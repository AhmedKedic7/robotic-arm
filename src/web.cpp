#include "web.h"
#include <atomic>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include "arm.h"
#include "config.h"
#include "secrets.h"
#include "web_ui.h"

namespace {

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// The client that last commanded motion. If it goes quiet the arm holds.
std::atomic<uint32_t> commanderId{0};
std::atomic<uint32_t> commanderLastMs{0};

String lastBroadcast;
uint32_t lastBroadcastMs = 0;

String buildConfigJson() {
  JsonDocument doc;
  doc["t"] = "cfg";
  JsonArray joints = doc["joints"].to<JsonArray>();
  for (const auto& j : JOINTS) {
    JsonObject o = joints.add<JsonObject>();
    o["name"] = j.name;
    o["min"] = j.minDeg;
    o["max"] = j.maxDeg;
  }
  JsonArray poses = doc["poses"].to<JsonArray>();
  for (const auto& p : POSES) poses.add(p.name);
  doc["speedMin"] = SPEED_MIN_DEG_S;
  doc["speedMax"] = SPEED_MAX_DEG_S;

  String out;
  serializeJson(doc, out);
  return out;
}

String buildStateJson() {
  ArmState s = arm.snapshot();

  JsonDocument doc;
  doc["t"] = "state";
  JsonArray cur = doc["cur"].to<JsonArray>();
  JsonArray tgt = doc["tgt"].to<JsonArray>();
  JsonArray on  = doc["on"].to<JsonArray>();
  for (int j = 0; j < NUM_JOINTS; j++) {
    cur.add(serialized(String(s.cur[j], 1)));
    tgt.add(serialized(String(s.tgt[j], 1)));
    on.add(s.enabled[j]);
  }
  doc["spd"] = serialized(String(s.speed, 0));
  doc["moving"] = s.moving;
  doc["estop"] = s.estopped;
  doc["fault"] = toString(s.fault);

  String out;
  serializeJson(doc, out);
  return out;
}

void sendError(AsyncWebSocketClient* client, const char* msg) {
  JsonDocument doc;
  doc["t"] = "err";
  doc["msg"] = msg;
  String out;
  serializeJson(doc, out);
  client->text(out);
}

void markCommander(AsyncWebSocketClient* client) {
  commanderId = client->id();
  commanderLastMs = millis();
}

void handleMessage(AsyncWebSocketClient* client, const uint8_t* data, size_t len) {
  if (client->id() == commanderId) commanderLastMs = millis();

  JsonDocument doc;
  if (deserializeJson(doc, data, len)) {
    sendError(client, "invalid JSON");
    return;
  }

  const char* type = doc["t"] | "";
  CmdResult res = CmdResult::Ok;

  if (!strcmp(type, "ping")) {
    return;
  } else if (!strcmp(type, "set")) {
    if (!doc["j"].is<int>() || !doc["a"].is<float>()) {
      res = CmdResult::BadValue;
    } else {
      res = arm.setTarget(doc["j"].as<int>(), doc["a"].as<float>());
      if (res == CmdResult::Ok) markCommander(client);
    }
  } else if (!strcmp(type, "pose")) {
    const char* name = doc["n"] | "";
    const PoseConfig* pose = nullptr;
    for (const auto& p : POSES) {
      if (!strcmp(p.name, name)) pose = &p;
    }
    if (!pose) {
      res = CmdResult::BadValue;
    } else {
      res = arm.moveToPose(pose->deg);
      if (res == CmdResult::Ok) {
        markCommander(client);
        Serial.printf("Pose: %s\n", pose->name);
      }
    }
  } else if (!strcmp(type, "speed")) {
    res = doc["v"].is<float>() ? arm.setSpeed(doc["v"].as<float>()) : CmdResult::BadValue;
  } else if (!strcmp(type, "stop")) {
    arm.eStop();
  } else if (!strcmp(type, "resume")) {
    arm.resume();
  } else {
    sendError(client, "unknown command");
    return;
  }

  if (res != CmdResult::Ok) sendError(client, toString(res));
}

void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type,
               void* arg, uint8_t* data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      if (server->count() > MAX_WS_CLIENTS) {
        client->close();
        return;
      }
      Serial.printf("WS client %u connected\n", client->id());
      client->text(buildConfigJson());
      client->text(buildStateJson());
      break;

    case WS_EVT_DISCONNECT:
      Serial.printf("WS client %u disconnected\n", client->id());
      if (client->id() == commanderId) {
        commanderId = 0;
        arm.hold();
      }
      break;

    case WS_EVT_DATA: {
      auto* info = static_cast<AwsFrameInfo*>(arg);
      // Commands are tiny; ignore fragmented or binary frames.
      if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
        handleMessage(client, data, len);
      }
      break;
    }

    default:
      break;
  }
}

}  // namespace

void webBegin() {
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);  // lower, steadier latency for control traffic
  if (!WiFi.softAP(AP_SSID, AP_PASS)) {
    Serial.println("ERROR: softAP failed (password must be 8-63 chars)");
  }
  Serial.printf("WiFi AP: %s  ->  http://%s\n", AP_SSID, WiFi.softAPIP().toString().c_str());

  if (MDNS.begin(MDNS_HOSTNAME)) {
    MDNS.addService("http", "tcp", 80);
    Serial.printf("mDNS: http://%s.local\n", MDNS_HOSTNAME);
  }

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send(200, "text/html", reinterpret_cast<const uint8_t*>(INDEX_HTML), strlen(INDEX_HTML));
  });

  // Read-only JSON for debugging / scripting. All control goes over /ws.
  server.on("/status", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send(200, "application/json", buildStateJson());
  });

  server.onNotFound([](AsyncWebServerRequest* req) { req->send(404, "text/plain", "Not found"); });

  server.begin();
}

void webUpdate() {
  uint32_t now = millis();
  if (now - lastBroadcastMs < BROADCAST_MS) return;
  lastBroadcastMs = now;

  ws.cleanupClients(MAX_WS_CLIENTS);

  // Dead-man: the commanding client vanished without a clean disconnect.
  uint32_t cmdId = commanderId;
  if (cmdId && now - commanderLastMs > DEADMAN_TIMEOUT_MS) {
    commanderId = 0;
    if (arm.snapshot().moving) {
      Serial.printf("Dead-man: client %u silent, holding\n", cmdId);
      arm.hold();
    }
  }

  if (ws.count() == 0) return;

  String state = buildStateJson();
  // Push on change, plus a 1 s heartbeat so clients can detect a stale link.
  static uint32_t lastHeartbeatMs = 0;
  if (state != lastBroadcast || now - lastHeartbeatMs >= 1000) {
    ws.textAll(state);
    lastBroadcast = state;
    lastHeartbeatMs = now;
  }
}
