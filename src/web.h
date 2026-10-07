#pragma once

// Wi-Fi access point, HTTP page, and the WebSocket control channel.
void webBegin();
void webUpdate();  // call from loop(): broadcasts state, enforces dead-man timeout
