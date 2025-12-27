//
// Created by kayla on 30.10.2025.
//

#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#endif //SERIAL_PROTOCOL_H
// firmware/secondary_core/serial_protocol.h
#include <ArduinoJson.h>

void handleCommand(String jsonLine) {
    StaticJsonDocument<200> doc;
    DeserializationError err = deserializeJson(doc, jsonLine);
    if (err) return;

    const char* cmd = doc["cmd"];
    if (strcmp(cmd, "drive") == 0) {
        float left = doc["speedL"];
        float right = doc["speedR"];
        setMotorSpeeds(left, right);
    }
    else if (strcmp(cmd, "ping") == 0) {
        Serial.println("{\"reply\": \"pong\"}");
    }
}
