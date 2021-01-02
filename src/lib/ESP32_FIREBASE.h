#ifndef ESP32_FIREBASE_H_
#define ESP32_FIREBASE_H_

#include <Arduino.h>
#include <WiFi.h>
#include <FirebaseArduino.h>
#include <String.h>

void connect_to_wifi();
void connect_to_firebase();
void push_value_to_firebase(string path, uint8_t value);
void get_value_from_firebase(string path, uint8_t value);

#endif
