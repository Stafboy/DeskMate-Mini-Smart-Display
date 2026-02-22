# DeskMate Mini Smart Display
DeskMate runs on an ESP32 with a 128x64 OLED driven by the U8g2 library over I2C. Three hardware buttons handle navigation and media control, and a buzzer manages alarms. The device connects to WiFi, syncs time using NTP, and stores settings in EEPROM.

For weather and crypto data, it sends HTTPS requests through WiFiClientSecure, parses JSON responses with ArduinoJson, and renders clean UI screens. Spotify control works through the Web API using secure HTTP calls and base64 authentication, enabling play, pause, skip, and replay.

RoboEyes uses the FluxGarage_RoboEyes library with a custom display adapter that maps Adafruit GFX functions to U8g2. The system switches between time, weather, crypto, quotes, alarm, and animated eye modes through a structured screen state manager.
