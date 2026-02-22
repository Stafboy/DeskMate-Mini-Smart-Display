# DeskMate Mini Smart Display

A feature-rich ESP32 smart display with animated robot eyes, clock, weather, Spotify integration, crypto prices, games, and more — running on a 128x64 OLED screen.
-> i used a Wemos Lite V1 ESP32, and connected a small 650mah 1s lithium ion battery pack to the esp to make it portable but this is optional
---

## Hardware

| Component | Pin |
|-----------|-----|
| SDA (OLED) | GPIO 22 |
| SCL (OLED) | GPIO 19 |
| OLED Address | 0x3C |
| Button Left | GPIO 15 |
| Button Middle | GPIO 2 |
| Button Right | GPIO 13 |
| Buzzer | GPIO 17 |

**Display:** SSD1306 128x64 I2C OLED (1.54inch)

---

## Required Libraries

Install these via Arduino IDE > Library Manager:

- `U8g2` by olikraus
- `FluxGarage_RoboEyes`
- `ArduinoJson`
- `NTPClient`
- `base64` (by Arturo Guadalupi)

---

## Configuration

Before uploading, open `roboeyes_v19.ino` and fill in the following placeholders near the top of the file:

### 1. WiFi

```cpp
const char* ssid     = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
```

Replace with your home network name and password.

---

### 2. Location (for weather)

```cpp
const double LATITUDE  = 0.00;
const double LONGITUDE = 0.00;
```

Enter your coordinates. You can find them by right-clicking your location on [Google Maps](https://maps.google.com) and copying the coordinates shown. For example, London is `51.50, -0.12`.

The weather data is fetched from [Open-Meteo](https://open-meteo.com) — **no API key required**.

---

### 3. Spotify (optional)

```cpp
const char* SP_ID      = "YOUR_SPOTIFY_CLIENT_ID";
const char* SP_SECRET  = "YOUR_SPOTIFY_CLIENT_SECRET";
const char* SP_REFRESH = "YOUR_SPOTIFY_REFRESH_TOKEN";
```

To get these values, follow the steps below.

#### How to get your Spotify credentials

**Step 1 — Create a Spotify App**

1. Go to [https://developer.spotify.com/dashboard](https://developer.spotify.com/dashboard) and log in.
2. Click **Create App**.
3. Give it any name and description.
4. Set the **Redirect URI** to `http://localhost:8888/callback`
5. Click **Save**.
6. On the app page, you will see your **Client ID** and **Client Secret** — copy these into `SP_ID` and `SP_SECRET`.

**Step 2 — Get a Refresh Token**

The refresh token lets the device access your Spotify account without you re-logging in every time.

Use the included helper script `get_refresh_token.py` (requires Python 3 and the `requests` library):

```bash
pip install requests
python get_refresh_token.py
```

The script will open a browser window asking you to authorize the app. After you approve it, it prints a refresh token — paste that into `SP_REFRESH`.

> If you prefer to do it manually, you can follow the [Spotify Authorization Code Flow guide](https://developer.spotify.com/documentation/web-api/tutorials/code-flow).

---

## Upload

1. Select your board: **ESP32 Dev Module** (or equivalent)
2. Set **Partition Scheme** to `Default 4MB with spiffs` or `Huge APP`
3. Upload speed: `115200`
4. Click **Upload**

On first boot the display shows **DeskMate Mini / Smart Display** and attempts to connect to WiFi.

---

## Screens & Navigation

| Screen | Description |
|--------|-------------|
| 0 | Clock + Weather |
| 1 | Detailed Weather |
| 2 | Games menu |
| 5 | Daily Quote |
| 9 | Spotify Now Playing |
| 10 | Crypto / Stock charts |

**Left / Right buttons** — cycle screens or navigate menus  
**Middle button** — confirm / interact  
**Long press** — secondary actions (settings, reset, etc.)

---

## Notes

- Weather updates every 10 minutes while awake, every hour while the screen is sleeping.
- The display auto-dims after 5 minutes of inactivity.
- High scores are saved to EEPROM and persist across reboots.
- Crypto data uses the free Kraken public API (no key needed) for BTC and PAXG, and Yahoo Finance for SPY.
