# Smart ACL Recovery Brace
 
## Files
 
- `smart_brace_esp32.ino` — runs on the injured-leg ESP32
- `haptics_code.ino` — LED / OLED / motor module, compiled with the main sketch
- `healthy_leg_esp32.ino` — runs on the healthy-leg ESP32
- `dashboard.html` — web dashboard, just open it in a browser
Put the first two in the same folder so the Arduino IDE picks them up as one sketch:
 
```
smart_brace_esp32/
    smart_brace_esp32.ino
    haptics_code.ino
healthy_leg_esp32/
    healthy_leg_esp32.ino
dashboard.html
```
 
## Hardware
 
- 2x ESP32 microcontrollers
- 2x MPU-6050 IMUs
- 2x HX711 + heel/toe load cell pairs (one set per leg)
- 1x SSD1306 OLED, 1x WS2812B LED, 1x coin vibration motor (injured leg only)
Pins on the injured-leg ESP32: I2C on 21/22, HX711 on 18/19, LED on 5, motor on 25, OLED on the I2C bus at 0x3C. Tie the thigh IMU's AD0 to GND and the shin IMU's AD0 to 3.3V so they get different I2C addresses.
 
The healthy-leg ESP32 only has the HX711 wired up, also on pins 18/19.
 
## Libraries
 
Install these from the Arduino Library Manager before building:
 
- WebSockets (Markus Sattler)
- Adafruit MPU6050 + Adafruit Unified Sensor
- HX711
- FastLED
- Adafruit NeoPixel
- Adafruit GFX + Adafruit SSD1306
You also need the ESP32 board package (Boards Manager → "esp32" by Espressif).
 
## Flashing
 
1. Open `smart_brace_esp32.ino` in the Arduino IDE, pick ESP32 Dev Module and the right COM port, and upload.
2. Do the same with `healthy_leg_esp32.ino` on the second board.
The default WiFi is SSID `ACL_Brace`, password `aclbrace1`. If you change one, change the other to match.
 
## Calibration
 
The `CALIB_HEEL` and `CALIB_TOE` constants near the top of both sketches are for our load cells and won't be right for yours. Put a known weight on each cell, divide the raw reading by the weight in kg, and replace the constant. Flip the sign if it reads negative.
 
If knee angle reads inverted, flip one of the `_SIGN` constants in `smart_brace_esp32.ino` from `1.0f` to `-1.0f`.
 
## Running it
 
1. Power on the injured-leg ESP32.
2. On a laptop or tablet, connect to the `ACL_Brace` WiFi network.
3. Open `dashboard.html`. The IP `192.168.4.1` is already filled in.
4. Have the patient stand straight, then click Connect — the brace tares and zeros itself at that moment.
5. Use the Tare and Calibrate buttons in the header to re-zero anytime. Export CSV dumps the session data.

## Model files / binaries

None. This project has no trained models or pre-built binaries. The Arduino IDE compiles the ESP32 firmware (~900 KB .bin per board) from the .ino sources at build time.
