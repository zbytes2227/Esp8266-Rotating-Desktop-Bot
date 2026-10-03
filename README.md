An ESP8266 D1 Mini table robot with a motorized pan head, expressive OLED eyes, touch reactions, sound effects, Wi‑Fi clock/weather screens, battery monitoring, and multiple behavior modes.

<img width="1536" height="1024" alt="image" src="https://github.com/user-attachments/assets/540d6b86-43a1-48ca-b9cc-208be256f63e" />


## Features

- Custom animated OLED face with pupils, blinking, saccades, and expressions.
- Smooth open-loop head panning with automatic return-to-center behavior.
- Four modes: `DEFAULT`, `GUARD`, `CLOCK`, and `DANCE`.
- Two touch inputs for reactions, effects, and sleep/wake control.
- Mode button for cycling behaviors.
- OLED screens for local time, weather, and battery status.
- Wi‑Fi/NTP time synchronization and weather data from `wttr.in`.
- Non-blocking buzzer effects and battery percentage estimation.
- One-way serial debugging at 115200 baud.

## Repository contents

| File | Description |
| --- | --- |
| [`table_robot_v9.ino`](table_robot_v9.ino) | Main firmware for the complete robot |
| [`oled_screen_effects.txt`](oled_screen_effects.txt) | Standalone OLED lyric/effects experiment; not included by the main sketch |

## Hardware

The firmware is written for an ESP8266 D1 Mini.

| Function | D1 Mini pin / GPIO |
| --- | --- |
| Head motor BIN1 | D1 / GPIO5 |
| Head motor BIN2 | D2 / GPIO4 |
| Head motor PWM | D7 / GPIO13 |
| Touch sensor 1 | D0 / GPIO16 |
| Touch sensor 2 | D4 / GPIO2 |
| OLED SCL | D5 |
| OLED SDA | D6 |
| Battery divider | A0 |
| Buzzer driver | RX / GPIO3 |
| Mode button | D3 / GPIO0, button to GND |

The OLED uses I2C address `0x3C` and is configured for 128×64 pixels. The buzzer is driven through a transistor; do not connect a buzzer directly to the ESP8266 GPIO.

## Required libraries

Install these libraries through the Arduino IDE Library Manager:

1. Adafruit GFX Library
2. Adafruit SSD1306
3. FluxGarage RoboEyes

The sketch keeps the RoboEyes dependency instantiated, while the visible face is rendered by its custom `FaceEngine` implementation.

## Setup

1. Install the ESP8266 board package in Arduino IDE.
2. Install the libraries listed above.
3. Open `table_robot_v9.ino`.
4. Set the Wi‑Fi credentials near the top of the sketch:

   ```cpp
   const char* WIFI_SSID     = "your-network";
   const char* WIFI_PASSWORD = "your-password";
   ```

   Do not commit real credentials to a public repository.

5. Select an ESP8266 D1 Mini board and the correct port.
6. Upload the sketch.

The sketch assumes the head starts physically centered at boot. Because the motor has no encoder, position is estimated from applied PWM and may require tuning for the mechanism.

## Controls

- **Mode button:** cycle through `DEFAULT` → `GUARD` → `CLOCK` → `DANCE`.
- **Touch 1, short tap:** trigger a default-mode reaction; cycle clock information screens in `CLOCK` mode.
- **Touch 2, short tap:** trigger a second reaction; cycle clock information screens in `CLOCK` mode.
- **Either touch, long press:** enter or leave sleep mode.
- **Both touch inputs:** trigger the combined reaction in `DEFAULT` mode.

In `CLOCK` mode, touch cycles through time, weather, and battery screens. Screens automatically return to the face after a short timeout.

## Wi‑Fi and weather

The firmware uses NTP servers to obtain IST time and periodically fetches weather for Lucknow from:

```text
http://wttr.in/Lucknow?format=%25C+%25t+(feels+%25f)
```

Change `WEATHER_URL` in the sketch if the robot is used in another location. Weather and time features continue to run opportunistically when Wi‑Fi is available.

## Important pin and boot notes

- GPIO3/RX is used by the buzzer, so the robot cannot receive serial commands from the PC. Serial output for debugging remains available.
- GPIO0/D3 is the ESP8266 flash-mode strap pin. Do not hold the mode button while powering on or resetting, or the board may enter the bootloader instead of starting the firmware.
- The battery must be connected through an appropriately scaled voltage divider. Verify the divider and calibration before relying on the displayed percentage.

## Tuning

Mechanisms and sensors vary. The most relevant constants are grouped near the top of the sketch:

- `MIN_PAN_SPEED`, `MAX_PAN_SPEED`, and `RAMP_STEP_SIZE` for motor smoothness.
- `GUARD_SIDE_TARGET` for the open-loop guard sweep range.
- `CENTER_DEADBAND` and `POSITION_TRAVEL_TIMEOUT_MS` for recentering.
- `VOLTAGE_CALIBRATION`, `BATTERY_FULL_V`, and `BATTERY_EMPTY_V` for battery readings.
- `GMT_OFFSET_SEC` and `WEATHER_URL` for location settings.

## License

No license is currently specified. Add a license file before redistributing the project.
