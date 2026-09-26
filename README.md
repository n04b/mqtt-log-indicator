# mqtt-log-indicator

MQTT broker activity indicator for Raspberry Pi on a **MAX7219 4 × 8×8** LED matrix mounted vertically (8 × 32).

Every MQTT message becomes one row at the bottom of the screen, older rows scroll up like a console log. The number of lit LEDs in a row shows the payload length (payload itself is never displayed).

```text
        8 LED
    ┌──────────┐
    │ ●....... │  oldest
    │ ●●●●●●.. │
    │ ●●●..... │
    │   ...    │
    │ ●●●●●●●● │  newest
    └──────────┘
      32 rows
```

## Features

- Subscribes to `#` with username/password, reconnects and re-subscribes automatically.
- Length in **characters** (UTF-8), invalid UTF-8 is handled with `errors="replace"`.
- Three scales: `linear`, `log` (default, base 3) and `adaptive` (percentile of recent traffic).
- Retained messages (Home Assistant discovery, zigbee2mqtt bridge dumps, …) are skipped by default.
- Wrong login/password → checkerboard on the whole display, inverted on every connection attempt.
- Blink command topic.
- Orientation test mode to match any module wiring.

## Hardware

| MAX7219   | Raspberry Pi 4 | Pin |
| --------- | -------------- | --: |
| VCC       | 5V             |   2 |
| GND       | GND            |   6 |
| DIN       | GPIO10 / MOSI  |  19 |
| CLK       | GPIO11 / SCLK  |  23 |
| CS / LOAD | GPIO8 / CE0    |  24 |

## Install

```bash
sudo raspi-config                      # Interface Options → SPI → enable
sudo apt install python3-spidev python3-paho-mqtt
sudo usermod -aG spi $USER

cp mqtt_display.py ~/
nano ~/mqtt_display.py                 # set MQTT_HOST / MQTT_USERNAME / MQTT_PASSWORD
```

All settings are in the `CONFIGURATION` block at the top of `mqtt_display.py`.

## Run

```bash
python3 mqtt_display.py                # MAX7219 via SPI
python3 mqtt_display.py --console      # no hardware, ASCII output
python3 mqtt_display.py --test         # cycle through orientation variants
python3 mqtt_display.py --test --once  # test pattern with current settings
```

Log output:

```text
SCALE=log | 1 LED: 1-2 | 2 LED: 3-8 | 3 LED: 9-26 | 4 LED: 27-80 | 5 LED: 81-242 | 6 LED: 243-728 | 7 LED: 729-2186 | 8 LED: 2187+
MQTT | connected to localhost:1883, subscribing to #
MQTT | skipped 590 retained messages
MQTT | topic=zigbee2mqtt/0x84b4dbfffe247b3c | length=609 | LEDs=6
```

## systemd

Edit `User=` and the path in `mqtt-display.service` if needed, then:

```bash
sudo cp mqtt-display.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now mqtt-display
journalctl -u mqtt-display -f
```

## Scales

| `SCALE`    | How LEDs are counted |
| ---------- | -------------------- |
| `linear`   | `ceil(length / CHARS_PER_LED)` |
| `log`      | each next LED needs `LOG_BASE` times more characters, starting at `LOG_MIN_CHARS` |
| `adaptive` | percentile of the length among the last `ADAPTIVE_WINDOW` messages × 8 |

All scales are capped at `MAX_LEDS_PER_ROW`; an empty payload gives an empty row (still added to history).

## Blink command

```bash
mosquitto_pub -t /mqtt-log-indicator/blink -m true    # 1 second
mosquitto_pub -t /mqtt-log-indicator/blink -m 5       # 5 seconds (max 60)
mosquitto_pub -t /mqtt-log-indicator/blink -m false   # stop
```

The display flashes at maximum brightness, then returns to the previous picture and brightness. The command does not add a row; retained blink commands are ignored.

## Orientation

Run `python3 mqtt_display.py --test`. The correct picture is one triangle over the whole height, pressed to the **left** edge: 1 LED wide at the top, +1 LED every 4 rows, 8 LEDs at the bottom. Copy the settings printed for the correct variant into the config.

| What you see | Change |
| ------------ | ------ |
| Triangle pressed to the right edge | `SCREEN_FLIP_X` |
| Triangle upside down | `SCREEN_FLIP_Y` |
| Inside each module the width shrinks downwards | `MODULE_FLIP_X` |
| Each module is correct but modules are in wrong order | `MODULE_TOP_IS_FIRST` |
| One module rotated differently | `MODULE_ROTATION_OVERRIDE = {index: degrees}` |
