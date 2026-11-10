# Changelog

All notable changes to this project are documented in this file.
The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Fixed

- **Wrong channel triggered.** Channel switching assumed that the first CH press only wakes
  the remote and that the firmware's stored channel stays correct. While the remote was still
  awake, each step advanced two channels instead of one, so commands reached the wrong blind and
  the error accumulated. The firmware now re-detects the remote's channel before every command
  and advances with single CH presses (see "Changed").
- Command queue memory corruption: the queue copied a full command struct into a pointer
  variable, overwriting neighbouring memory. Commands are now stored by value; the heap
  allocation and the memory leak on a full queue are gone.
- A command for channel 5 blocked the queue forever. Only channels 0–4 are accepted now.
- Unknown payloads (e.g. `UP`, `OPEN`) were queued as "no command" and pressed the CH button,
  which desynchronised the channel tracking. They are now ignored.
- Payloads were matched by prefix (`upxyz` was treated as `up`). Matching is now exact and
  case-insensitive.
- Button presses relied on reading back an output pin, which does not work on all ESP32 core
  versions. Buttons are now driven LOW, then HIGH, explicitly.
- The LED edge counter shared with the interrupt is now `volatile` and read/reset atomically;
  the input pin is configured before the interrupt is attached.
- Status messages were published as retained by accident (`80` was passed as the `retained`
  flag) and their payload was not valid JSON.
- Missing or mistyped entries in `config.json` crashed the device (`strlen(NULL)`), and a too
  small JSON buffer could make parsing fail silently. Errors are now reported on Serial and
  setup stops with a clear message.
- Wrong include name `ESP8266Wifi.h` for ESP8266 builds (now `ESP8266WiFi.h`).

### Changed

- Channel selection: CH is pressed until the remote shows "all" (fast pulses on the CH1 LED
  line) or channel 1 (steady LOW), then the target is reached one press at a time. The LEDs are
  checked after every press; on a mismatch the sequence is restarted (up to 3 attempts). On
  failure the command is dropped and an error is published. A command now takes up to about
  4 s before the button is pressed.
- MQTT topics are derived from `MQTT_TOPIC_BASE` (default `esp/jalousie`) instead of
  hard-coded `shades/terasse/...` strings.
- `.../state/CH_SELECT` and `.../state/COMMAND_LAST` now publish valid JSON, e.g.
  `{"channel": 2}` and `{"command": "s_UP"}`, retained. They are also published right after
  each command.
- The "Hello World" message is no longer published (retained) to `cmd/channel99`, where the
  firmware received its own message on every reconnect.
- The MQTT connection is serviced while a command is running, so commands arriving meanwhile
  are queued. Reconnect attempts are made every 5 s instead of on every loop pass.
- `mqttPort` in `config.json` may be given as a string (`"1883"`) or a number (`1883`).
- The configuration file content (including passwords) is no longer printed to Serial.
- Remote timings are collected as constants at the top of `src/main.cpp`.

### Added

- Availability topic `.../state/availability` (`online` / `offline` via MQTT last will).
- Error topic `.../state/ERROR` for failed channel switches and a full queue.
- `STARTUP_ALL_UP` switch for the existing behaviour of sending "up" to all blinds after
  power-up (enabled by default).
- Serial diagnostics for every LED observation (`observe -> edges: N, CH1 level: X`).

### Removed

- Use of the `nickgammon/Regexp` library. It is still listed in `platformio.ini` and can be
  removed there.

### Upgrade notes

- Clear the retained message left by the old firmware:
  `mosquitto_pub -t <base>/cmd/channel99 -r -n`
- If you consume `CH_SELECT` / `COMMAND_LAST`, update your parsers to the new JSON payloads.
- Optionally add `availability_topic: "esp/jalousie/state/availability"` to the Home Assistant
  cover definitions.
