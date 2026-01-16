# esp32-pinout-tester

Small PlatformIO project for identifying ESP32 GPIOs.

Three modes:
- PWM sensing (default): drives each GPIO with a 1 kHz square wave and detects which pin is connected to the sense input.
- Morse output: blinks each GPIO number in Morse code.
- Pin toggle (button-stepped): pulses each GPIO on button press and logs the pin/state.

## Quick start

1) Pick the environment for your board in `platformio.ini`:
- `esp32s3`
- `d1mini_esp32`

2) Build and upload (default PWM sensing):
```sh
pio run -e esp32s3 -t upload --upload-port /dev/cu.usbmodemXXXX
```

3) Monitor:
```sh
pio device monitor
```

## Mode selection

Modes are toggled by uncommenting flags in the global `[env]` section of `platformio.ini`:
```ini
[env]
build_flags =
  ; -DMORSE
  ; -DTEST_BUTTON
  ; -DPINS_TOGGLE
  ; -DPINS_LEVEL=HIGH
```

- Default (no flags): PWM sensing mode.
- `-DMORSE`: Morse mode.
- `-DTEST_BUTTON`: enables the Morse test button behavior (Morse mode only).
- `-DPINS_TOGGLE`: enables button-stepped pin toggling.
- `-DPINS_LEVEL=HIGH` or `-DPINS_LEVEL=LOW`: idle level for `PINS_TOGGLE`.

After changing flags, rebuild and upload.

## PWM sensing mode

Wiring:
- Connect the pin you want to identify to the sense input pin (`SENSE_IN_PIN`).
- The status LED (`STATUS_LED_PIN`) blinks Morse status:
  - `E`, `I` during confirm passes
  - `S` on success
  - `O` on disconnect

The serial log prints on successful detection:
```
Connected: GPIO35
Connected: GPIO35 (2)
```
The number in parentheses increments only on reconnects to the same pin.

Pin mapping is configured in `platformio.ini` only (no edits in `src/main.cpp`).  
Set the pins in the env you are using:
```ini
build_flags =
  -DBOARD_ESP32S3
  -DSENSE_IN_PIN=44
  -DSTATUS_LED_PIN=40
```

## Morse mode

Each GPIO blinks its numeric pin ID in Morse code, in parallel. Timing is based on `kDashMs` in `src/main.cpp`.

### Morse test button (optional)

If `-DTEST_BUTTON` is enabled in Morse mode:
- A button press forces `TEST_LED_PIN` to blink at 5 Hz.
- Releasing returns the pin to Morse output.

Pin configuration example:
```ini
build_flags =
  -DBOARD_ESP32S3
  -DMORSE
  -DTEST_BUTTON
  -DTEST_BUTTON_PIN=17
  -DTEST_LED_PIN=40
```

## Pin toggle mode

Each button press targets one pin, pulses it to the opposite of `PINS_LEVEL` for 200 ms, then returns it to `PINS_LEVEL`.
The serial log prints the pin and the pulse/idle levels.

Pin configuration example:
```ini
build_flags =
  -DBOARD_ESP32S3
  -DPINS_TOGGLE
  -DPINS_LEVEL=HIGH
  -DTEST_BUTTON_PIN=17
```

## Board environments

Configured in `platformio.ini`:
- `esp32s3`
- `d1mini_esp32`

Each env defines the board and USB-CDC settings as needed.

## Notes

- Avoid flash pins, strapping pins, and input-only pins unless you know the risks.
