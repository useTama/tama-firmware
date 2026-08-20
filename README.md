<img src="assets/icon.png" width="88" alt="">

# tama-firmware

Firmware for a physical Tama device: hold a button, talk, and a markdown note appears in a
folder you own. Talks to [tama-server](https://github.com/useTama/tama-server).

    [button held] --> ES7210 mic --> WAV --> POST /capture --> tama-server --> your vault

No wake word, no on-device model, no cloud account. The device records and uploads; the
server transcribes locally with whisper.cpp. **No inference ever runs on the ESP32** — it
holds a button, a mic, a screen and a socket.

## Status

**Boots and runs on real hardware** as of 2026-08-20. Confirmed working on the actual board:
display stack, both audio codecs over I²C, and the PSRAM record buffer.

Not yet verified: the network leg end to end (needs WiFi credentials in `secrets.h`), and
audio quality — see [Open questions](#open-questions).

## Hardware

Cheeko Gotchi board, silkscreen `OSTB_XIAOZHI_V1.2` — ESP32-S3, 8 MB octal PSRAM, 16 MB
flash, 240x296 ST7789 SPI display, CST810 touch, ES8311 speaker codec, ES7210 4-channel mic
ADC, LIS2DH12 accelerometer.

Capture runs through the **ES7210 only**. The ES8311's mic inputs are physically not
connected on this PCB, so firmware that records through it captures silence forever — a
board fact, not a driver bug.

## Build and flash

No IDE required. `arduino-cli` only:

    brew install arduino-cli
    arduino-cli config set board_manager.additional_urls \
      https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
    arduino-cli core update-index
    arduino-cli core install esp32:esp32

    cp tama_capture/secrets.h.example tama_capture/secrets.h
    # then fill in WiFi credentials, server address, and a device token

Mint the device token from a running tama-server:

    curl -X POST http://<host>:8080/tokens -H "Authorization: Bearer <adminToken>" \
      -H 'content-type: application/json' -d '{"deviceName":"cheeko-01"}'

Then:

    FQBN="esp32:esp32:XIAO_ESP32S3:PSRAM=opi,PartitionScheme=tinyuf2_noota"
    PORT=$(ls /dev/cu.usbmodem*)

    arduino-cli compile --fqbn "$FQBN" tama_capture
    arduino-cli upload  --fqbn "$FQBN" -p "$PORT" tama_capture
    arduino-cli monitor -p "$PORT" -c baudrate=115200

`PSRAM=opi` is mandatory — this board has *octal* PSRAM, and without the flag the record
buffer allocation returns null. There is no board definition for this exact hardware; the
Seeed XIAO ESP32-S3 profile is pin-compatible enough and, critically, stable. This FQBN also
writes a TinyUF2 recovery bootloader at `0x410000`, which is the safety net if a flash is
ever interrupted.

## Open questions

Both are unverified against real hardware and both fail *silently* rather than loudly, so
they are called out in the code as well:

- **Sample rate.** `board.json` claims the ES7210 runs at 24 kHz; the proven bring-up I²S
  config uses 16 kHz. If the ADC really delivers 24 kHz while the WAV header declares 16 kHz,
  audio replays 1.5x slow and transcripts come back as garbage that looks like a whisper
  problem. Verify by recording a known-length phrase and checking the server's reported
  `audioSeconds` against a stopwatch.
- **Channel mapping.** The ES7210 has four channels and MIC3 is a speaker loopback for echo
  cancellation. The reference bring-up reads it as plain 2-channel I²S and its level meter
  worked, so that is the evidence-backed choice, but which physical mics land in L and R is
  not established.

## Credit

The pin map, display init sequence, and ES7210 register sequence come from
[raviramp36/cheeko-gotchi](https://github.com/raviramp36/cheeko-gotchi), which contains the
board schematic and the original working bring-up sketches for this hardware. That work is
what made this firmware a config-and-glue job instead of a driver-writing one.

There is also an official Cheeko Gotchi SDK presenting a clean C++ app API. Its firmware
layer is currently a scaffold — the display, touch and audio services are empty stubs with
`// TODO(board)` comments — so it will not drive this hardware as-is. Useful as an
architectural reference.

## Licence

Apache-2.0
