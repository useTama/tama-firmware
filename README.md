# tama-firmware

Firmware for a physical Tama device: a button, a mic, a speaker, a socket. Talks to
[tama-server](https://github.com/sxivansx/tama-server).

**Status: parked.** Nothing here yet. The board is in hand and its bring-up details are
already verified; work resumes once the server and its first software client have been
used for a while and are worth building hardware around.

## The plan, when it resumes

Fork [78/xiaozhi-esp32](https://github.com/78/xiaozhi-esp32) and add a board definition for
the target hardware, pointed at a `tama-server` instance instead of its stock backend.
Drivers, provisioning, OTA and the audio path are upstream already; the new part is a small
state machine with no ESP dependencies, so it can be written and unit-tested off-device
before anything is flashed.

**No inference ever runs on the device.** No speech-to-text, no language model, no
embeddings on the ESP32. The board is I/O and a network client; the server does the work.

## Target hardware

A [Cheeko Gotchi](https://github.com/ALTIO-AI-PRIVATE-LIMITED/cheeko-gotchi) board
(`OSTB_XIAOZHI_V1.2`), xiaozhi lineage: ESP32-S3, 8 MB octal PSRAM, ST7789 display, ES8311
codec, ES7210 mic array. Full pin map and known hardware traps live with the project notes,
not here yet — they'll land in this repo once firmware work actually starts.

## Licence

Apache-2.0
