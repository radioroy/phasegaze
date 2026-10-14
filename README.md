# PhaseGaze

Swept-LO phase-gradient hemisphere viewer for QuadRF.

The Wi-Fi mode receiver (`src/wifi.c`) ports its 802.11a/g sync and
equalization from [gr-ieee802-11](https://github.com/bastibl/gr-ieee802-11)
by Bastian Bloessl (GPL-3.0-or-later).

## Build

```bash
cmake -S . -B build
cmake --build build -j
```
