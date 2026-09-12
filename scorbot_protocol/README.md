# scorbot_protocol

The contract between the ROS 2 hardware interface on the Raspberry Pi and the ESP32
motor controller firmware. One `.proto`, nanopb on both sides, COBS/CRC framing on
serial. No ROS dependencies, so the firmware and bench tools use it as-is.

The wire specification for firmware authors is **`docs/protocol.md`**.

## Layout

| Path | What |
| --- | --- |
| `proto/scorbot/v1/scorbot.proto` | the message definitions (source of truth) |
| `proto/scorbot/v1/scorbot.options` | nanopb sizes: fixed arrays (8 joints), fixed strings |
| `generated/scorbot/v1/scorbot.pb.{h,c}` | nanopb output, **committed**; regenerate with `scripts/regenerate.sh` |
| `third_party/nanopb/` | nanopb 0.4.9.1 runtime (zlib licence), vendored so both sides match |
| `include/scorbot_protocol/cobs.hpp` | COBS encode/decode, header-only |
| `include/scorbot_protocol/crc16.hpp` | CRC-16/CCITT-FALSE, header-only |
| `include/scorbot_protocol/framing.hpp` | `encodeFrame`, `StreamDecoder` (resynchronising, no allocation) |
| `include/scorbot_protocol/messages.hpp` | encode/decode an `Envelope`, typed request builders, enum names |
| `include/scorbot_protocol/transport.hpp` | `Transport` interface, `FdTransport`, `SerialTransport`, `TransportPair` |
| `include/scorbot_protocol/link.hpp` | `Link`: envelopes over a transport, sequence numbers, stats, request/response |
| `src/cli.cpp` | `scorbot_protocol_cli`: ping, info, monitor, enable, home, ... |
| `test/` | gtest: COBS vectors and fuzz, CRC, framing resync, message round trips, link over a socketpair and a pty |

## Using it

From a ROS 2 workspace it is an ordinary ament package (`find_package(scorbot_protocol)`,
link `scorbot_protocol`). Standalone: `cmake -B build && cmake --build build` works with
no ROS installed (tests need GoogleTest).

Bench:

```bash
ros2 run scorbot_protocol scorbot_protocol_cli --port /dev/ttyUSB0 ping
ros2 run scorbot_protocol scorbot_protocol_cli --port /dev/ttyUSB0 info
ros2 run scorbot_protocol scorbot_protocol_cli --port /dev/ttyUSB0 monitor 5
ros2 run scorbot_protocol scorbot_protocol_cli --port /dev/ttyUSB0 enable --allow-unhomed
```

## Firmware side

The ESP-IDF component needs exactly these files: `generated/scorbot/v1/scorbot.pb.{h,c}`,
`third_party/nanopb/pb*.{h,c}`, and a C port of `cobs.hpp`/`crc16.hpp` (they are
plain functions; the C version is part of the firmware work). Build with
`PB_ENABLE_MALLOC=0`. Copy them, do not fork them: CI checks that the generated files
match the `.proto`, and the firmware copy should be diffed against this package in its
own CI.

## Changing the protocol

1. Edit `scorbot.proto` (add fields only; never renumber) and `scorbot.options` if sizes change.
2. `python3 -m pip install "nanopb==0.4.9.1" grpcio-tools && scripts/regenerate.sh`
3. Bump `kProtocolMinor` (additions) or `kProtocolMajor` (semantic changes) in `messages.hpp`
   and the firmware's `GetInfo` response.
4. Update `docs/protocol.md`.
