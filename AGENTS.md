# AGENTS.md — droneengage_IR_camera

DroneEngage IR Camera module (`de_ir_camera`). Thermal imaging detection
and RGB+IR fusion over an MI48 sensor; publishes thermal blobs as
observations (`obs`, 6546) on the bus. **Detection/fusion only** — object
tracking is `droneengage_tracking`. C++17, OpenCV, mi48 SDK.

## Quickstart

    ./build.sh                 # DEBUG -> bin/de_ir_camera
    make -C test test          # hotspot_detector unit tests

## Index — `.agents/wiki/`

| File | What it covers |
|---|---|
| [`agents_overview.md`](.agents/wiki/agents_overview.md) | module description, build/deps (mi48), config files, source layout |

See `../../AGENTS.md` for the virtual video device pipeline contract and
`de_common` vendoring.
