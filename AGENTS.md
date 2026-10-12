# AGENTS.md — droneengage_IR_camera

DroneEngage IR Camera module (`de_ir_camera`, binary `de_ir_tracker`).
Thermal imaging detection and RGB+IR fusion: reads an MI48 thermal
sensor + an RGB camera, detects hottest/coldest points per frame
optionally fuses thermal with RGB, and publishes thermal blobs as
observations (`obs`, 6546: class `hotspot`, optionally `coldspot`, with
`attrs.temp_c`, boxes top-left 0..1 in the upright thermal image) to the
DroneEngage bus. Detection is `src/ir_camera/hotspot_detector.hpp` (pure,
tested with `make -C test test`); the `"track"` config group confirms spots
and gives each a stable `trk`. The old hot/cold message (1043) is gone. **Detection/fusion
only** — object tracking is the separate `droneengage_tracking` module.

This is a **video pipeline** module: it reads from input video devices,
processes, and writes the fused result to its `output_video_device_name`
virtual device (`v4l2loopback`) for the next module in the chain. See
parent `../../AGENTS.md` for the virtual video device pipeline contract
and `de_common` vendoring.

## Build

    ./build.sh                 # DEBUG (only build script shipped)

Out-of-source in `build/`. Binary: `bin/de_ir_camera`. On a host whose
`mi48` install links `-lopencv_barcode` (OpenCV 4.14) the link step fails
with `cannot find -lopencv_barcode`; the compile is unaffected.

### CMake options

- `DDEBUG` — detailed debug.
- Version hard-coded (`0.9.1`), no `.version` auto-increment.

### Dependencies

OpenCV 4.5+, Threads, **mi48** thermal sensor SDK
(`find_package(mi48 REQUIRED PATHS $ENV{HOME}/.local/lib/cmake/mi48)` —
installed under `~/.local`). The MI48 sensor is accessed over serial.

## Config

- `de_ir_camera.config.module.json` — module config (WebClient UI).
- `de_ir_camera.config.module2.json` — alternate config.
- `de_ir_camera.config.local` — instance identity.
- `TEMPLATE.JSON` — UI schema (note: uppercase name).
- Key fields: `output_video_device_name` / `output_video_device` (the
  virtual device the fused stream is written to), IR/RGB device
  selection, fusion mode, EMA smoothing factor.

## Source layout & docs

`src/` — `main.cpp`, `ir_camera/`, `de_common/` (vendored), `defines.hpp`,
`version.hpp`. `res/` — demo assets. `README.md` — full architecture
write-up including the virtual device chain diagram.
