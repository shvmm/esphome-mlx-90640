# esphome-mlx-90640
Performant and feature complete implementation for the Melexis MLX 90640 in ESPHome

Start by compiling the example config and take it from there.
`esphome compile thermal_cam.yaml`

Verified working using a XIAO ESP32-S3.
<img width="730" height="886" alt="image" src="https://github.com/user-attachments/assets/ab878865-ad5b-49d4-9270-a2f2322adcd6" />

<img width="1503" height="898" alt="image" src="https://github.com/user-attachments/assets/bebe8951-c62e-45fc-ab6d-93fbee14540b" />

# MLX90640 thermal camera for ESPHome

An ESPHome external component for the Melexis **MLX90640** 32×24 thermal sensor. It gives you:

- **A live heatmap page** at `http://<device>/thermal`. It has a hover readout, palettes, and controls for every setting.
- **A Home Assistant camera entity** (JPEG). It shows a smooth, upscaled, colour-mapped image with an optional min/max/centre overlay.
- **Sensors**: min, max, mean and centre temperature, sensor die temperature, frame rate, supply voltage, bad pixels, and I2C errors.
- **Every sensor setting as an entity** (select / number / switch / button). Changes are saved and survive reboots. The web page and Home Assistant stay in sync.
- **Data endpoints** for your own tools: `/thermal.json`, `/thermal.bin` and `/thermal.jpg`.

It is tested with ESPHome 2026.11 on an ESP32-S3 (Seeed XIAO ESP32-S3) and needs ESPHome 2026.8 or newer. Any ESP32 works. PSRAM is optional.

## Wiring

| MLX90640 | XIAO ESP32-S3 | Generic ESP32 |
|----------|---------------|---------------|
| VIN      | 3V3           | 3V3           |
| GND      | GND           | GND           |
| SDA      | D4 (GPIO5)    | any GPIO      |
| SCL      | D5 (GPIO6)    | any GPIO      |

Most breakout boards already have I2C pull-ups. Keep the wires short if you run the bus at 1 MHz.

## Installation

```yaml
external_components:
  - source: github://OWNER/REPO   # this repository
    components: [mlx90640]

i2c:
  sda: GPIO5
  scl: GPIO6
  frequency: 1MHz

mlx90640:
  max_temperature:
    name: "Max Temperature"
  camera:
    name: "Thermal Camera"
```

[example/thermal-camera.yaml](example/thermal-camera.yaml) is a complete configuration that uses every option and entity.

## Configuration

### `mlx90640:`

| Option | Default | Description |
|--------|---------|-------------|
| `address` | `0x33` | I2C address |
| `refresh_rate` | `4Hz` | Sub-page rate: `0.5Hz` … `64Hz`. One full image is two sub-pages. |
| `resolution` | `18` | ADC resolution: `16`–`19` bits |
| `emissivity` | `0.95` | Default emissivity |
| `update_interval` | `10s` | How often the sensors are published. The heatmap and camera always run at the full rate. |
| `web_ui` | `true` | Serve the heatmap page and data endpoints under `/thermal`. Set to `false` for Home Assistant-only devices; the web server is then not included. |
| `camera` | – | Adds the camera entity (see below) |
| `min_temperature`, `max_temperature`, `mean_temperature`, `center_temperature`, `ambient_temperature`, `frame_rate`, `supply_voltage`, `bad_pixels`, `i2c_errors` | – | Optional sensors (all normal sensor options apply) |

The `refresh_rate`, `resolution` and `emissivity` values are only defaults. A matching select or number entity, once you add one, takes over with its saved value.

### `camera:` (under `mlx90640:`)

| Option | Default | Description |
|--------|---------|-------------|
| `name` | – | Entity name |
| `resolution` | `320x240` | `160x120`, `320x240`, `480x360` or `640x480` |
| `jpeg_quality` | `80` | Default for the JPEG quality number (10–100) |
| `overlay` | `true` | Default for the overlay switch: temperatures, hotspot crosshair and colour bar |
| `max_framerate` | `4` | Frame-rate cap while Home Assistant streams |
| `idle_framerate` | `0.1` | Preview refresh when nobody streams (`0` = off) |

The image uses the same palette, range, mirror and rotate settings as the heatmap. ESPHome allows only one camera per device, so `camera:` cannot be combined with `esp32_camera`.

### Entities

Add any of these. Each one is optional.

| Platform | Keys |
|----------|------|
| `select` | `refresh_rate`, `resolution`, `pattern` (chess / interleaved), `palette` (iron / rainbow / grey) |
| `number` | `emissivity`, `reflected_offset` (reflected temperature = die temperature − offset), `smoothing` (0–0.95), `range_min`, `range_max` (used when auto range is off), `publish_interval`, `jpeg_quality` |
| `switch` | `interpolate_bad_pixels`, `auto_range`, `mirror`, `rotate_180`, `freeze`, `camera_overlay` |
| `button` | `reinitialize` (re-reads the calibration and re-applies the settings) |

```yaml
select:
  - platform: mlx90640
    palette:
      name: "Heatmap Palette"
```

## Web endpoints

These are served when `web_ui: true`. If `web_server:` has `auth:` set, it protects them too.

| Endpoint | Description |
|----------|-------------|
| `GET /thermal` | Heatmap page (about 4.5 kB gzipped, cached by the browser) |
| `GET /thermal.json` | Status, settings (`cfg`), and pixels `t` (768 values in °C, display orientation, `null` = no value) |
| `GET /thermal.bin` | The same data in about 1.6 kB. The layout is documented in `handle_binary_` in [mlx90640.cpp](components/mlx90640/mlx90640.cpp). |
| `GET /thermal.jpg` | Camera snapshot. Needs `camera:`. |
| `POST /thermal/set?key=<setting>&value=<number>` | Change a setting. Keys are the names in `cfg`. |

## Performance notes

- **I2C speed.** Each sub-page is 1.7 kB. It is read in small chunks spread across loop passes, so even a slow bus doesn't block the rest of ESPHome. For clean images at high refresh rates, use 1 MHz for 32 Hz and above, and at least 400 kHz for 8–16 Hz. If reads can't keep up, the log says so and the I2C error / frame-rate sensors show it.
- **Image updates.** The image updates once per sub-page, and only the pixels in that sub-page change. The frame-rate sensor counts these updates, so it is about equal to the refresh rate.
- **Memory.** The sensor part uses about 17 kB of RAM. The camera adds roughly 50 kB at 320×240 (encoder buffers, the latest JPEG and a 10 kB task stack), and more at 640×480. The large buffers use PSRAM when it is available.
- **Stale readings.** If the sensor stops responding (wiring, power), the sensors switch to *unknown* and the component retries every 5 s. If the sensor resets, its settings are re-applied automatically.

## Development

- [tests/](tests/) has a PC test that compares the temperature maths with the Melexis reference driver: `tests/run_calc_test.sh`.
- [.github/workflows/ci.yml](.github/workflows/ci.yml) runs that test, then validates and compiles the example for an ESP32-S3 and a plain ESP32.

## License

Apache License 2.0, see [LICENSE](LICENSE). [NOTICE](NOTICE) lists the third-party code: the Melexis reference driver (Apache-2.0), which the calibration maths follows, and the public-domain `jpge` JPEG encoder.