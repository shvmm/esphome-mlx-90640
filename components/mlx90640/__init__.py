"""MLX90640 32x24 thermal camera with a built-in web heatmap (/thermal).

Runtime settings are exposed through the number / select / switch / button
platforms of this component (see those sub-packages). An optional `camera:`
block adds a camera entity (JPEG) that Home Assistant shows natively.
"""

import gzip
import hashlib
from pathlib import Path

import esphome.codegen as cg
from esphome.components import i2c, sensor
import esphome.config_validation as cv
from esphome.core.entity_helpers import setup_entity
import esphome.final_validate as fv
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_VOLTAGE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_CELSIUS,
    UNIT_VOLT,
)

DEPENDENCIES = ["i2c", "esp32"]


def AUTO_LOAD(config=None):
    # A falsy config is a tooling probe for the full set.
    loads = ["sensor"]
    if not config or config.get(CONF_WEB_UI, True):
        loads.append("web_server_base")
    if not config or CONF_CAMERA in config:
        loads.append("camera")
    return loads


CONF_MLX90640_ID = "mlx90640_id"
CONF_REFRESH_RATE = "refresh_rate"
CONF_RESOLUTION = "resolution"
CONF_EMISSIVITY = "emissivity"
CONF_CAMERA = "camera"
CONF_JPEG_QUALITY = "jpeg_quality"
CONF_MAX_FRAMERATE = "max_framerate"
CONF_IDLE_FRAMERATE = "idle_framerate"
CONF_OVERLAY = "overlay"
CONF_WEB_UI = "web_ui"

PAGE_FILE = Path(__file__).parent / "heatmap.html"

CAMERA_RESOLUTIONS = {
    "160x120": (160, 120),
    "320x240": (320, 240),
    "480x360": (480, 360),
    "640x480": (640, 480),
}

# Option labels; their order is the C++ setting value (index).
REFRESH_RATE_OPTIONS = ["0.5 Hz", "1 Hz", "2 Hz", "4 Hz", "8 Hz", "16 Hz", "32 Hz", "64 Hz"]
RESOLUTION_OPTIONS = ["16-bit", "17-bit", "18-bit", "19-bit"]
PATTERN_OPTIONS = ["Chess", "Interleaved"]
PALETTE_OPTIONS = ["Iron", "Rainbow", "Grey"]

# YAML accepts "4Hz", "4 Hz", "0.5hz" ...
REFRESH_RATES = {o.replace(" ", "").lower(): i for i, o in enumerate(REFRESH_RATE_OPTIONS)}
RESOLUTIONS = {o.replace("-bit", ""): i for i, o in enumerate(RESOLUTION_OPTIONS)}

mlx90640_ns = cg.esphome_ns.namespace("mlx90640")
MLX90640Component = mlx90640_ns.class_(
    "MLX90640Component", cg.PollingComponent, i2c.I2CDevice
)
MLX90640Camera = mlx90640_ns.class_("MLX90640Camera", cg.Component, cg.EntityBase)

CAMERA_SCHEMA = cv.ENTITY_BASE_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(MLX90640Camera),
        cv.Optional(CONF_RESOLUTION, default="320x240"): cv.one_of(
            *CAMERA_RESOLUTIONS, lower=True
        ),
        # Defaults for the runtime "JPEG Quality" number / "Camera Overlay" switch
        cv.Optional(CONF_JPEG_QUALITY, default=80): cv.int_range(min=10, max=100),
        cv.Optional(CONF_OVERLAY, default=True): cv.boolean,
        # Upper limit while Home Assistant streams (sensor gives 2-4 frames/s at 4-8 Hz)
        cv.Optional(CONF_MAX_FRAMERATE, default=4.0): cv.float_range(min=0.1, max=16.0),
        # Background refresh of the HA preview when nobody streams (0 = off)
        cv.Optional(CONF_IDLE_FRAMERATE, default=0.1): cv.float_range(min=0.0, max=1.0),
    }
).extend(cv.COMPONENT_SCHEMA)

TEMPERATURE_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_CELSIUS,
    accuracy_decimals=1,
    device_class=DEVICE_CLASS_TEMPERATURE,
    state_class=STATE_CLASS_MEASUREMENT,
)

# key -> (setter, schema)
SENSORS = {
    "min_temperature": ("set_min_sensor", TEMPERATURE_SCHEMA),
    "max_temperature": ("set_max_sensor", TEMPERATURE_SCHEMA),
    "mean_temperature": ("set_mean_sensor", TEMPERATURE_SCHEMA),
    "center_temperature": ("set_center_sensor", TEMPERATURE_SCHEMA),
    "ambient_temperature": (
        "set_ambient_sensor",
        sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
    ),
    "frame_rate": (
        "set_frame_rate_sensor",
        sensor.sensor_schema(
            unit_of_measurement="fps",
            accuracy_decimals=1,
            icon="mdi:camera-timer",
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
    ),
    "i2c_errors": (
        "set_i2c_errors_sensor",
        sensor.sensor_schema(
            accuracy_decimals=0,
            icon="mdi:alert-circle-outline",
            state_class=STATE_CLASS_TOTAL_INCREASING,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
    ),
    "bad_pixels": (
        "set_bad_pixels_sensor",
        sensor.sensor_schema(
            accuracy_decimals=0,
            icon="mdi:grid-off",
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
    ),
    "supply_voltage": (
        "set_supply_voltage_sensor",
        sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
    ),
}

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MLX90640Component),
            # Heatmap page and data endpoints under /thermal (uses the web server on port 80)
            cv.Optional(CONF_WEB_UI, default=True): cv.boolean,
            # Defaults; the matching select/number entities (if configured)
            # override these with their saved values.
            cv.Optional(CONF_REFRESH_RATE, default="4Hz"): cv.enum(
                REFRESH_RATES, lower=True, space=""
            ),
            cv.Optional(CONF_RESOLUTION, default="18"): cv.enum(
                RESOLUTIONS, string=True
            ),
            cv.Optional(CONF_EMISSIVITY, default=0.95): cv.float_range(
                min=0.1, max=1.0
            ),
            **{cv.Optional(key): schema for key, (_, schema) in SENSORS.items()},
            cv.Optional(CONF_CAMERA): CAMERA_SCHEMA,
        }
    )
    .extend(cv.polling_component_schema("10s"))
    .extend(i2c.i2c_device_schema(0x33)),
    # Oldest release with the entity/camera/web server APIs used here
    cv.require_esphome_version(2026, 8, 0),
)


def _final_validate(config):
    full_config = fv.full_config.get()
    if CONF_CAMERA in config and "esp32_camera" in full_config:
        raise cv.Invalid(
            "ESPHome supports one camera per device: remove 'camera:' from mlx90640 "
            "or the esp32_camera component",
            path=[CONF_CAMERA],
        )
    return config


FINAL_VALIDATE_SCHEMA = _final_validate


def _add_page(var):
    """Embed heatmap.html gzipped, with an ETag so browsers can cache it."""
    page = gzip.compress(PAGE_FILE.read_bytes(), compresslevel=9, mtime=0)
    etag = '"' + hashlib.sha256(page).hexdigest()[:16] + '"'
    data = ",".join(str(b) for b in page)
    cg.add_global(
        cg.RawStatement(
            f"static const uint8_t MLX90640_HEATMAP_GZ[{len(page)}] = {{{data}}};"
        )
    )
    cg.add(var.set_page(cg.RawExpression("MLX90640_HEATMAP_GZ"), len(page), etag))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    if config[CONF_WEB_UI]:
        cg.add_define("USE_MLX90640_WEB_UI")
        _add_page(var)

    cg.add(var.set_refresh_rate(REFRESH_RATES[config[CONF_REFRESH_RATE]]))
    cg.add(var.set_resolution(RESOLUTIONS[config[CONF_RESOLUTION]]))
    cg.add(var.set_emissivity(config[CONF_EMISSIVITY]))

    for key, (setter, _) in SENSORS.items():
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(var, setter)(sens))

    if cam := config.get(CONF_CAMERA):
        cg.add_define("USE_CAMERA")
        cg.add_define("USE_MLX90640_CAMERA")
        cam_var = cg.new_Pvariable(cam[CONF_ID])
        await setup_entity(cam_var, cam, "camera")
        await cg.register_component(cam_var, cam)
        await cg.register_parented(cam_var, var)
        width, height = CAMERA_RESOLUTIONS[cam[CONF_RESOLUTION]]
        cg.add(cam_var.set_resolution(width, height))
        cg.add(cam_var.set_max_update_interval(int(1000 / cam[CONF_MAX_FRAMERATE])))
        idle = cam[CONF_IDLE_FRAMERATE]
        cg.add(cam_var.set_idle_update_interval(int(1000 / idle) if idle > 0 else 0))
        cg.add(var.set_jpeg_quality(cam[CONF_JPEG_QUALITY]))
        cg.add(var.set_camera_overlay(cam[CONF_OVERLAY]))
        cg.add(var.set_camera(cam_var))
