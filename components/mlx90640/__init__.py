"""MLX90640 32x24 thermal camera with a built-in web heatmap (/thermal).

Runtime settings are exposed through the number / select / switch / button
platforms of this component (see those sub-packages).
"""

import esphome.codegen as cg
from esphome.components import i2c, sensor, web_server_base
from esphome.components.web_server_base import CONF_WEB_SERVER_BASE_ID
import esphome.config_validation as cv
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
AUTO_LOAD = ["sensor", "web_server_base"]

CONF_MLX90640_ID = "mlx90640_id"
CONF_REFRESH_RATE = "refresh_rate"
CONF_RESOLUTION = "resolution"
CONF_EMISSIVITY = "emissivity"

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

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MLX90640Component),
            cv.GenerateID(CONF_WEB_SERVER_BASE_ID): cv.use_id(
                web_server_base.WebServerBase
            ),
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
        }
    )
    .extend(cv.polling_component_schema("10s"))
    .extend(i2c.i2c_device_schema(0x33))
)


async def to_code(config):
    base = await cg.get_variable(config[CONF_WEB_SERVER_BASE_ID])
    var = cg.new_Pvariable(config[CONF_ID], base)
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    cg.add(var.set_refresh_rate(REFRESH_RATES[config[CONF_REFRESH_RATE]]))
    cg.add(var.set_resolution(RESOLUTIONS[config[CONF_RESOLUTION]]))
    cg.add(var.set_emissivity(config[CONF_EMISSIVITY]))

    for key, (setter, _) in SENSORS.items():
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(var, setter)(sens))
