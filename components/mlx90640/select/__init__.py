import esphome.codegen as cg
from esphome.components import select
import esphome.config_validation as cv
from esphome.const import ENTITY_CATEGORY_CONFIG

from .. import (
    CONF_MLX90640_ID,
    PALETTE_OPTIONS,
    PATTERN_OPTIONS,
    REFRESH_RATE_OPTIONS,
    RESOLUTION_OPTIONS,
    MLX90640Component,
    mlx90640_ns,
)

DEPENDENCIES = ["mlx90640"]

MLX90640Select = mlx90640_ns.class_("MLX90640Select", select.Select, cg.Component)

# key -> (C++ setting, options, icon)
SELECTS = {
    "refresh_rate": ("SETTING_REFRESH_RATE", REFRESH_RATE_OPTIONS, "mdi:camera-timer"),
    "resolution": ("SETTING_RESOLUTION", RESOLUTION_OPTIONS, "mdi:tune-vertical"),
    "pattern": ("SETTING_PATTERN", PATTERN_OPTIONS, "mdi:checkerboard"),
    "palette": ("SETTING_PALETTE", PALETTE_OPTIONS, "mdi:palette"),
}

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_MLX90640_ID): cv.use_id(MLX90640Component),
        **{
            cv.Optional(key): select.select_schema(
                MLX90640Select, entity_category=ENTITY_CATEGORY_CONFIG, icon=icon
            ).extend(cv.COMPONENT_SCHEMA)
            for key, (_, _, icon) in SELECTS.items()
        },
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_MLX90640_ID])
    for key, (setting, options, _) in SELECTS.items():
        if conf := config.get(key):
            var = await select.new_select(conf, options=options)
            await cg.register_component(var, conf)
            await cg.register_parented(var, hub)
            cg.add(var.set_setting(getattr(mlx90640_ns, setting)))
