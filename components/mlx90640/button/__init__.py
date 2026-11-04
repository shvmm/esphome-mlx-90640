import esphome.codegen as cg
from esphome.components import button
import esphome.config_validation as cv
from esphome.const import DEVICE_CLASS_RESTART, ENTITY_CATEGORY_CONFIG

from .. import CONF_MLX90640_ID, MLX90640Component, mlx90640_ns

DEPENDENCIES = ["mlx90640"]

MLX90640ReinitButton = mlx90640_ns.class_("MLX90640ReinitButton", button.Button)

CONF_REINITIALIZE = "reinitialize"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_MLX90640_ID): cv.use_id(MLX90640Component),
        cv.Optional(CONF_REINITIALIZE): button.button_schema(
            MLX90640ReinitButton,
            device_class=DEVICE_CLASS_RESTART,
            entity_category=ENTITY_CATEGORY_CONFIG,
            icon="mdi:camera-retake",
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_MLX90640_ID])
    if conf := config.get(CONF_REINITIALIZE):
        var = await button.new_button(conf)
        await cg.register_parented(var, hub)
