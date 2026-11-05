import esphome.codegen as cg
from esphome.components import switch
import esphome.config_validation as cv
from esphome.const import ENTITY_CATEGORY_CONFIG

from .. import CONF_MLX90640_ID, MLX90640Component, mlx90640_ns

DEPENDENCIES = ["mlx90640"]

MLX90640Switch = mlx90640_ns.class_("MLX90640Switch", switch.Switch, cg.Component)

# key -> (C++ setting, icon, default restore mode)
# A restored state wins; with no saved state the component default is used
# (interpolation and auto range on, the rest off).
SWITCHES = {
    "interpolate_bad_pixels": ("SETTING_INTERPOLATE_BAD_PIXELS", "mdi:auto-fix", "RESTORE_DEFAULT_ON"),
    "auto_range": ("SETTING_AUTO_RANGE", "mdi:arrow-expand-vertical", "RESTORE_DEFAULT_ON"),
    "mirror": ("SETTING_MIRROR", "mdi:flip-horizontal", "RESTORE_DEFAULT_OFF"),
    "rotate_180": ("SETTING_ROTATE_180", "mdi:rotate-3d-variant", "RESTORE_DEFAULT_OFF"),
    "freeze": ("SETTING_FREEZE", "mdi:pause", "ALWAYS_OFF"),
    "camera_overlay": ("SETTING_CAMERA_OVERLAY", "mdi:layers-outline", "RESTORE_DEFAULT_ON"),
}

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_MLX90640_ID): cv.use_id(MLX90640Component),
        **{
            cv.Optional(key): switch.switch_schema(
                MLX90640Switch,
                block_inverted=True,
                default_restore_mode=restore,
                entity_category=ENTITY_CATEGORY_CONFIG,
                icon=icon,
            ).extend(cv.COMPONENT_SCHEMA)
            for key, (_, icon, restore) in SWITCHES.items()
        },
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_MLX90640_ID])
    for key, (setting, _, _) in SWITCHES.items():
        if conf := config.get(key):
            var = await switch.new_switch(conf)
            await cg.register_component(var, conf)
            await cg.register_parented(var, hub)
            cg.add(var.set_setting(getattr(mlx90640_ns, setting)))
