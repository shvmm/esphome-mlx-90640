import esphome.codegen as cg
from esphome.components import number
import esphome.config_validation as cv
from esphome.const import ENTITY_CATEGORY_CONFIG, UNIT_CELSIUS, UNIT_SECOND

from .. import CONF_MLX90640_ID, MLX90640Component, mlx90640_ns

DEPENDENCIES = ["mlx90640"]

MLX90640Number = mlx90640_ns.class_("MLX90640Number", number.Number, cg.Component)

# key -> (C++ setting, min, max, step, unit, icon)
NUMBERS = {
    "emissivity": ("SETTING_EMISSIVITY", 0.1, 1.0, 0.01, None, "mdi:texture-box"),
    "reflected_offset": ("SETTING_REFLECTED_OFFSET", -20, 40, 0.5, UNIT_CELSIUS, "mdi:mirror"),
    "smoothing": ("SETTING_SMOOTHING", 0, 0.95, 0.05, None, "mdi:blur"),
    "range_min": ("SETTING_RANGE_MIN", -40, 300, 0.5, UNIT_CELSIUS, "mdi:thermometer-low"),
    "range_max": ("SETTING_RANGE_MAX", -40, 300, 0.5, UNIT_CELSIUS, "mdi:thermometer-high"),
    "publish_interval": ("SETTING_PUBLISH_INTERVAL", 1, 3600, 1, UNIT_SECOND, "mdi:timer-outline"),
}


def _schema(unit, icon):
    kwargs = {"entity_category": ENTITY_CATEGORY_CONFIG, "icon": icon}
    if unit is not None:
        kwargs["unit_of_measurement"] = unit
    return number.number_schema(MLX90640Number, **kwargs).extend(cv.COMPONENT_SCHEMA)


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_MLX90640_ID): cv.use_id(MLX90640Component),
        **{
            cv.Optional(key): _schema(unit, icon)
            for key, (_, _, _, _, unit, icon) in NUMBERS.items()
        },
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_MLX90640_ID])
    for key, (setting, min_value, max_value, step, _, _) in NUMBERS.items():
        if conf := config.get(key):
            var = await number.new_number(
                conf, min_value=min_value, max_value=max_value, step=step
            )
            await cg.register_component(var, conf)
            await cg.register_parented(var, hub)
            cg.add(var.set_setting(getattr(mlx90640_ns, setting)))
