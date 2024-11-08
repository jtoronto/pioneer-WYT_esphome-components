import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c
from esphome.const import CONF_ID

CODEOWNERS = ["@mikesmitty"]
DEPENDENCIES = ["i2c"]
MULTI_CONF = True

CONF_PLAATO_AIRLOCK_ID = "plaato_airlock_id"

ICON_THERMOMETER_ALERT = "mdi:thermometer-alert"

plaato_airlock_ns = cg.esphome_ns.namespace("plaato_airlock")
PlaatoAirlockComponent = plaato_airlock_ns.class_(
    "PlaatoAirlockComponent", cg.Component, i2c.I2CDevice
)

PLAATO_AIRLOCK_COMPONENT_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_PLAATO_AIRLOCK_ID): cv.use_id(PlaatoAirlockComponent),
    }
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(PlaatoAirlockComponent),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(0x03))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
