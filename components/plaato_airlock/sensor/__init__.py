import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_GAS,
    STATE_CLASS_TOTAL_INCREASING,
)
from .. import (
    CONF_PLAATO_AIRLOCK_ID,
    PLAATO_AIRLOCK_COMPONENT_SCHEMA,
    plaato_airlock_ns,
)

CODEOWNERS = ["@mikesmitty"]
DEPENDENCIES = ["plaato_airlock"]

CONF_BUBBLE_COUNT = "bubble_count"
CONF_BUBBLE_TOTAL = "bubble_total"

PlaatoAirlockSensor = plaato_airlock_ns.class_(
    "PlaatoAirlockSensor", cg.PollingComponent
)

CONFIG_SCHEMA = PLAATO_AIRLOCK_COMPONENT_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(PlaatoAirlockSensor),
        cv.Optional(CONF_BUBBLE_COUNT): sensor.sensor_schema(
            state_class=STATE_CLASS_TOTAL_INCREASING,
        ),
        cv.Optional(CONF_BUBBLE_TOTAL): sensor.sensor_schema(
            device_class=DEVICE_CLASS_GAS,
            state_class=STATE_CLASS_TOTAL_INCREASING,
        ),
    }
).extend(cv.polling_component_schema("60s"))


async def to_code(config):
    paren = await cg.get_variable(config[CONF_PLAATO_AIRLOCK_ID])
    var = cg.new_Pvariable(config[CONF_ID], paren)
    await cg.register_component(var, config)

    if bubble_count_config := config.get(CONF_BUBBLE_COUNT):
        sens = await sensor.new_sensor(bubble_count_config)
        cg.add(var.set_bubble_count_sensor(sens))
    if bubble_total_config := config.get(CONF_BUBBLE_TOTAL):
        sens = await sensor.new_sensor(bubble_total_config)
        cg.add(var.set_bubble_total_sensor(sens))
