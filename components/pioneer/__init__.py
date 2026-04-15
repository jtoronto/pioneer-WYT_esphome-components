import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import core
from esphome.components import climate
from esphome.const import CONF_ID, CONF_NAME, CONF_MODE

CONF_PIONEER_CLIMATE = "pioneer"

CODEOWNERS = ["@mikesmitty"]

CONFIG_SCHEMA = cv.All(
    climate.CLIMATE_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(cg.Pvariable),
            cv.Optional(CONF_NAME): cv.string,
            cv.Optional(CONF_MODE): cv.enum(climate.CLIMATE_MODES),
            cv.Optional("beeper", default=False): cv.boolean,
            cv.Optional("display", default=True): cv.boolean,
            cv.Optional("defrost_status"): cv.use_id.sensor,
            cv.Optional("indoor_fan_speed"): cv.use_id.sensor,
            cv.Optional("outdoor_fan_speed"): cv.use_id.sensor,
            cv.Optional("outdoor_temperature"): cv.use_id.sensor,
            cv.Optional("power_usage"): cv.use_id.sensor,
            cv.Optional("pending"): cv.use_id.binary_sensor,
            cv.Optional("uart_phase"): cv.use_id.text_sensor,
        }
    ),
    cv.require_platform("climate"),
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID], config.get(CONF_NAME, "Pioneer Climate"))
    await climate.register_climate(var, config)

    if "beeper" in config:
        cg.add(var.set_beeper(config["beeper"]))
    if "display" in config:
        cg.add(var.set_display(config["display"]))
    if "defrost_status" in config:
        sens = await cg.get_variable(config["defrost_status"])
        cg.add(var.set_defrost_binary_sensor(sens))
    if "indoor_fan_speed" in config:
        sens = await cg.get_variable(config["indoor_fan_speed"])
        cg.add(var.set_indoor_fan_speed_sensor(sens))
    if "outdoor_fan_speed" in config:
        sens = await cg.get_variable(config["outdoor_fan_speed"])
        cg.add(var.set_outdoor_fan_speed_sensor(sens))
    if "outdoor_temperature" in config:
        sens = await cg.get_variable(config["outdoor_temperature"])
        cg.add(var.set_outdoor_temperature_sensor(sens))
    if "power_usage" in config:
        sens = await cg.get_variable(config["power_usage"])
        cg.add(var.set_power_sensor(sens))
    if "pending" in config:
        sens = await cg.get_variable(config["pending"])
        cg.add(var.set_pending_command_sensor(sens))
    if "uart_phase" in config:
        sens = await cg.get_variable(config["uart_phase"])
        cg.add(var.set_uart_phase_sensor(sens))