from esphome.components import climate, uart
import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.const import (
    CONF_BEEPER,
    CONF_DISPLAY,
    CONF_ID,
)

CODEOWNERS = ["@mikesmitty"]
DEPENDENCIES = ["climate", "uart"]
pioneer_wyt_ns = cg.esphome_ns.namespace("pioneer").namespace("wyt")
WytClimate = pioneer_wyt_ns.class_(
    "WytClimate", climate.Climate, cg.PollingComponent, uart.UARTDevice
)


CONFIG_SCHEMA = cv.All(
    climate.CLIMATE_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(WytClimate),
            cv.Optional(CONF_BEEPER, default=False): cv.boolean,
            cv.Optional(CONF_DISPLAY, default=True): cv.boolean,
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
    .extend(cv.polling_component_schema("3s"))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    await climate.register_climate(var, config)
    cg.add(var.set_beeper(config[CONF_BEEPER]))
    cg.add(var.set_display(config[CONF_DISPLAY]))
