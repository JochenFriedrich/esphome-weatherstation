from esphome import automation
import esphome.codegen as cg
from esphome.components import remote_base, remote_receiver, binary_sensor
import esphome.config_validation as cv

from esphome.const import (
    CONF_BATTERY_LEVEL,
    CONF_CHANNEL,
    CONF_HUMIDITY,
    CONF_ID,
    CONF_TEMPERATURE,
    CONF_WIND_DIRECTION_DEGREES,
    CONF_WIND_SPEED,
)

CODEOWNERS = ["@JochenFriedrich"]
DEPENDENCIES = ["remote_receiver"]

ns = weatherstation_ns = cg.esphome_ns.namespace("weatherstation")

WeatherStationComponent = weatherstation_ns.class_("WeatherStationComponent", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(WeatherStationComponent),
        cv.GenerateID(remote_base.CONF_RECEIVER_ID): cv.use_id(
            remote_receiver.RemoteReceiverComponent
        ),
    }
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

AUTO_LOAD = ["binary_sensor"]

CONF_RECEIVER_ID = "receiver_id"
CONF_TRANSMITTER_ID = "transmitter_id"
CONF_FIRST = "first"
CONF_RAIN = "rain"
CONF_WIND_GUST = "wind_gust"

remote_base_ns = cg.esphome_ns.namespace("remote_base")
RemoteProtocol = remote_base_ns.class_("RemoteProtocol")
RemoteReceiverListener = remote_base_ns.class_("RemoteReceiverListener")
RemoteReceiverBinarySensorBase = remote_base_ns.class_(
    "RemoteReceiverBinarySensorBase", binary_sensor.BinarySensor, cg.Component
)
RemoteReceiverTrigger = remote_base_ns.class_(
    "RemoteReceiverTrigger", automation.Trigger, RemoteReceiverListener
)
RemoteReceiverDumperBase = remote_base_ns.class_("RemoteReceiverDumperBase")
RemoteTransmittable = remote_base_ns.class_("RemoteTransmittable")
RemoteTransmitterActionBase = remote_base_ns.class_(
    "RemoteTransmitterActionBase", RemoteTransmittable, automation.Action
)
RemoteReceiverBase = remote_base_ns.class_("RemoteReceiverBase")
RemoteTransmitterBase = remote_base_ns.class_("RemoteTransmitterBase")

def declare_protocol(name):
    data = ns.struct(f"{name}Data")
    binary_sensor_ = ns.class_(f"{name}BinarySensor", RemoteReceiverBinarySensorBase)
    trigger = ns.class_(f"{name}Trigger", RemoteReceiverTrigger)
    action = ns.class_(f"{name}Action", RemoteTransmitterActionBase)
    dumper = ns.class_(f"{name}Dumper", RemoteReceiverDumperBase)
    return data, binary_sensor_, trigger, action, dumper

(
    WeatherStationData,
    WeatherStationBinarySensor,
    WeatherStationTrigger,
    WeatherStationAction,
    WeatherStationDumper,
) = declare_protocol("WeatherStation")

WEATHERSTATION_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_ID): cv.uint16_t,
        cv.Optional(CONF_BATTERY_LEVEL): cv.float_,
        cv.Optional(CONF_CHANNEL): cv.uint8_t,
        cv.Optional(CONF_TEMPERATURE): cv.float_,
        cv.Optional(CONF_HUMIDITY): cv.uint8_t,
        cv.Optional(CONF_RAIN): cv.float_,
        cv.Optional(CONF_WIND_DIRECTION_DEGREES): cv.uint16_t,
        cv.Optional(CONF_WIND_SPEED): cv.float_,
        cv.Optional(CONF_WIND_GUST): cv.float_,
    }
)

def weatherstation_binary_sensor(var, config):
    pass


def weatherstation_trigger(var, config):
    pass


def weatherstation_action(var, config, args):
    if CONF_ID in config:
        cg.add(var.set_id((yield cg.templatable(config[CONF_ID], args, cg.uint16))))
    if CONF_BATTERY_LEVEL in config:
        cg.add(
            var.set_battery_level(
                (yield cg.templatable(config[CONF_BATTERY_LEVEL], args, cg.float_))
            )
        )
    if CONF_CHANNEL in config:
        cg.add(
            var.set_channel(
                (yield cg.templatable(config[CONF_CHANNEL], args, cg.uint8))
            )
        )
    if CONF_TEMPERATURE in config:
        cg.add(
            var.set_temperature(
                (yield cg.templatable(config[CONF_TEMPERATURE], args, cg.float_))
            )
        )
    if CONF_HUMIDITY in config:
        cg.add(
            var.set_humidity(
                (yield cg.templatable(config[CONF_HUMIDITY], args, cg.uint8))
            )
        )
    if CONF_RAIN in config:
        cg.add(var.set_rain((yield cg.templatable(config[CONF_RAIN], args, cg.float_))))
    if CONF_WIND_DIRECTION_DEGREES in config:
        cg.add(
            var.set_rain(
                (
                    yield cg.templatable(
                        config[CONF_WIND_DIRECTION_DEGREES], args, cg.uint16
                    )
                )
            )
        )
    if CONF_WIND_SPEED in config:
        cg.add(
            var.set_rain(
                (yield cg.templatable(config[CONF_WIND_SPEED], args, cg.float_))
            )
        )
    if CONF_WIND_GUST in config:
        cg.add(
            var.set_rain(
                (yield cg.templatable(config[CONF_WIND_GUST], args, cg.float_))
            )
        )


def weatherstation_dumper(var, config):
    pass


def register_weatherstation_protocol(name):
    lname = name.lower()
    remote_base.register_binary_sensor(
        f"weatherstation_{lname}",
        ns.class_(f"WeatherStation{name}BinarySensor", RemoteReceiverBinarySensorBase),
        WEATHERSTATION_SCHEMA,
    )(weatherstation_binary_sensor)
    remote_base.register_trigger(
        f"weatherstation_{lname}",
        ns.class_(f"WeatherStation{name}Trigger", RemoteReceiverTrigger),
        WeatherStationData,
    )(weatherstation_trigger)
    remote_base.register_action(
        f"weatherstation_{lname}",
        ns.class_(f"WeatherStation{name}Action", RemoteTransmitterActionBase),
        WEATHERSTATION_SCHEMA,
    )(weatherstation_action)
    remote_base.register_dumper(
        f"weatherstation_{lname}",
        ns.class_(f"WeatherStation{name}Dumper", RemoteReceiverDumperBase),
    )(weatherstation_dumper)

register_weatherstation_protocol("2032")
register_weatherstation_protocol("4LD")
register_weatherstation_protocol("AHFL")
register_weatherstation_protocol("Bresser3CH")
register_weatherstation_protocol("Eurochron")
register_weatherstation_protocol("H10515")
register_weatherstation_protocol("H13726")
register_weatherstation_protocol("L08037A")
register_weatherstation_protocol("Nexus")
register_weatherstation_protocol("Z31743")
register_weatherstation_protocol("Z32171")
register_weatherstation_protocol("Hideki")

