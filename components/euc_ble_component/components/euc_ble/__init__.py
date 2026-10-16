"""ESPHome external component: Veteran / LeaperKim (Patton S) -> sensory Home Assistant.

Plik definiuje schemat YAML (CONFIG_SCHEMA) i generuje kod C++ (to_code).
Logika BLE i parser sa w euc_ble.cpp / veteran_parser.h.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import ble_client, binary_sensor, sensor
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_BATTERY,
    DEVICE_CLASS_BATTERY_CHARGING,
    DEVICE_CLASS_CONNECTIVITY,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_VOLTAGE,
    STATE_CLASS_MEASUREMENT,
    UNIT_AMPERE,
    UNIT_CELSIUS,
    UNIT_MILLIVOLT,
    UNIT_PERCENT,
    UNIT_VOLT,
)

CODEOWNERS = []
DEPENDENCIES = ["ble_client"]
AUTO_LOAD = ["sensor", "binary_sensor"]

euc_ble_ns = cg.esphome_ns.namespace("euc_ble")
EucBle = euc_ble_ns.class_(
    "EucBle", cg.PollingComponent, ble_client.BLEClientNode
)

CONF_CELLS_PER_BMS = "cells_per_bms"
CONF_VOLTAGE = "voltage"
CONF_SOC = "soc"
CONF_CHARGING = "charging"
CONF_CONNECTED = "connected"

# (klucz w YAML, schemat). Kolejnosc = enum BmsSensorKind w euc_ble.h!
BMS_SENSORS = [
    (
        "current",
        sensor.sensor_schema(
            unit_of_measurement=UNIT_AMPERE,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_CURRENT,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
    ),
    (
        "cell_min",
        sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
    ),
    (
        "cell_max",
        sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
    ),
    (
        "cell_diff",
        sensor.sensor_schema(
            unit_of_measurement=UNIT_MILLIVOLT,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
    ),
    (
        "temp_min",
        sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
    ),
    (
        "temp_max",
        sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
    ),
]

BMS_SCHEMA = cv.Schema({cv.Optional(key): schema for key, schema in BMS_SENSORS})
BMS_KEYS = ("bms1", "bms2")

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(EucBle),
            cv.Optional(CONF_CELLS_PER_BMS, default=30): cv.int_range(min=1, max=42),
            cv.Optional(CONF_VOLTAGE): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_SOC): sensor.sensor_schema(
                unit_of_measurement=UNIT_PERCENT,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_BATTERY,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_CHARGING): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_BATTERY_CHARGING
            ),
            cv.Optional(CONF_CONNECTED): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_CONNECTIVITY
            ),
            cv.Optional(BMS_KEYS[0]): BMS_SCHEMA,
            cv.Optional(BMS_KEYS[1]): BMS_SCHEMA,
        }
    )
    .extend(cv.polling_component_schema("10s"))
    .extend(ble_client.BLE_CLIENT_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await ble_client.register_ble_node(var, config)

    cg.add(var.set_cells_per_bms(config[CONF_CELLS_PER_BMS]))

    if CONF_VOLTAGE in config:
        s = await sensor.new_sensor(config[CONF_VOLTAGE])
        cg.add(var.set_voltage_sensor(s))
    if CONF_SOC in config:
        s = await sensor.new_sensor(config[CONF_SOC])
        cg.add(var.set_soc_sensor(s))
    if CONF_CHARGING in config:
        b = await binary_sensor.new_binary_sensor(config[CONF_CHARGING])
        cg.add(var.set_charging_binary_sensor(b))
    if CONF_CONNECTED in config:
        b = await binary_sensor.new_binary_sensor(config[CONF_CONNECTED])
        cg.add(var.set_connected_binary_sensor(b))

    for bms_idx, bms_key in enumerate(BMS_KEYS):
        if bms_key not in config:
            continue
        for kind_idx, (key, _) in enumerate(BMS_SENSORS):
            if key in config[bms_key]:
                s = await sensor.new_sensor(config[bms_key][key])
                cg.add(var.set_bms_sensor(bms_idx, kind_idx, s))
