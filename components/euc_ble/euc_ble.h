#pragma once

#include "esphome/core/component.h"
#include "esphome/components/ble_client/ble_client.h"
#include "esphome/components/esp32_ble_tracker/esp32_ble_tracker.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

#include "veteran_parser.h"

#ifdef USE_ESP32
#include <esp_gattc_api.h>

namespace esphome {
namespace euc_ble {

namespace espbt = esphome::esp32_ble_tracker;

// Kolejnosc musi byc zgodna z BMS_SENSORS w __init__.py
enum BmsSensorKind : uint8_t {
  BMS_CURRENT = 0,
  BMS_CELL_MIN,
  BMS_CELL_MAX,
  BMS_CELL_DIFF,
  BMS_TEMP_MIN,
  BMS_TEMP_MAX,
  BMS_SENSOR_COUNT
};

class EucBle : public PollingComponent, public ble_client::BLEClientNode {
 public:
  void loop() override { this->disable_loop(); }  // praca przez callbacki BLE i update()
  void update() override;
  void dump_config() override;
  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param) override;

  void set_cells_per_bms(uint8_t n) { this->cells_per_bms_ = n; }
  void set_voltage_sensor(sensor::Sensor *s) { this->voltage_sensor_ = s; }
  void set_soc_sensor(sensor::Sensor *s) { this->soc_sensor_ = s; }
  void set_charging_binary_sensor(binary_sensor::BinarySensor *s) { this->charging_ = s; }
  void set_connected_binary_sensor(binary_sensor::BinarySensor *s) { this->connected_ = s; }
  void set_bms_sensor(uint8_t bms, uint8_t kind, sensor::Sensor *s) {
    if (bms < 2 && kind < BMS_SENSOR_COUNT)
      this->bms_sensors_[bms][kind] = s;
  }

 protected:
  void publish_all_(bool fresh);

  VeteranParser parser_;
  uint8_t cells_per_bms_{30};
  uint16_t handle_{0};
  uint32_t last_frames_ok_{0};
  bool nan_published_{false};

  sensor::Sensor *voltage_sensor_{nullptr};
  sensor::Sensor *soc_sensor_{nullptr};
  binary_sensor::BinarySensor *charging_{nullptr};
  binary_sensor::BinarySensor *connected_{nullptr};
  sensor::Sensor *bms_sensors_[2][BMS_SENSOR_COUNT]{};
};

}  // namespace euc_ble
}  // namespace esphome

#endif  // USE_ESP32
