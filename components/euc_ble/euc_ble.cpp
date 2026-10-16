#include "euc_ble.h"
#include "esphome/core/log.h"

#ifdef USE_ESP32

namespace esphome {
namespace euc_ble {

static const char *const TAG = "euc_ble";

// Veteran / Gotway: jeden serwis i jedna charakterystyka (styl HM-10), notify
static const uint16_t SERVICE_UUID16 = 0xFFE0;
static const uint16_t CHAR_UUID16 = 0xFFE1;

static void pub(sensor::Sensor *s, float v) {
  if (s != nullptr)
    s->publish_state(v);
}

void EucBle::dump_config() {
  ESP_LOGCONFIG(TAG, "EUC BLE (Veteran / LeaperKim):");
  ESP_LOGCONFIG(TAG, "  Cells per BMS: %u", this->cells_per_bms_);
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Pack voltage", this->voltage_sensor_);
  LOG_SENSOR("  ", "SOC", this->soc_sensor_);
  LOG_BINARY_SENSOR("  ", "Charging", this->charging_);
  LOG_BINARY_SENSOR("  ", "Connected", this->connected_);
  static const char *const names[BMS_SENSOR_COUNT] = {"current", "cell min", "cell max",
                                                       "cell diff", "temp min", "temp max"};
  for (uint8_t b = 0; b < 2; b++)
    for (uint8_t k = 0; k < BMS_SENSOR_COUNT; k++)
      if (this->bms_sensors_[b][k] != nullptr)
        ESP_LOGCONFIG(TAG, "  BMS%u %s: %s", b + 1, names[k], this->bms_sensors_[b][k]->get_name().c_str());
}

void EucBle::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                                 esp_ble_gattc_cb_param_t *param) {
  switch (event) {
    case ESP_GATTC_OPEN_EVT: {
      if (param->open.status == ESP_GATT_OK) {
        ESP_LOGI(TAG, "Polaczono z kolem");
        this->parser_.reset();
        this->last_frames_ok_ = 0;
      }
      break;
    }
    case ESP_GATTC_CLOSE_EVT: {
      ESP_LOGW(TAG, "Rozlaczono");
      this->handle_ = 0;
      this->parser_.reset();
      this->last_frames_ok_ = 0;
      this->publish_all_(false);  // od razu pokaz "brak danych" w HA
      break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT: {
      this->handle_ = 0;
      auto *chr = this->parent()->get_characteristic(SERVICE_UUID16, CHAR_UUID16);
      if (chr == nullptr) {
        ESP_LOGW(TAG, "Brak charakterystyki FFE0/FFE1 - to na pewno Veteran?");
        this->status_set_warning();
        break;
      }
      this->handle_ = chr->handle;
      auto status = this->parent()->register_for_notify(chr->handle);
      if (status)
        ESP_LOGW(TAG, "register_for_notify nieudane, status=%d", status);
      break;
    }
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
      if (param->reg_for_notify.handle == this->handle_) {
        if (param->reg_for_notify.status != ESP_GATT_OK) {
          ESP_LOGW(TAG, "Blad rejestracji notify, status=%d", param->reg_for_notify.status);
          break;
        }
        this->status_clear_warning();
        this->node_state = espbt::ClientState::ESTABLISHED;
        ESP_LOGD(TAG, "Notify FFE1 aktywne");
      }
      break;
    }
    case ESP_GATTC_NOTIFY_EVT: {
      if (param->notify.handle != this->handle_ || param->notify.value_len == 0)
        break;
      this->parser_.feed(param->notify.value, param->notify.value_len);
      break;
    }
    default:
      break;
  }
}

void EucBle::update() {
  // Swiezosc: od ostatniego update() musiala przyjsc przynajmniej jedna poprawna ramka.
  const uint32_t ok = this->parser_.frames_ok;
  const bool fresh = this->parser_.telemetry().valid && ok != this->last_frames_ok_;
  this->last_frames_ok_ = ok;
  this->publish_all_(fresh);
}

void EucBle::publish_all_(bool fresh) {
  if (this->connected_ != nullptr)
    this->connected_->publish_state(fresh);

  if (!fresh) {
    // Raz publikujemy NaN (w HA: "unknown"), potem cisza az do powrotu danych.
    if (this->nan_published_)
      return;
    this->nan_published_ = true;
    pub(this->voltage_sensor_, NAN);
    pub(this->soc_sensor_, NAN);
    for (uint8_t b = 0; b < 2; b++)
      for (uint8_t k = 0; k < BMS_SENSOR_COUNT; k++)
        pub(this->bms_sensors_[b][k], NAN);
    // Binary sensor "charging" zostaje przy ostatniej wartosci (ESPHome nie ma stanu "unknown" dla binary).
    return;
  }
  this->nan_published_ = false;

  const Telemetry &t = this->parser_.telemetry();
  pub(this->voltage_sensor_, t.voltage_v());
  if (this->soc_sensor_ != nullptr) {
    int soc = soc_percent(t.model, t.voltage_raw);
    pub(this->soc_sensor_, soc < 0 ? NAN : (float) soc);
  }
  if (this->charging_ != nullptr)
    this->charging_->publish_state(t.charge_mode != 0);

  for (uint8_t b = 0; b < 2; b++) {
    const BmsData &d = this->parser_.bms(b);
    sensor::Sensor **s = this->bms_sensors_[b];

    pub(s[BMS_CURRENT], d.has_current ? d.current : NAN);

    float mn = 0, mx = 0;
    if (d.cell_stats(this->cells_per_bms_, &mn, &mx)) {
      pub(s[BMS_CELL_MIN], mn);
      pub(s[BMS_CELL_MAX], mx);
      pub(s[BMS_CELL_DIFF], (mx - mn) * 1000.0f);  // mV
    } else {
      pub(s[BMS_CELL_MIN], NAN);
      pub(s[BMS_CELL_MAX], NAN);
      pub(s[BMS_CELL_DIFF], NAN);
    }

    if (d.temp_stats(&mn, &mx)) {
      pub(s[BMS_TEMP_MIN], mn);
      pub(s[BMS_TEMP_MAX], mx);
    } else {
      pub(s[BMS_TEMP_MIN], NAN);
      pub(s[BMS_TEMP_MAX], NAN);
    }
  }
}

}  // namespace euc_ble
}  // namespace esphome

#endif  // USE_ESP32
