#pragma once
// Parser ramek Veteran / LeaperKim (Patton S itd.) - czysty C++, BEZ zaleznosci od ESPHome.
// Dzieki temu mozna go testowac na PC (test/test_parser.cpp) i uzywac w innych projektach.
//
// Zrodlo ukladu ramek: WheelLog VeteranAdapter.java + Twoj protocol_veteran.cpp.
//  0-2 DC 5A 5C | 3 len | 4-5 napiecie /100 V | 16-17 prad fazowy /10 A | 22-23 chargeMode
//  28-29 wersja (ver/1000 = ID modelu, 7 = Patton S) | 46 pnum (tylko model >= 5)
//  ostatnie 4 B = CRC32 (IEEE) z poprzedzajacych len bajtow, big-endian; total = len + 4
//
// SmartBMS (rotujace ramki, pnum = bajt 46; pnum < 4 -> BMS1, >= 4 -> BMS2):
//  pnum 0/4: prad BMS1 @69, prad BMS2 @71 (signed /100 A)
//  pnum 1/5: cele 1-15  @53+2i (signed /1000 V)
//  pnum 2/6: cele 16-30 @53+2i (unsigned /1000 V)
//  pnum 3/7: temperatury 1-6 @47+2i (signed /100 C), cele 31-42 @59+2i
//  pnum 8:   nieznany (ignorowany)

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace esphome {
namespace euc_ble {

static const uint8_t kMaxCells = 42;
static const uint16_t kRxBufSize = 300;  // len max 255 + 4

static inline int16_t i16be(const uint8_t *p) { return (int16_t)(((uint16_t) p[0] << 8) | p[1]); }
static inline uint16_t u16be(const uint8_t *p) { return (uint16_t)(((uint16_t) p[0] << 8) | p[1]); }

// CRC32 IEEE (jak zlib), bitowo - ramki sa krotkie, a czestosc niska
static inline uint32_t crc32_ieee(const uint8_t *d, uint16_t n) {
  uint32_t c = 0xFFFFFFFFu;
  for (uint16_t i = 0; i < n; i++) {
    c ^= d[i];
    for (int k = 0; k < 8; k++)
      c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
  }
  return ~c;
}

// SOC wg WheelLog, wariant standardowy; napiecie w jednostkach 0.01 V.
// Zwraca -1 dla modeli, ktorych wzoru tu nie ma.
static inline int soc_percent(uint8_t model, uint16_t volt_raw) {
  if (model == 4 || model == 7 || model == 43) {  // Patton, Patton S, Nosfet Aero
    if (volt_raw <= 9918)
      return 0;
    if (volt_raw >= 12337)
      return 100;
    return (int) lroundf((volt_raw - 9918) / 24.2f);
  }
  return -1;
}

struct Telemetry {
  bool valid{false};
  uint16_t voltage_raw{0};   // 0.01 V
  uint16_t charge_mode{0};   // offset 22; zakladamy 0 = brak, 1 = laduje (do potwierdzenia)
  uint16_t ver{0};           // offset 28
  uint8_t model{0};          // ver / 1000
  float voltage_v() const { return voltage_raw / 100.0f; }
};

struct BmsData {
  float cell[kMaxCells];  // V
  float temp[6];          // C
  float current;          // A
  bool has_current;
  bool has_temp;
  bool got_cells_a;  // odebrano pnum 1/5 (cele 1-15)
  bool got_cells_b;  // odebrano pnum 2/6 (cele 16-30)

  void clear() { memset(this, 0, sizeof(*this)); }

  // min/max napiecia cel [V] z pierwszych n cel. Pomija wartosci spoza 0.5..5 V
  // (np. zera, gdy dany BMS nie istnieje). false = brak kompletnych danych.
  bool cell_stats(uint8_t n, float *mn, float *mx) const {
    if (!(got_cells_a && got_cells_b))
      return false;
    if (n > kMaxCells)
      n = kMaxCells;
    bool any = false;
    float lo = 0, hi = 0;
    for (uint8_t i = 0; i < n; i++) {
      float c = cell[i];
      if (!(c > 0.5f && c < 5.0f))
        continue;
      if (!any || c < lo)
        lo = c;
      if (!any || c > hi)
        hi = c;
      any = true;
    }
    if (!any)
      return false;
    *mn = lo;
    *mx = hi;
    return true;
  }

  // min/max temperatury [C]. Pomija 0.00 (zalozenie: brak czujnika) i wartosci spoza -40..125.
  bool temp_stats(float *mn, float *mx) const {
    if (!has_temp)
      return false;
    bool any = false;
    float lo = 0, hi = 0;
    for (uint8_t i = 0; i < 6; i++) {
      float t = temp[i];
      if (t == 0.0f || !(t > -40.0f && t < 125.0f))
        continue;
      if (!any || t < lo)
        lo = t;
      if (!any || t > hi)
        hi = t;
      any = true;
    }
    if (!any)
      return false;
    *mn = lo;
    *mx = hi;
    return true;
  }
};

class VeteranParser {
 public:
  uint32_t frames_ok{0};
  uint32_t frames_bad_crc{0};

  VeteranParser() { reset(); }

  void reset() {
    rx_len_ = 0;
    telem_ = Telemetry();
    bms_[0].clear();
    bms_[1].clear();
  }

  // Wywoluj dla kazdej porcji bajtow z notyfikacji (dowolnie pofragmentowanej).
  void feed(const uint8_t *data, size_t len) {
    while (len > 0) {
      size_t room = kRxBufSize - rx_len_;
      if (room == 0) {  // zabezpieczenie; przy poprawnym przetwarzaniu nie powinno sie zdarzyc
        rx_len_ = 0;
        room = kRxBufSize;
      }
      size_t n = len < room ? len : room;
      memcpy(rx_ + rx_len_, data, n);
      rx_len_ += (uint16_t) n;
      data += n;
      len -= n;
      process_();
    }
  }

  const Telemetry &telemetry() const { return telem_; }
  const BmsData &bms(uint8_t i) const { return bms_[i ? 1 : 0]; }

 protected:
  uint8_t rx_[kRxBufSize];
  uint16_t rx_len_{0};
  Telemetry telem_;
  BmsData bms_[2];

  // Szukaj DC 5A 5C, potem czekaj az uzbiera sie len+4 bajtow.
  void process_() {
    for (;;) {
      uint16_t s = 0;
      while (s + 2 < rx_len_ && !(rx_[s] == 0xDC && rx_[s + 1] == 0x5A && rx_[s + 2] == 0x5C))
        s++;  // przy braku naglowka zostaja ostatnie 2 bajty (moga byc jego poczatkiem)
      if (s > 0) {
        memmove(rx_, rx_ + s, rx_len_ - s);
        rx_len_ -= s;
      }
      if (rx_len_ < 4)
        return;
      if (!(rx_[0] == 0xDC && rx_[1] == 0x5A && rx_[2] == 0x5C))
        return;  // czekamy na wiecej
      uint16_t total = (uint16_t) rx_[3] + 4;
      if (total < 38) {  // nieprawdopodobna dlugosc - to nie naglowek
        memmove(rx_, rx_ + 1, --rx_len_);
        continue;
      }
      if (rx_len_ < total)
        return;  // jeszcze niekompletna
      parse_frame_(rx_, total);
      memmove(rx_, rx_ + total, rx_len_ - total);
      rx_len_ -= total;
    }
  }

  void parse_frame_(const uint8_t *f, uint16_t total) {
    uint32_t calc = crc32_ieee(f, total - 4);
    uint32_t got = ((uint32_t) f[total - 4] << 24) | ((uint32_t) f[total - 3] << 16) |
                   ((uint32_t) f[total - 2] << 8) | f[total - 1];
    if (calc != got) {
      frames_bad_crc++;
      return;
    }
    frames_ok++;

    telem_.voltage_raw = u16be(f + 4);
    telem_.charge_mode = u16be(f + 22);
    telem_.ver = u16be(f + 28);
    telem_.model = (uint8_t)(telem_.ver / 1000);
    telem_.valid = true;

    if (telem_.model < 5 || total <= 46)
      return;  // starsze modele / krotkie ramki: bez SmartBMS

    const uint8_t pnum = f[46];
    if (pnum > 7)
      return;  // 8 = nieznany pakiet
    BmsData &b = bms_[pnum < 4 ? 0 : 1];

    if (pnum == 0 || pnum == 4) {
      if (total > 72) {
        bms_[0].current = i16be(f + 69) / 100.0f;
        bms_[1].current = i16be(f + 71) / 100.0f;
        bms_[0].has_current = bms_[1].has_current = true;
      }
    } else if (pnum == 1 || pnum == 5) {
      for (uint8_t i = 0; i < 15 && 53 + i * 2 + 2 <= total; i++)
        b.cell[i] = i16be(f + 53 + i * 2) / 1000.0f;
      b.got_cells_a = true;
    } else if (pnum == 2 || pnum == 6) {
      for (uint8_t i = 0; i < 15 && 53 + i * 2 + 2 <= total; i++)
        b.cell[i + 15] = u16be(f + 53 + i * 2) / 1000.0f;
      b.got_cells_b = true;
    } else {  // 3 / 7
      for (uint8_t i = 0; i < 12 && 59 + i * 2 + 2 <= total; i++)
        b.cell[i + 30] = u16be(f + 59 + i * 2) / 1000.0f;
      for (uint8_t i = 0; i < 6; i++)
        b.temp[i] = i16be(f + 47 + i * 2) / 100.0f;
      b.has_temp = true;
    }
  }
};

}  // namespace euc_ble
}  // namespace esphome
