// Test parsera na PC:  g++ -std=c++11 -Wall -I../components/euc_ble test_parser.cpp -o test_parser && ./test_parser
// Ramki syntetyczne zbudowane wg tabeli offsetow z veteran_parser.h (NIE z prawdziwego kola).
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include "veteran_parser.h"

using namespace esphome::euc_ble;

static int fails = 0;
#define CHECK(cond)                                                      \
  do {                                                                   \
    if (!(cond)) {                                                       \
      printf("FAIL line %d: %s\n", __LINE__, #cond);                     \
      fails++;                                                           \
    }                                                                    \
  } while (0)
#define NEAR(a, b, eps) CHECK(fabsf((float) (a) - (float) (b)) < (eps))

static void put16(std::vector<uint8_t> &f, int off, int v) {
  f[off] = (uint8_t)((v >> 8) & 0xFF);
  f[off + 1] = (uint8_t)(v & 0xFF);
}

// total = dlugosc ramki z CRC
static std::vector<uint8_t> make_frame(int total, int volt_raw, int chg, int ver, int pnum) {
  std::vector<uint8_t> f(total, 0);
  f[0] = 0xDC; f[1] = 0x5A; f[2] = 0x5C;
  f[3] = (uint8_t)(total - 4);
  put16(f, 4, volt_raw);
  put16(f, 22, chg);
  put16(f, 28, ver);
  f[46] = (uint8_t) pnum;
  return f;
}
static void seal(std::vector<uint8_t> &f) {
  uint32_t c = crc32_ieee(f.data(), (uint16_t)(f.size() - 4));
  f[f.size() - 4] = c >> 24; f[f.size() - 3] = c >> 16; f[f.size() - 2] = c >> 8; f[f.size() - 1] = c;
}

static void feed_chunks(VeteranParser &p, const std::vector<uint8_t> &d, size_t chunk) {
  for (size_t i = 0; i < d.size(); i += chunk)
    p.feed(d.data() + i, (d.size() - i < chunk) ? d.size() - i : chunk);
}

int main() {
  const int TOTAL = 100;   // dluzsze niz minimum, cele 53..82 miesci sie z zapasem
  const int VER = 7012;    // model 7 = Patton S
  const int VOLT = 11080;  // 110.80 V
  std::vector<uint8_t> stream;
  auto add = [&](std::vector<uint8_t> f) { seal(f); stream.insert(stream.end(), f.begin(), f.end()); };

  // smieci (tekstowy zrzut) przed pierwsza ramka
  const char *junk = "Hello wheel log\r\n\xDC\x5A";
  stream.insert(stream.end(), junk, junk + strlen(junk));

  // BMS1: pnum 0 (prady), 1 (cele 1-15), 2 (cele 16-30), 3 (temp)
  { auto f = make_frame(TOTAL, VOLT, 1, VER, 0); put16(f, 69, 1234); put16(f, 71, (uint16_t) -250); add(f); }
  { auto f = make_frame(TOTAL, VOLT, 1, VER, 1);
    for (int i = 0; i < 15; i++) put16(f, 53 + 2 * i, 3690 + i);   // 3.690 .. 3.704
    add(f); }
  { auto f = make_frame(TOTAL, VOLT, 1, VER, 2);
    for (int i = 0; i < 15; i++) put16(f, 53 + 2 * i, 3700 + i);   // 3.700 .. 3.714
    add(f); }
  { auto f = make_frame(TOTAL, VOLT, 1, VER, 3);
    put16(f, 47, 2890); put16(f, 49, 3010); put16(f, 51, 2750); put16(f, 53, 0); put16(f, 55, 2950); put16(f, 57, 3100);
    add(f); }
  // BMS2: pnum 5, 6, 7 (cele 15S=0? tu pelne 30 cel o innych wartosciach)
  { auto f = make_frame(TOTAL, VOLT, 1, VER, 5);
    for (int i = 0; i < 15; i++) put16(f, 53 + 2 * i, 3600 + i);
    add(f); }
  { auto f = make_frame(TOTAL, VOLT, 1, VER, 6);
    for (int i = 0; i < 15; i++) put16(f, 53 + 2 * i, 3650 + i);
    add(f); }
  { auto f = make_frame(TOTAL, VOLT, 1, VER, 7);
    put16(f, 47, 2500); put16(f, 49, 2600); put16(f, 51, 2700); put16(f, 53, 2800); put16(f, 55, 2900); put16(f, 57, 3000);
    add(f); }
  // pnum 8 - ma byc zignorowany
  add(make_frame(TOTAL, VOLT, 1, VER, 8));
  // uszkodzona ramka (zly CRC) - ma zostac odrzucona
  { auto f = make_frame(TOTAL, VOLT, 1, VER, 1); seal(f); f[60] ^= 0xFF; stream.insert(stream.end(), f.begin(), f.end()); }

  // rozne rozmiary porcji (jak notyfikacje BLE: 20 B, 1 B, 7 B, cala ramka)
  const size_t chunks[] = {20, 1, 7, 100, 244};
  for (size_t ci = 0; ci < sizeof(chunks) / sizeof(chunks[0]); ci++) {
    VeteranParser p;
    feed_chunks(p, stream, chunks[ci]);
    printf("chunk=%zu frames_ok=%u bad_crc=%u\n", chunks[ci], p.frames_ok, p.frames_bad_crc);

    CHECK(p.frames_ok == 8);
    CHECK(p.frames_bad_crc == 1);
    const Telemetry &t = p.telemetry();
    CHECK(t.valid);
    CHECK(t.model == 7);
    CHECK(t.charge_mode == 1);
    NEAR(t.voltage_v(), 110.80f, 0.001f);
    CHECK(soc_percent(t.model, t.voltage_raw) == (int) lroundf((11080 - 9918) / 24.2f));

    const BmsData &b1 = p.bms(0);
    const BmsData &b2 = p.bms(1);
    CHECK(b1.has_current && b2.has_current);
    NEAR(b1.current, 12.34f, 0.001f);
    NEAR(b2.current, -2.50f, 0.001f);

    float mn, mx;
    CHECK(b1.cell_stats(30, &mn, &mx));
    NEAR(mn, 3.690f, 0.0005f);
    NEAR(mx, 3.714f, 0.0005f);
    CHECK(b2.cell_stats(30, &mn, &mx));
    NEAR(mn, 3.600f, 0.0005f);
    NEAR(mx, 3.664f, 0.0005f);

    CHECK(b1.temp_stats(&mn, &mx));   // 0.00 pominiete
    NEAR(mn, 27.50f, 0.001f);
    NEAR(mx, 31.00f, 0.001f);
    CHECK(b2.temp_stats(&mn, &mx));
    NEAR(mn, 25.00f, 0.001f);
    NEAR(mx, 30.00f, 0.001f);
  }

  // Brak kompletu cel -> brak statystyk
  {
    VeteranParser p;
    std::vector<uint8_t> s;
    auto f = make_frame(100, 11080, 0, 7012, 1);
    for (int i = 0; i < 15; i++) put16(f, 53 + 2 * i, 3700);
    seal(f);
    p.feed(f.data(), f.size());
    float mn, mx;
    CHECK(!p.bms(0).cell_stats(30, &mn, &mx));  // brak pnum 2
    CHECK(p.telemetry().charge_mode == 0);
  }

  // Model < 5: bez BMS
  {
    VeteranParser p;
    auto f = make_frame(100, 11080, 0, 4005, 1);
    seal(f);
    p.feed(f.data(), f.size());
    CHECK(p.frames_ok == 1);
    CHECK(!p.bms(0).got_cells_a);
    CHECK(p.telemetry().model == 4);
  }

  printf(fails ? "\n%d BLEDOW\n" : "\nWSZYSTKO OK\n", fails);
  return fails ? 1 : 0;
}
