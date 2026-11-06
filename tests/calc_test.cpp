// Differential test: components/mlx90640/mlx90640_calc.* against the Melexis
// reference driver (fetched by run_calc_test.sh). Builds plausible calibration
// and frame data, runs both implementations and compares Vdd, Ta, every
// object temperature and the bad-pixel list.
//
// Known, intentional differences that the data avoids:
//  * TGC is set to 0 (it is 0 on production sensors). With TGC != 0 the current
//    Melexis driver compensates alpha with the mean of both CP alphas, the
//    datasheet with the sub-page's own one, which this component follows.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <random>
#include <vector>

#include "../components/mlx90640/mlx90640_calc.h"

extern "C" {
#include "MLX90640_API.h"
// The reference API.c links against these; nothing here talks to hardware.
int MLX90640_I2CRead(uint8_t, uint16_t, uint16_t, uint16_t *) { return -1; }
int MLX90640_I2CWrite(uint8_t, uint16_t, uint16_t) { return -1; }
int MLX90640_I2CGeneralReset(void) { return -1; }
void MLX90640_I2CInit(void) {}
void MLX90640_I2CFreqSet(int) {}
}

using namespace esphome::mlx90640;

static int failures = 0;
static int checks = 0;

static void expect_close(const char *what, int test, double ours, double ref, double tol) {
  checks++;
  if (!(std::fabs(ours - ref) <= tol)) {
    if (failures < 25)
      std::printf("FAIL test %d %s: ours %.6f ref %.6f (tol %.6f)\n", test, what, ours, ref, tol);
    failures++;
  }
}

// Calibration data in realistic ranges (field layout per the datasheet).
static void make_eeprom(std::mt19937 &rng, bool calibrated_interleaved, uint16_t *ee) {
  auto r = [&](int lo, int hi) { return std::uniform_int_distribution<int>(lo, hi)(rng); };
  auto nibbles = [&]() {  // four signed 4-bit values in -3..3
    uint16_t w = 0;
    for (int k = 0; k < 4; k++)
      w |= static_cast<uint16_t>(r(-3, 3) & 0xF) << (4 * k);
    return w;
  };
  std::memset(ee, 0, MLX_EEPROM_WORDS * sizeof(uint16_t));
  for (int i = 0; i < 16; i++)
    ee[i] = static_cast<uint16_t>(r(0, 0xFFFF));
  ee[10] = static_cast<uint16_t>((ee[10] & ~0x0800 & ~0x0040) | (calibrated_interleaved ? 0x0800 : 0));
  ee[16] = (4 << 12) | (2 << 8) | (1 << 4) | 0;  // alphaPTAT 9, occ row/col/rem scales 2/1/0
  ee[17] = static_cast<uint16_t>(static_cast<int16_t>(r(-80, -40)));  // offset reference
  for (int i = 18; i < 32; i++)
    ee[i] = nibbles();                       // offset row / column corrections
  ee[32] = (7 << 12) | (3 << 8) | (2 << 4) | 1;  // alpha scale 37, acc row/col/rem scales 3/2/1
  ee[33] = static_cast<uint16_t>(r(11500, 12500));  // alpha reference
  for (int i = 34; i < 48; i++)
    ee[i] = nibbles();                       // alpha row / column corrections
  ee[48] = static_cast<uint16_t>(r(6000, 6600));    // gain
  ee[49] = static_cast<uint16_t>(r(12000, 12400));  // vPTAT25
  ee[50] = static_cast<uint16_t>((r(3, 7) << 10) | r(330, 360));  // KvPTAT, KtPTAT
  ee[51] = static_cast<uint16_t>((r(0x98, 0xA0) << 8) | r(0x60, 0x70));  // kVdd, vdd25
  ee[52] = static_cast<uint16_t>((r(2, 5) << 12) | (r(2, 5) << 8) | (r(2, 5) << 4) | r(2, 5));  // Kv groups
  ee[53] = static_cast<uint16_t>(r(0, 0xFFFF));  // interleaved / chess corrections
  ee[54] = static_cast<uint16_t>((r(0x40, 0x60) << 8) | r(0x40, 0x60));  // Kta groups
  ee[55] = static_cast<uint16_t>((r(0x40, 0x60) << 8) | r(0x40, 0x60));
  ee[56] = (2 << 12) | (3 << 8) | (6 << 4) | static_cast<uint16_t>(r(0, 1));  // res 18, Kv/Kta scales
  ee[57] = static_cast<uint16_t>((r(0x28, 0x3F) << 10) | r(60, 80));  // CP alpha
  ee[58] = static_cast<uint16_t>((r(0, 7) << 10) | ((1024 - r(60, 90)) & 0x3FF));  // CP offsets
  ee[59] = static_cast<uint16_t>((r(0x30, 0x50) << 8) | r(0x40, 0x60));  // CP Kv, CP Kta
  ee[60] = static_cast<uint16_t>(r(0xE0, 0xFF) << 8);  // KsTa, TGC = 0
  ee[61] = static_cast<uint16_t>((r(0x90, 0xA0) << 8) | r(0x90, 0xA0));  // KsTo ranges 1, 2
  ee[62] = static_cast<uint16_t>((r(0x90, 0xA0) << 8) | r(0x90, 0xA0));  // KsTo ranges 3, 4
  ee[63] = (1 << 12) | (5 << 8) | (0xF << 4) | 9;  // corner temps 150/200, KsTo scale
  for (int px = 0; px < MLX_PIXELS; px++) {
    uint16_t w = static_cast<uint16_t>((r(0, 63) << 10) | (r(0, 63) << 4) | (r(0, 7) << 1));
    ee[64 + px] = w == 0 ? 0x0010 : w;
  }
  // A few deviating pixels, far apart (the reference rejects adjacent ones).
  ee[64 + 100] = 0;       // broken
  ee[64 + 400] = 0;       // broken
  ee[64 + 250] |= 0x0001;  // outlier
  ee[64 + 600] |= 0x0001;  // outlier
}

static void make_frame(std::mt19937 &rng, const MLX90640Params &p, bool chess, int adc_code, int sub_page,
                       uint16_t *frame) {
  auto r = [&](int lo, int hi) { return std::uniform_int_distribution<int>(lo, hi)(rng); };
  std::memset(frame, 0, MLX_FRAME_WORDS * sizeof(uint16_t));
  for (int px = 0; px < MLX_PIXELS; px++)
    frame[px] = static_cast<uint16_t>(static_cast<int16_t>(p.offset[px] + r(-20, 400)));
  frame[768] = static_cast<uint16_t>(r(20800, 21200));  // VBE
  frame[776] = static_cast<uint16_t>(static_cast<int16_t>(p.cpOffset[0] + r(-5, 5)));
  frame[778] = static_cast<uint16_t>(p.gainEE + r(-50, 50));
  frame[800] = static_cast<uint16_t>(r(1650, 1750));  // PTAT
  frame[808] = static_cast<uint16_t>(static_cast<int16_t>(p.cpOffset[1] + r(-5, 5)));
  frame[810] = static_cast<uint16_t>(static_cast<int16_t>(p.vdd25 + r(-200, 200)));
  frame[832] = static_cast<uint16_t>((chess ? 0x1000 : 0) | (adc_code << 10) | (3 << 7) | 0x0001);
  frame[833] = static_cast<uint16_t>(sub_page);
}

int main() {
  static paramsMLX90640 ref;
  static MLX90640Params ours;
  uint16_t ee[MLX_EEPROM_WORDS], frame[MLX_FRAME_WORDS], frame_copy[MLX_FRAME_WORDS];
  std::vector<float> to_ours(MLX_PIXELS), to_ref(MLX_PIXELS);
  int test = 0, finite = 0, compared = 0;

  for (int seed = 1; seed <= 6; seed++) {
    for (int cal_il = 0; cal_il < 2; cal_il++) {
      std::mt19937 rng(seed * 7919 + cal_il);
      make_eeprom(rng, cal_il != 0, ee);
      if (!mlx_eeprom_plausible(ee)) {
        std::printf("FAIL seed %d: generated calibration judged implausible\n", seed);
        failures++;
      }
      mlx_extract_parameters(ee, &ours);
      std::memcpy(frame_copy, ee, sizeof(ee));  // the reference takes a non-const pointer
      const int err = MLX90640_ExtractParameters(frame_copy, &ref);
      if (err != 0) {
        std::printf("FAIL seed %d: reference rejected calibration (%d)\n", seed, err);
        failures++;
        continue;
      }
      // Bad pixels: ours flags them per pixel, the reference lists them.
      for (int px = 0; px < MLX_PIXELS; px++) {
        int expected = 0;
        for (uint16_t b : ref.brokenPixels)
          if (b == px)
            expected = 1;
        for (uint16_t o : ref.outlierPixels)
          if (o == px)
            expected = 2;
        expect_close("bad pixel flag", seed, ours.bad[px], expected, 0);
      }

      for (int chess = 0; chess < 2; chess++) {
        for (int adc = 2; adc <= 3; adc++) {  // 3 = differs from calibration (resolution correction)
          for (int sp = 0; sp < 2; sp++) {
            for (float emissivity : {0.95f, 1.0f}) {
              test++;
              make_frame(rng, ours, chess != 0, adc, sp, frame);
              if (!mlx_frame_valid(frame)) {
                std::printf("FAIL test %d: generated frame judged invalid\n", test);
                failures++;
              }
              std::memcpy(frame_copy, frame, sizeof(frame));
              const float vdd_ref = MLX90640_GetVdd(frame_copy, &ref);
              const float ta_ref = MLX90640_GetTa(frame_copy, &ref);
              expect_close("vdd", test, mlx_get_vdd(frame, &ours), vdd_ref, 1e-4);
              expect_close("ta", test, mlx_get_ta(frame, &ours), ta_ref, 2e-3);

              const float tr = ta_ref - 8.0f;
              constexpr float UNTOUCHED = -12345.0f;
              std::fill(to_ours.begin(), to_ours.end(), UNTOUCHED);
              std::fill(to_ref.begin(), to_ref.end(), UNTOUCHED);
              mlx_calculate_to(frame, &ours, emissivity, tr, to_ours.data());
              MLX90640_CalculateTo(frame_copy, &ref, emissivity, tr, to_ref.data());
              for (int px = 0; px < MLX_PIXELS; px++) {
                const bool mine = mlx_pixel_subpage(px, chess != 0) == sp;
                // Both must update exactly this sub-page's pixels.
                expect_close("pixel updated", test, to_ours[px] != UNTOUCHED, mine, 0);
                expect_close("reference updated", test, to_ref[px] != UNTOUCHED, mine, 0);
                if (!mine)
                  continue;
                compared++;
                if (std::isfinite(to_ref[px]) && std::isfinite(to_ours[px])) {
                  finite++;
                  // float vs double maths and the reference's re-quantised alpha/Kta/Kv
                  expect_close("To", test, to_ours[px], to_ref[px], 0.02 + 2e-4 * std::fabs(to_ref[px]));
                } else {
                  expect_close("To finite", test, std::isfinite(to_ours[px]), std::isfinite(to_ref[px]), 0);
                }
              }
            }
          }
        }
      }
    }
  }

  std::printf("%d tests, %d checks, %d pixel temperatures compared (%d finite), %d failures\n", test, checks,
              compared, finite, failures);
  if (compared == 0 || finite < compared * 95 / 100) {
    std::printf("FAIL: too few finite temperatures, the generated data is not realistic enough\n");
    return 1;
  }
  return failures == 0 ? 0 : 1;
}
