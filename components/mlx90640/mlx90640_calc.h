#pragma once
// MLX90640 calibration maths, following the algorithm in the Melexis MLX90640
// datasheet (section 11) and the Melexis reference driver (Apache-2.0, see
// NOTICE). No ESPHome dependencies, so it can be tested on a PC (tests/).
#include <cstdint>

namespace esphome::mlx90640 {

static constexpr int MLX_COLS = 32;
static constexpr int MLX_ROWS = 24;
static constexpr int MLX_PIXELS = MLX_COLS * MLX_ROWS;
static constexpr int MLX_EEPROM_WORDS = 832;
static constexpr int MLX_RAM_WORDS = 832;    // 768 pixels + 64 auxiliary words
static constexpr int MLX_FRAME_WORDS = 834;  // RAM words + control register + sub-page number

struct MLX90640Params {
  int16_t kVdd;
  int16_t vdd25;
  float KvPTAT;
  float KtPTAT;
  uint16_t vPTAT25;
  float alphaPTAT;
  int16_t gainEE;
  float tgc;
  float cpKv;
  float cpKta;
  uint8_t resolutionEE;
  uint8_t calibrationModeEE;
  float KsTa;
  float ksTo[5];
  int16_t ct[5];
  float alpha[MLX_PIXELS];
  float kta[MLX_PIXELS];
  int16_t offset[MLX_PIXELS];
  float kvRC[4];  // Kv only depends on row/column parity: index 2 * (row % 2) + (col % 2)
  float cpAlpha[2];
  int16_t cpOffset[2];
  float ilChessC[3];
  uint8_t bad[MLX_PIXELS];  // 0 = ok, 1 = broken, 2 = outlier
};

/// Sub-page (0 or 1) that measures pixel `px`, in chess (true) or interleaved (false) mode.
inline int mlx_pixel_subpage(int px, bool chess) {
  const int il = (px / MLX_COLS) & 1;
  return chess ? (il ^ (px & 1)) : il;
}

/// Returns false if the calibration data cannot be right (blank or wrong device).
bool mlx_eeprom_plausible(const uint16_t *ee);
void mlx_extract_parameters(const uint16_t *ee, MLX90640Params *p);
/// Checks a frame for the 0x7FFF placeholders the sensor leaves in words it has not written yet.
bool mlx_frame_valid(const uint16_t *frame);
float mlx_get_vdd(const uint16_t *frame, const MLX90640Params *p);
float mlx_get_ta(const uint16_t *frame, const MLX90640Params *p);
/// Updates only the pixels belonging to the sub-page in frame[833].
void mlx_calculate_to(const uint16_t *frame, const MLX90640Params *p, float emissivity, float tr, float *result);

}  // namespace esphome::mlx90640
