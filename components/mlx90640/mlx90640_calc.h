#pragma once
// MLX90640 calibration maths, following the algorithm in the Melexis MLX90640
// datasheet (section 11) and the reference driver (Apache-2.0). No ESPHome
// dependencies, so it can be unit-tested on a PC.
#include <cstdint>

namespace esphome::mlx90640 {

static constexpr int MLX_COLS = 32;
static constexpr int MLX_ROWS = 24;
static constexpr int MLX_PIXELS = MLX_COLS * MLX_ROWS;
static constexpr int MLX_EEPROM_WORDS = 832;
static constexpr int MLX_FRAME_WORDS = 834;  // 832 RAM words + control reg + sub-page

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
  float offset[MLX_PIXELS];
  float kta[MLX_PIXELS];
  float kv[MLX_PIXELS];
  float cpAlpha[2];
  int16_t cpOffset[2];
  float ilChessC[3];
  uint8_t bad[MLX_PIXELS];  // 0 = ok, 1 = broken, 2 = outlier
};

void mlx_extract_parameters(const uint16_t *ee, MLX90640Params *p);
float mlx_get_vdd(const uint16_t *frame, const MLX90640Params *p);
float mlx_get_ta(const uint16_t *frame, const MLX90640Params *p);
// Updates only the pixels belonging to the sub-page in frame[833].
void mlx_calculate_to(const uint16_t *frame, const MLX90640Params *p, float emissivity, float tr, float *result);

}  // namespace esphome::mlx90640
