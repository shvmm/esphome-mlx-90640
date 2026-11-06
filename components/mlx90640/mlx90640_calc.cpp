#include "mlx90640_calc.h"

#include <cmath>

namespace esphome::mlx90640 {

// Sign-extend an unsigned bit field of the given width.
static inline int sx(int v, int bits) {
  const int half = 1 << (bits - 1);
  return v >= half ? v - (1 << bits) : v;
}
static inline float s16(uint16_t v) { return static_cast<float>(static_cast<int16_t>(v)); }

bool mlx_eeprom_plausible(const uint16_t *ee) {
  bool all_same = true;
  for (int i = 1; i < MLX_EEPROM_WORDS && all_same; i++)
    all_same = ee[i] == ee[0];
  // These calibration values divide later on, so zero means the data is not real.
  const bool zero_divisor = ((ee[51] >> 8) & 0xFF) == 0 || ee[48] == 0 || (ee[50] & 0x3FF) == 0;
  return !all_same && !zero_divisor;
}

void mlx_extract_parameters(const uint16_t *ee, MLX90640Params *p) {
  // --- Supply voltage ---
  p->kVdd = static_cast<int16_t>(sx((ee[51] >> 8) & 0xFF, 8) * 32);
  p->vdd25 = static_cast<int16_t>((((ee[51] & 0xFF) - 256) * 32) - 8192);

  // --- PTAT (die temperature) ---
  p->KvPTAT = sx((ee[50] >> 10) & 0x3F, 6) / 4096.0f;
  p->KtPTAT = sx(ee[50] & 0x3FF, 10) / 8.0f;
  p->vPTAT25 = ee[49];
  p->alphaPTAT = ((ee[16] >> 12) & 0xF) / 4.0f + 8.0f;

  // --- Gain, TGC, resolution, KsTa ---
  p->gainEE = static_cast<int16_t>(ee[48]);
  p->tgc = sx(ee[60] & 0xFF, 8) / 32.0f;
  p->resolutionEE = (ee[56] >> 12) & 0x3;
  p->KsTa = sx((ee[60] >> 8) & 0xFF, 8) / 8192.0f;

  // --- KsTo and corner temperatures ---
  const int step = ((ee[63] >> 12) & 0x3) * 10;
  p->ct[0] = -40;
  p->ct[1] = 0;
  p->ct[2] = static_cast<int16_t>(((ee[63] >> 4) & 0xF) * step);
  p->ct[3] = static_cast<int16_t>(p->ct[2] + ((ee[63] >> 8) & 0xF) * step);
  p->ct[4] = 400;
  const float ks_to_scale = static_cast<float>(1UL << ((ee[63] & 0xF) + 8));
  p->ksTo[0] = sx(ee[61] & 0xFF, 8) / ks_to_scale;
  p->ksTo[1] = sx((ee[61] >> 8) & 0xFF, 8) / ks_to_scale;
  p->ksTo[2] = sx(ee[62] & 0xFF, 8) / ks_to_scale;
  p->ksTo[3] = sx((ee[62] >> 8) & 0xFF, 8) / ks_to_scale;
  p->ksTo[4] = -0.0002f;

  // --- Compensation pixel ---
  const int alpha_scale_cp = ((ee[32] >> 12) & 0xF) + 27;
  const int offset_sp0 = sx(ee[58] & 0x3FF, 10);
  const int offset_sp1 = sx((ee[58] >> 10) & 0x3F, 6) + offset_sp0;
  const float alpha_sp0 = ldexpf(static_cast<float>(sx(ee[57] & 0x3FF, 10)), -alpha_scale_cp);
  const float alpha_sp1 = (1.0f + sx((ee[57] >> 10) & 0x3F, 6) / 128.0f) * alpha_sp0;
  const int kta_scale1 = ((ee[56] >> 4) & 0xF) + 8;
  const int kta_scale2 = ee[56] & 0xF;
  const int kv_scale = (ee[56] >> 8) & 0xF;
  p->cpKta = ldexpf(static_cast<float>(sx(ee[59] & 0xFF, 8)), -kta_scale1);
  p->cpKv = ldexpf(static_cast<float>(sx((ee[59] >> 8) & 0xFF, 8)), -kv_scale);
  p->cpAlpha[0] = alpha_sp0;
  p->cpAlpha[1] = alpha_sp1;
  p->cpOffset[0] = static_cast<int16_t>(offset_sp0);
  p->cpOffset[1] = static_cast<int16_t>(offset_sp1);

  // --- Per-pixel sensitivity (alpha) ---
  const int acc_rem_scale = ee[32] & 0xF;
  const int acc_col_scale = (ee[32] >> 4) & 0xF;
  const int acc_row_scale = (ee[32] >> 8) & 0xF;
  const int alpha_scale = ((ee[32] >> 12) & 0xF) + 30;
  const int alpha_ref = ee[33];
  int acc_row[MLX_ROWS], acc_col[MLX_COLS];
  for (int i = 0; i < 6; i++)
    for (int k = 0; k < 4; k++)
      acc_row[i * 4 + k] = sx((ee[34 + i] >> (4 * k)) & 0xF, 4);
  for (int i = 0; i < 8; i++)
    for (int k = 0; k < 4; k++)
      acc_col[i * 4 + k] = sx((ee[40 + i] >> (4 * k)) & 0xF, 4);

  // --- Per-pixel offset ---
  const int occ_rem_scale = ee[16] & 0xF;
  const int occ_col_scale = (ee[16] >> 4) & 0xF;
  const int occ_row_scale = (ee[16] >> 8) & 0xF;
  const int offset_ref = static_cast<int16_t>(ee[17]);
  int occ_row[MLX_ROWS], occ_col[MLX_COLS];
  for (int i = 0; i < 6; i++)
    for (int k = 0; k < 4; k++)
      occ_row[i * 4 + k] = sx((ee[18 + i] >> (4 * k)) & 0xF, 4);
  for (int i = 0; i < 8; i++)
    for (int k = 0; k < 4; k++)
      occ_col[i * 4 + k] = sx((ee[24 + i] >> (4 * k)) & 0xF, 4);

  // --- Kta / Kv row-column groups ---
  // Index: 2 * (row % 2) + (col % 2)  ->  [RoCo, RoCe, ReCo, ReCe] (1-based odd/even)
  int kta_rc[4], kv_rc[4];
  kta_rc[0] = sx((ee[54] >> 8) & 0xFF, 8);
  kta_rc[2] = sx(ee[54] & 0xFF, 8);
  kta_rc[1] = sx((ee[55] >> 8) & 0xFF, 8);
  kta_rc[3] = sx(ee[55] & 0xFF, 8);
  kv_rc[0] = sx((ee[52] >> 12) & 0xF, 4);
  kv_rc[2] = sx((ee[52] >> 8) & 0xF, 4);
  kv_rc[1] = sx((ee[52] >> 4) & 0xF, 4);
  kv_rc[3] = sx(ee[52] & 0xF, 4);
  for (int i = 0; i < 4; i++)
    p->kvRC[i] = ldexpf(static_cast<float>(kv_rc[i]), -kv_scale);

  for (int row = 0; row < MLX_ROWS; row++) {
    for (int col = 0; col < MLX_COLS; col++) {
      const int px = row * MLX_COLS + col;
      const uint16_t w = ee[64 + px];
      const int split = 2 * (row % 2) + (col % 2);

      int a = sx((w >> 4) & 0x3F, 6) * (1 << acc_rem_scale);
      a = alpha_ref + acc_row[row] * (1 << acc_row_scale) + acc_col[col] * (1 << acc_col_scale) + a;
      p->alpha[px] = ldexpf(static_cast<float>(a), -alpha_scale);

      int o = sx((w >> 10) & 0x3F, 6) * (1 << occ_rem_scale);
      p->offset[px] = static_cast<int16_t>(offset_ref + occ_row[row] * (1 << occ_row_scale) +
                                           occ_col[col] * (1 << occ_col_scale) + o);

      int k = sx((w >> 1) & 0x7, 3) * (1 << kta_scale2);
      p->kta[px] = ldexpf(static_cast<float>(kta_rc[split] + k), -kta_scale1);

      p->bad[px] = (w == 0) ? 1 : ((w & 0x1) ? 2 : 0);
    }
  }

  // --- Interleaved / chess correction ---
  p->calibrationModeEE = static_cast<uint8_t>(((ee[10] & 0x0800) >> 4) ^ 0x80);
  p->ilChessC[0] = sx(ee[53] & 0x3F, 6) / 16.0f;
  p->ilChessC[1] = sx((ee[53] >> 6) & 0x1F, 5) / 2.0f;
  p->ilChessC[2] = sx((ee[53] >> 11) & 0x1F, 5) / 8.0f;
}

bool mlx_frame_valid(const uint16_t *frame) {
  // First pixel of every row the current sub-page wrote (same check as the Melexis driver)
  const int sub_page = frame[833] & 1;
  for (int row = sub_page; row < MLX_ROWS; row += 2)
    if (frame[row * MLX_COLS] == 0x7FFF)
      return false;
  // Auxiliary words that carry data (gaps are unused addresses)
  static const uint8_t AUX_RANGES[][2] = {{0, 1}, {8, 19}, {20, 23}, {24, 33}, {40, 51}, {52, 55}, {56, 64}};
  const uint16_t *aux = frame + MLX_PIXELS;
  for (const auto &r : AUX_RANGES)
    for (int i = r[0]; i < r[1]; i++)
      if (aux[i] == 0x7FFF)
        return false;
  return true;
}

float mlx_get_vdd(const uint16_t *frame, const MLX90640Params *p) {
  const float vdd = s16(frame[810]);
  const int resolution_ram = (frame[832] & 0x0C00) >> 10;
  const float res_corr = ldexpf(1.0f, p->resolutionEE - resolution_ram);
  return (res_corr * vdd - p->vdd25) / p->kVdd + 3.3f;
}

float mlx_get_ta(const uint16_t *frame, const MLX90640Params *p) {
  const float vdd = mlx_get_vdd(frame, p);
  const float ptat = s16(frame[800]);
  float ptat_art = s16(frame[768]);
  ptat_art = (ptat / (ptat * p->alphaPTAT + ptat_art)) * 262144.0f;  // 2^18
  const float ta = ptat_art / (1.0f + p->KvPTAT * (vdd - 3.3f)) - p->vPTAT25;
  return ta / p->KtPTAT + 25.0f;
}

void mlx_calculate_to(const uint16_t *frame, const MLX90640Params *p, float emissivity, float tr, float *result) {
  const int sub_page = frame[833] & 1;
  const float vdd = mlx_get_vdd(frame, p);
  const float ta = mlx_get_ta(frame, p);

  const float ta4 = powf(ta + 273.15f, 4.0f);
  const float tr4 = powf(tr + 273.15f, 4.0f);
  const float ta_tr = tr4 - (tr4 - ta4) / emissivity;

  float alpha_corr_r[4];
  alpha_corr_r[0] = 1.0f / (1.0f + p->ksTo[0] * 40.0f);
  alpha_corr_r[1] = 1.0f;
  alpha_corr_r[2] = 1.0f + p->ksTo[1] * p->ct[2];
  alpha_corr_r[3] = alpha_corr_r[2] * (1.0f + p->ksTo[2] * (p->ct[3] - p->ct[2]));

  const float gain = p->gainEE / s16(frame[778]);
  const int mode = (frame[832] & 0x1000) >> 5;  // 0x80 = chess, 0 = interleaved

  const float dta = ta - 25.0f;
  const float dvdd = vdd - 3.3f;
  float ir_cp[2];
  ir_cp[0] = s16(frame[776]) * gain;
  ir_cp[1] = s16(frame[808]) * gain;
  ir_cp[0] -= p->cpOffset[0] * (1.0f + p->cpKta * dta) * (1.0f + p->cpKv * dvdd);
  if (mode == p->calibrationModeEE) {
    ir_cp[1] -= p->cpOffset[1] * (1.0f + p->cpKta * dta) * (1.0f + p->cpKv * dvdd);
  } else {
    ir_cp[1] -= (p->cpOffset[1] + p->ilChessC[0]) * (1.0f + p->cpKta * dta) * (1.0f + p->cpKv * dvdd);
  }

  for (int px = 0; px < MLX_PIXELS; px++) {
    const int il_pattern = (px / 32) % 2;
    const int chess_pattern = il_pattern ^ (px % 2);
    const int pattern = (mode == 0) ? il_pattern : chess_pattern;
    if (pattern != sub_page)
      continue;
    const int split = 2 * il_pattern + (px & 1);

    float ir = s16(frame[px]) * gain;
    ir -= p->offset[px] * (1.0f + p->kta[px] * dta) * (1.0f + p->kvRC[split] * dvdd);
    if (mode != p->calibrationModeEE) {
      const int conversion_pattern =
          ((px + 2) / 4 - (px + 3) / 4 + (px + 1) / 4 - px / 4) * (1 - 2 * il_pattern);
      ir += p->ilChessC[2] * (2 * il_pattern - 1) - p->ilChessC[1] * conversion_pattern;
    }
    ir -= p->tgc * ir_cp[sub_page];
    ir /= emissivity;

    const float alpha_c = (p->alpha[px] - p->tgc * p->cpAlpha[sub_page]) * (1.0f + p->KsTa * dta);

    float s = alpha_c * alpha_c * alpha_c * (ir + alpha_c * ta_tr);
    s = sqrtf(sqrtf(s)) * p->ksTo[1];
    float to = sqrtf(sqrtf(ir / (alpha_c * (1.0f - p->ksTo[1] * 273.15f) + s) + ta_tr)) - 273.15f;

    int range;
    if (to < p->ct[1])
      range = 0;
    else if (to < p->ct[2])
      range = 1;
    else if (to < p->ct[3])
      range = 2;
    else
      range = 3;

    to = sqrtf(sqrtf(ir / (alpha_c * alpha_corr_r[range] * (1.0f + p->ksTo[range] * (to - p->ct[range]))) + ta_tr)) -
         273.15f;
    result[px] = to;
  }
}

}  // namespace esphome::mlx90640
