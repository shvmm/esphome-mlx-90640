#include "mlx90640.h"
#include "heatmap_page.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome::mlx90640 {

static const char *const TAG = "mlx90640";

static constexpr uint16_t REG_STATUS = 0x8000;
static constexpr uint16_t REG_CONTROL1 = 0x800D;
static constexpr uint16_t ADDR_RAM = 0x0400;
static constexpr uint16_t ADDR_EEPROM = 0x2400;
static constexpr size_t CHUNK_WORDS = 64;  // 128-byte I2C transactions
static constexpr uint32_t INIT_RETRY_MS = 5000;
static const float RATE_HZ[8] = {0.5f, 1, 2, 4, 8, 16, 32, 64};

struct SettingInfo {
  const char *key;  // name used by the web page / JSON
  float min;
  float max;
  float def;
  bool integer;
};

// Order must match enum Setting.
static const SettingInfo SETTINGS[SETTING_COUNT] = {
    {"refresh_rate", 0, 7, 3, true},
    {"resolution", 0, 3, 2, true},
    {"pattern", 0, 1, 0, true},
    {"palette", 0, 2, 0, true},
    {"emissivity", 0.1f, 1.0f, 0.95f, false},
    {"reflected_offset", -20, 40, 8, false},
    {"smoothing", 0, 0.95f, 0, false},
    {"range_min", -40, 300, 20, false},
    {"range_max", -40, 300, 35, false},
    {"publish_interval", 1, 3600, 10, false},
    {"interpolate_bad_pixels", 0, 1, 1, true},
    {"auto_range", 0, 1, 1, true},
    {"mirror", 0, 1, 0, true},
    {"rotate_180", 0, 1, 0, true},
    {"freeze", 0, 1, 0, true},
};

MLX90640Component::MLX90640Component(web_server_base::WebServerBase *base) : base_(base) {
  for (int i = 0; i < SETTING_COUNT; i++)
    this->settings_[i] = SETTINGS[i].def;
}

// ---------------------------------------------------------------- settings

float MLX90640Component::get_setting(Setting s) const {
  if (s == SETTING_PUBLISH_INTERVAL)
    return this->get_update_interval() / 1000.0f;
  return this->settings_[s];
}

float MLX90640Component::set_setting(Setting s, float value) {
  if (s >= SETTING_COUNT || !std::isfinite(value))
    return this->get_setting(s);
  const SettingInfo &info = SETTINGS[s];
  value = std::clamp(value, info.min, info.max);
  if (info.integer)
    value = std::round(value);
  const float old = this->get_setting(s);
  this->settings_[s] = value;

  switch (s) {
    case SETTING_REFRESH_RATE:
    case SETTING_RESOLUTION:
    case SETTING_PATTERN:
      if (this->ready_ && value != old && !this->apply_control_register_())
        this->status_set_warning();
      break;
    case SETTING_PUBLISH_INTERVAL:
      this->set_update_interval(static_cast<uint32_t>(value * 1000.0f));
      if (this->setup_done_ && value != old) {
        this->stop_poller();
        this->start_poller();
      }
      break;
    case SETTING_SMOOTHING:
    case SETTING_INTERPOLATE_BAD_PIXELS:
      this->subpages_seen_ = 0;  // rebuild the image cleanly with the new processing
      break;
    default:
      break;
  }
  if (value != old)
    ESP_LOGD(TAG, "Setting %s = %g", info.key, value);
  return value;
}

void MLX90640Component::apply_pending_web_changes_() {
  uint32_t mask = this->pending_mask_.exchange(0);
  for (int i = 0; mask != 0 && i < SETTING_COUNT; i++) {
    if (!(mask & (1u << i)))
      continue;
    mask &= ~(1u << i);
    const auto s = static_cast<Setting>(i);
    const float v = this->pending_values_[i];
    if (this->entities_[s] != nullptr) {
      this->entities_[s]->apply_setting(v);  // publishes + persists
    } else {
      this->set_setting(s, v);
    }
  }
}

// ---------------------------------------------------------------- I2C

bool MLX90640Component::read_words_(uint16_t addr, uint16_t *dst, size_t count) {
  uint8_t buf[CHUNK_WORDS * 2];
  while (count > 0) {
    const size_t n = std::min(count, CHUNK_WORDS);
    if (this->read_register16(addr, buf, n * 2) != i2c::ERROR_OK) {
      this->i2c_errors_++;
      return false;
    }
    for (size_t i = 0; i < n; i++)
      dst[i] = static_cast<uint16_t>((buf[2 * i] << 8) | buf[2 * i + 1]);
    addr += n;
    dst += n;
    count -= n;
  }
  return true;
}

bool MLX90640Component::write_word_(uint16_t addr, uint16_t value) {
  const uint8_t data[2] = {static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value & 0xFF)};
  if (this->write_register16(addr, data, 2) != i2c::ERROR_OK) {
    this->i2c_errors_++;
    return false;
  }
  return true;
}

// Control register 1: refresh rate bits 7..9, ADC resolution bits 10..11, chess mode bit 12.
bool MLX90640Component::apply_control_register_() {
  uint16_t ctrl;
  if (!this->read_words_(REG_CONTROL1, &ctrl, 1))
    return false;
  uint16_t wanted = ctrl & ~static_cast<uint16_t>((0x7 << 7) | (0x3 << 10) | (0x1 << 12));
  wanted |= (static_cast<uint16_t>(this->settings_[SETTING_REFRESH_RATE]) & 0x7) << 7;
  wanted |= (static_cast<uint16_t>(this->settings_[SETTING_RESOLUTION]) & 0x3) << 10;
  if (!this->flag_(SETTING_PATTERN))
    wanted |= 0x1000;  // chess
  if (wanted != ctrl) {
    uint16_t check = 0;
    if (!this->write_word_(REG_CONTROL1, wanted) || !this->read_words_(REG_CONTROL1, &check, 1) || check != wanted) {
      ESP_LOGW(TAG, "Failed to write control register (0x%04X)", wanted);
      return false;
    }
  }
  this->subpages_seen_ = 0;
  return true;
}

bool MLX90640Component::init_sensor_() {
  this->last_init_attempt_ms_ = millis();
  this->ready_ = false;
  this->subpages_seen_ = 0;
  if (!this->read_words_(ADDR_EEPROM, this->eeprom_, MLX_EEPROM_WORDS)) {
    ESP_LOGW(TAG, "Sensor not responding at 0x%02X - check wiring; retrying", this->address_);
    this->status_set_error();
    return false;
  }
  mlx_extract_parameters(this->eeprom_, &this->params_);
  this->bad_pixel_count_ = 0;
  for (uint8_t b : this->params_.bad)
    if (b)
      this->bad_pixel_count_++;
  if (!this->apply_control_register_()) {
    this->status_set_error();
    return false;
  }
  this->ready_ = true;
  this->status_clear_error();
  this->status_clear_warning();
  ESP_LOGI(TAG, "Sensor initialised (%u bad/outlier pixels)", static_cast<unsigned>(this->bad_pixel_count_));
  return true;
}

// ---------------------------------------------------------------- lifecycle

void MLX90640Component::setup() {
  this->base_->init();
  this->base_->add_handler(this);
  this->init_sensor_();
  this->setup_done_ = true;
}

void MLX90640Component::dump_config() {
  ESP_LOGCONFIG(TAG, "MLX90640 thermal camera:");
  LOG_I2C_DEVICE(this);
  ESP_LOGCONFIG(TAG,
                "  Status: %s\n"
                "  Sub-page rate: %.1f Hz, ADC: %d-bit, pattern: %s\n"
                "  Emissivity: %.2f\n"
                "  Bad/outlier pixels: %u\n"
                "  Heatmap: http://<device>/thermal  (JSON: /thermal.json)",
                this->ready_ ? "OK" : "NOT RESPONDING", RATE_HZ[static_cast<int>(this->settings_[SETTING_REFRESH_RATE])],
                16 + static_cast<int>(this->settings_[SETTING_RESOLUTION]),
                this->flag_(SETTING_PATTERN) ? "interleaved" : "chess", this->settings_[SETTING_EMISSIVITY],
                static_cast<unsigned>(this->bad_pixel_count_));
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Min", this->min_sensor_);
  LOG_SENSOR("  ", "Max", this->max_sensor_);
  LOG_SENSOR("  ", "Mean", this->mean_sensor_);
  LOG_SENSOR("  ", "Center", this->center_sensor_);
  LOG_SENSOR("  ", "Ambient", this->ambient_sensor_);
  LOG_SENSOR("  ", "Frame rate", this->frame_rate_sensor_);
  LOG_SENSOR("  ", "I2C errors", this->i2c_errors_sensor_);
  LOG_SENSOR("  ", "Bad pixels", this->bad_pixels_sensor_);
  LOG_SENSOR("  ", "Supply voltage", this->supply_voltage_sensor_);
}

void MLX90640Component::loop() {
  if (this->pending_mask_.load() != 0)
    this->apply_pending_web_changes_();

  const uint32_t now = millis();
  if (this->reinit_requested_) {
    this->reinit_requested_ = false;
    ESP_LOGI(TAG, "Re-initialising sensor");
    this->init_sensor_();
    return;
  }
  if (!this->ready_) {
    if (now - this->last_init_attempt_ms_ >= INIT_RETRY_MS)
      this->init_sensor_();
    return;
  }

  if (now - this->last_poll_ms_ < 10)
    return;
  this->last_poll_ms_ = now;

  uint16_t status;
  if (!this->read_words_(REG_STATUS, &status, 1) || !(status & 0x0008))
    return;  // no new sub-page yet

  // Clear "new data" flag (keeps overwrite enabled), then read RAM + control register.
  this->write_word_(REG_STATUS, 0x0030);
  uint16_t ctrl;
  if (!this->read_words_(ADDR_RAM, this->frame_, 832) || !this->read_words_(REG_CONTROL1, &ctrl, 1))
    return;
  this->frame_[832] = ctrl;
  this->frame_[833] = status & 0x0001;
  if (this->frame_[768] == 0x7FFF)
    return;  // invalid aux data

  const float ta = mlx_get_ta(this->frame_, &this->params_);
  if (!std::isfinite(ta) || ta < -50 || ta > 150)
    return;
  this->vdd_ = mlx_get_vdd(this->frame_, &this->params_);
  mlx_calculate_to(this->frame_, &this->params_, this->settings_[SETTING_EMISSIVITY],
                   ta - this->settings_[SETTING_REFLECTED_OFFSET], this->work_);

  this->subpages_seen_ |= 1 << this->frame_[833];
  if (this->subpages_seen_ == 0x3)
    this->finish_frame_(ta);
}

void MLX90640Component::finish_frame_(float ta) {
  this->ta_ = ta;
  if (this->flag_(SETTING_FREEZE))
    return;

  const bool interpolate = this->flag_(SETTING_INTERPOLATE_BAD_PIXELS);
  // Exponential smoothing; skipped for the first frame after a (re)start.
  const float alpha = this->has_frame_ ? this->settings_[SETTING_SMOOTHING] : 0.0f;
  float mn = INFINITY, mx = -INFINITY, sum = 0;
  int n = 0;
  for (int px = 0; px < MLX_PIXELS; px++) {
    float v = this->work_[px];
    if (interpolate && this->params_.bad[px]) {
      // Replace broken/outlier pixels with the mean of their good 4-neighbours.
      const int r = px / MLX_COLS, c = px % MLX_COLS;
      float acc = 0;
      int cnt = 0;
      const int nb[4][2] = {{r - 1, c}, {r + 1, c}, {r, c - 1}, {r, c + 1}};
      for (auto &q : nb) {
        if (q[0] < 0 || q[0] >= MLX_ROWS || q[1] < 0 || q[1] >= MLX_COLS)
          continue;
        const int qi = q[0] * MLX_COLS + q[1];
        if (this->params_.bad[qi] || !std::isfinite(this->work_[qi]))
          continue;
        acc += this->work_[qi];
        cnt++;
      }
      v = cnt ? acc / cnt : NAN;
    }
    const float prev = this->output_[px];
    if (alpha > 0 && std::isfinite(prev) && std::isfinite(v))
      v = alpha * prev + (1.0f - alpha) * v;
    this->output_[px] = v;
    if (std::isfinite(v)) {
      mn = std::min(mn, v);
      mx = std::max(mx, v);
      sum += v;
      n++;
    }
  }
  if (n == 0)
    return;

  const int c0 = 11 * MLX_COLS + 15;  // 2x2 block in the middle of the array
  this->center_ = (this->output_[c0] + this->output_[c0 + 1] + this->output_[c0 + MLX_COLS] +
                   this->output_[c0 + MLX_COLS + 1]) /
                  4.0f;
  this->min_ = mn;
  this->max_ = mx;
  this->mean_ = sum / n;

  const uint32_t now = millis();
  if (this->last_frame_ms_ != 0 && now > this->last_frame_ms_) {
    const float inst = 1000.0f / (now - this->last_frame_ms_);
    this->fps_ = this->fps_ == 0 ? inst : this->fps_ * 0.8f + inst * 0.2f;
  }
  this->last_frame_ms_ = now;
  this->frame_count_++;
  this->has_frame_ = true;
}

void MLX90640Component::update() {
  if (this->i2c_errors_sensor_ != nullptr)
    this->i2c_errors_sensor_->publish_state(this->i2c_errors_);
  if (this->bad_pixels_sensor_ != nullptr && this->ready_)
    this->bad_pixels_sensor_->publish_state(this->bad_pixel_count_);
  if (!this->has_frame_ || !this->ready_) {
    ESP_LOGD(TAG, "No frame yet (I2C errors: %u)", static_cast<unsigned>(this->i2c_errors_));
    return;
  }
  if (this->min_sensor_ != nullptr)
    this->min_sensor_->publish_state(this->min_);
  if (this->max_sensor_ != nullptr)
    this->max_sensor_->publish_state(this->max_);
  if (this->mean_sensor_ != nullptr)
    this->mean_sensor_->publish_state(this->mean_);
  if (this->center_sensor_ != nullptr)
    this->center_sensor_->publish_state(this->center_);
  if (this->ambient_sensor_ != nullptr)
    this->ambient_sensor_->publish_state(this->ta_);
  if (this->frame_rate_sensor_ != nullptr)
    this->frame_rate_sensor_->publish_state(this->fps_);
  if (this->supply_voltage_sensor_ != nullptr)
    this->supply_voltage_sensor_->publish_state(this->vdd_);
}

// ---------------------------------------------------------------- web

bool MLX90640Component::canHandle(AsyncWebServerRequest *request) const {
  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  StringRef url = request->url_to(url_buf);
  if (request->method() == HTTP_POST)
    return url == "/thermal/set";
  if (request->method() != HTTP_GET)
    return false;
  return url == "/thermal" || url == "/thermal/" || url == "/thermal.json";
}

void MLX90640Component::handleRequest(AsyncWebServerRequest *request) {
  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  StringRef url = request->url_to(url_buf);

  // POST /thermal/set?key=<setting>&value=<number>
  if (url == "/thermal/set") {
    const std::string key = request->arg("key");
    const std::string value = request->arg("value");
    char *end = nullptr;
    const float v = strtof(value.c_str(), &end);
    if (value.empty() || end == value.c_str() || !std::isfinite(v)) {
      request->send(400, "application/json", "{\"error\":\"bad value\"}");
      return;
    }
    for (int i = 0; i < SETTING_COUNT; i++) {
      if (key == SETTINGS[i].key) {
        this->pending_values_[i] = v;
        this->pending_mask_.fetch_or(1u << i);
        request->send(200, "application/json", "{\"ok\":true}");
        return;
      }
    }
    request->send(404, "application/json", "{\"error\":\"unknown setting\"}");
    return;
  }

  if (url != "/thermal.json") {
    request->send(200, "text/html", HEATMAP_PAGE);
    return;
  }

  // Runs on the HTTP server task; output_ may be updated concurrently, which at
  // worst mixes two consecutive frames in one response.
  AsyncResponseStream *stream = request->beginResponseStream("application/json");
  stream->printf("{\"w\":%d,\"h\":%d,\"ready\":%s,\"ta\":%.2f,\"vdd\":%.3f,\"min\":%.2f,\"max\":%.2f,\"mean\":%.2f,"
                 "\"fps\":%.2f,\"frame\":%u,\"i2c_errors\":%u,\"cfg\":{",
                 MLX_COLS, MLX_ROWS, this->ready_ ? "true" : "false", std::isfinite(this->ta_) ? this->ta_ : 0.0f,
                 std::isfinite(this->vdd_) ? this->vdd_ : 0.0f, std::isfinite(this->min_) ? this->min_ : 0.0f,
                 std::isfinite(this->max_) ? this->max_ : 0.0f, std::isfinite(this->mean_) ? this->mean_ : 0.0f,
                 this->fps_, static_cast<unsigned>(this->frame_count_), static_cast<unsigned>(this->i2c_errors_));
  for (int i = 0; i < SETTING_COUNT; i++)
    stream->printf("%s\"%s\":%g", i ? "," : "", SETTINGS[i].key, this->get_setting(static_cast<Setting>(i)));
  stream->print("},\"t\":");

  if (!this->has_frame_) {
    stream->print("null}");
    request->send(stream);
    return;
  }

  // Pixels are emitted in display orientation (mirror / rotate applied here).
  const bool mirror = this->flag_(SETTING_MIRROR);
  const bool rotate = this->flag_(SETTING_ROTATE_180);
  stream->print("[");
  char buf[16];
  for (int y = 0; y < MLX_ROWS; y++) {
    for (int x = 0; x < MLX_COLS; x++) {
      int sx = x, sy = y;
      if (rotate) {
        sx = MLX_COLS - 1 - sx;
        sy = MLX_ROWS - 1 - sy;
      }
      if (mirror)
        sx = MLX_COLS - 1 - sx;
      const float v = this->output_[sy * MLX_COLS + sx];
      const char *sep = (x || y) ? "," : "";
      if (std::isfinite(v)) {
        snprintf(buf, sizeof(buf), "%s%.1f", sep, v);
      } else {
        snprintf(buf, sizeof(buf), "%snull", sep);
      }
      stream->print(buf);
    }
  }
  stream->print("]}");
  request->send(stream);
}

}  // namespace esphome::mlx90640
