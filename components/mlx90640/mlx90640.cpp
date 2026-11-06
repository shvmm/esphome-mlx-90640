#include "mlx90640.h"
#ifdef USE_MLX90640_CAMERA
#include "mlx90640_camera.h"
#endif

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <new>
#include <vector>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome::mlx90640 {

static const char *const TAG = "mlx90640";

static constexpr uint16_t REG_STATUS = 0x8000;
static constexpr uint16_t REG_CONTROL1 = 0x800D;
static constexpr uint16_t ADDR_RAM = 0x0400;
static constexpr uint16_t ADDR_EEPROM = 0x2400;
static constexpr uint16_t STATUS_SUBPAGE = 0x0001;
static constexpr uint16_t STATUS_DATA_READY = 0x0008;
static constexpr uint16_t STATUS_CLEAR = 0x0030;  // clears "data ready", keeps "overwrite enabled"
// Control register 1: refresh rate bits 7..9, ADC resolution bits 10..11, chess mode bit 12.
static constexpr uint16_t CTRL_MASK = (0x7 << 7) | (0x3 << 10) | (0x1 << 12);
static constexpr uint16_t CTRL_CHESS = 0x1000;

static constexpr size_t CHUNK_WORDS = 64;        // 128-byte I2C reads
static constexpr uint32_t READ_BUDGET_MS = 8;    // I2C time per loop() pass while reading a sub-page
static constexpr uint32_t INIT_RETRY_MS = 5000;  // retry interval while the sensor is missing
static constexpr uint8_t LOST_AFTER_ERRORS = 10;  // consecutive I2C failures before giving up on the sensor
static constexpr uint32_t MIN_STALL_MS = 3000;    // no-data watchdog (at least 4 sub-page periods)
static constexpr int HIGH_FREQ_RATE_CODE = 5;     // 16 Hz and up: keep loop() running continuously
static const float RATE_HZ[8] = {0.5f, 1, 2, 4, 8, 16, 32, 64};

// Binary frame format for GET /thermal.bin (little-endian), see heatmap.html for the reader.
static constexpr uint32_t BIN_MAGIC = 0x31584C4D;  // "MLX1"
static constexpr size_t BIN_HEADER_SIZE = 44;
static constexpr int16_t BIN_NO_VALUE = INT16_MIN;

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
    {"camera_overlay", 0, 1, 1, true},
    {"jpeg_quality", 10, 100, 80, true},
};

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
  if (value == old)
    return value;

  this->settings_gen_++;
  switch (s) {
    case SETTING_REFRESH_RATE:
    case SETTING_RESOLUTION:
    case SETTING_PATTERN:
      if (this->ready_ && !this->apply_control_register_())
        this->status_set_warning();
      break;
    case SETTING_PUBLISH_INTERVAL:
      this->set_update_interval(static_cast<uint32_t>(value * 1000.0f));
      if (this->setup_done_) {
        this->stop_poller();
        this->start_poller();
      }
      break;
    case SETTING_SMOOTHING:
    case SETTING_INTERPOLATE_BAD_PIXELS:
      this->subpages_seen_ = 0;  // rebuild the image cleanly with the new processing
      this->restart_image_ = true;
      break;
    default:
      break;
  }
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
      if (this->consecutive_errors_ < 255)
        this->consecutive_errors_++;
      return false;
    }
    for (size_t i = 0; i < n; i++)
      dst[i] = static_cast<uint16_t>((buf[2 * i] << 8) | buf[2 * i + 1]);
    addr += n;
    dst += n;
    count -= n;
  }
  this->consecutive_errors_ = 0;
  return true;
}

bool MLX90640Component::write_word_(uint16_t addr, uint16_t value) {
  const uint8_t data[2] = {static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value & 0xFF)};
  if (this->write_register16(addr, data, 2) != i2c::ERROR_OK) {
    this->i2c_errors_++;
    if (this->consecutive_errors_ < 255)
      this->consecutive_errors_++;
    return false;
  }
  this->consecutive_errors_ = 0;
  return true;
}

uint16_t MLX90640Component::wanted_control_bits_() const {
  uint16_t bits = (static_cast<uint16_t>(this->settings_[SETTING_REFRESH_RATE]) & 0x7) << 7;
  bits |= (static_cast<uint16_t>(this->settings_[SETTING_RESOLUTION]) & 0x3) << 10;
  if (!this->flag_(SETTING_PATTERN))
    bits |= CTRL_CHESS;
  return bits;
}

bool MLX90640Component::apply_control_register_() {
  uint16_t ctrl;
  if (!this->read_words_(REG_CONTROL1, &ctrl, 1))
    return false;
  const uint16_t wanted = (ctrl & ~CTRL_MASK) | this->wanted_control_bits_();
  if (wanted != ctrl) {
    uint16_t check = 0;
    if (!this->write_word_(REG_CONTROL1, wanted) || !this->read_words_(REG_CONTROL1, &check, 1) || check != wanted) {
      ESP_LOGW(TAG, "Failed to write control register (0x%04X)", wanted);
      return false;
    }
  }
  this->reading_ = false;  // a sub-page being read now may still use the old settings
  this->subpages_seen_ = 0;
  this->restart_image_ = true;
  return true;
}

uint32_t MLX90640Component::subpage_period_ms_() const {
  return static_cast<uint32_t>(1000.0f / RATE_HZ[static_cast<int>(this->settings_[SETTING_REFRESH_RATE]) & 0x7]);
}

bool MLX90640Component::init_sensor_() {
  this->last_init_attempt_ms_ = millis();
  this->ready_ = false;
  this->reading_ = false;
  this->subpages_seen_ = 0;
  this->consecutive_errors_ = 0;

  // The raw calibration data is only needed here, so it lives on the heap briefly.
  std::unique_ptr<uint16_t[]> ee(new (std::nothrow) uint16_t[MLX_EEPROM_WORDS]);
  if (!ee) {
    ESP_LOGE(TAG, "Out of memory reading calibration data");
    this->status_set_error();
    return false;
  }
  if (!this->read_words_(ADDR_EEPROM, ee.get(), MLX_EEPROM_WORDS)) {
    ESP_LOGW(TAG, "Sensor not responding at 0x%02X - check wiring; retrying", this->address_);
    this->status_set_error();
    return false;
  }
  if (!mlx_eeprom_plausible(ee.get())) {
    ESP_LOGW(TAG, "Calibration data at 0x%02X looks invalid (is this an MLX90640?); retrying", this->address_);
    this->status_set_error();
    return false;
  }
  mlx_extract_parameters(ee.get(), &this->params_);
  this->bad_pixel_count_ = 0;
  for (uint8_t b : this->params_.bad)
    if (b)
      this->bad_pixel_count_++;
  if (!this->apply_control_register_()) {
    this->status_set_error();
    return false;
  }
  this->ready_ = true;
  this->last_subpage_ms_ = millis();  // starts the no-data watchdog
  this->status_clear_error();
  this->status_clear_warning();
  ESP_LOGI(TAG, "Sensor initialised (%u bad/outlier pixels)", static_cast<unsigned>(this->bad_pixel_count_));
  return true;
}

void MLX90640Component::sensor_lost_(const char *reason) {
  ESP_LOGW(TAG, "Lost the sensor (%s); retrying every %u s", reason, static_cast<unsigned>(INIT_RETRY_MS / 1000));
  this->ready_ = false;
  this->reading_ = false;
  this->has_frame_ = false;
  this->subpages_seen_ = 0;
  this->consecutive_errors_ = 0;
  this->last_init_attempt_ms_ = millis();
  this->fps_ = 0;
  this->last_frame_ms_ = 0;
  this->status_set_error();
  // Show "unknown" in Home Assistant instead of the last readings.
  for (auto *s : {this->min_sensor_, this->max_sensor_, this->mean_sensor_, this->center_sensor_,
                  this->ambient_sensor_, this->frame_rate_sensor_, this->supply_voltage_sensor_}) {
    if (s != nullptr)
      s->publish_state(NAN);
  }
}

// ---------------------------------------------------------------- lifecycle

void MLX90640Component::setup() {
#ifdef USE_MLX90640_WEB_UI
  auto *base = web_server_base::global_web_server_base;
  if (base != nullptr) {
    base->init();
    base->add_handler(this);  // also applies web_server auth, if configured
  }
#endif
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
                "  Bad/outlier pixels: %u",
                this->ready_ ? "OK" : "NOT RESPONDING", RATE_HZ[static_cast<int>(this->settings_[SETTING_REFRESH_RATE])],
                16 + static_cast<int>(this->settings_[SETTING_RESOLUTION]),
                this->flag_(SETTING_PATTERN) ? "interleaved" : "chess", this->settings_[SETTING_EMISSIVITY],
                static_cast<unsigned>(this->bad_pixel_count_));
#ifdef USE_MLX90640_WEB_UI
  ESP_LOGCONFIG(TAG, "  Heatmap: http://<device>/thermal  (data: /thermal.json, /thermal.bin)");
#endif
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
  } else if (!this->ready_) {
    if (now - this->last_init_attempt_ms_ >= INIT_RETRY_MS)
      this->init_sensor_();
  } else if (this->reading_) {
    this->continue_read_();
  } else {
    this->poll_status_(now);
  }

  if (this->ready_ && this->consecutive_errors_ >= LOST_AFTER_ERRORS)
    this->sensor_lost_("I2C errors");

  if (this->ready_ && (this->reading_ || this->settings_[SETTING_REFRESH_RATE] >= HIGH_FREQ_RATE_CODE)) {
    this->high_freq_.start();
  } else {
    this->high_freq_.stop();
  }
}

// Waits for the sensor's "new sub-page" flag, then starts reading it.
void MLX90640Component::poll_status_(uint32_t now) {
  const uint32_t period = this->subpage_period_ms_();
  const uint32_t since = now - this->last_subpage_ms_;
  if (since > std::max<uint32_t>(4 * period, MIN_STALL_MS)) {
    this->sensor_lost_("no new data");
    return;
  }
  // Nothing to ask for until the next sub-page is nearly due; then ask every pass (>= 2 ms apart).
  if (since < period * 3 / 4 || now - this->last_poll_ms_ < 2)
    return;
  this->last_poll_ms_ = now;

  uint16_t status;
  if (!this->read_words_(REG_STATUS, &status, 1) || !(status & STATUS_DATA_READY))
    return;
  if (!this->write_word_(REG_STATUS, STATUS_CLEAR))
    return;
  this->reading_ = true;
  this->read_pos_ = 0;
  this->read_subpage_ = status & STATUS_SUBPAGE;
  this->last_subpage_ms_ = now;
  this->continue_read_();
}

// Reads the sub-page in 128-byte chunks, spending at most READ_BUDGET_MS per pass so
// a slow I2C bus never blocks the main loop for long (the loop runs continuously meanwhile).
void MLX90640Component::continue_read_() {
  const uint32_t start = millis();
  do {
    const size_t n = std::min<size_t>(CHUNK_WORDS, MLX_RAM_WORDS - this->read_pos_);
    if (!this->read_words_(ADDR_RAM + this->read_pos_, this->frame_ + this->read_pos_, n)) {
      this->reading_ = false;  // drop it; the next sub-page starts over
      return;
    }
    this->read_pos_ += n;
  } while (this->read_pos_ < MLX_RAM_WORDS && millis() - start < READ_BUDGET_MS);
  if (this->read_pos_ < MLX_RAM_WORDS)
    return;
  this->reading_ = false;

  uint16_t ctrl, status;
  if (!this->read_words_(REG_CONTROL1, &ctrl, 1) || !this->read_words_(REG_STATUS, &status, 1))
    return;
  if (status & STATUS_DATA_READY) {
    // The next sub-page finished while this one was being read, so it may be mixed.
    if (this->consecutive_drops_ < 255)
      this->consecutive_drops_++;
    if (this->consecutive_drops_ == 5) {
      ESP_LOGW(TAG, "I2C is too slow for %.1f Hz: lower the refresh rate or raise the I2C frequency",
               RATE_HZ[static_cast<int>(this->settings_[SETTING_REFRESH_RATE])]);
    }
    if (this->consecutive_drops_ < 3) {
      // Skip it; the next poll picks up the new one straight away.
      this->dropped_subpages_++;
      this->last_subpage_ms_ = millis() - this->subpage_period_ms_();
      return;
    }
    // Reads never finish in time: show the (possibly mixed) data rather than nothing.
  } else {
    this->consecutive_drops_ = 0;
  }
  this->process_subpage_(ctrl);
}

void MLX90640Component::process_subpage_(uint16_t ctrl) {
  this->frame_[832] = ctrl;
  this->frame_[833] = this->read_subpage_;
  if ((ctrl & CTRL_MASK) != this->wanted_control_bits_()) {
    // The sensor was reset or power-cycled and is back on its defaults.
    ESP_LOGW(TAG, "Sensor settings were reset (control register 0x%04X); re-applying", ctrl);
    if (!this->apply_control_register_())
      this->status_set_warning();
    return;
  }
  if (!mlx_frame_valid(this->frame_)) {
    ESP_LOGV(TAG, "Incomplete sub-page skipped");
    return;
  }

  const float ta = mlx_get_ta(this->frame_, &this->params_);
  if (!std::isfinite(ta) || ta < -50 || ta > 150)
    return;
  this->vdd_ = mlx_get_vdd(this->frame_, &this->params_);
  mlx_calculate_to(this->frame_, &this->params_, this->settings_[SETTING_EMISSIVITY],
                   ta - this->settings_[SETTING_REFLECTED_OFFSET], this->work_);

  const int sub_page = this->read_subpage_;
  this->subpages_seen_ |= 1 << sub_page;
  if (this->subpages_seen_ == 0x3)
    this->finish_frame_(ta, sub_page, (ctrl & CTRL_CHESS) != 0);
}

// Builds the output image after each sub-page. Only that sub-page's pixels are new, so
// only they are updated (otherwise smoothing would be applied twice per measurement).
void MLX90640Component::finish_frame_(float ta, int sub_page, bool chess) {
  this->ta_ = ta;
  if (this->flag_(SETTING_FREEZE))
    return;

  const bool interpolate = this->flag_(SETTING_INTERPOLATE_BAD_PIXELS);
  // First image after a (re)start or a processing change: take every pixel as is.
  const bool all = !this->has_frame_ || this->restart_image_;
  this->restart_image_ = false;
  const float alpha = all ? 0.0f : this->settings_[SETTING_SMOOTHING];
  float mn = INFINITY, mx = -INFINITY, sum = 0;
  int n = 0;
  for (int px = 0; px < MLX_PIXELS; px++) {
    if (all || mlx_pixel_subpage(px, chess) == sub_page) {
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
    }
    const float v = this->output_[px];
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
  if (!this->has_frame()) {
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

// ---------------------------------------------------------------- frame access

int MLX90640Component::display_source_index_(int x, int y) const {
  if (this->flag_(SETTING_ROTATE_180)) {
    x = MLX_COLS - 1 - x;
    y = MLX_ROWS - 1 - y;
  }
  if (this->flag_(SETTING_MIRROR))
    x = MLX_COLS - 1 - x;
  return y * MLX_COLS + x;
}

void MLX90640Component::copy_display_frame(float *dst) const {
  for (int y = 0; y < MLX_ROWS; y++)
    for (int x = 0; x < MLX_COLS; x++)
      dst[y * MLX_COLS + x] = this->output_[this->display_source_index_(x, y)];
}

// ---------------------------------------------------------------- web
#ifdef USE_MLX90640_WEB_UI

// The handlers below run on the HTTP server task. output_ and the stats may be
// updated concurrently, which at worst mixes two consecutive images in one response.

bool MLX90640Component::canHandle(AsyncWebServerRequest *request) const {
  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  StringRef url = request->url_to(url_buf);
  if (request->method() == HTTP_POST)
    return url == "/thermal/set";
  if (request->method() != HTTP_GET)
    return false;
  return url == "/thermal" || url == "/thermal/" || url == "/thermal.json" || url == "/thermal.bin" ||
         url == "/thermal.jpg";
}

void MLX90640Component::handleRequest(AsyncWebServerRequest *request) {
  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  StringRef url = request->url_to(url_buf);
  if (url == "/thermal/set") {
    this->handle_set_(request);
  } else if (url == "/thermal.bin") {
    this->handle_binary_(request);
  } else if (url == "/thermal.json") {
    this->handle_json_(request);
  } else if (url == "/thermal.jpg") {
    this->handle_snapshot_(request);
  } else {
    this->handle_page_(request);
  }
}

// GET /thermal - gzipped page with an ETag, so browsers only download it once per firmware.
void MLX90640Component::handle_page_(AsyncWebServerRequest *request) {
  auto if_none_match = request->get_header("If-None-Match");
  if (if_none_match.has_value() && *if_none_match == this->page_etag_) {
    auto *response = request->beginResponse(304, "text/html");
    response->addHeader("ETag", this->page_etag_);
    response->addHeader("Cache-Control", "no-cache");
    request->send(response);
    return;
  }
  auto *response = request->beginResponse(200, "text/html", this->page_data_, this->page_size_);
  response->addHeader("Content-Encoding", "gzip");
  response->addHeader("ETag", this->page_etag_);
  response->addHeader("Cache-Control", "no-cache");
  request->send(response);
}

// POST /thermal/set?key=<setting>&value=<number> - applied on the main loop.
void MLX90640Component::handle_set_(AsyncWebServerRequest *request) {
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
}

// GET /thermal.json - status, settings and pixels (display orientation, 0.1 degC).
void MLX90640Component::handle_json_(AsyncWebServerRequest *request) {
  auto finite = [](float v) { return std::isfinite(v) ? v : 0.0f; };
  AsyncResponseStream *stream = request->beginResponseStream("application/json");
  stream->printf("{\"w\":%d,\"h\":%d,\"ready\":%s,\"ta\":%.2f,\"vdd\":%.3f,\"min\":%.2f,\"max\":%.2f,\"mean\":%.2f,"
                 "\"fps\":%.2f,\"frame\":%u,\"i2c_errors\":%u,\"dropped\":%u,\"camera\":%s,\"cfg\":{",
                 MLX_COLS, MLX_ROWS, this->ready_ ? "true" : "false", finite(this->ta_), finite(this->vdd_),
                 finite(this->min_), finite(this->max_), finite(this->mean_), this->fps_,
                 static_cast<unsigned>(this->frame_count_), static_cast<unsigned>(this->i2c_errors_),
                 static_cast<unsigned>(this->dropped_subpages_), this->camera_ != nullptr ? "true" : "false");
  for (int i = 0; i < SETTING_COUNT; i++)
    stream->printf("%s\"%s\":%g", i ? "," : "", SETTINGS[i].key, this->get_setting(static_cast<Setting>(i)));
  stream->print("},\"t\":");

  if (!this->has_frame()) {
    stream->print("null}");
    request->send(stream);
    return;
  }
  stream->print("[");
  char buf[16];
  for (int y = 0; y < MLX_ROWS; y++) {
    for (int x = 0; x < MLX_COLS; x++) {
      const float v = this->output_[this->display_source_index_(x, y)];
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

// GET /thermal.bin - the same data as /thermal.json in ~1.6 kB instead of ~4.5 kB (used by the page).
// Little-endian layout:
//   0 u32 magic "MLX1"   4 u16 width   6 u16 height   8 u32 frame   12 u32 i2c_errors
//   16 f32 ta  20 f32 vdd  24 f32 min  28 f32 max  32 f32 mean  36 f32 fps
//   40 u8 flags (1 ready, 2 has image, 4 camera)   41 u8 setting count (n)   42 u16 header size
//   44 f32 settings[n] (same order as "cfg" in /thermal.json)
//   header size: i16 pixels[w*h] in 0.01 degC (display orientation), -32768 = no value
void MLX90640Component::handle_binary_(AsyncWebServerRequest *request) {
  const bool image = this->has_frame();
  const size_t header = BIN_HEADER_SIZE + 4 * SETTING_COUNT;
  std::vector<uint8_t> buf(header + (image ? 2 * MLX_PIXELS : 0));  // heap: the HTTP task stack is small
  uint8_t *p = buf.data();
  auto put_u16 = [p](size_t off, uint16_t v) { std::memcpy(p + off, &v, sizeof(v)); };
  auto put_u32 = [p](size_t off, uint32_t v) { std::memcpy(p + off, &v, sizeof(v)); };
  auto put_f32 = [p](size_t off, float v) { std::memcpy(p + off, &v, sizeof(v)); };

  put_u32(0, BIN_MAGIC);
  put_u16(4, MLX_COLS);
  put_u16(6, MLX_ROWS);
  put_u32(8, this->frame_count_);
  put_u32(12, this->i2c_errors_);
  put_f32(16, this->ta_);
  put_f32(20, this->vdd_);
  put_f32(24, this->min_);
  put_f32(28, this->max_);
  put_f32(32, this->mean_);
  put_f32(36, this->fps_);
  p[40] = (this->ready_ ? 1 : 0) | (image ? 2 : 0) | (this->camera_ != nullptr ? 4 : 0);
  p[41] = SETTING_COUNT;
  put_u16(42, static_cast<uint16_t>(header));
  for (int i = 0; i < SETTING_COUNT; i++)
    put_f32(BIN_HEADER_SIZE + 4 * i, this->get_setting(static_cast<Setting>(i)));
  if (image) {
    for (int y = 0; y < MLX_ROWS; y++) {
      for (int x = 0; x < MLX_COLS; x++) {
        const float v = this->output_[this->display_source_index_(x, y)];
        int16_t c = BIN_NO_VALUE;
        if (std::isfinite(v))
          c = static_cast<int16_t>(std::clamp<long>(std::lround(v * 100.0f), -32767, 32767));
        put_u16(header + 2 * (y * MLX_COLS + x), static_cast<uint16_t>(c));
      }
    }
  }
  auto *response = request->beginResponse(200, "application/octet-stream", p, buf.size());
  response->addHeader("Cache-Control", "no-store");
  request->send(response);  // sent synchronously, while `buf` is still alive
}

// GET /thermal.jpg - camera image (cached one if it is current, else a fresh one).
void MLX90640Component::handle_snapshot_(AsyncWebServerRequest *request) {
#ifdef USE_MLX90640_CAMERA
  if (this->camera_ != nullptr) {
    if (!this->has_frame()) {
      request->send(503, "text/plain", "Sensor not ready");
      return;
    }
    std::shared_ptr<camera::CameraImage> image = this->camera_->get_fresh_image(1000);
    if (image) {
      auto *response = request->beginResponse(200, "image/jpeg", image->get_data_buffer(), image->get_data_length());
      response->addHeader("Cache-Control", "no-store");
      request->send(response);  // sent synchronously, while `image` keeps the buffer alive
      return;
    }
    request->send(503, "text/plain", "No image yet");
    return;
  }
#endif
  request->send(404, "text/plain", "Camera not enabled (add 'camera:' under 'mlx90640:')");
}

#endif  // USE_MLX90640_WEB_UI

}  // namespace esphome::mlx90640
