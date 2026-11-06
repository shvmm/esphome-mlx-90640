#pragma once

#include <atomic>

#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/helpers.h"

#ifdef USE_MLX90640_WEB_UI
#include "esphome/components/web_server_base/web_server_base.h"
#endif

#include "mlx90640_calc.h"

namespace esphome::mlx90640 {

// Every runtime-adjustable setting. The number/select/switch platforms bind an
// entity to one of these; the web page reads/writes them by name.
enum Setting : uint8_t {
  // selects (value = option index)
  SETTING_REFRESH_RATE = 0,  // 0..7 -> 0.5..64 Hz sub-page rate
  SETTING_RESOLUTION,        // 0..3 -> 16..19 bit ADC
  SETTING_PATTERN,           // 0 = chess, 1 = interleaved
  SETTING_PALETTE,           // 0 = iron, 1 = rainbow, 2 = grey
  // numbers
  SETTING_EMISSIVITY,
  SETTING_REFLECTED_OFFSET,  // reflected temperature = Ta - offset
  SETTING_SMOOTHING,         // 0 = off .. 0.95 = heavy temporal smoothing
  SETTING_RANGE_MIN,         // manual colour range
  SETTING_RANGE_MAX,
  SETTING_PUBLISH_INTERVAL,  // seconds between summary sensor publishes
  // switches (value 0/1)
  SETTING_INTERPOLATE_BAD_PIXELS,
  SETTING_AUTO_RANGE,
  SETTING_MIRROR,
  SETTING_ROTATE_180,
  SETTING_FREEZE,
  SETTING_CAMERA_OVERLAY,  // min/max/centre text, hotspot and colour bar on the camera image
  // numbers (camera)
  SETTING_JPEG_QUALITY,  // 10..100
  SETTING_COUNT,
};

class MLX90640Camera;

// Implemented by the entity classes so web-page changes go through the entity
// (state gets published to Home Assistant / web UI and persisted).
class SettingEntity {
 public:
  virtual void apply_setting(float value) = 0;
};

class MLX90640Component : public PollingComponent,
                          public i2c::I2CDevice
#ifdef USE_MLX90640_WEB_UI
    ,
                          public AsyncWebHandler
#endif
{
 public:
  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::WIFI - 1.0f; }

#ifdef USE_MLX90640_WEB_UI
  // GET /thermal (page), /thermal.json, /thermal.bin, /thermal.jpg; POST /thermal/set
  bool canHandle(AsyncWebServerRequest *request) const override;
  void handleRequest(AsyncWebServerRequest *request) override;
  /// Gzipped heatmap page, generated from heatmap.html at compile time.
  void set_page(const uint8_t *data, size_t size, const char *etag) {
    this->page_data_ = data;
    this->page_size_ = size;
    this->page_etag_ = etag;
  }
#endif

  // Settings API (main loop only). set_setting clamps, stores, applies, and returns the stored value.
  float get_setting(Setting s) const;
  float set_setting(Setting s, float value);
  /// Increments whenever any setting changes (lets the camera tell whether a cached image is current).
  uint32_t get_settings_generation() const { return this->settings_gen_; }
  void register_entity(Setting s, SettingEntity *e) { this->entities_[s] = e; }
  void request_reinit() { this->reinit_requested_ = true; }

  // Frame access for the camera. Copies the last image in display orientation
  // (mirror / rotate applied).
  bool has_frame() const { return this->has_frame_ && this->ready_; }
  /// Increments with every new image (once per sub-page).
  uint32_t get_frame_count() const { return this->frame_count_; }
  void copy_display_frame(float *dst) const;
  void set_camera(MLX90640Camera *camera) { this->camera_ = camera; }

  // YAML defaults
  void set_jpeg_quality(uint8_t q) { this->settings_[SETTING_JPEG_QUALITY] = q; }
  void set_camera_overlay(bool on) { this->settings_[SETTING_CAMERA_OVERLAY] = on ? 1.0f : 0.0f; }
  void set_refresh_rate(uint8_t code) { this->settings_[SETTING_REFRESH_RATE] = code; }
  void set_resolution(uint8_t code) { this->settings_[SETTING_RESOLUTION] = code; }
  void set_emissivity(float e) { this->settings_[SETTING_EMISSIVITY] = e; }

  void set_min_sensor(sensor::Sensor *s) { this->min_sensor_ = s; }
  void set_max_sensor(sensor::Sensor *s) { this->max_sensor_ = s; }
  void set_mean_sensor(sensor::Sensor *s) { this->mean_sensor_ = s; }
  void set_center_sensor(sensor::Sensor *s) { this->center_sensor_ = s; }
  void set_ambient_sensor(sensor::Sensor *s) { this->ambient_sensor_ = s; }
  void set_frame_rate_sensor(sensor::Sensor *s) { this->frame_rate_sensor_ = s; }
  void set_i2c_errors_sensor(sensor::Sensor *s) { this->i2c_errors_sensor_ = s; }
  void set_bad_pixels_sensor(sensor::Sensor *s) { this->bad_pixels_sensor_ = s; }
  void set_supply_voltage_sensor(sensor::Sensor *s) { this->supply_voltage_sensor_ = s; }

 protected:
  bool read_words_(uint16_t addr, uint16_t *dst, size_t count);
  bool write_word_(uint16_t addr, uint16_t value);
  bool init_sensor_();
  void sensor_lost_(const char *reason);
  uint16_t wanted_control_bits_() const;
  bool apply_control_register_();
  uint32_t subpage_period_ms_() const;
  void poll_status_(uint32_t now);
  void continue_read_();
  void process_subpage_(uint16_t ctrl);
  void apply_pending_web_changes_();
  void finish_frame_(float ta, int sub_page, bool chess);
  bool flag_(Setting s) const { return this->settings_[s] > 0.5f; }
  // Sensor-array index of display pixel (x, y), honouring mirror / rotate.
  int display_source_index_(int x, int y) const;

#ifdef USE_MLX90640_WEB_UI
  void handle_page_(AsyncWebServerRequest *request);
  void handle_json_(AsyncWebServerRequest *request);
  void handle_binary_(AsyncWebServerRequest *request);
  void handle_set_(AsyncWebServerRequest *request);
  void handle_snapshot_(AsyncWebServerRequest *request);

  const uint8_t *page_data_{nullptr};
  size_t page_size_{0};
  const char *page_etag_{""};
#endif

  MLX90640Camera *camera_{nullptr};

  float settings_[SETTING_COUNT]{};
  uint32_t settings_gen_{0};
  SettingEntity *entities_[SETTING_COUNT]{};
  // Changes posted from the HTTP task, applied on the main loop.
  float pending_values_[SETTING_COUNT]{};
  std::atomic<uint32_t> pending_mask_{0};

  sensor::Sensor *min_sensor_{nullptr};
  sensor::Sensor *max_sensor_{nullptr};
  sensor::Sensor *mean_sensor_{nullptr};
  sensor::Sensor *center_sensor_{nullptr};
  sensor::Sensor *ambient_sensor_{nullptr};
  sensor::Sensor *frame_rate_sensor_{nullptr};
  sensor::Sensor *i2c_errors_sensor_{nullptr};
  sensor::Sensor *bad_pixels_sensor_{nullptr};
  sensor::Sensor *supply_voltage_sensor_{nullptr};

  MLX90640Params params_{};
  uint16_t frame_[MLX_FRAME_WORDS]{};
  float work_[MLX_PIXELS]{};    // latest calculated value of every pixel (both sub-pages)
  float output_[MLX_PIXELS]{};  // processed image (sensor orientation), served over HTTP

  // Runs loop() continuously while a sub-page is being read and at high refresh rates.
  HighFrequencyLoopRequester high_freq_;

  bool setup_done_{false};
  bool ready_{false};
  bool reinit_requested_{false};
  bool has_frame_{false};
  bool restart_image_{false};    // next image replaces every pixel (no smoothing with the old one)
  bool reading_{false};          // a sub-page is being read in chunks
  uint8_t read_subpage_{0};
  uint8_t subpages_seen_{0};
  uint8_t consecutive_errors_{0};
  uint8_t consecutive_drops_{0};
  uint16_t read_pos_{0};         // words of RAM read so far
  uint16_t bad_pixel_count_{0};
  uint32_t last_init_attempt_ms_{0};
  uint32_t last_poll_ms_{0};
  uint32_t last_subpage_ms_{0};
  uint32_t last_frame_ms_{0};
  uint32_t frame_count_{0};
  uint32_t i2c_errors_{0};
  uint32_t dropped_subpages_{0};
  float fps_{0};
  float ta_{NAN};
  float vdd_{NAN};
  float min_{NAN}, max_{NAN}, mean_{NAN}, center_{NAN};
};

}  // namespace esphome::mlx90640
