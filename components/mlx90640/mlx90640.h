#pragma once

#include <atomic>

#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/web_server_base/web_server_base.h"
#include "esphome/core/component.h"

#include "mlx90640_calc.h"

namespace esphome::mlx90640 {

// Every runtime-adjustable setting. The number/select/switch platforms bind an
// entity to one of these; the web page reads/writes them by name.
enum Setting : uint8_t {
  // selects (value = option index)
  SETTING_REFRESH_RATE = 0,  // 0..7 -> 0.5..64 Hz sub-page rate
  SETTING_RESOLUTION,        // 0..3 -> 16..19 bit ADC
  SETTING_PATTERN,           // 0 = chess, 1 = interleaved
  SETTING_PALETTE,           // 0 = iron, 1 = rainbow, 2 = grey (heatmap page)
  // numbers
  SETTING_EMISSIVITY,
  SETTING_REFLECTED_OFFSET,  // reflected temperature = Ta - offset
  SETTING_SMOOTHING,         // 0 = off .. 0.95 = heavy temporal smoothing
  SETTING_RANGE_MIN,         // manual colour range (heatmap page)
  SETTING_RANGE_MAX,
  SETTING_PUBLISH_INTERVAL,  // seconds between summary sensor publishes
  // switches (value 0/1)
  SETTING_INTERPOLATE_BAD_PIXELS,
  SETTING_AUTO_RANGE,
  SETTING_MIRROR,
  SETTING_ROTATE_180,
  SETTING_FREEZE,
  SETTING_COUNT,
};

// Implemented by the entity classes so web-page changes go through the entity
// (state gets published to Home Assistant / web UI and persisted).
class SettingEntity {
 public:
  virtual void apply_setting(float value) = 0;
};

class MLX90640Component : public PollingComponent, public i2c::I2CDevice, public AsyncWebHandler {
 public:
  explicit MLX90640Component(web_server_base::WebServerBase *base);

  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::WIFI - 1.0f; }

  // Web handler: GET /thermal (page), GET /thermal.json (frame + settings), POST /thermal/set
  bool canHandle(AsyncWebServerRequest *request) const override;
  void handleRequest(AsyncWebServerRequest *request) override;

  // Settings API (main loop only). set_setting clamps, stores, applies, and returns the stored value.
  float get_setting(Setting s) const;
  float set_setting(Setting s, float value);
  void register_entity(Setting s, SettingEntity *e) { this->entities_[s] = e; }
  void request_reinit() { this->reinit_requested_ = true; }

  // YAML defaults
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
  bool apply_control_register_();
  void apply_pending_web_changes_();
  void finish_frame_(float ta);
  bool flag_(Setting s) const { return this->settings_[s] > 0.5f; }

  web_server_base::WebServerBase *base_;

  float settings_[SETTING_COUNT]{};
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
  uint16_t eeprom_[MLX_EEPROM_WORDS]{};
  uint16_t frame_[MLX_FRAME_WORDS]{};
  float work_[MLX_PIXELS]{};    // per-subpage results accumulate here
  float output_[MLX_PIXELS]{};  // last complete image (sensor orientation), served over HTTP

  bool setup_done_{false};
  bool ready_{false};
  bool reinit_requested_{false};
  uint8_t subpages_seen_{0};
  bool has_frame_{false};
  uint16_t bad_pixel_count_{0};
  uint32_t last_init_attempt_ms_{0};
  uint32_t last_poll_ms_{0};
  uint32_t last_frame_ms_{0};
  uint32_t frame_count_{0};
  uint32_t i2c_errors_{0};
  float fps_{0};
  float ta_{NAN};
  float vdd_{NAN};
  float min_{NAN}, max_{NAN}, mean_{NAN}, center_{NAN};
};

}  // namespace esphome::mlx90640
