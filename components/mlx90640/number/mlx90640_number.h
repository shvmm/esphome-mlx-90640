#pragma once

#include <cmath>

#include "esphome/components/number/number.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/core/preferences.h"

#include "../mlx90640.h"

namespace esphome::mlx90640 {

class MLX90640Number : public number::Number, public Component, public Parented<MLX90640Component>, public SettingEntity {
 public:
  void set_setting(Setting s) { this->setting_ = s; }

  // Runs before the hub's setup so restored values are in place when the sensor is configured.
  float get_setup_priority() const override { return setup_priority::DATA; }

  void setup() override {
    this->parent_->register_entity(this->setting_, this);
    this->pref_ = this->make_entity_preference<float>();
    float value;
    if (!this->pref_.load(&value) || !std::isfinite(value))
      value = this->parent_->get_setting(this->setting_);
    this->publish_state(this->parent_->set_setting(this->setting_, value));
  }

  static constexpr const char *TAG = "mlx90640.number";
  void dump_config() override { LOG_NUMBER("", "MLX90640 Number", this); }

  void apply_setting(float value) override { this->control(value); }

 protected:
  void control(float value) override {
    float applied = this->parent_->set_setting(this->setting_, value);
    this->publish_state(applied);
    this->pref_.save(&applied);
  }

  Setting setting_{SETTING_EMISSIVITY};
  ESPPreferenceObject pref_;
};

}  // namespace esphome::mlx90640
