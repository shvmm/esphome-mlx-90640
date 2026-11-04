#pragma once

#include "esphome/components/select/select.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/core/preferences.h"

#include "../mlx90640.h"

namespace esphome::mlx90640 {

class MLX90640Select : public select::Select, public Component, public Parented<MLX90640Component>, public SettingEntity {
 public:
  void set_setting(Setting s) { this->setting_ = s; }

  // Runs before the hub's setup so restored values are in place when the sensor is configured.
  float get_setup_priority() const override { return setup_priority::DATA; }

  void setup() override {
    this->parent_->register_entity(this->setting_, this);
    this->pref_ = this->make_entity_preference<uint32_t>();
    uint32_t index;
    if (!this->pref_.load(&index) || !this->has_index(index))
      index = static_cast<uint32_t>(this->parent_->get_setting(this->setting_));
    index = static_cast<uint32_t>(this->parent_->set_setting(this->setting_, static_cast<float>(index)));
    this->publish_state(static_cast<size_t>(index));
  }

  static constexpr const char *TAG = "mlx90640.select";
  void dump_config() override { LOG_SELECT("", "MLX90640 Select", this); }

  void apply_setting(float value) override {
    if (value >= 0 && this->has_index(static_cast<size_t>(value)))
      this->control(static_cast<size_t>(value));
  }

 protected:
  void control(size_t index) override {
    uint32_t applied = static_cast<uint32_t>(this->parent_->set_setting(this->setting_, static_cast<float>(index)));
    this->publish_state(static_cast<size_t>(applied));
    this->pref_.save(&applied);
  }

  Setting setting_{SETTING_REFRESH_RATE};
  ESPPreferenceObject pref_;
};

}  // namespace esphome::mlx90640
