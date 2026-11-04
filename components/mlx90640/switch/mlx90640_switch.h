#pragma once

#include "esphome/components/switch/switch.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include "../mlx90640.h"

namespace esphome::mlx90640 {

class MLX90640Switch : public switch_::Switch, public Component, public Parented<MLX90640Component>, public SettingEntity {
 public:
  void set_setting(Setting s) { this->setting_ = s; }

  // Runs before the hub's setup so restored values are in place when the sensor is configured.
  float get_setup_priority() const override { return setup_priority::DATA; }

  void setup() override {
    this->parent_->register_entity(this->setting_, this);
    // Honours restore_mode (RESTORE_* / ALWAYS_*); falls back to the component default.
    optional<bool> initial = this->get_initial_state_with_restore_mode();
    bool state = initial.has_value() ? *initial : this->parent_->get_setting(this->setting_) > 0.5f;
    this->publish_state(this->parent_->set_setting(this->setting_, state ? 1.0f : 0.0f) > 0.5f);
  }

  static constexpr const char *TAG = "mlx90640.switch";
  void dump_config() override { LOG_SWITCH("", "MLX90640 Switch", this); }

  void apply_setting(float value) override { this->write_state(value > 0.5f); }

 protected:
  void write_state(bool state) override {
    this->publish_state(this->parent_->set_setting(this->setting_, state ? 1.0f : 0.0f) > 0.5f);
  }

  Setting setting_{SETTING_MIRROR};
};

}  // namespace esphome::mlx90640
