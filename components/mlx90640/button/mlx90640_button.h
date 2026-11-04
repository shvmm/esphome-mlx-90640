#pragma once

#include "esphome/components/button/button.h"
#include "esphome/core/helpers.h"

#include "../mlx90640.h"

namespace esphome::mlx90640 {

// Re-reads the calibration EEPROM and re-applies the control register.
class MLX90640ReinitButton : public button::Button, public Parented<MLX90640Component> {
 protected:
  void press_action() override { this->parent_->request_reinit(); }
};

}  // namespace esphome::mlx90640
