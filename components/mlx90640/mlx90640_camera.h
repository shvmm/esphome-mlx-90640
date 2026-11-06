#pragma once
// Camera entity for the MLX90640: renders the thermal image (palette, smooth
// upscale, optional overlay) to JPEG and hands it to ESPHome's camera pipeline,
// which serves it to Home Assistant over the native API.

#include "esphome/core/defines.h"
#ifdef USE_MLX90640_CAMERA

#include <atomic>
#include <memory>
#include <mutex>
#include <vector>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "esphome/components/camera/camera.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

#include "mlx90640.h"

namespace esphome::mlx90640 {

/// One encoded JPEG, shared by every client that sends it.
class MLX90640CameraImage : public camera::CameraImage {
 public:
  MLX90640CameraImage(uint8_t *data, size_t length, uint8_t requesters)
      : data_(data), length_(length), requesters_(requesters) {}
  ~MLX90640CameraImage() override;

  uint8_t *get_data_buffer() override { return this->data_; }
  size_t get_data_length() override { return this->length_; }
  bool was_requested_by(camera::CameraRequester requester) const override {
    return (this->requesters_ & (1 << requester)) != 0;
  }

 protected:
  uint8_t *data_;
  size_t length_;
  uint8_t requesters_;
};

/// Tracks how much of an image one API connection has sent.
class MLX90640CameraImageReader : public camera::CameraImageReader {
 public:
  void set_image(std::shared_ptr<camera::CameraImage> image) override {
    this->image_ = std::move(image);
    this->offset_ = 0;
  }
  size_t available() const override { return this->image_ ? this->image_->get_data_length() - this->offset_ : 0; }
  uint8_t *peek_data_buffer() override { return this->image_->get_data_buffer() + this->offset_; }
  void consume_data(size_t consumed) override { this->offset_ += consumed; }
  void return_image() override {
    this->image_.reset();
    this->offset_ = 0;
  }

 protected:
  std::shared_ptr<camera::CameraImage> image_;
  size_t offset_{0};
};

class MLX90640Camera : public camera::Camera, public Parented<MLX90640Component> {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_resolution(uint16_t width, uint16_t height) {
    this->width_ = width;
    this->height_ = height;
  }
  void set_max_update_interval(uint32_t ms) { this->max_update_interval_ = ms; }
  void set_idle_update_interval(uint32_t ms) { this->idle_update_interval_ = ms; }

  // -------- camera::Camera --------
  void add_listener(camera::CameraListener *listener) override { this->listeners_.push_back(listener); }
  camera::CameraImageReader *create_image_reader() override { return new MLX90640CameraImageReader(); }
  void request_image(camera::CameraRequester requester) override {
    this->single_requesters_.fetch_or(static_cast<uint8_t>(1U << requester));
  }
  void start_stream(camera::CameraRequester requester) override;
  void stop_stream(camera::CameraRequester requester) override;

  /// For the web snapshot endpoint (any task): returns the newest image straight away
  /// if it shows the current frame and settings (or is very recent); otherwise requests
  /// a new one and waits up to timeout_ms for it. May return null.
  std::shared_ptr<camera::CameraImage> get_fresh_image(uint32_t timeout_ms);

 protected:
  /// Everything the encoder task needs, captured on the main loop.
  struct RenderJob {
    float temps[MLX_PIXELS];
    uint8_t palette;
    uint8_t quality;
    bool overlay;
    bool auto_range;
    float range_min;
    float range_max;
    uint8_t requesters;
    uint32_t frame;       // parent frame count when captured
    uint32_t generation;  // parent settings generation when captured
  };
  enum State : uint8_t { STATE_IDLE = 0, STATE_BUSY, STATE_DONE };

  static void task_entry_(void *arg);
  bool render_encode_();

  uint16_t width_{320};
  uint16_t height_{240};
  uint32_t max_update_interval_{250};
  uint32_t idle_update_interval_{10000};

  std::vector<camera::CameraListener *> listeners_;
  std::atomic<uint8_t> single_requesters_{0};
  std::atomic<uint8_t> stream_requesters_{0};

  TaskHandle_t task_{nullptr};
  std::atomic<uint8_t> state_{STATE_IDLE};
  RenderJob job_{};

  // Encoder output (owned by the task while BUSY, by the loop when DONE)
  uint8_t *result_data_{nullptr};
  size_t result_length_{0};
  bool result_ok_{false};

  // Pre-computed horizontal interpolation for the configured width
  std::vector<uint8_t> x0_, x1_;
  std::vector<float> wx_;
  uint8_t *line_{nullptr};

  // Newest image and what it shows, for /thermal.jpg (guarded by latest_mutex_)
  std::mutex latest_mutex_;
  std::shared_ptr<camera::CameraImage> latest_;
  uint32_t latest_frame_{0};
  uint32_t latest_generation_{0};
  uint32_t latest_ms_{0};
  std::atomic<uint32_t> image_seq_{0};

  uint32_t last_render_ms_{0};
  uint32_t last_idle_ms_{0};
  uint32_t last_frame_rendered_{0};
  uint32_t last_generation_rendered_{0};
  uint32_t last_encode_ms_{0};
  size_t last_size_{0};
};

}  // namespace esphome::mlx90640

#endif  // USE_MLX90640_CAMERA
