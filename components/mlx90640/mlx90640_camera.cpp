#include "mlx90640_camera.h"

#ifdef USE_MLX90640_CAMERA

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include "jpge.h"

namespace esphome::mlx90640 {

static const char *const TAG = "mlx90640.camera";

// Same colour stops as the /thermal page (iron, rainbow, grey).
static const uint8_t PAL_IRON[][3] = {{0, 0, 0},       {30, 0, 90},    {120, 0, 150},  {200, 30, 100},
                                      {240, 90, 0},    {255, 170, 0},  {255, 240, 80}, {255, 255, 255}};
static const uint8_t PAL_RAINBOW[][3] = {{0, 0, 130},   {0, 60, 255},  {0, 200, 255}, {0, 255, 120},
                                         {200, 255, 0}, {255, 160, 0}, {255, 0, 0}};
static const uint8_t PAL_GREY[][3] = {{0, 0, 0}, {255, 255, 255}};

// 5x7 font for the overlay; each row is 5 bits, MSB = leftmost column.
// '^' is drawn as a degree sign.
struct Glyph {
  char c;
  uint8_t rows[7];
};
static const Glyph FONT[] = {
    {'0', {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E}}, {'1', {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E}},
    {'2', {0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F}}, {'3', {0x1F, 0x02, 0x04, 0x02, 0x01, 0x11, 0x0E}},
    {'4', {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02}}, {'5', {0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E}},
    {'6', {0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E}}, {'7', {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08}},
    {'8', {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E}}, {'9', {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C}},
    {'.', {0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C}}, {'-', {0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00}},
    {'^', {0x0C, 0x12, 0x12, 0x0C, 0x00, 0x00, 0x00}}, {'A', {0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11}},
    {'C', {0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E}}, {'I', {0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E}},
    {'M', {0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11}}, {'N', {0x11, 0x11, 0x19, 0x15, 0x13, 0x11, 0x11}},
    {'R', {0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11}}, {'T', {0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04}},
    {'X', {0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11}},
};

static const uint8_t *glyph_for(char c) {
  for (const auto &g : FONT)
    if (g.c == c)
      return g.rows;
  return nullptr;  // space / unknown -> blank
}

/// Growable JPEG output buffer (prefers PSRAM when available).
class JpegOutput : public mlx_jpge::output_stream {
 public:
  ~JpegOutput() override {
    if (this->data_ != nullptr)
      RAMAllocator<uint8_t>().deallocate(this->data_, this->capacity_);
  }
  bool put_buf(const void *buf, int len) override {
    if (len <= 0)
      return true;
    const size_t need = this->length_ + static_cast<size_t>(len);
    if (need > this->capacity_) {
      size_t cap = std::max<size_t>({need, this->capacity_ * 2, 16 * 1024});
      RAMAllocator<uint8_t> alloc;
      uint8_t *p = this->data_ == nullptr ? alloc.allocate(cap) : alloc.reallocate(this->data_, cap);
      if (p == nullptr)
        return false;
      this->data_ = p;
      this->capacity_ = cap;
    }
    std::memcpy(this->data_ + this->length_, buf, len);
    this->length_ = need;
    return true;
  }
  mlx_jpge::uint get_size() const override { return this->length_; }

  /// Hand the buffer over to the caller (who must free it with RAMAllocator).
  uint8_t *release(size_t *length) {
    uint8_t *p = this->data_;
    *length = this->length_;
    this->data_ = nullptr;
    this->length_ = this->capacity_ = 0;
    return p;
  }

 protected:
  uint8_t *data_{nullptr};
  size_t length_{0};
  size_t capacity_{0};
};

MLX90640CameraImage::~MLX90640CameraImage() {
  if (this->data_ != nullptr)
    RAMAllocator<uint8_t>().deallocate(this->data_, this->length_);
}

// ---------------------------------------------------------------- lifecycle

void MLX90640Camera::setup() {
  // Horizontal sample positions/weights are the same for every row.
  this->x0_.resize(this->width_);
  this->x1_.resize(this->width_);
  this->wx_.resize(this->width_);
  for (int x = 0; x < this->width_; x++) {
    float fx = (x + 0.5f) * MLX_COLS / this->width_ - 0.5f;
    fx = std::clamp(fx, 0.0f, static_cast<float>(MLX_COLS - 1));
    const int x0 = static_cast<int>(fx);
    this->x0_[x] = static_cast<uint8_t>(x0);
    this->x1_[x] = static_cast<uint8_t>(std::min(x0 + 1, MLX_COLS - 1));
    this->wx_[x] = fx - x0;
  }
  this->line_ = RAMAllocator<uint8_t>().allocate(this->width_ * 3);
  if (this->line_ == nullptr) {
    ESP_LOGE(TAG, "Out of memory for scanline buffer");
    this->mark_failed();
    return;
  }
  // Low priority so encoding never starves Wi-Fi / the main loop.
  if (xTaskCreate(&MLX90640Camera::task_entry_, "mlx_cam", 10240, this, 1, &this->task_) != pdPASS) {
    ESP_LOGE(TAG, "Failed to start encoder task");
    this->mark_failed();
    return;
  }
}

void MLX90640Camera::dump_config() {
  ESP_LOGCONFIG(TAG,
                "MLX90640 Camera:\n"
                "  Resolution: %ux%u\n"
                "  Max framerate: %.1f fps\n"
                "  Idle framerate: %.2f fps\n"
                "  Snapshot URL: http://<device>/thermal.jpg",
                this->width_, this->height_, 1000.0f / this->max_update_interval_,
                this->idle_update_interval_ ? 1000.0f / this->idle_update_interval_ : 0.0f);
}

void MLX90640Camera::start_stream(camera::CameraRequester requester) {
  for (auto *listener : this->listeners_)
    listener->on_stream_start();
  this->stream_requesters_.fetch_or(static_cast<uint8_t>(1U << requester));
}

void MLX90640Camera::stop_stream(camera::CameraRequester requester) {
  for (auto *listener : this->listeners_)
    listener->on_stream_stop();
  this->stream_requesters_.fetch_and(static_cast<uint8_t>(~(1U << requester)));
}

std::shared_ptr<camera::CameraImage> MLX90640Camera::get_fresh_image(uint32_t timeout_ms) {
  const uint32_t seq = this->image_seq_.load();
  this->request_image(camera::WEB_REQUESTER);
  const uint32_t start = millis();
  while (this->image_seq_.load() == seq && millis() - start < timeout_ms)
    vTaskDelay(pdMS_TO_TICKS(20));
  std::lock_guard<std::mutex> lock(this->latest_mutex_);
  return this->latest_;
}

void MLX90640Camera::loop() {
  // 1. Publish a finished frame (listeners must be called from the main loop).
  if (this->state_.load() == STATE_DONE) {
    if (this->result_ok_) {
      auto image = std::make_shared<MLX90640CameraImage>(this->result_data_, this->result_length_, this->job_.requesters);
      this->result_data_ = nullptr;
      {
        std::lock_guard<std::mutex> lock(this->latest_mutex_);
        this->latest_ = image;
      }
      this->image_seq_.fetch_add(1);
      for (auto *listener : this->listeners_)
        listener->on_camera_image(image);
    } else if (this->result_data_ != nullptr) {
      RAMAllocator<uint8_t>().deallocate(this->result_data_, this->result_length_);
      this->result_data_ = nullptr;
    }
    this->state_.store(STATE_IDLE);
  }

  // 2. Start a new frame if someone wants one.
  if (this->state_.load() != STATE_IDLE || this->task_ == nullptr || !this->parent_->has_frame())
    return;

  const uint32_t now = millis();
  const uint8_t streaming = this->stream_requesters_.load();
  const bool idle_due = this->idle_update_interval_ != 0 && now - this->last_idle_ms_ >= this->idle_update_interval_;
  bool wanted = this->single_requesters_.load() != 0 || idle_due;
  if (!wanted && streaming != 0) {
    // While streaming, only encode new sensor frames, at most max_framerate.
    wanted = now - this->last_render_ms_ >= this->max_update_interval_ &&
             this->parent_->get_frame_count() != this->last_frame_rendered_;
  }
  if (!wanted)
    return;

  uint8_t requesters = this->single_requesters_.exchange(0) | streaming;
  if (idle_due) {
    requesters |= 1U << camera::IDLE;
    this->last_idle_ms_ = now;
  }

  RenderJob &job = this->job_;
  this->parent_->copy_display_frame(job.temps);
  job.palette = static_cast<uint8_t>(this->parent_->get_setting(SETTING_PALETTE));
  job.quality = static_cast<uint8_t>(this->parent_->get_setting(SETTING_JPEG_QUALITY));
  job.overlay = this->parent_->get_setting(SETTING_CAMERA_OVERLAY) > 0.5f;
  job.auto_range = this->parent_->get_setting(SETTING_AUTO_RANGE) > 0.5f;
  job.range_min = this->parent_->get_setting(SETTING_RANGE_MIN);
  job.range_max = this->parent_->get_setting(SETTING_RANGE_MAX);
  job.requesters = requesters;

  this->last_render_ms_ = now;
  this->last_frame_rendered_ = this->parent_->get_frame_count();
  this->state_.store(STATE_BUSY);
  xTaskNotifyGive(this->task_);
}

void MLX90640Camera::task_entry_(void *arg) {
  auto *self = static_cast<MLX90640Camera *>(arg);
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    const uint32_t start = millis();
    self->result_ok_ = self->render_encode_();
    self->last_encode_ms_ = millis() - start;
    ESP_LOGV(TAG, "Encoded %ux%u in %u ms, %u bytes", self->width_, self->height_,
             static_cast<unsigned>(self->last_encode_ms_), static_cast<unsigned>(self->result_length_));
    self->state_.store(STATE_DONE);
    App.wake_loop_threadsafe();  // publish without waiting for the next loop tick
  }
}

// ---------------------------------------------------------------- rendering

namespace {

struct Label {
  int x, y;
  char text[16];
  int len;
};

inline void put_px(uint8_t *line, int width, int x, uint8_t r, uint8_t g, uint8_t b) {
  if (x < 0 || x >= width)
    return;
  uint8_t *p = line + x * 3;
  p[0] = r;
  p[1] = g;
  p[2] = b;
}

inline void darken_px(uint8_t *line, int width, int x) {
  if (x < 0 || x >= width)
    return;
  uint8_t *p = line + x * 3;
  p[0] = static_cast<uint8_t>(p[0] * 0.35f);
  p[1] = static_cast<uint8_t>(p[1] * 0.35f);
  p[2] = static_cast<uint8_t>(p[2] * 0.35f);
}

}  // namespace

bool MLX90640Camera::render_encode_() {
  const int W = this->width_, H = this->height_;
  float *t = this->job_.temps;
  this->result_length_ = 0;

  // Stats; replace missing pixels with the frame mean so interpolation stays sane.
  float mn = INFINITY, mx = -INFINITY, sum = 0;
  int n = 0, hot = 0;
  for (int i = 0; i < MLX_PIXELS; i++) {
    if (!std::isfinite(t[i]))
      continue;
    sum += t[i];
    n++;
    mn = std::min(mn, t[i]);
    if (t[i] > mx) {
      mx = t[i];
      hot = i;
    }
  }
  if (n == 0)
    return false;
  const float mean = sum / n;
  for (int i = 0; i < MLX_PIXELS; i++)
    if (!std::isfinite(t[i]))
      t[i] = mean;
  const int c0 = 11 * MLX_COLS + 15;
  const float centre = (t[c0] + t[c0 + 1] + t[c0 + MLX_COLS] + t[c0 + MLX_COLS + 1]) / 4.0f;

  float lo, hi;
  if (this->job_.auto_range) {
    lo = mn;
    hi = std::max(mx, mn + 2.0f);
  } else {
    lo = this->job_.range_min;
    hi = this->job_.range_max;
    if (!(hi > lo))
      hi = lo + 1.0f;
  }
  const float to_index = 255.0f / (hi - lo);

  // Palette lookup table
  const uint8_t(*stops)[3] = PAL_IRON;
  int nstops = sizeof(PAL_IRON) / 3;
  if (this->job_.palette == 1) {
    stops = PAL_RAINBOW;
    nstops = sizeof(PAL_RAINBOW) / 3;
  } else if (this->job_.palette == 2) {
    stops = PAL_GREY;
    nstops = sizeof(PAL_GREY) / 3;
  }
  uint8_t lut[256][3];
  for (int i = 0; i < 256; i++) {
    const float f = i / 255.0f * (nstops - 1);
    const int k = std::min(static_cast<int>(f), nstops - 2);
    const float w = f - k;
    for (int c = 0; c < 3; c++)
      lut[i][c] = static_cast<uint8_t>(std::lround(stops[k][c] + (stops[k + 1][c] - stops[k][c]) * w));
  }

  // Overlay layout (scaled with the image height)
  const bool overlay = this->job_.overlay;
  const int s = std::max(1, H / 120);
  const int char_w = 6 * s, char_h = 7 * s, pad = 2 * s, margin = 3 * s;
  Label labels[5];
  int nlabels = 0;
  auto add_label = [&](int x, int y, bool right_align, const char *fmt, float v) {
    Label &l = labels[nlabels++];
    l.len = snprintf(l.text, sizeof(l.text), fmt, v);
    l.len = std::min<int>(l.len, sizeof(l.text) - 1);
    const int tw = l.len * char_w - s;
    l.x = right_align ? x - tw : x;
    l.y = y;
  };
  const int top_y = margin + pad;
  const int bot_y = H - margin - pad - char_h;
  const int bar_w = 4 * s;
  const int bar_x1 = W - margin;  // exclusive
  const int bar_x0 = bar_x1 - bar_w;
  const int bar_y0 = top_y + char_h + pad + 2 * s;
  const int bar_y1 = bot_y - pad - 2 * s;  // exclusive
  int hot_x = 0, hot_y = 0, box_x0 = 0, box_x1 = 0, box_y0 = 0, box_y1 = 0;
  if (overlay) {
    add_label(margin + pad, top_y, false, "MAX %.1f^", mx);
    add_label(margin + pad, bot_y, false, "MIN %.1f^", mn);
    add_label(W / 2 - (9 * char_w) / 2, top_y, false, "CTR %.1f^", centre);
    add_label(W - margin - pad, top_y, true, "%.1f", hi);
    add_label(W - margin - pad, bot_y, true, "%.1f", lo);
    hot_x = static_cast<int>((hot % MLX_COLS + 0.5f) * W / MLX_COLS);
    hot_y = static_cast<int>((hot / MLX_COLS + 0.5f) * H / MLX_ROWS);
    box_x0 = 15 * W / MLX_COLS;
    box_x1 = 17 * W / MLX_COLS;
    box_y0 = 11 * H / MLX_ROWS;
    box_y1 = 13 * H / MLX_ROWS;
  }
  const int arm = 7 * s, thick = std::max(1, s / 2 + (s > 1 ? 1 : 0));

  JpegOutput out;
  mlx_jpge::params params;
  params.m_quality = std::clamp<int>(this->job_.quality, 10, 100);
  params.m_subsampling = mlx_jpge::H2V2;
  mlx_jpge::jpeg_encoder encoder;
  if (!encoder.init(&out, W, H, 3, params)) {
    ESP_LOGW(TAG, "JPEG encoder init failed (out of memory?)");
    return false;
  }

  uint8_t *line = this->line_;
  for (int y = 0; y < H; y++) {
    // ---- bilinear interpolation of the 32x24 frame ----
    float fy = (y + 0.5f) * MLX_ROWS / H - 0.5f;
    fy = std::clamp(fy, 0.0f, static_cast<float>(MLX_ROWS - 1));
    const int y0 = static_cast<int>(fy);
    const int y1 = std::min(y0 + 1, MLX_ROWS - 1);
    const float wy = fy - y0;
    const float *r0 = t + y0 * MLX_COLS;
    const float *r1 = t + y1 * MLX_COLS;
    for (int x = 0; x < W; x++) {
      const int a = this->x0_[x], b = this->x1_[x];
      const float wx = this->wx_[x];
      const float top = r0[a] + (r0[b] - r0[a]) * wx;
      const float bot = r1[a] + (r1[b] - r1[a]) * wx;
      const float v = top + (bot - top) * wy;
      const int idx = std::clamp(static_cast<int>((v - lo) * to_index), 0, 255);
      uint8_t *p = line + x * 3;
      p[0] = lut[idx][0];
      p[1] = lut[idx][1];
      p[2] = lut[idx][2];
    }

    if (overlay) {
      // Colour bar on the right edge (hot at the top)
      if (y >= bar_y0 && y < bar_y1 && bar_y1 > bar_y0) {
        const int idx = 255 * (bar_y1 - 1 - y) / std::max(1, bar_y1 - 1 - bar_y0);
        for (int x = bar_x0 - 1; x <= bar_x1; x++) {
          if (x == bar_x0 - 1 || x == bar_x1)
            put_px(line, W, x, 0, 0, 0);  // thin dark border
          else
            put_px(line, W, x, lut[idx][0], lut[idx][1], lut[idx][2]);
        }
      }
      // Label backgrounds + text
      for (int i = 0; i < nlabels; i++) {
        const Label &l = labels[i];
        if (y < l.y - pad || y >= l.y + char_h + pad)
          continue;
        const int tw = l.len * char_w - s;
        for (int x = l.x - pad; x < l.x + tw + pad; x++)
          darken_px(line, W, x);
        if (y < l.y || y >= l.y + char_h)
          continue;
        const int gy = (y - l.y) / s;
        for (int ci = 0; ci < l.len; ci++) {
          const uint8_t *rows = glyph_for(l.text[ci]);
          if (rows == nullptr)
            continue;
          const uint8_t bits = rows[gy];
          for (int col = 0; col < 5; col++) {
            if (!(bits & (0x10 >> col)))
              continue;
            const int px = l.x + ci * char_w + col * s;
            for (int k = 0; k < s; k++)
              put_px(line, W, px + k, 255, 255, 255);
          }
        }
      }
      // Centre 2x2-pixel box (outline)
      if (y >= box_y0 && y < box_y1) {
        const bool edge_row = y < box_y0 + thick || y >= box_y1 - thick;
        for (int x = box_x0; x < box_x1; x++) {
          if (edge_row || x < box_x0 + thick || x >= box_x1 - thick)
            put_px(line, W, x, 255, 255, 255);
        }
      }
      // Hotspot crosshair: dark outline, then white
      const int dy = y - hot_y;
      if (dy >= -arm - 1 && dy <= arm + 1) {
        const int half = thick / 2;
        if (dy >= -half - 1 && dy <= half + 1)
          for (int x = hot_x - arm - 1; x <= hot_x + arm + 1; x++)
            put_px(line, W, x, 0, 0, 0);
        for (int x = hot_x - half - 1; x <= hot_x + half + 1; x++)
          put_px(line, W, x, 0, 0, 0);
        if (dy >= -arm && dy <= arm) {
          if (dy >= -half && dy <= half)
            for (int x = hot_x - arm; x <= hot_x + arm; x++)
              put_px(line, W, x, 255, 255, 255);
          for (int x = hot_x - half; x <= hot_x + half; x++)
            put_px(line, W, x, 255, 255, 255);
        }
      }
    }

    if (!encoder.process_scanline(line)) {
      ESP_LOGW(TAG, "JPEG encode failed at row %d (out of memory?)", y);
      return false;
    }
  }
  if (!encoder.process_scanline(nullptr))
    return false;
  encoder.deinit();

  this->result_data_ = out.release(&this->result_length_);
  this->last_size_ = this->result_length_;
  return this->result_data_ != nullptr && this->result_length_ > 0;
}

}  // namespace esphome::mlx90640

#endif  // USE_MLX90640_CAMERA
