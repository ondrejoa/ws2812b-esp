#include "ws2812b.hpp"

#include <driver/gpio.h>
#include <driver/rmt.h>
#include <esp_err.h>
#include <esp_pthread.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>
#include <iostream>
#include <cmath>

namespace ws2812b {
using std::this_thread::sleep_for;
using namespace std::chrono_literals;

namespace {
constexpr uint64_t clk_div = 4;

constexpr uint64_t clk_mhz = 80;

constexpr uint64_t t0h_ns = 400;
constexpr uint64_t t1h_ns = 800;
constexpr uint64_t t0l_ns = 850;
constexpr uint64_t t1l_ns = 450;
constexpr uint64_t res_ns = 150;  // At least 50

constexpr uint64_t clk_divided() {
    return clk_mhz / clk_div;
}

constexpr uint64_t clk_to_ns() {
    return (1000 * 1000 * 1000) / (clk_divided() * 1000 * 1000);
}

constexpr uint32_t periods(uint64_t required_ns) {
    return uint32_t(required_ns / clk_to_ns());
}

constexpr uint32_t t0h_periods = periods(t0h_ns);
constexpr uint32_t t1h_periods = periods(t1h_ns);
constexpr uint32_t t0l_periods = periods(t0l_ns);
constexpr uint32_t t1l_periods = periods(t1l_ns);

constexpr int color_bits = 24;
}  // namespace

class LedStripPrivate {
   public:
    LedStripPrivate(gpio_num_t gpio, uint16_t led_count, rmt_channel_t channel);

    const uint16_t led_count_;

    void auto_flush(std::chrono::milliseconds interval);

    void write_color(LedStrip::LedColor color, uint16_t index);
    LedStrip::LedColor read_color(uint16_t index) const;

   private:
    std::vector<LedStrip::LedColor> front_buffer_;
    std::vector<LedStrip::LedColor> back_buffer_;

    std::atomic_bool back_buffer_dirty;
    mutable std::mutex back_buffer_mutex_;

    mutable std::vector<rmt_item32_t> item_buffer_;

    rmt_channel_t rmt_channel_;
    gpio_num_t gpio_;

    void show() const;
    void copy_to_front();
};

LedStrip::Ptr LedStrip::create(int gpio, uint16_t led_count, int channel) {
    return Ptr(new LedStrip(gpio, led_count, channel));
}

LedStrip::LedStrip(int gpio, uint16_t led_count, int channel) : d_ptr_(new LedStripPrivate(gpio_num_t(gpio), led_count, rmt_channel_t(channel))) {
}

LedStrip::~LedStrip() {
}

void LedStrip::run(uint32_t auto_flush_ms) {
    auto config = esp_pthread_get_default_config();
    config.thread_name = "Led strip";
    config.stack_size = 1280;
    config.pin_to_core = 1;
    ESP_ERROR_CHECK(esp_pthread_set_cfg(&config));
    std::thread{std::mem_fn(&LedStripPrivate::auto_flush), d_ptr_.get(), std::chrono::milliseconds(auto_flush_ms)}.detach();
    config = esp_pthread_get_default_config();
    ESP_ERROR_CHECK(esp_pthread_set_cfg(&config));
}

LedStripPrivate::LedStripPrivate(gpio_num_t gpio, uint16_t led_count, rmt_channel_t channel) : led_count_(led_count), rmt_channel_(channel), gpio_(gpio) {
    rmt_config_t conf = {
        .rmt_mode = RMT_MODE_TX,
        .channel = channel,
        .gpio_num = gpio,
        .clk_div = uint8_t(clk_div),
        .mem_block_num = 1,
        .tx_config = {
            .carrier_freq_hz = 38000,
            .carrier_level = RMT_CARRIER_LEVEL_LOW,
            .idle_level = RMT_IDLE_LEVEL_LOW,
            .carrier_duty_percent = 50,
            .carrier_en = false,
            .loop_en = false,
            .idle_output_en = true,
        }};
    auto config = esp_pthread_get_default_config();
    config.pin_to_core = 1;
    ESP_ERROR_CHECK(esp_pthread_set_cfg(&config));
    auto init = std::thread{[this, conf]() {
        ESP_ERROR_CHECK(rmt_config(&conf));
        ESP_ERROR_CHECK(rmt_driver_install(conf.channel, 0, 0));

        item_buffer_.resize(led_count_ * color_bits);

        front_buffer_.resize(led_count_);
        back_buffer_.resize(led_count_);
    }};
    config = esp_pthread_get_default_config();
    ESP_ERROR_CHECK(esp_pthread_set_cfg(&config));
    init.join();

}

void byte_to_items(uint8_t byte, rmt_item32_t *items) {
    constexpr uint8_t mask = 0b1000'0000u;
    for (int i = 0; i < 8; ++i) {
        rmt_item32_t item;
        if (((byte << i) & mask) == 0) {
            item.duration0 = t0h_periods;
            item.level0 = 1;
            item.duration1 = t0l_periods;
            item.level1 = 0;

        } else {
            item.duration0 = t1h_periods;
            item.level0 = 1;
            item.duration1 = t1l_periods;
            item.level1 = 0;
        }
        items[i] = item;
    }
}

void color_to_items(LedStrip::LedColor color, rmt_item32_t *items) {
    byte_to_items(color.green, items + 8);
    byte_to_items(color.red, items);
    byte_to_items(color.blue, items + 16);
}

void LedStripPrivate::show() const {
    auto data = front_buffer_.data();
    ESP_ERROR_CHECK(rmt_wait_tx_done(rmt_channel_, portMAX_DELAY));
    for (int ldi = 0; ldi < led_count_; ++ldi) {
        color_to_items(data[ldi], item_buffer_.data() + color_bits * ldi);
    }
    ESP_ERROR_CHECK(rmt_write_items(rmt_channel_, item_buffer_.data(), item_buffer_.size(), false));
}

void LedStripPrivate::auto_flush(std::chrono::milliseconds interval) {
    while (true) {
        if (back_buffer_dirty.exchange(false)) {
            copy_to_front();
            show();
        }
        sleep_for(interval);
    }
}

void LedStripPrivate::copy_to_front() {
    std::lock_guard<std::mutex> lock(back_buffer_mutex_);
    for (int i = 0; i < led_count_; ++i) {
        front_buffer_[i] = back_buffer_[i];
    }
}

void LedStripPrivate::write_color(LedStrip::LedColor color, uint16_t index) {
    std::lock_guard<std::mutex> lock(back_buffer_mutex_);
    back_buffer_[index] = color;
    back_buffer_dirty.store(true);
}

LedStrip::LedColor LedStripPrivate::read_color(uint16_t index) const {
    std::lock_guard<std::mutex> lock(back_buffer_mutex_);
    return back_buffer_[index];
}

LedStrip::Led LedStrip::led(uint16_t index) {
    if (index < d_ptr_->led_count_) {
        return Led(weak_from_this(), index);
    }
    return Led({}, 0);
}

LedStrip::Led::Led(LedStrip::Ptr::weak_type strip, uint16_t index) : strip_(strip), index_(index) {
}

LedStrip::Led::operator LedStrip::LedColor() const {
    if (auto strip = strip_.lock(); strip) {
        return strip->d_ptr_->read_color(index_);
    }
    return {};
}

LedStrip::LedColor LedStrip::Led::operator=(LedStrip::LedColor color) {
    if (auto strip = strip_.lock(); strip) {
        strip->d_ptr_->write_color(color, index_);
        return color;
    }
    return {};
}

uint16_t LedStrip::Led::index() const {
    return index_;
}

LedColor LedColor::from_hsv(float H, float S, float V) {
    LedColor color{.red = 0, .green = 0, .blue = 0};
    if (H > 360.0f || H < 0.0f || S > 100.0f || S < 0.0f || V > 100.0f || V < 0.0f) {
        std::cerr << "HSV not in range" << std::endl;
        return color;
    }
    const float s = S / 100.0f;
    const float v = V / 100.0f;
    const float C = s * v;
    const float X = C * (1.0f - std::abs(std::fmod(H / 60.0f, 2) - 1));
    const float m = v - C;
    float r, g, b;
    if (H >= 0.0f && H < 60.0f) {
        r = C;
        g = X;
        b = 0;
    } else if (H >= 60.0f && H < 120.0f) {
        r = X;
        g = C;
        b = 0;
    } else if (H >= 120.0f && H < 180.0f) {
        r = 0;
        g = C;
        b = X;
    } else if (H >= 180.0f && H < 240.0f) {
        r = 0;
        g = X;
        b = C;
    } else if (H >= 240.0f && H < 300.0f) {
        r = X;
        g = 0;
        b = C;
    } else {
        r = C;
        g = 0;
        b = X;
    }

    color.red = uint8_t((r + m) * 255.0f);
    color.green = uint8_t((g + m) * 255.0f);
    color.blue = uint8_t((b + m) * 255.0f);

    return color;
}

std::ostream &operator<<(std::ostream &os, LedColor color) {
    auto [r, g, b] = color;
    return os << "Color{ .red = " << int(r) << ", .green = " << int(g) << ", .blue = " << int(b) << " }";
}

}  // namespace ws2812b