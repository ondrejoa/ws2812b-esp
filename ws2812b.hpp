#pragma once

#include <cstdint>
#include <memory>

namespace ws2812b {
class LedStripPrivate;
class LedStrip : public std::enable_shared_from_this<LedStrip> {
   public:
    using Ptr = std::shared_ptr<LedStrip>;
    ~LedStrip();

    [[nodiscard]] static Ptr create(int gpio, uint16_t led_count, int channel = 0);

    struct __attribute__((packed)) LedColor {
        uint8_t red = 0;
        uint8_t green = 0;
        uint8_t blue = 0;
    };

    void run(uint32_t auto_flush_ms);

    class Led {
       public:
        explicit operator LedColor() const;
        LedColor operator=(LedColor color);
        uint16_t index() const;

        Led(const Led&) = default;

       private:
        Led(Ptr::weak_type strip, uint16_t index);
        Ptr::weak_type strip_;
        uint16_t index_;
        friend class LedStrip;
    };

    Led led(uint16_t index);

   private:
    LedStrip(int gpio, uint16_t led_count, int channel);
    friend class Led;

   protected:
    std::unique_ptr<LedStripPrivate> d_ptr_;
};

using LedColor = LedStrip::LedColor;

static LedColor constexpr operator"" _rgb(unsigned long long val) {
    constexpr unsigned long long mask = 0xFF;
    return LedColor{
        .red = uint8_t((val >> 16) & mask),
        .green = uint8_t((val >> 8) & mask),
        .blue = uint8_t(val & mask),
    };
}
}  // namespace ws2812b