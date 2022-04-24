## Requirements
* esp-idf
* C++20 (-std=c++2a)

## Installation

Add as git submodule to your components directory or just copy it into your source directories.

## Usage

```cpp
#include <ws2812b.hpp>
using namespace ws2812b;
```

### Creating a new instance


LedStrip can only be created as a shared_ptr. Feel free to copy and store it.
```cpp
auto gpio = 33;
auto led_count = 10;
LedStrip::Ptr led_strip = LedStrip::create(gpio, led_count);
```

After you created your strip you need specify the update interval in milliseconds.
Values under led_count * 1250 + 50 ns won't work.
```cpp
led_strip->run(40);
``` 

If you have multiple led strips you should specify the RMT channel.

```cpp 
auto led_strip_1 = LedStrip::create(gpio, led_count, 0);
auto led_strip_2 = LedStrip::create(gpio + 1, led_count, 1);
```



### Accessing individual leds

```cpp
LedStrip::Led led = led_strip->led(0);
```
Leds internally contain a weak_ptr to the whole strip. Access from multiple tasks (threads) is synchronized through the LedStrip instance.

The LedStrip must outlive the LedStrip::Led instance otherwise it will silently fail.

You can assign and read colors with the led instances.

```cpp
led = 0xff0000_rgb; // assigning red
auto color = LedColor(led) // reading, explicit cast
```

### Creating colors

```cpp
// Designated initalizer
auto color = LedColor{
    .red = 255,
    .green = 0,
    .blue = 128
};

// User defined literal
color = 0x00ff00_rgb;

// from hsv
color = LedColor::from_hsv(360, 100, 100);
```

## Memory usage

### Internally allocated buffers:
* Back buffer: led_count * 3 byte
* Front buffer: led_count * 3 byte
* RMT Item buffer: led_count * 24 * 4 byte

Total: led_count * 102 byte

### Stack size
1280 byte.
