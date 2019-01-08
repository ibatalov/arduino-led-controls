// This class controls an array of RGB LEDs WB2812B
// It should also work for WB2811 and WB2812
#ifndef WS2812B_h
#define WS2812B_h
#include <Arduino.h>
#include <util/delay.h>

#define NUM_BITS      (8)         // Constant value: bits per byte
#define PORT (PORTB)
#define PORT_PIN (PORTB0)

class WS2812B {
  private:
    uint16_t num_bytes;
    uint8_t *rgb_arr = NULL;
    uint32_t t_f;

public:
    uint16_t num_rgb;
    void init(uint16_t, uint16_t, uint8_t);
    void setColorRGB(uint16_t, uint8_t, uint8_t, uint8_t);
    void render(void);
    void turnLEDsOff();
    uint8_t* getPixelGRB(uint16_t);
};
#endif

