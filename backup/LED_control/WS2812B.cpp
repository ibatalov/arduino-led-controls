// This class controls an array of RGB LEDs WB2812B
// It should also work for WB2811 and WB2812
#include <Arduino.h>
#include "WS2812B.h"

void WS2812B::init(uint16_t num_rgb, uint16_t num_bytes, uint8_t digital_pin) {
  this->num_rgb = num_rgb;
  this->num_bytes = num_bytes;

  pinMode(digital_pin, OUTPUT);
  digitalWrite(digital_pin, 0);

  rgb_arr = (uint8_t *)malloc(num_bytes);
  Serial.print("num bytes: ");
  Serial.println(this->num_bytes);
  if (rgb_arr)
  {
    memset(rgb_arr, 0, num_bytes); // fills array with zeros
  }
}

void WS2812B::setColorRGB(uint16_t idx, uint8_t r, uint8_t g, uint8_t b) {
  if (idx < num_rgb) {
    uint8_t *p = &rgb_arr[idx * 3];
    *p++ = g;
    *p++ = r;
    *p = b;
  }
}

uint8_t* WS2812B::getPixelGRB(uint16_t idx) {
  if (idx < num_rgb) {
    return &rgb_arr[idx * 3];
  } else return NULL;
}

void WS2812B::render(void) {
  if (!rgb_arr) return;

  while ((micros() - t_f) < 50L); // wait for 50us (data latch)

  cli(); // Disable interrupts so that timing is as precise as possible
  volatile uint8_t
  *p    = rgb_arr,   // Copy the start address of our data array
   val  = *p++,      // Get the current byte value & point to next byte
   high = PORT |  _BV(PORT_PIN), // Bitmask for sending HIGH to pin
   low  = PORT & ~_BV(PORT_PIN), // Bitmask for sending LOW to pin
   tmp  = low,       // Swap variable to adjust duty cycle
   nbits = NUM_BITS; // Bit counter for inner loop
  volatile uint16_t
  nbytes = num_bytes; // Byte counter for outer loop
  asm volatile(
    // The volatile attribute is used to tell the compiler not to optimize
    // this section.  We want every instruction to be left as is.
    //
    // Generating an 800KHz signal (1.25us period) implies that we have
    // exactly 20 instructions clocked at 16MHz (0.0625us duration) to
    // generate either a 1 or a 0---we need to do it within a single
    // period.
    //
    // By choosing 1 clock cycle as our time unit we can keep track of
    // the signal's phase (T) after each instruction is executed.
    //
    // To generate a value of 1, we need to hold the signal HIGH (maximum)
    // for 0.8us, and then LOW (minimum) for 0.45us.  Since our timing has a
    // resolution of 0.0625us we can only approximate these values. Luckily,
    // the WS281X chips were designed to accept a +/- 300ns variance in the
    // duration of the signal.  Thus, if we hold the signal HIGH for 13
    // cycles (0.8125us), and LOW for 7 cycles (0.4375us), then the variance
    // is well within the tolerated range.
    //
    // To generate a value of 0, we need to hold the signal HIGH (maximum)
    // for 0.4us, and then LOW (minimum) for 0.85us.  Thus, holding the
    // signal HIGH for 6 cycles (0.375us), and LOW for 14 cycles (0.875us)
    // will maintain the variance within the tolerated range.
    //
    // For a full description of each assembly instruction consult the AVR
    // manual here: http://www.atmel.com/images/doc0856.pdf
    // Instruction        CLK     Description                 Phase
    "nextbit:\n\t"         // -    label                       (T =  0)
    "sbi  %0, %1\n\t"     // 2    signal HIGH                 (T =  2)
    "sbrc %4, 7\n\t"      // 1-2  if MSB set                  (T =  ?)
    "mov  %6, %3\n\t"    // 0-1   tmp'll set signal high     (T =  4)
    "dec  %5\n\t"         // 1    decrease bitcount           (T =  5)
    "nop\n\t"             // 1    nop (idle 1 clock cycle)    (T =  6)
    "st   %a2, %6\n\t"    // 2    set PORT to tmp             (T =  8)
    "mov  %6, %7\n\t"     // 1    reset tmp to low (default)  (T =  9)
    "breq nextbyte\n\t"   // 1-2  if bitcount ==0 -> nextbyte (T =  ?)
    "rol  %4\n\t"         // 1    shift MSB leftwards         (T = 11)
    "rjmp .+0\n\t"        // 2    nop nop                     (T = 13)
    "cbi   %0, %1\n\t"    // 2    signal LOW                  (T = 15)
    "rjmp .+0\n\t"        // 2    nop nop                     (T = 17)
    "nop\n\t"             // 1    nop                         (T = 18)
    "rjmp nextbit\n\t"    // 2    bitcount !=0 -> nextbit     (T = 20)
    "nextbyte:\n\t"        // -    label                       -
    "ldi  %5, 8\n\t"      // 1    reset bitcount              (T = 11)
    "ld   %4, %a8+\n\t"   // 2    val = *p++                  (T = 13)
    "cbi   %0, %1\n\t"    // 2    signal LOW                  (T = 15)
    "rjmp .+0\n\t"        // 2    nop nop                     (T = 17)
    "sbiw %9,1\n\t"       // 2    decrease bytecount          (T = 19)
    "brne nextbit\n\t"    // 2    if bytecount !=0 -> nextbit (T = 20)
    ::
    // Input operands         Operand Id (w/ constraint)
    "I" (_SFR_IO_ADDR(PORT)), // %0
    "I" (PORT_PIN),           // %1
    "e" (&PORT),              // %a2
    "r" (high),               // %3
    "r" (val),                // %4
    "r" (nbits),              // %5
    "r" (tmp),                // %6
    "r" (low),                // %7
    "e" (p),                  // %a8
    "w" (nbytes)              // %9
  );
  sei();                          // Enable interrupts
  t_f = micros();                 // t_f will be used to measure the 50us
  // latching period in the next call of the
  // function.
}

