#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif 

#define LOG_OUT 1
#define OCTAVE 0
#define LIN_OUT 0
#define FHT_N 256
#define SCALE 256
#define OCT_NORM 0

#include "WS2812B.h"
#include <math.h>
#include <FHT.h>
#include <avr/pgmspace.h>
#include <EEPROM.h>

float pi = 3.1415f;
uint8_t *power_table; // stores borders between frequency bins

const static uint8_t noise_table[] PROGMEM = {127, 112, 48, 37, 38, 37, 33, 33, 33, 37, 32, 33, 33, 30, 32, 33, 27, 35, 35, 33, 30, 33, 35, 30, 27, 30, 30, 30, 30, 30, 33, 30, 19, 32, 32, 27, 27, 30, 27, 30, 24, 30, 30, 30, 30, 30, 30, 27, 19, 27, 27, 25, 27, 27, 27, 27, 19, 27, 24, 27, 27, 19, 27, 27, 16, 27, 27, 27, 27, 27, 27, 25, 19, 27, 27, 27, 27, 25, 27, 27, 19, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 19, 27, 27, 25, 27, 27, 27, 27, 24, 25, 27, 27, 27, 27, 27, 30, 19, 25, 27, 19, 19, 19, 27, 19, 24, 19, 19, 19, 19, 19, 19, 19};
uint16_t* freq_bin_noise;

uint16_t total_noise = 3668;
uint16_t SAMPLE_RATE_KHZ = 10;             // Sample rate of the audio in hertz.
uint16_t TIMER_DELAY = 1000 / SAMPLE_RATE_KHZ;
const uint8_t AUDIO_INPUT_PIN = 0;        // Input ADC pin for audio data.
const uint8_t LED_OUTPUT_PIN = 8;
uint8_t FREQ_BINS = -1;
uint8_t MAX_FREQ_BINS = 16;
uint8_t led_bins = -1;

uint32_t last_read = 0;
uint8_t f_zero = 34; //default

WS2812B LEDS;

uint32_t last_update = 0;
uint8_t NUM_RGB = 128;

uint16_t *max_amplitudes;
uint16_t *average_brightnesses;

uint8_t *bin_frequencies; // stores frequency numbers for LED bins
float amplitude_memory_coeff = 0.6f;
uint16_t *smoothed_amplitudes;
uint8_t noise_duration = 2;
uint32_t noise_start;

uint8_t mode = 0;
uint8_t led_bin_order = 0;

float decay_coeff = 0.07f;

uint8_t red;
uint8_t green;
uint8_t blue;

uint8_t **bin_leds;
uint8_t custom_led_bins = 0; // 0 - no bin led data to expect; 1 - custom bin led data should be set
uint8_t variable_bin_length = 0; // 0 - constant length, 1 - variable length; 2 - spectrum mode
uint8_t variable_length; // 0 LEDs fill the whole length, 1 - length depends on amplitude
uint8_t bin_brightness_type; // 0 - independent, 1 - average, 2 - interpolation (linear)
uint8_t f_split_method; // 0 - noise equalization, 1 - Bark method

uint16_t *last_vals;
float total_val;
float max_total_val;
float min_val = 0.15f;

const uint8_t max_message_length = 40;
char message[max_message_length];
uint8_t message_length = 0;

uint8_t color_order; // 0 - monotonic, 1 - random
uint16_t old_tone; // tone is from 0 to 355
uint16_t new_tone; // tone is from 0 to 355
uint8_t old_color[3];
uint8_t new_color[3];
long color_duration = 1000000;
uint8_t color_change_method; // 0 - continuous, 1 - instant, 2 - smoothed step
float color_brightness = 1;
long last_color_changed;

void setup() {
  // set prescale for the ADC to 128. The default is 128
  // Don't set the prescale lower than 16, the resuls will be unreliable!
  // for prescale=32 max_f = 19200 Hz; min_f = 19200/128 = 150
  sbi(ADCSRA,ADPS2) ;
  sbi(ADCSRA,ADPS1) ;
  sbi(ADCSRA,ADPS0) ;

  if (EEPROM.read(0) == 255) {
    for (uint16_t i = 0; i < EEPROM.length(); i++) {
      EEPROM.write(i, 0);
    }
  }
  //EEPROM.update(0, 0);

  Serial.begin(115200);
  pinMode(AUDIO_INPUT_PIN, INPUT);
  randomSeed(analogRead(0));

  boolean numLedsSaved = false;
  EEPROM.get(131, numLedsSaved);
  if (numLedsSaved) {
    EEPROM.get(132, NUM_RGB);
  }
  LEDS.init(NUM_RGB, (uint16_t)3 * NUM_RGB, LED_OUTPUT_PIN);
  randomSeed(analogRead(1));
}

float hue;
float sat = 1.f;
float val = 1.f;

void loop() {
  readSerialData();

  if (mode == 3) {

    uint8_t max_fht_log_out[FHT_N / 2];
    memset(max_fht_log_out, 0, sizeof(max_fht_log_out));
    last_read = micros();
    total_noise = 0;
    while ((last_read - noise_start) < (uint32_t)noise_duration * 1000000 && (last_read - noise_start) > 0) {
      // sample audio data
      acquireAnalogData();
      //calculate frequency amplitudes
      fht_window();
      fht_reorder();
      fht_run();
      fht_mag_log();

      for (uint8_t i = 0; i < FHT_N / 2; i++) {
        if (fht_log_out[i] > max_fht_log_out[i]) {
          total_noise += fht_log_out[i] - max_fht_log_out[i];
          max_fht_log_out[i] = fht_log_out[i];
        }
      }

    }
    Serial.println(F("Noise recorded"));

    EEPROM.update(0, 1); // flag
    for (uint8_t f = 0; f < FHT_N / 2; f++) {
      EEPROM.update(f + 1, max_fht_log_out[f]);
    }
    EEPROM.put(FHT_N / 2 + 1, total_noise); // EEPROM is now occupied till i = 130. Next empty slot - 131
    split_freq_bins();
    mode = 4;

  } else if (mode == 4) {
    if (custom_led_bins && !bin_leds[led_bins - 1]) {
      return;
    }
    // sample audio data
    acquireAnalogData();

    //calculate frequency amplitudes
    fht_window();
    fht_reorder();
    fht_run();
    fht_mag_log();

    for (uint8_t bin = 0; bin < FREQ_BINS; bin++) {
      uint16_t curr_amplitude = 0;
      for (uint8_t f = power_table[bin]; f < power_table[bin + 1]; f++) {
        curr_amplitude += fht_log_out[f];
      }

      curr_amplitude -= min(freq_bin_noise[bin], curr_amplitude);
      //Serial.print(curr_amplitude);

      smoothed_amplitudes[bin] = (uint16_t)((float)smoothed_amplitudes[bin] * (1 - amplitude_memory_coeff) + (float)curr_amplitude * amplitude_memory_coeff);
      if (max_amplitudes[bin] < smoothed_amplitudes[bin]) {
        max_amplitudes[bin] = smoothed_amplitudes[bin];
      }

    }

    sat = 1;
    total_val = 0;
    for (uint8_t led_bin = 0; led_bin < FREQ_BINS; led_bin++) {
      val = (float)smoothed_amplitudes[bin_frequencies[led_bin]] / (max_amplitudes[bin_frequencies[led_bin]] + 0.1f);
      if (min_val > 0) {
        val *= 1 - min_val;
        val += min_val;
      } else {
        val += 0.001; // so I don't divide by zero, which is, of course, illegal
      }
      average_brightnesses[bin_frequencies[led_bin]] = average_brightnesses[bin_frequencies[led_bin]] * (1.f - decay_coeff) + (uint16_t)(65535 * val * decay_coeff);
      last_vals[bin_frequencies[led_bin]] = (uint16_t)(val * 65535);
      if (led_bin < led_bins) {
        total_val += val;
      }
    }
    if (max_total_val < total_val) {
      max_total_val = total_val;
    }
    LEDS.turnLEDsOff();
    updateLEDBins();

    LEDS.render();
    if (led_bin_order > 0) {
      updateBandLocations();
    }
  } else if (mode == 7) { // LED animation
    if (micros() - last_read < 0) {
      last_read = 0;
    }
    while (micros() - last_read < 20000) {

    }
    last_read = micros();

    updateAnimatedColor();
    setRGBColorAll();
    LEDS.render();
  }
}

boolean inputReceived = true;

boolean readSerialData() {

  if (Serial.available()) {
    char c = (char)Serial.read();

    if (c == '<') {
      message_length = 0;
      memset(message, ' ', strlen(message));
      inputReceived = false;
      sei();
      Serial.write(c);
    } else {
      return false;
    }

    while (!inputReceived) {
      if (Serial.available()) {
        c = (char)Serial.read();
        if (c == '>') {
          inputReceived = true;
        } else if (c == '<') {
          message_length = 0;
          memset(message, ' ', strlen(message));
        } else {
          message[message_length] = c;
          message_length++;
        }
      }
    }

    Serial.flush();
    uint8_t index = 0;
    uint8_t sent_length = atoi(getWord(message, &index, message_length));
    index++; // skip the space between size and the message beginning

    // compare the sent and received lengths (not including the length number)
    if (sent_length != message_length - index) {
      Serial.write("0"); // code for fail
      Serial.println();
      Serial.println(message);
      return false;
    } else {
      Serial.write("1"); // code for success
      Serial.println();
      Serial.println(message);
    }
    parseBluetoothMessage(index);
  }
}

void parseBluetoothMessage(uint8_t index) {
  char* first_word = getWord(message, &index, message_length);
  index++;
  if (strcmp(first_word, "0") == 0) { // MODE_OFF
    mode = 0;
    red = 0;
    green = 0;
    blue = 0;
    setRGBColorAll();
    LEDS.render();
  } else if (strcmp(first_word, "1") == 0) { // MODE_RGB
    mode = 1;
    red = atoi(getWord(message, &index, message_length));
    index++;
    green = atoi(getWord(message, &index, message_length));
    index++;
    blue = atoi(getWord(message, &index, message_length));
    setRGBColorAll();
    LEDS.render();
  } else if (strcmp(first_word, "2") == 0) { // MODE_HSV
    mode = 2;
    hue = atoi(getWord(message, &index, message_length));
    index++;
    sat = atof(getWord(message, &index, message_length));
    index++;
    val = atof(getWord(message, &index, message_length));

    HSVtoRGB(hue, sat, val);
    setRGBColorAll();

    LEDS.render();
  } else if (strcmp(first_word, "3") == 0) { // MODE_NOISE
    if (mode == 4) {
      mode = 3;
      noise_duration = atoi(getWord(message, &index, message_length));
      noise_start = micros();
    }
  } else if (strcmp(first_word, "4") == 0) { // MODE_AUDIO
    mode = 4;
    uint8_t old_freq_bins = FREQ_BINS;
    FREQ_BINS = atoi(getWord(message, &index, message_length));
    uint8_t old_led_bins = led_bins;
    index++;
    led_bins = atoi(getWord(message, &index, message_length));
    index++;
    led_bin_order = atoi(getWord(message, &index, message_length));
    index++;
    decay_coeff = atof(getWord(message, &index, message_length));
    index++;
    amplitude_memory_coeff = atof(getWord(message, &index, message_length));
    index++;
    variable_bin_length = atoi(getWord(message, &index, message_length));
    index++;
    variable_length = atoi(getWord(message, &index, message_length));
    index++;
    bin_brightness_type = atoi(getWord(message, &index, message_length));
    index++;
    min_val = atof(getWord(message, &index, message_length));

    Serial.print(FREQ_BINS);
    Serial.print(F("    "));
    Serial.print(led_bins);
    Serial.print(F("    "));
    Serial.print(led_bin_order);
    Serial.print(F("    "));
    Serial.print(decay_coeff);
    Serial.print(F("    "));
    Serial.print(amplitude_memory_coeff);
    Serial.print(F("    "));
    Serial.print(variable_bin_length);
    Serial.print(F("    "));
    Serial.print(variable_length);
    Serial.print(F("    "));
    Serial.println(bin_brightness_type);

    // I decided to allocate max memory and don't reallocate every time FREQ_BINS changes
    // to prevent SRAM fragmentation
    if (!smoothed_amplitudes) {
      max_amplitudes = (uint16_t*)calloc(MAX_FREQ_BINS, sizeof(uint16_t));
      average_brightnesses = (uint16_t*)calloc(MAX_FREQ_BINS, sizeof(uint16_t));
      smoothed_amplitudes = (uint16_t*)calloc(MAX_FREQ_BINS, sizeof(uint16_t));
      power_table = (uint8_t*)calloc(MAX_FREQ_BINS + 1, sizeof(uint8_t));
      bin_frequencies = (uint8_t*)calloc(MAX_FREQ_BINS, sizeof(uint8_t));
      last_vals = (uint16_t*)calloc(MAX_FREQ_BINS, sizeof(uint16_t));
      freq_bin_noise = (uint16_t*)calloc(MAX_FREQ_BINS, sizeof(uint16_t));
    }

    memset(max_amplitudes, 0, sizeof(max_amplitudes));
    memset(average_brightnesses, 0, sizeof(average_brightnesses));
    memset(smoothed_amplitudes, 0, sizeof(smoothed_amplitudes));
    memset(last_vals, 0, sizeof(last_vals));
    max_total_val = 0;

    split_freq_bins();

  } else if (strcmp(first_word, "bin") == 0) { // bin leds
    // message format: bin 123(34)[1,2,4-7,8,13,15,15-23]
    //index++; // skip the space bewtween "bins" and the bin number
    uint8_t bin_num = atoi(getWord(message, &index, message_length));

    index++; // skip the '(' character
    uint8_t bin_size = atoi(getWord(message, &index, message_length));

    index += 2; // skip ')['
    bin_leds[bin_num] = (uint8_t*)calloc(bin_size, sizeof(uint8_t));

    for (uint8_t i = 0; i < bin_size; i++) {
      bin_leds[bin_num][i] = atoi(getWord(message, &index, message_length));
      if (message[index] == '-') {
        index++;
        uint8_t last_led = atoi(getWord(message, &index, message_length));
        for (uint8_t led = bin_leds[bin_num][i] + 1; led <= last_led; led++) {
          i++;
          bin_leds[bin_num][i] = led;
        }
      } else if (message[index] == ',') {
        index++;
      } else if (message[index] == ']') {
        return;
      }
    }
  } else if (strcmp(first_word, "5") == 0) { // set a single LED
    // message format: 5 led_num red green blue
    // message example: 5 34 123 94 0
    mode = 5;
    uint8_t led_num = atoi(getWord(message, &index, message_length));
    index++;
    red = atoi(getWord(message, &index, message_length));
    index++;
    green = atoi(getWord(message, &index, message_length));
    index++;
    blue = atoi(getWord(message, &index, message_length));
    LEDS.setColorRGB(led_num, red, green, blue);
    LEDS.render();
  } else if (strcmp(first_word, "6") == 0) { // change LED count
    // message format: 5 led_num red green blue
    // message example: 5 34 123 94 0
    mode = 6;
    NUM_RGB = atoi(getWord(message, &index, message_length));
    EEPROM.put(131, true);
    EEPROM.put(132, NUM_RGB);
    softwareReset(); // reset the program, this is better for memory
  } else if (strcmp(first_word, "fsm") == 0) { // how frequencies are split in bins
    uint8_t temp_split_method = atoi(getWord(message, &index, message_length));

    if (mode == 4) {
      if (temp_split_method != f_split_method) {
        f_split_method = temp_split_method;
        split_freq_bins();
      }
    }
  } else if (strcmp(first_word, "anim") == 0) { // animated LEDs
    // format: anim  color_order color_change_method  color_duration/1000000  color_brightness
    uint8_t temp_color_order = atoi(getWord(message, &index, message_length));
    index++;
    uint8_t temp_color_change_method = atoi(getWord(message, &index, message_length));
    index++;
    long temp_color_duration = atol(getWord(message, &index, message_length));
    index++;
    float temp_color_brightness = atof(getWord(message, &index, message_length));

    if (mode == 7) {
      if (temp_color_duration != color_duration || temp_color_order != color_order) {
        if (temp_color_duration == 0) {
          old_tone = 0;
          if (temp_color_change_method == 0) {
            new_tone = 30;
          } else {
            new_tone = old_tone;
          }
        } else if (temp_color_order == 1) {
          old_tone = random(356);
          if (temp_color_change_method == 0) {
            new_tone = random(356);
          } else {
            new_tone = old_tone;
          }
        }
        last_color_changed = micros();
      }
      HSVtoRGB(old_tone, 1.f, temp_color_brightness);
      old_color[0] = red;
      old_color[1] = green;
      old_color[2] = blue;

      HSVtoRGB(new_tone, 1.f, temp_color_brightness);
      new_color[0] = red;
      new_color[1] = green;
      new_color[2] = blue;

      color_order = temp_color_order;
      color_change_method = temp_color_change_method;
      color_duration = temp_color_duration;
      color_brightness = temp_color_brightness;

    } else {
      mode = 7;
      color_order = temp_color_order;
      color_change_method = temp_color_change_method;
      color_duration = temp_color_duration;
      color_brightness = temp_color_brightness;

      if (temp_color_duration == 0) {
        old_tone = 0;
        if (temp_color_change_method == 0) {
          new_tone = 30;
        } else {
          new_tone = old_tone;
        }
      } else if (temp_color_order == 1) {
        old_tone = random(356);
        if (temp_color_change_method == 0) {
          new_tone = random(356);
        } else {
          new_tone = old_tone;
        }
      }
      HSVtoRGB(old_tone, 1.f, color_brightness);
      old_color[0] = red;
      old_color[1] = green;
      old_color[2] = blue;

      HSVtoRGB(new_tone, 1.f, color_brightness);
      new_color[0] = red;
      new_color[1] = green;
      new_color[2] = blue;
      last_color_changed = micros();
    }
  }

}

void split_freq_bins() {
  uint16_t noise_left;
  uint8_t custom_noise_saved;
  EEPROM.get(0, custom_noise_saved);
  if (custom_noise_saved) {
    EEPROM.get(FHT_N / 2 + 1, total_noise);
    noise_left = total_noise - EEPROM.read(1) - EEPROM.read(2);
  } else {
    total_noise = 3668;
    noise_left = total_noise - pgm_read_byte_near(noise_table) - pgm_read_byte_near(noise_table + 1);
  }

  if (f_split_method == 0) { // equal noise mode
    uint8_t f = 2;
    for (uint8_t bin = 0; bin < FREQ_BINS; bin++) {
      power_table[bin] = f;
      bin_frequencies[bin] = bin; // default led bin sorting
      freq_bin_noise[bin] = 0;

      while ((float)noise_left / max(freq_bin_noise[bin], 0.1f) > (FREQ_BINS - bin) && f < FHT_N / 2) {
        if (custom_noise_saved) {
          freq_bin_noise[bin] += EEPROM.read(f + 1);
        } else {
          freq_bin_noise[bin] += pgm_read_byte_near(noise_table + f);
        }
        f++;
      }
      noise_left -= freq_bin_noise[bin];
    }
    power_table[FREQ_BINS] = FHT_N / 2;

  } else if (f_split_method == 1) { // Bark scale
    if (f_zero > 0) {
      uint8_t bin = 0;
      float max_bark_scale = getBarkScale(FHT_N / 2);
      power_table[0] = 2;
      freq_bin_noise[0] = 0;
      for (uint8_t f = 2; f < FHT_N / 2; f++) {
        float temp = ((FREQ_BINS + 0.49f) / max_bark_scale) * getBarkScale(f);
        if (temp - 0.5f > bin + 1) {
          bin++;
          power_table[bin] = f;
          bin_frequencies[bin] = bin; // default led bin sorting
          freq_bin_noise[bin] = 0;
        }
        if (custom_noise_saved) {
          freq_bin_noise[bin] += EEPROM.read(f + 1);
        } else {
          freq_bin_noise[bin] += pgm_read_byte_near(noise_table + f);
        }
      }
      power_table[FREQ_BINS] = FHT_N / 2;
    }
  }
  Serial.print(F("Fundamental frequency: "));
  Serial.println(f_zero);
  for (uint8_t bin = 0; bin <= FREQ_BINS; bin++) {
    Serial.print(power_table[bin]);
    Serial.print(F("  "));
  }
  Serial.println();
}

char subarray[10];
char* substring(const char* array, int start, int l) {
  memcpy(subarray, &array[start], l);
  subarray[l] = NULL;
  return subarray;
}

char* getWord(const char* array, uint8_t *start, int array_length) {
  uint8_t l = 0;
  for (int i = *start; i < array_length; i++) {
    if (isDelimiter(array[i])) {
      char *result = substring(array, *start, l);
      *start += l;
      return result;
    }
    l++;
  }
  char *result = substring(array, *start, l);
  *start += l;
  return result;
}
char delimiters[] = {' ', ',', '(', ')', '[', ']', '-'};
uint8_t delimiter_count = 7;

boolean isDelimiter(char c) {
  for (uint8_t i = 0; i < delimiter_count; i++) {
    if (c == delimiters[i]) {
      return true;
    }
  }
  return false;
}

float getBarkScale(uint8_t led_num) {
  uint16_t f = (uint16_t)f_zero * led_num;
  return 13 * (float)atan(0.00076f * f) + 3.5f * (float)atan(sq((float)f / 7500));
}

void updateBandLocations() {
  uint16_t temp[FREQ_BINS];
  for (int8_t i = 0; i < FREQ_BINS; i++) {
    temp[i] = average_brightnesses[i];
    bin_frequencies[i] = i;
  }

  for (int i = 0; i < (FREQ_BINS - 1); i++) {
    for (int j = 0; j < (FREQ_BINS - (i + 1)); j++) {
      if (temp[j] < temp[j + 1]) {
        float t1 = temp[j];
        temp[j] = temp[j + 1];
        temp[j + 1] = t1;
        uint8_t t2 = bin_frequencies[j];
        bin_frequencies[j] = bin_frequencies[j + 1];
        bin_frequencies[j + 1] = t2;
      }
    }
  }

  if (led_bin_order == 2) {
    for (int i = 0; i < (led_bins - 1); i++) {
      for (int j = 0; j < (led_bins - (i + 1)); j++) {
        if (bin_frequencies[j] > bin_frequencies[j + 1]) {
          uint8_t t1 = bin_frequencies[j];
          bin_frequencies[j] = bin_frequencies[j + 1];
          bin_frequencies[j + 1] = t1;
        }
      }
    }
  }
}

void updateLEDBins() {
  if (variable_bin_length == 0) {
    uint8_t dist = 0;
    uint8_t num_leds = NUM_RGB / 2 / led_bins;
    for (uint8_t led_bin = 0; led_bin < led_bins; led_bin++) {
      hue = (float)bin_frequencies[led_bin] / FREQ_BINS * 359;
      val = (float)last_vals[bin_frequencies[led_bin]] / 65535;
      HSVtoRGB(hue, sat, val);
      if (custom_led_bins && bin_leds[led_bins - 1]) {
        uint8_t *grb;
        for (uint8_t i = 0; i < sizeof(bin_leds[led_bin]) / sizeof(uint8_t); i++) {
          grb = LEDS.getPixelGRB(bin_leds[led_bin][i]);
          *grb++ = green; //setting green LED
          *grb++ = red; // setting red LED
          *grb++ = blue; // setting blue LED
        }
      } else {
        uint8_t min_led = min(NUM_RGB / 2 + dist, NUM_RGB - 1);
        setRGBColor(LEDS.getPixelGRB(min_led), num_leds);

        min_led = max(0, (int)(NUM_RGB - 1) / 2 - (int)dist - (int)num_leds + 1);
        setRGBColor(LEDS.getPixelGRB(min_led), num_leds);

        dist += num_leds;
      }
    }
  } else if (variable_bin_length == 1) { // variable bin length
    if (bin_brightness_type == 1) {
      val = total_val / led_bins;
    }
    uint8_t half_length;
    if (variable_length == 1) {
      half_length = (uint8_t)(min(1, (2 * total_val / max_total_val)) * (NUM_RGB)) / 2;
    } else {
      half_length = (NUM_RGB) / 2;
    }

    uint8_t dist = 0;

    for (uint8_t led_bin = 0; led_bin < led_bins; led_bin++) {
      hue = (float)bin_frequencies[led_bin] / FREQ_BINS * 359;
      if (bin_brightness_type == 0) {
        val = (float)last_vals[bin_frequencies[led_bin]] / 65535;
      }
      HSVtoRGB(hue, sat, val);

      uint8_t num_leds;
      if (led_bin != led_bins - 1) {
        num_leds = (uint8_t)(half_length * ((float)last_vals[bin_frequencies[led_bin]] / 65535 / total_val));
      } else {
        num_leds = half_length - dist;
      }
      if (bin_brightness_type == 0 || bin_brightness_type == 1) {
        uint8_t min_led = min(NUM_RGB / 2 + dist, NUM_RGB - 1);
        setRGBColor(LEDS.getPixelGRB(min_led), num_leds);

        min_led = max(0, (int)(NUM_RGB - 1) / 2 - (int)dist - (int)num_leds + 1);
        setRGBColor(LEDS.getPixelGRB(min_led), num_leds);

      } else if (bin_brightness_type == 2) {
        float left_edge = (float)last_vals[bin_frequencies[led_bin]] * last_vals[bin_frequencies[max(0, led_bin - 1)]];
        float right_edge = (float)last_vals[bin_frequencies[led_bin]] * last_vals[bin_frequencies[min(led_bins - 1, led_bin + 1)]];

        left_edge /= (float)last_vals[bin_frequencies[led_bin]] + last_vals[bin_frequencies[max(0, led_bin - 1)]];
        right_edge /= (float)last_vals[bin_frequencies[led_bin]] + last_vals[bin_frequencies[min(led_bins - 1, led_bin + 1)]];

        left_edge *= 2;
        right_edge *= 2;

        uint8_t i;
        for (i = 0; i <= num_leds / 2; i++) {
          val = (last_vals[bin_frequencies[led_bin]] + (left_edge - last_vals[bin_frequencies[led_bin]]) * ((float)(num_leds / 2 - i) / num_leds * 1.9f)) / 65535;
          HSVtoRGB(hue, sat, val);
          setRGBColor(LEDS.getPixelGRB(NUM_RGB / 2 + dist + i), 1); // right side
          setRGBColor(LEDS.getPixelGRB((NUM_RGB - 1) / 2 - dist - i), 1); // left side
        }
        for (i = num_leds / 2 + 1; i < num_leds; i++) {
          val = (last_vals[bin_frequencies[led_bin]] + (right_edge - last_vals[bin_frequencies[led_bin]]) * ((float)(i - num_leds / 2) / num_leds * 1.9f)) / 65535;
          HSVtoRGB(hue, sat, val);
          setRGBColor(LEDS.getPixelGRB(NUM_RGB / 2 + dist + i), 1); // right side
          setRGBColor(LEDS.getPixelGRB((NUM_RGB - 1) / 2 - dist - i), 1); // left side
        }
      }
      dist += num_leds;
    }
  } else if (variable_bin_length == 2) { // RAINBOW MODE!!!!
    if (bin_brightness_type == 1) {
      val = total_val / led_bins;
    }
    float hueDist = 1.f / (FREQ_BINS + 1) * 359;
    float minHue = 0;
    uint8_t dist = 0;
    uint8_t half_length;
    if (variable_length == 1) {
      half_length = (uint8_t)(min(1, (2 * total_val / max_total_val)) * (NUM_RGB)) / 2;
    } else {
      half_length = (NUM_RGB) / 2;
    }

    for (uint8_t led_bin = 0; led_bin < led_bins; led_bin++) {
      if (bin_brightness_type == 0) {
        val = ((float)last_vals[bin_frequencies[led_bin]]) / 65535;
      }

      uint8_t num_leds;
      if (led_bin != led_bins - 1) {
        num_leds = (uint8_t)(half_length * ((float)last_vals[bin_frequencies[led_bin]] / 65535 / total_val));
      } else {
        num_leds = half_length - dist;
      }

      if (bin_brightness_type != 2) {
        for (uint8_t i = 0; i < num_leds; i++) {
          hue = minHue + hueDist * i / num_leds;
          HSVtoRGB(hue, sat, val);
          setRGBColor(LEDS.getPixelGRB(NUM_RGB / 2 + dist + i), 1); // right side
          setRGBColor(LEDS.getPixelGRB((NUM_RGB - 1) / 2 - dist - i), 1); // left side
        }
      } else {
        float left_edge = (float)last_vals[bin_frequencies[led_bin]] * last_vals[bin_frequencies[max(0, led_bin - 1)]];
        float right_edge = (float)last_vals[bin_frequencies[led_bin]] * last_vals[bin_frequencies[min(led_bins - 1, led_bin + 1)]];

        left_edge /= (float)last_vals[bin_frequencies[led_bin]] + last_vals[bin_frequencies[max(0, led_bin - 1)]];
        right_edge /= (float)last_vals[bin_frequencies[led_bin]] + last_vals[bin_frequencies[min(led_bins - 1, led_bin + 1)]];

        left_edge *= 2;
        right_edge *= 2;

        uint8_t i;
        for (i = 0; i <= num_leds / 2; i++) {
          val = (last_vals[bin_frequencies[led_bin]] + (left_edge - last_vals[bin_frequencies[led_bin]]) * ((float)(num_leds / 2 - i) / num_leds * 1.9f)) / 65535;
          hue = minHue + hueDist * i / num_leds;
          HSVtoRGB(hue, sat, val);
          setRGBColor(LEDS.getPixelGRB(NUM_RGB / 2 + dist + i), 1); // right side
          setRGBColor(LEDS.getPixelGRB((NUM_RGB - 1) / 2 - dist - i), 1); // left side
        }
        for (i = num_leds / 2 + 1; i < num_leds; i++) {
          val = (last_vals[bin_frequencies[led_bin]] + (right_edge - last_vals[bin_frequencies[led_bin]]) * ((float)(i - num_leds / 2) / num_leds * 1.9f)) / 65535;
          hue = minHue + hueDist * i / num_leds;
          HSVtoRGB(hue, sat, val);
          setRGBColor(LEDS.getPixelGRB(NUM_RGB / 2 + dist + i), 1); // right side
          setRGBColor(LEDS.getPixelGRB((NUM_RGB - 1) / 2 - dist - i), 1); // left side
        }
      }
      dist += num_leds;
      minHue += hueDist;
    }
  }
}

void updateAnimatedColor() {
  // handle long overflow
  if (micros() - last_color_changed < 0) {
    last_color_changed = 0;
  }
  // generate new color if enough time passed
  if (micros() - last_color_changed > color_duration) {
    old_tone = new_tone;
    old_color[0] = new_color[0];
    old_color[1] = new_color[1];
    old_color[2] = new_color[2];
    if (color_order == 0) {
      new_tone = (new_tone + 30) % 360;
    } else if (color_order == 1) {
      new_tone = random(356);
    }
    HSVtoRGB(new_tone, 1.f, color_brightness);
    new_color[0] = red;
    new_color[1] = green;
    new_color[2] = blue;

    last_color_changed = micros();
  }

  // show the color
  if (color_change_method == 0) { // continuous color change
    float progress = (float)(micros() - last_color_changed) / (float)color_duration;
    red = (uint8_t)(old_color[0] + ((int)new_color[0] - (int)old_color[0]) * progress);
    green = (uint8_t)(old_color[1] + ((int)new_color[1] - (int)old_color[1]) * progress);
    blue = (uint8_t)(old_color[2] + ((int)new_color[2] - (int)old_color[2]) * progress);

  } else if (color_change_method == 1) { // instant color change (step function)
    red = new_color[0];
    green = new_color[1];
    blue = new_color[2];
  } else if (color_change_method == 2) { // smoothed color transition (smoothed step function)
    float progress = (float)(micros() - last_color_changed) / (float)min(color_duration, 20000000l);
    if (progress < 0.25f) {
      progress *= 4; // normalize for transition function
      red = (uint8_t)(old_color[0] + ((int)new_color[0] - (int)old_color[0]) * progress);
      green = (uint8_t)(old_color[1] + ((int)new_color[1] - (int)old_color[1]) * progress);
      blue = (uint8_t)(old_color[2] + ((int)new_color[2] - (int)old_color[2]) * progress);
    } else {
      red = new_color[0];
      green = new_color[1];
      blue = new_color[2];
    }
  }
}

// smooth transition between x ~ 0 and x ~ 1
float transition_function(float x) {
  return (sin((x - 0.5f) / pi) + 1) / 2;
  //return 1.f/(1 + exp(-11*(x - 0.5f)));
  // better, but slower version:
  // (1.f/(1 + exp(-10*(x - 0.5f))) - 0.00669f)/(1 - 2*0.00669f)
}

void setRGBColorAll() {
  uint8_t* grb = LEDS.getPixelGRB(0);
  for (uint16_t i = 0; i < NUM_RGB; i++) {
    *grb++ = green; //setting green LED
    *grb++ = red; // setting red LED
    *grb++ = blue; // setting blue LED
  }
}

void setRGBColor(uint8_t* grb, uint16_t num_leds) {
  for (uint16_t i = 0; i < num_leds; i++) {
    *grb++ = green; //setting green LED
    *grb++ = red; // setting red LED
    *grb++ = blue; // setting blue LED
  }
}

void HSVtoRGB(float h, float s, float v) {
  h /= 60;
  h = fmod(h, 6);

  //s = min(1, s);
  //s = max(0; s);

  //v = min(1, s);
  //v = max(0, v);

  float r = 0;
  float g = 0;
  float b = 0;

  if (h >= 0 && h < 1) {
    r = v;
    g = v * (1 - s * abs(fmod(h, 2) - 1));
    b = v * (1 - s);
  } else if (h >= 1 && h < 2) {
    r = v * (1 - s * abs(fmod(h, 2) - 1));
    g = v;
    b = v * (1 - s);
  } else if (h >= 2 && h < 3) {
    r = v * (1 - s);
    g = v;
    b = v * (1 - s * abs(fmod(h, 2) - 1));
  } else if (h >= 3 && h < 4) {
    r = v * (1 - s);
    g = v * (1 - s * abs(fmod(h, 2) - 1));
    b = v;
  } else if (h >= 4 && h < 5) {
    r = v * (1 - s * abs(fmod(h, 2) - 1));
    g = v * (1 - s);
    b = v;
  } else if (h >= 5 && h < 6) {
    r = v;
    g = v * (1 - s);
    b = v * (1 - s * abs(fmod(h, 2) - 1));
  }

  red = (uint8_t) (r * 255);
  green = (uint8_t) (g * 255);
  blue = (uint8_t) (b * 255);
}
void acquireAnalogData() {
  uint16_t sampleCounter = 0;
  last_read = micros();
  while (sampleCounter < FHT_N) {
    fht_input[sampleCounter] = analogRead(AUDIO_INPUT_PIN);
    sampleCounter++;
  }
  f_zero = (uint8_t)(1000000.f / (micros() - last_read));
  last_read = micros();
}

void softwareReset() {// Restarts program from beginning but does not reset the peripherals and registers
  asm volatile ("  jmp 0");
}
