#include <Joystick.h>
#include <inttypes.h>

/*===========================================================================*/
/*LED SETUP*/

#include <FastLED.h>
#define DATA_PIN 10
#define NUM_LEDS 240
CRGB leds[NUM_LEDS];

struct Feature {
  uint16_t LEDstart;
  uint16_t LEDend;
};

const Feature LED_Range[] = {
  //RIGHT - pin 0
  { 0, 47 },  //full square

  //DOWN - pin 1
  { 48, 95 },  //full square

  //LEFT - pin 2
  { 96, 143 },  //full square

  //UP - pin 3
  { 144, 191 },  //full square

  //CENTER no pin
  { 192, 239 },  //full square
};

enum Panelnum {
  Right,  //0
  Down,   //1
  Left,
  Up,
  Center
};


// Set the color profile for the lights
int COLOR_PROFILE = 0;

// Re-usable palletes
CRGB ALL_WHITE[5] = { CRGB::White, CRGB::White, CRGB::White, CRGB::White, CRGB::White };
CRGB ALL_BLACK[5] = { CRGB::Black, CRGB::Black, CRGB::Black, CRGB::Black, CRGB::Black };
CRGB ALL_RED[5] = { CRGB::Red, CRGB::Red, CRGB::Red, CRGB::Red, CRGB::Red };
CRGB ALL_BLUE[5] = { CRGB::Blue, CRGB::Blue, CRGB::Blue, CRGB::Blue, CRGB::Blue };
CRGB ALL_GREEN[5] = { CRGB::Green, CRGB::Green, CRGB::Green, CRGB::Green, CRGB::Green };
CRGB ALL_GOLD[5] = { CRGB::Gold, CRGB::Gold, CRGB::Gold, CRGB::Gold, CRGB::Gold };

// Idle lights, light up when the panel isn't being pressed
CRGB IDLE_COLORS[][5] = {
  //  Left,             Up,               Down,             Right,            Center
  /* 0  Test   */ { CRGB::Gold, CRGB::Red, CRGB::Green, CRGB::Purple, CRGB::Green },
  /* 1  ITG    */ { CRGB::Blue, CRGB::Red, CRGB::Red, CRGB::Blue, CRGB::Black },
  /* 2  DDR    */ { CRGB::DeepSkyBlue, CRGB::DeepPink, CRGB::DeepPink, CRGB::DeepSkyBlue, CRGB::Black },
  /* 3  Brazil */ { CRGB::Gold, CRGB::Gold, CRGB::Gold, CRGB::Gold, CRGB::Green },
  /* 4  Frozen */ { CRGB::White, CRGB::White, CRGB::White, CRGB::White, CRGB::White },
  /* 5  Italy  */ { CRGB::Green, CRGB::White, CRGB::White, CRGB::Red, CRGB::White },
  /* 6  One    */ { CRGB::Black, CRGB::Black, CRGB::Black, CRGB::Black, CRGB::Black },
  /* 7  Prncss */ { CRGB::MediumPurple, CRGB::MediumPurple, CRGB::MediumPurple, CRGB::MediumPurple, CRGB::Magenta },
  /* 8  Navi   */ { CRGB::Blue, CRGB::Blue, CRGB::Blue, CRGB::Blue, CRGB::Green },
  /* 9  USA    */ { CRGB::Red, CRGB::Red, CRGB::Red, CRGB::Red, CRGB::Blue },
  /* 10 Y->BLK */ ALL_GOLD,
  /* 11 R->BLK */ ALL_RED,
  /* 12 B->BLK */ ALL_BLUE,
  /* 13 G->BLK */ ALL_GREEN,
  /* 14 W->BLK */ ALL_WHITE,
  /* 15 BLK->W */ ALL_BLACK,
  /* 16 BLK->R */ ALL_BLACK,
  /* 17 BLK->B */ ALL_BLACK,
  /* 18 BLK->G */ ALL_BLACK,
  /* 19 BLK->DR*/ ALL_BLACK,
  /* 20 BLK->IT*/ ALL_BLACK,
  /* 21 RED->BL*/ ALL_RED,
  /* 22 BLU->RD*/ ALL_BLUE,
  /* 23 RED->GN*/ ALL_RED,
  /* 24 GRE->RD*/ ALL_GREEN,
  /* 25 YEL->RD*/ ALL_GOLD,
};

// Active lights, light up when the panel is pressed
CRGB ACTIVE_COLORS[][5] = {
  // Left,             Up,               Down,             Right,            Center
  /* 0  Test   */ ALL_WHITE,
  /* 1  ITG    */ ALL_WHITE,
  /* 2  DDR    */ ALL_WHITE,
  /* 3  Brazil */ ALL_WHITE,
  /* 4  Frozen */ { CRGB::Blue, CRGB::Blue, CRGB::Blue, CRGB::Blue, CRGB::Blue },
  /* 5  Italy  */ { CRGB::Blue, CRGB::Blue, CRGB::Blue, CRGB::Blue, CRGB::Blue },
  /* 6  One    */ ALL_WHITE,
  /* 7  Prncss */ { CRGB::Gold, CRGB::Gold, CRGB::Gold, CRGB::Gold, CRGB::Gold },
  /* 8  Navi   */ ALL_WHITE,
  /* 9  USA    */ ALL_WHITE,
  /* 10 Y->BLK */ ALL_BLACK,
  /* 11 R->BLK */ ALL_BLACK,
  /* 12 B->BLK */ ALL_BLACK,
  /* 13 G->BLK */ ALL_BLACK,
  /* 14 W->BLK */ ALL_BLACK,
  /* 15 BLK->W */ ALL_WHITE,
  /* 16 BLK->R */ ALL_RED,
  /* 17 BLK->B */ ALL_BLUE,
  /* 18 BLK->G */ ALL_GREEN,
  /* 19 BLK->DR*/ { CRGB::DeepSkyBlue, CRGB::DeepPink, CRGB::DeepPink, CRGB::DeepSkyBlue, CRGB::Black },
  /* 20 BLK->IT*/ { CRGB::Blue, CRGB::Red, CRGB::Red, CRGB::Blue, CRGB::Black },
  /* 21 RED->BL*/ ALL_BLUE,
  /* 22 BLU->RD*/ ALL_RED,
  /* 23 RED->GN*/ ALL_GREEN,
  /* 24 GRE->RD*/ ALL_RED,
  /* 25 YEL->RD*/ ALL_RED,
};
void panel_active(uint32_t panel) {

  for (int i = LED_Range[panel].LEDstart; i <= LED_Range[panel].LEDend; i++) {
    leds[i] = ACTIVE_COLORS[COLOR_PROFILE][panel];
  };
}

void panel_idle(uint32_t panel) {

  for (int i = LED_Range[panel].LEDstart; i <= LED_Range[panel].LEDend; i++) {
    leds[i] = IDLE_COLORS[COLOR_PROFILE][panel];
  };
}

void setIdleColors() {
  for (uint32_t panel = 0; panel <= 4; panel++) {
    panel_idle(panel);
  }
  FastLED.show();
}

/*END LED SETUP*/
/*===========================================================================*/




#if !defined(__AVR_ATmega32U4__) && !defined(__AVR_ATmega328P__) && !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
#define CAN_AVERAGE
#endif

#if defined(_SFR_BYTE) && defined(_BV) && defined(ADCSRA)
#define CLEAR_BIT(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define SET_BIT(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif


#ifdef CORE_TEENSY
// Use the Joystick library for Teensy
void ButtonStart() {
// Use Joystick.begin() for everything that's not Teensy 2.0.
#ifndef __AVR_ATmega32U4__
  Joystick.begin();
#endif
  Joystick.useManualSend(true);
}
void ButtonPress(uint8_t button_num) {
  Joystick.button(button_num, 1);
}
void ButtonRelease(uint8_t button_num) {
  Joystick.button(button_num, 0);
}
#else
#include <Keyboard.h>
// And the Keyboard library for Arduino
void ButtonStart() {
  Keyboard.begin();
}
void ButtonPress(uint8_t button_num) {
  Keyboard.press('a' + button_num - 1);
}
void ButtonRelease(uint8_t button_num) {
  Keyboard.release('a' + button_num - 1);
}
#endif

// Default threshold value for each of the sensors.
const int16_t kDefaultThreshold = 200;
// Max window size for both of the moving averages classes.
const size_t kWindowSize = 50;
// Baud rate used for Serial communication. Technically ignored by Teensys.
const long kBaudRate = 9600;
// Max number of sensors per panel.
// NOTE(teejusb): This is arbitrary, if you need to support more sensors
// per panel then just change the following number.
const size_t kMaxSharedSensors = 2;
// Button numbers should start with 1 (Button0 is not a valid Joystick input).
// Automatically incremented when creating a new SensorState.
uint8_t curButtonNum = 1;

/*===========================================================================*/

// EXPERIMENTAL. Used to turn on the lights feature. Note, this might conflict
// some existing sensor pins so if you see some weird behavior it might be
// because of this. Uncomment the following line to enable the feature.

#define ENABLE_LIGHTS

// We don't want to use digital pins 0 and 1 as they're needed for Serial
// communication so we start curLightPin from 2.
// Automatically incremented when creating a new SensorState.
#if defined(ENABLE_LIGHTS)
uint8_t curLightPin = 10;
#endif

/*===========================================================================*/

// Calculates the Weighted Moving Average for a given period size.
// Values provided to this class should fall in [−32,768, 32,767] otherwise it
// may overflow. We use a 32-bit integer for the intermediate sums which we
// then restrict back down to 16-bits.
class WeightedMovingAverage {
public:
  WeightedMovingAverage(size_t size)
    : size_(min(size, kWindowSize)), cur_sum_(0), cur_weighted_sum_(0),
      values_{}, cur_count_(0) {}

  int16_t GetAverage(int16_t value) {
    // Add current value and remove oldest value.
    // e.g. with value = 5 and cur_count_ = 0
    // [4, 3, 2, 1] -> 10 becomes 10 + 5 - 4 = 11 -> [5, 3, 2, 1]
    int32_t next_sum = cur_sum_ + value - values_[cur_count_];
    // Update weighted sum giving most weight to the newest value.
    // [1*4, 2*3, 3*2, 4*1] -> 20 becomes 20 + 4*5 - 10 = 30
    //     -> [4*5, 1*3, 2*2, 3*1]
    // Subtracting by cur_sum_ is the same as removing 1 from each of the weight
    // coefficients.
    int32_t next_weighted_sum = cur_weighted_sum_ + size_ * value - cur_sum_;
    cur_sum_ = next_sum;
    cur_weighted_sum_ = next_weighted_sum;
    values_[cur_count_] = value;
    cur_count_ = (cur_count_ + 1) % size_;
    // Integer division is fine here since both the numerator and denominator
    // are integers and we need to return an int anyways. Off by one isn't
    // substantial here.
    // Sum of weights = sum of all integers from [1, size_]
    int16_t sum_weights = ((size_ * (size_ + 1)) / 2);
    return next_weighted_sum / sum_weights;
  }

  // Delete default constructor. Size MUST be explicitly specified.
  WeightedMovingAverage() = delete;

private:
  size_t size_;
  int32_t cur_sum_;
  int32_t cur_weighted_sum_;
  // Keep track of all values we have in a circular array.
  int16_t values_[kWindowSize];
  size_t cur_count_;
};

// Calculates the Hull Moving Average. This is one of the better smoothing
// algorithms that will smooth the input values without wildly distorting the
// input values while still being responsive to input changes.
//
// The algorithm is essentially:
//   1. Calculate WMA of input values with a period of n/2 and double it.
//   2. Calculate WMA of input values with a period of n and subtract it from
//      step 1.
//   3. Calculate WMA of the values from step 2 with a period of sqrt(2).
//
// HMA = WMA( 2 * WMA(input, n/2) - WMA(input, n), sqrt(n) )
class HullMovingAverage {
public:
  HullMovingAverage(size_t size)
    : wma1_(size / 2), wma2_(size), hull_(sqrt(size)) {}

  int16_t GetAverage(int16_t value) {
    int16_t wma1_value = wma1_.GetAverage(value);
    int16_t wma2_value = wma2_.GetAverage(value);
    int16_t hull_value = hull_.GetAverage(2 * wma1_value - wma2_value);

    return hull_value;
  }

  // Delete default constructor. Size MUST be explicitly specified.
  HullMovingAverage() = delete;

private:
  WeightedMovingAverage wma1_;
  WeightedMovingAverage wma2_;
  WeightedMovingAverage hull_;
};

/*===========================================================================*/

// The class that actually evaluates a sensor and actually triggers the button
// press or release event. If there are multiple sensors added to a
// SensorState, they will all be evaluated first before triggering the event.
class SensorState {
public:
  SensorState()
    : num_sensors_(0),
#if defined(ENABLE_LIGHTS)
      kLightsPin(curLightPin++),
#endif
      buttonNum(curButtonNum++) {
    for (size_t i = 0; i < kMaxSharedSensors; ++i) {
      sensor_ids_[i] = 0;
      individual_states_[i] = SensorState::OFF;
    }
#if defined(ENABLE_LIGHTS)
    pinMode(kLightsPin, OUTPUT);
#endif
  }

  void Init() {
    if (initialized_) {
      return;
    }
    buttonNum = curButtonNum++;
    initialized_ = true;
  }

  // Adds a new sensor to share this state with. If we try adding a sensor that
  // we don't have space for, it's essentially dropped.
  void AddSensor(uint8_t sensor_id) {
    if (num_sensors_ < kMaxSharedSensors) {
      sensor_ids_[num_sensors_++] = sensor_id;
    }
  }

  // Evaluates a single sensor as part of the shared state.
  void EvaluateSensor(uint8_t sensor_id,
                      int16_t cur_value,
                      int16_t user_threshold) {
    if (!initialized_) {
      return;
    }
    size_t sensor_index = GetIndexForSensor(sensor_id);

    // The sensor we're evaluating is not part of this shared state.
    // This should not happen.
    if (sensor_index == SIZE_MAX) {
      return;
    }

    // If we're above the threshold, turn the individual sensor on.
    if (cur_value >= user_threshold + kPaddingWidth) {
      individual_states_[sensor_index] = SensorState::ON;
    }

    // If we're below the threshold, turn the individual sensor off.
    if (cur_value < user_threshold - kPaddingWidth) {
      individual_states_[sensor_index] = SensorState::OFF;
    }

    // If we evaluated all the sensors this state applies to, only then
    // should we determine if we want to send a press/release event.
    bool all_evaluated = (sensor_index == num_sensors_ - 1);

    if (all_evaluated) {
      switch (combined_state_) {
        case SensorState::OFF:
          {
            // If ANY of the sensors triggered, then we trigger a button press.
            bool turn_on = false;
            for (size_t i = 0; i < num_sensors_; ++i) {
              if (individual_states_[i] == SensorState::ON) {
                turn_on = true;
                break;
              }
            }
            if (turn_on) {
              ButtonPress(buttonNum);
              combined_state_ = SensorState::ON;
#if defined(ENABLE_LIGHTS)
              digitalWrite(kLightsPin, HIGH);
              // Light on
              // Set panel LED to active color

              panel_active(buttonNum-1);
              panel_active(4);
              FastLED.show();
#endif
            }
          }
          break;
        case SensorState::ON:
          {
            // ALL of the sensors must be off to trigger a release.
            // i.e. If any of them are ON we do not release.
            bool turn_off = true;
            for (size_t i = 0; i < num_sensors_; ++i) {
              if (individual_states_[i] == SensorState::ON) {
                turn_off = false;
              }
            }
            if (turn_off) {
              ButtonRelease(buttonNum);
              combined_state_ = SensorState::OFF;
#if defined(ENABLE_LIGHTS)
              digitalWrite(kLightsPin, LOW);
              // Light off
              // Reset panel LED back to idle color (or off, if black)
              panel_idle(buttonNum-1);
              panel_idle(4);
              FastLED.show();
#endif
            }
          }
          break;
      }
    }
  }

  // Given a sensor_id, returns the index in the sensor_ids_ array.
  // Returns SIZE_MAX if not found.
  size_t GetIndexForSensor(uint8_t sensor_id) {
    for (size_t i = 0; i < num_sensors_; ++i) {
      if (sensor_ids_[i] == sensor_id) {
        return i;
      }
    }
    return SIZE_MAX;
  }

private:
  // Ensures that Init() has been called at exactly once on this SensorState.
  bool initialized_;

  // The collection of sensors shared with this state.
  uint8_t sensor_ids_[kMaxSharedSensors];
  // The number of sensors this state combines with.
  size_t num_sensors_;

  // Used to determine the state of each individual sensor, as well as
  // the aggregated state.
  enum State { OFF,
               ON };
  // The evaluated state for each individual sensor.
  State individual_states_[kMaxSharedSensors];
  // The aggregated state.
  State combined_state_ = SensorState::OFF;

  // One-tailed width size to create a window around user_threshold to
  // mitigate fluctuations by noise.
  // TODO(teejusb): Make this a user controllable variable.
  const int16_t kPaddingWidth = 1;

// The light pin this state corresponds to.
#if defined(ENABLE_LIGHTS)
  const uint8_t kLightsPin;
#endif

  // The button number this state corresponds to.
  // Set once in Init().
  uint8_t buttonNum;
};

/*===========================================================================*/

// Class containing all relevant information per sensor.
class Sensor {
public:
  Sensor(uint8_t pin_value, SensorState* sensor_state = nullptr)
    : initialized_(false), pin_value_(pin_value),
      user_threshold_(kDefaultThreshold),
#if defined(CAN_AVERAGE)
      moving_average_(kWindowSize),
#endif
      offset_(0), sensor_state_(sensor_state),
      should_delete_state_(false) {
  }

  ~Sensor() {
    if (should_delete_state_) {
      delete sensor_state_;
    }
  }

  void Init(uint8_t sensor_id) {
    // Sensor should only be initialized once.
    if (initialized_) {
      return;
    }
    // Sensor IDs should be 1-indexed thus they must be non-zero.
    if (sensor_id == 0) {
      return;
    }

    // There is no state for this sensor, create one.
    if (sensor_state_ == nullptr) {
      sensor_state_ = new SensorState();
      // If this sensor created the state, then it's in charge of deleting it.
      should_delete_state_ = true;
    }

    // Initialize the sensor state.
    // This sets the button number corresponding to the sensor state.
    // Trying to re-initialize a sensor_state_ is a no-op, so no harm in
    sensor_state_->Init();

    // If this sensor hasn't been added to the state, then try adding it.
    if (sensor_state_->GetIndexForSensor(sensor_id) == SIZE_MAX) {
      sensor_state_->AddSensor(sensor_id);
    }
    sensor_id_ = sensor_id;
    initialized_ = true;
  }

  // Fetches the sensor value and maybe triggers the button press/release.
  void EvaluateSensor(bool willSend) {
    if (!initialized_) {
      return;
    }
    // If this sensor was never added to the state, then return early.
    if (sensor_state_->GetIndexForSensor(sensor_id_) == SIZE_MAX) {
      return;
    }

    int16_t sensor_value = analogRead(pin_value_);

#if defined(CAN_AVERAGE)
    // Fetch the updated Weighted Moving Average.
    cur_value_ = moving_average_.GetAverage(sensor_value) - offset_;
    cur_value_ = constrain(cur_value_, 0, 1023);
#else
    // Don't use averaging for Arduino Leonardo, Uno, Mega1280, and Mega2560
    // since averaging seems to be broken with it. This should also include
    // the Teensy 2.0 as it's the same board as the Leonardo.
    // TODO(teejusb): Figure out why and fix. Maybe due to different integer
    // widths?
    cur_value_ = sensor_value - offset_;
#endif

    if (willSend) {
      sensor_state_->EvaluateSensor(
        sensor_id_, cur_value_, user_threshold_);
    }
  }

  void UpdateThreshold(int16_t new_threshold) {
    user_threshold_ = new_threshold;
  }

  int16_t UpdateOffset() {
    // Update the offset with the last read value. UpdateOffset should be
    // called with no applied pressure on the panels so that it will be
    // calibrated correctly.
    offset_ = cur_value_;
    return offset_;
  }

  int16_t GetCurValue() {
    return cur_value_;
  }

  int16_t GetThreshold() {
    return user_threshold_;
  }

  // Delete default constructor. Pin number MUST be explicitly specified.
  Sensor() = delete;

private:
  // Ensures that Init() has been called at exactly once on this Sensor.
  bool initialized_;
  // The pin on the Teensy/Arduino corresponding to this sensor.
  uint8_t pin_value_;

  // The user defined threshold value to activate/deactivate this sensor at.
  int16_t user_threshold_;

#if defined(CAN_AVERAGE)
  // The smoothed moving average calculated to reduce some of the noise.
  HullMovingAverage moving_average_;
#endif

  // The latest value obtained for this sensor.
  int16_t cur_value_;
  // How much to shift the value read by during each read.
  int16_t offset_;

  // Since many sensors may refer to the same input this may be shared among
  // other sensors.
  SensorState* sensor_state_;
  // Used to indicate if the state is owned by this instance, or if it was
  // passed in from outside
  bool should_delete_state_;

  // A unique number corresponding to this sensor. Set during Init().
  uint8_t sensor_id_;
};

/*===========================================================================*/

// Defines the sensor collections and sets the pins for them appropriately.
//
// If you want to use multiple sensors in one panel, you will want to share
// state across them. In the following example, the first and second sensors
// share state. The maximum number of sensors that can be shared for one panel
// is controlled by the kMaxSharedSensors constant at the top of this file, but
// can be modified as needed.
//
// SensorState state1;
// Sensor kSensors[] = {
//   Sensor(A0, &state1),
//   Sensor(A1, &state1),
//   Sensor(A2),
//   Sensor(A3),
//   Sensor(A4),
// };

Sensor kSensors[] = {
  Sensor(A0),  //RIGHT
  Sensor(A1),  //UP
  Sensor(A2),  //LEFT
  Sensor(A3),  //DOWN
};
const size_t kNumSensors = sizeof(kSensors) / sizeof(Sensor);

/*===========================================================================*/

class SerialProcessor {
public:
  void Init(long baud_rate) {
    Serial.begin(baud_rate);
  }

  void CheckAndMaybeProcessData() {
    while (Serial.available() > 0) {
      size_t bytes_read = Serial.readBytesUntil(
        '\n', buffer_, kBufferSize - 1);
      buffer_[bytes_read] = '\0';

      if (bytes_read == 0) { return; }

      switch (buffer_[0]) {
        case 'o':
        case 'O':
          UpdateOffsets();
          break;
        case 'v':
        case 'V':
          PrintValues();
          break;
        case 't':
        case 'T':
          PrintThresholds();
          break;
        case 'c':
        case 'C':
          UpdateColorProfile(bytes_read);
          break;
        case '0' ... '9':  // Case ranges are non-standard but work in gcc
          UpdateAndPrintThreshold(bytes_read);
        default:
          break;
      }
    }
  }

  void UpdateAndPrintThreshold(size_t bytes_read) {
    // Need to specify:
    // Sensor number + Threshold value, separated by a space.
    // {0, 1, 2, 3,...} + "0"-"1023"
    // e.g. 3 180 (fourth FSR, change threshold to 180)

    if (bytes_read < 3 || bytes_read > 7) { return; }

    char* next = nullptr;
    size_t sensor_index = strtoul(buffer_, &next, 10);
    if (sensor_index >= kNumSensors) { return; }

    int16_t sensor_threshold = strtol(next, nullptr, 10);
    if (sensor_threshold < 0 || sensor_threshold > 1023) { return; }

    kSensors[sensor_index].UpdateThreshold(sensor_threshold);
    PrintThresholds();
  }

  void UpdateOffsets() {
    for (size_t i = 0; i < kNumSensors; ++i) {
      kSensors[i].UpdateOffset();
    }
  }

  void PrintValues() {
    Serial.print("v");
    for (size_t i = 0; i < kNumSensors; ++i) {
      Serial.print(" ");
      Serial.print(kSensors[i].GetCurValue());
    }
    Serial.print("\n");
  }

  void PrintThresholds() {
    Serial.print("t");
    for (size_t i = 0; i < kNumSensors; ++i) {
      Serial.print(" ");
      Serial.print(kSensors[i].GetThreshold());
    }
    Serial.print("\n");
  }

  void PrintColorProfile() {
    Serial.print("c");
    Serial.print(" ");
    Serial.print(COLOR_PROFILE);
    Serial.print("\n");
  }

  void UpdateColorProfile(size_t bytes_read) {
    // Need to specify:
    // C + color profile.
    // e.g. C3 (selects third color profile)

    // If the value isn't there (just "C") only print the profile
    if (bytes_read < 2 || bytes_read > 5) {
      PrintColorProfile();
      return;
    }
    // Update the COLOR_PROFILE variable with whatever the value is
    COLOR_PROFILE = strtoul(buffer_ + 1, nullptr, 10);
    // Print it out
    PrintColorProfile();
    // Set the new idle colors
    setIdleColors();
  }

private:
  static const size_t kBufferSize = 64;
  char buffer_[kBufferSize];
};

/*===========================================================================*/

SerialProcessor SerialProcessor;
// Timestamps are always "unsigned long" regardless of board type So don't need
// to explicitly worry about the widths.
unsigned long lastSend = 0;
// loopTime is used to estimate how long it takes to run one iteration of
// loop().
long loopTime = -1;

void setup() {

  // Add the LEDs
  FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, NUM_LEDS).setCorrection(TypicalSMD5050).setTemperature(CarbonArc);
  // FastLED provides these pre-conigured incandescent color profiles:
  //     Candle, Tungsten40W, Tungsten100W, Halogen, CarbonArc,
  //     HighNoonSun, DirectSunlight, OvercastSky, ClearBlueSky,
  // FastLED provides these pre-configured gaseous-light color profiles:
  //     WarmFluorescent, StandardFluorescent, CoolWhiteFluorescent,
  //     FullSpectrumFluorescent, GrowLightFluorescent, BlackLightFluorescent,
  //     MercuryVapor, SodiumVapor, MetalHalide, HighPressureSodium,
  // FastLED also provides an "Uncorrected temperature" profile
  //    UncorrectedTemperature;

  // Set each light to its idle color on start
  setIdleColors();

  SerialProcessor.Init(kBaudRate);
  ButtonStart();
  for (size_t i = 0; i < kNumSensors; ++i) {
    // Button numbers should start with 1.
    kSensors[i].Init(i + 1);
  }
}

void loop() {
  unsigned long startMicros = micros();
  // We only want to send over USB every millisecond, but we still want to
  // read the analog values as fast as we can to have the most up to date
  // values for the average.
  static bool willSend;
  // Separate out the initialization and the update steps for willSend.
  // Since willSend is static, we want to make sure we update the variable
  // every time we loop.
  willSend = (loopTime == -1 || startMicros - lastSend + loopTime >= 1000);

  SerialProcessor.CheckAndMaybeProcessData();

  for (size_t i = 0; i < kNumSensors; ++i) {
    kSensors[i].EvaluateSensor(willSend);
  }

  if (willSend) {
    lastSend = startMicros;
#ifdef CORE_TEENSY
    Joystick.send_now();
#endif
  }

  if (loopTime == -1) {
    loopTime = micros() - startMicros;
  }
}