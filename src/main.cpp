#include <Arduino.h>

#if defined(BOARD_D1MINI_ESP32)
// D1 Mini ESP32 safe GPIOs.
// Excludes flash pins (6-11), input-only (34-39), strapping pins (0, 2, 12, 15),
// and UART pins (1, 3) so upload/serial stay stable.
static const uint8_t kGpios[] = {
    4, 5,
    13, 14, 16, 17, 18, 19,
    21, 22, 23,
    25, 26, 27,
    32, 33,
    2,
};
#elif defined(BOARD_ESP32S3)
// ESP32-S3 safe GPIOs (avoid strapping, USB, and UART0 pins by default).
// Adjust this list for your specific S3 module as needed.
static const uint8_t kGpios[] = {
    4, 5, 6, 7, 8, 9, 10, 11,
    12, 13, 14, 15, 16, 18,
    21,
    35, 36, 37, 38, 39, 41, 42,
    40
};
#else
#error "Define a BOARD_* build flag to select the GPIO list."
#endif

// MORSE selects the GPIO-blinking mode; PINS_TOGGLE selects the button-stepped mode.
// PWM sensing is the default when neither is defined.
#if defined(MORSE)
static const unsigned long kDashMs = 500;
#else
static const unsigned long kDashMs = 200;
#endif
static const unsigned long kDotMs = kDashMs / 3;
static const unsigned long kIntraElementGapMs = kDotMs;
static const unsigned long kInterLetterGapMs = 3 * kDotMs;
static const unsigned long kInterWordGapMs = 7 * kDotMs;
static const unsigned long kErrorLogRateMs = 1000;

static const char* MorseForChar(char c) {
  switch (c) {
    case 'A': return ".-";
    case 'B': return "-...";
    case 'C': return "-.-.";
    case 'D': return "-..";
    case 'E': return ".";
    case 'F': return "..-.";
    case 'G': return "--.";
    case 'H': return "....";
    case 'I': return "..";
    case 'J': return ".---";
    case 'K': return "-.-";
    case 'L': return ".-..";
    case 'M': return "--";
    case 'N': return "-.";
    case 'O': return "---";
    case 'P': return ".--.";
    case 'Q': return "--.-";
    case 'R': return ".-.";
    case 'S': return "...";
    case 'T': return "-";
    case 'U': return "..-";
    case 'V': return "...-";
    case 'W': return ".--";
    case 'X': return "-..-";
    case 'Y': return "-.--";
    case 'Z': return "--..";
    case '0': return "-----";
    case '1': return ".----";
    case '2': return "..---";
    case '3': return "...--";
    case '4': return "....-";
    case '5': return ".....";
    case '6': return "-....";
    case '7': return "--...";
    case '8': return "---..";
    case '9': return "----.";
    default: return "";
  }
}

struct MorseSequence {
  uint16_t durations[64];
  uint8_t levels[64];
  uint8_t len;
};

struct PinState {
  uint8_t pin;
  uint8_t idx;
  unsigned long next_ms;
};

// Guard against board variants where a listed pin isn't output-capable.
static bool IsValidOutputPin(uint8_t pin) {
  return digitalPinCanOutput(pin);
}

static void LogInvalidPin(uint8_t pin) {
  static unsigned long last_log_ms = 0;
  unsigned long now = millis();
  if (now - last_log_ms >= kErrorLogRateMs) {
#if defined(BOARD_ESP32S3)
    // Keep logs on USB CDC for S3; other boards may not expose serial.
    Serial.print("Skipping invalid output pin: GPIO");
    Serial.println(pin);
#endif
    last_log_ms = now;
  }
}

#if defined(MORSE) && defined(PINS_TOGGLE)
#error "Define only one of MORSE or PINS_TOGGLE."
#endif

#if !defined(MORSE) && !defined(PINS_TOGGLE)
// PWM sensing inputs and status indicator.
#if !defined(SENSE_IN_PIN) || !defined(STATUS_LED_PIN)
#error "Define SENSE_IN_PIN and STATUS_LED_PIN in platformio.ini build_flags."
#endif
static const uint8_t kSenseInPin = SENSE_IN_PIN;
static const uint8_t kStatusLedPin = STATUS_LED_PIN;
static const uint32_t kTestToneHz = 1000;
static const unsigned long kDetectWindowMs = 25;
static const unsigned long kConfirmWindowMs = 60;
// Minimum observed edges in the window to accept a connection.
static const uint16_t kMinEdges = 30;
static const uint8_t kPwmChannel = 0;
static const uint8_t kPwmResolutionBits = 8;
#endif

#if defined(MORSE) && defined(TEST_BUTTON)
// Button-driven override for Morse mode (per-board pins via TEST_* macros).
#if !defined(TEST_BUTTON_PIN) || !defined(TEST_LED_PIN)
#error "Define TEST_BUTTON_PIN and TEST_LED_PIN in platformio.ini build_flags."
#endif
static const uint8_t kTestButtonPin = TEST_BUTTON_PIN;
static const uint8_t kTestLedPin = TEST_LED_PIN;
static const unsigned long kTestBlinkMs = 100;
static unsigned long g_test_next_ms = 0;
static bool g_test_level = false;
#endif

#if defined(PINS_TOGGLE)
#if !defined(TEST_BUTTON_PIN)
#error "Define TEST_BUTTON_PIN in platformio.ini build_flags."
#endif
#if !defined(PINS_LEVEL)
#error "Define PINS_LEVEL=HIGH or PINS_LEVEL=LOW in platformio.ini build_flags."
#endif
static const uint8_t kTestButtonPin = TEST_BUTTON_PIN;
static const uint8_t kPinsLevel = PINS_LEVEL;
static const unsigned long kPinsTogglePulseMs = 200;
#endif

static void AddStep(MorseSequence& seq, uint8_t level, uint16_t duration) {
  if (seq.len >= sizeof(seq.durations) / sizeof(seq.durations[0])) {
    return;
  }
  seq.levels[seq.len] = level;
  seq.durations[seq.len] = duration;
  seq.len++;
}

static void AppendChar(MorseSequence& seq, char c) {
  if (c >= 'a' && c <= 'z') {
    c = static_cast<char>(c - 'a' + 'A');
  }
  const char* pattern = MorseForChar(c);
  if (pattern[0] == '\0') {
    return;
  }
  for (size_t p = 0; pattern[p] != '\0'; ++p) {
    AddStep(seq, HIGH, pattern[p] == '-' ? kDashMs : kDotMs);
    if (pattern[p + 1] != '\0') {
      AddStep(seq, LOW, kIntraElementGapMs);
    }
  }
}

static void BuildPinSequence(MorseSequence& seq, uint8_t pin) {
  seq.len = 0;

  char digits[6];
  snprintf(digits, sizeof(digits), "%u", pin);
  // Spell the pin number in Morse as separate digits.
  for (size_t i = 0; digits[i] != '\0'; ++i) {
    AppendChar(seq, digits[i]);
    if (digits[i + 1] != '\0') {
      AddStep(seq, LOW, kInterLetterGapMs);
    }
  }
  AddStep(seq, LOW, kInterWordGapMs);
}

void setup() {
#if defined(BOARD_ESP32S3)
  Serial.begin(115200);
#endif
  const size_t count = sizeof(kGpios) / sizeof(kGpios[0]);
  for (size_t i = 0; i < count; i++) {
    if (!IsValidOutputPin(kGpios[i])) {
      LogInvalidPin(kGpios[i]);
      continue;
    }
    pinMode(kGpios[i], OUTPUT);
    digitalWrite(kGpios[i], LOW);
  }
#if !defined(MORSE) && !defined(PINS_TOGGLE)
#if defined(BOARD_ESP32S3)
  Serial.println("USB CDC test mode active");
#endif
  pinMode(kSenseInPin, INPUT);
  pinMode(kStatusLedPin, OUTPUT);
  digitalWrite(kStatusLedPin, LOW);
#endif
#if defined(MORSE) && defined(TEST_BUTTON)
  pinMode(kTestButtonPin, INPUT_PULLUP);
  pinMode(kTestLedPin, OUTPUT);
#endif
#if defined(PINS_TOGGLE)
  pinMode(kTestButtonPin, INPUT_PULLUP);
  for (size_t i = 0; i < count; i++) {
    if (IsValidOutputPin(kGpios[i])) {
      digitalWrite(kGpios[i], kPinsLevel);
    }
  }
#endif
}

void loop() {
#if !defined(MORSE) && !defined(PINS_TOGGLE)
  // PWM sensing mode: scan outputs, look for the 1 kHz signature on kSenseInPin.
  static int last_pin = -1;
  static uint16_t last_count = 0;
  static bool connected = false;
  int detected = -1;
  const size_t count = sizeof(kGpios) / sizeof(kGpios[0]);

  auto detect_edges = [&](uint8_t pin, unsigned long window_ms) -> bool {
    ledcAttach(pin, kTestToneHz, kPwmResolutionBits);
    ledcWrite(pin, 128);

    unsigned long start_ms = millis();
    int edges = 0;
    int last = digitalRead(kSenseInPin);
    while (static_cast<long>(millis() - start_ms) < static_cast<long>(window_ms)) {
      int v = digitalRead(kSenseInPin);
      if (v != last) {
        edges++;
        last = v;
      }
    }

    ledcWrite(pin, 0);
    ledcDetach(pin);
    return edges >= kMinEdges;
  };

  // Status signaling: E/I/S for confirm passes, O on failure/disconnect.
  auto blink_morse = [&](char c, bool final_gap) {
    const char* pattern = MorseForChar(c);
    for (size_t i = 0; pattern[i] != '\0'; ++i) {
      digitalWrite(kStatusLedPin, HIGH);
      delay(pattern[i] == '-' ? kDashMs : kDotMs);
      digitalWrite(kStatusLedPin, LOW);
      if (pattern[i + 1] != '\0') {
        delay(kIntraElementGapMs);
      }
    }
    if (final_gap) {
      delay(kInterLetterGapMs);
    }
  };

  for (size_t i = 0; i < count; i++) {
    if (!IsValidOutputPin(kGpios[i])) {
      LogInvalidPin(kGpios[i]);
      continue;
    }
    if (detect_edges(kGpios[i], kDetectWindowMs)) {
      detected = kGpios[i];
      break;
    }
  }

  if (detected >= 0) {
    if (connected) {
      return;
    }
    bool ok = true;
    for (int pass = 0; pass < 3; pass++) {
      if (!detect_edges(detected, kConfirmWindowMs)) {
        ok = false;
        break;
      }
      if (pass == 0) {
        blink_morse('E', true);
      } else if (pass == 1) {
        blink_morse('I', true);
      }
    }
    if (ok) {
      if (!connected) {
        if (detected == last_pin) {
          last_count++;
        } else {
          last_pin = detected;
          last_count = 1;
        }
        Serial.print("Connected: GPIO");
        Serial.print(detected);
        if (last_count > 1) {
          Serial.print(" (");
          Serial.print(last_count);
          Serial.print(")");
        }
        Serial.println();
        blink_morse('S', false);
        connected = true;
      }
    } else {
      blink_morse('O', true);
    }
  } else if (connected) {
    blink_morse('O', true);
    connected = false;
  }
  return;
#endif
#if defined(PINS_TOGGLE)
  // Button-stepped mode: each press pulses the current pin opposite to PINS_LEVEL.
  static size_t idx = 0;
  static int last_state = HIGH;
  static unsigned long last_press_ms = 0;
  const unsigned long now_ms = millis();
  const size_t count2 = sizeof(kGpios) / sizeof(kGpios[0]);
  const int state = digitalRead(kTestButtonPin);

  if (state == LOW && last_state == HIGH) {
    if (now_ms - last_press_ms >= 50) {
      if (idx >= count2) {
        idx = 0;
      }
      const uint8_t pin = kGpios[idx];
      if (!IsValidOutputPin(pin)) {
        LogInvalidPin(pin);
        idx++;
      } else {
        const uint8_t pulse_level = (kPinsLevel == HIGH) ? LOW : HIGH;
        Serial.print("Pin ");
        Serial.print(pin);
        Serial.print(" ");
        Serial.print(pulse_level == HIGH ? "HIGH" : "LOW");
        Serial.print(" -> ");
        Serial.println(kPinsLevel == HIGH ? "HIGH" : "LOW");
        digitalWrite(pin, pulse_level);
        delay(kPinsTogglePulseMs);
        digitalWrite(pin, kPinsLevel);
        idx++;
      }
      last_press_ms = now_ms;
    }
  }
  last_state = state;
  return;
#endif
#if defined(MORSE)
  // Morse mode: non-blocking per-pin scheduler.
  static bool initialized = false;
  static MorseSequence sequences[sizeof(kGpios) / sizeof(kGpios[0])];
  static PinState states[sizeof(kGpios) / sizeof(kGpios[0])];

  const size_t count2 = sizeof(kGpios) / sizeof(kGpios[0]);
  if (!initialized) {
    unsigned long init_now = millis();
    for (size_t i = 0; i < count2; i++) {
      BuildPinSequence(sequences[i], kGpios[i]);
      states[i].pin = kGpios[i];
      states[i].idx = 0;
      states[i].next_ms = init_now + sequences[i].durations[0];
      digitalWrite(states[i].pin, sequences[i].levels[0]);
    }
    initialized = true;
  }

  // TEMP stub: drive all pins HIGH, then pull each LOW for 200 ms in sequence.
  static bool all_high = false;
  static size_t idx = 0;
  if (!all_high) {
    for (size_t i = 0; i < count2; i++) {
      if (IsValidOutputPin(kGpios[i])) {
        digitalWrite(kGpios[i], HIGH);
      }
    }
    all_high = true;
  }
  if (idx >= count2) {
    idx = 0;
  }
  if (IsValidOutputPin(kGpios[idx])) {
    digitalWrite(kGpios[idx], LOW);
    delay(200);
    digitalWrite(kGpios[idx], HIGH);
  } else {
    delay(200);
  }
  idx++;
  return;

  unsigned long now_ms = millis();
#if defined(MORSE) && defined(TEST_BUTTON)
  const bool test_pressed = digitalRead(kTestButtonPin) == LOW;
  if (test_pressed) {
    if (static_cast<long>(now_ms - g_test_next_ms) >= 0) {
      g_test_level = !g_test_level;
      digitalWrite(kTestLedPin, g_test_level ? HIGH : LOW);
      g_test_next_ms = now_ms + kTestBlinkMs;
    }
  }
#endif
  for (size_t i = 0; i < count2; i++) {
    if (static_cast<long>(now_ms - states[i].next_ms) >= 0) {
      states[i].idx = static_cast<uint8_t>((states[i].idx + 1) % sequences[i].len);
      if (
#if defined(MORSE) && defined(TEST_BUTTON)
          !(test_pressed && states[i].pin == kTestLedPin) &&
#endif
          true
      ) {
        if (!IsValidOutputPin(states[i].pin)) {
          LogInvalidPin(states[i].pin);
        } else {
        digitalWrite(states[i].pin, sequences[i].levels[states[i].idx]);
        }
      }
      states[i].next_ms = now_ms + sequences[i].durations[states[i].idx];
    }
  }
#if defined(MORSE) && defined(TEST_BUTTON)
  if (!test_pressed) {
    for (size_t i = 0; i < count2; i++) {
      if (states[i].pin == kTestLedPin) {
        digitalWrite(states[i].pin, sequences[i].levels[states[i].idx]);
        break;
      }
    }
  }
#endif
#endif
}
