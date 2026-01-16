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
};
#elif defined(BOARD_ESP32S3)
// ESP32-S3 safe GPIOs (avoid strapping, USB, and UART0 pins by default).
// Adjust this list for your specific S3 module as needed.
static const uint8_t kGpios[] = {
    4, 5, 6, 7, 8, 9, 10, 11,
    12, 13, 14, 15, 16, 18,
    21,
    35, 36, 37, 38, 39, 40, 41, 42
};
#else
#error "Define a BOARD_* build flag to select the GPIO list."
#endif

static const unsigned long kDashMs = 500;
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

static bool IsValidOutputPin(uint8_t pin) {
  return digitalPinCanOutput(pin);
}

static void LogInvalidPin(uint8_t pin) {
  static unsigned long last_log_ms = 0;
  unsigned long now = millis();
  if (now - last_log_ms >= kErrorLogRateMs) {
#if defined(BOARD_ESP32S3)
    Serial.print("Skipping invalid output pin: GPIO");
    Serial.println(pin);
#endif
    last_log_ms = now;
  }
}

#if defined(BOARD_ESP32S3) && defined(UART_TEST_MODE)
static const uint8_t kUartTestInPin = 44;
static const unsigned long kScanSettleMs = kDotMs ? kDotMs : 1;
static const unsigned long kScanHoldMs = kDotMs ? kDotMs : 1;
#endif

#if defined(BOARD_ESP32S3) && defined(TEST_MODE)
static const uint8_t kTestButtonPin = 17;
static const uint8_t kTestLedPin = 40;
static const unsigned long kTestBlinkMs = 100;
static unsigned long g_test_next_ms = 0;
static bool g_test_level = false;
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
#if defined(BOARD_ESP32S3) && defined(UART_TEST_MODE)
#if defined(BOARD_ESP32S3)
  Serial.println("USB CDC test mode active");
#endif
  pinMode(kUartTestInPin, INPUT_PULLDOWN);
#endif
#if defined(BOARD_ESP32S3) && defined(TEST_MODE)
  pinMode(kTestButtonPin, INPUT_PULLUP);
  pinMode(kTestLedPin, OUTPUT);
#endif
}

void loop() {
#if defined(BOARD_ESP32S3) && defined(UART_TEST_MODE)
  static int last_detected = -1;
  static unsigned long last_report_ms = 0;
  int detected = -1;
  unsigned long now = millis();

  for (size_t i = 0; i < sizeof(kGpios) / sizeof(kGpios[0]); i++) {
    if (IsValidOutputPin(kGpios[i])) {
      digitalWrite(kGpios[i], LOW);
    }
  }
  for (size_t i = 0; i < sizeof(kGpios) / sizeof(kGpios[0]); i++) {
    if (!IsValidOutputPin(kGpios[i])) {
      LogInvalidPin(kGpios[i]);
      continue;
    }
    digitalWrite(kGpios[i], HIGH);
    delay(kScanSettleMs);
    if (digitalRead(kUartTestInPin) == HIGH) {
      detected = kGpios[i];
      digitalWrite(kGpios[i], LOW);
      break;
    }
    delay(kScanHoldMs);
    digitalWrite(kGpios[i], LOW);
  }

  const bool changed = detected != last_detected;
  if (changed || (now - last_report_ms) >= 1000) {
    if (detected >= 0) {
      Serial.print("Connected: GPIO");
      Serial.println(detected);
    } else {
      Serial.println("Connected: none");
    }
    last_detected = detected;
    last_report_ms = now;
  }
  return;
#endif
  static bool initialized = false;
  static MorseSequence sequences[sizeof(kGpios) / sizeof(kGpios[0])];
  static PinState states[sizeof(kGpios) / sizeof(kGpios[0])];

  const size_t count = sizeof(kGpios) / sizeof(kGpios[0]);
  if (!initialized) {
    unsigned long init_now = millis();
    for (size_t i = 0; i < count; i++) {
      BuildPinSequence(sequences[i], kGpios[i]);
      states[i].pin = kGpios[i];
      states[i].idx = 0;
      states[i].next_ms = init_now + sequences[i].durations[0];
      digitalWrite(states[i].pin, sequences[i].levels[0]);
    }
    initialized = true;
  }

  unsigned long now = millis();
#if defined(BOARD_ESP32S3) && defined(TEST_MODE)
  const bool test_pressed = digitalRead(kTestButtonPin) == LOW;
  if (test_pressed) {
    if (static_cast<long>(now - g_test_next_ms) >= 0) {
      g_test_level = !g_test_level;
      digitalWrite(kTestLedPin, g_test_level ? HIGH : LOW);
      g_test_next_ms = now + kTestBlinkMs;
    }
  }
#endif
  for (size_t i = 0; i < count; i++) {
    if (static_cast<long>(now - states[i].next_ms) >= 0) {
      states[i].idx = static_cast<uint8_t>((states[i].idx + 1) % sequences[i].len);
      if (
#if defined(BOARD_ESP32S3) && defined(TEST_MODE)
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
      states[i].next_ms = now + sequences[i].durations[states[i].idx];
    }
  }
#if defined(BOARD_ESP32S3) && defined(TEST_MODE)
  if (!test_pressed) {
    for (size_t i = 0; i < count; i++) {
      if (states[i].pin == kTestLedPin) {
        digitalWrite(states[i].pin, sequences[i].levels[states[i].idx]);
        break;
      }
    }
  }
#endif
}
