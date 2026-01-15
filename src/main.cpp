#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// --------- Pins ----------
static const int TDS_PIN = 15;
static const int BTN_PIN = 17;       // button to GND
static const int LED_STRIP_PIN = 23;
static const int LED_COUNT = 191;
static const int LED_BRIGHTNESS = 80;

// --------- Sampling ----------
static const int SAMPLES = 30;
static const int SAMPLE_DELAY_MS = 5;
static const float TEMP_C = 25.0f;
static const float LP_ALPHA = 0.25f;
static const uint32_t TDS_PERIOD_MS = 250;

// --------- Button behavior ----------
static const uint32_t DEBOUNCE_MS = 40;
static const uint32_t LONG_PRESS_MS = 2000;

// --------- Values ----------
static float offsetPpm = 0.0f;
static float scaleK = 1.0f;
static float filteredPpm = 0.0f;

Adafruit_NeoPixel strip(LED_COUNT, LED_STRIP_PIN, NEO_GRBW + NEO_KHZ800);

// --------- UI state machine ----------
enum UiState { STATE_RESET_IDLE = 0, STATE_LOADING, STATE_SHOW_TDS };
static UiState uiState = STATE_RESET_IDLE;
static uint32_t stateStartMs = 0;

// --------- Timing ----------
static uint32_t lastTdsMs = 0;

// --------- Button debouncer ----------
struct Button {
  int pin;
  bool stablePressed = false;      // debounced stable state
  bool lastReading = false;        // last raw reading
  uint32_t lastChangeMs = 0;       // last time raw changed
  uint32_t pressStartMs = 0;       // time when stable pressed started
  bool longFired = false;

  void begin(int p) {
    pin = p;
    pinMode(pin, INPUT_PULLUP);
    bool raw = (digitalRead(pin) == LOW);
    stablePressed = raw;
    lastReading = raw;
    lastChangeMs = millis();
    pressStartMs = stablePressed ? millis() : 0;
    longFired = false;
  }

  void update(bool &shortPress, bool &longPress) {
    shortPress = false;
    longPress  = false;

    uint32_t now = millis();
    bool raw = (digitalRead(pin) == LOW);

    if (raw != lastReading) {
      lastReading = raw;
      lastChangeMs = now;
    }

    if ((now - lastChangeMs) < DEBOUNCE_MS) return;

    if (raw != stablePressed) {
      stablePressed = raw;

      if (stablePressed) {
        pressStartMs = now;
        longFired = false;
      } else {
        if (!longFired) shortPress = true;
      }
    }

    if (stablePressed && !longFired) {
      if ((now - pressStartMs) >= LONG_PRESS_MS) {
        longPress = true;
        longFired = true;
      }
    }
  }
};

static Button btn;

// ----- TDS math -----
static float tdsFromVoltage(float v, float tempC)
{
  float compensation = 1.0f + 0.02f * (tempC - 25.0f);
  float vComp = v / compensation;
  float tds = (133.42f * vComp * vComp * vComp - 255.86f * vComp * vComp + 857.39f * vComp) * 0.5f;
  return tds;
}

static float readVoltageAveraged()
{
  uint32_t mvSum = 0;
  for (int i = 0; i < SAMPLES; i++) {
    mvSum += analogReadMilliVolts(TDS_PIN);
    delay(SAMPLE_DELAY_MS);
  }
  float mv = (float)mvSum / (float)SAMPLES;
  return mv / 1000.0f;
}

static float lowpass(float prev, float x, float alpha)
{
  return prev + alpha * (x - prev);
}

// ----- Color mapping -----
static void hsvToRgb(float h, float s, float v, uint8_t &r, uint8_t &g, uint8_t &b)
{
  float c = v * s;
  float hh = fmodf(h / 60.0f, 6.0f);
  float x = c * (1 - fabsf(fmodf(hh, 2.0f) - 1));
  float m = v - c;
  float rf=0, gf=0, bf=0;

  if (hh < 1) { rf = c; gf = x; bf = 0; }
  else if (hh < 2) { rf = x; gf = c; bf = 0; }
  else if (hh < 3) { rf = 0; gf = c; bf = x; }
  else if (hh < 4) { rf = 0; gf = x; bf = c; }
  else if (hh < 5) { rf = x; gf = 0; bf = c; }
  else { rf = c; gf = 0; bf = x; }

  r = (uint8_t)roundf((rf + m) * 255.0f);
  g = (uint8_t)roundf((gf + m) * 255.0f);
  b = (uint8_t)roundf((bf + m) * 255.0f);
}

static void ppmToColor(float ppm, uint8_t &r, uint8_t &g, uint8_t &b, uint8_t &w)
{
  float minPpm = 100.0f;
  float maxPpm = 420.0f;

  if (ppm < minPpm) ppm = minPpm;
  if (ppm > maxPpm) ppm = maxPpm;

  float t = (ppm - minPpm) / (maxPpm - minPpm);

  float hue = (t < 0.5f)
                ? (160.0f + (60.0f - 160.0f) * (t / 0.5f))
                : (60.0f + (0.0f - 60.0f) * ((t - 0.5f) / 0.5f));

  hsvToRgb(hue, 1.0f, 1.0f, r, g, b);
  w = 0;
}

// ----- LEDs -----
static void clearStrip()
{
  strip.clear();
  strip.show();
}

static void clearValues()
{
  filteredPpm = 0.0f;
  offsetPpm = 0.0f;
  scaleK = 1.0f;
}

static void animateResetIdlePulse()
{
  // light blue pulse forever
  const uint8_t baseR = 0;
  const uint8_t baseG = 70;
  const uint8_t baseB = 140;
  const uint8_t baseW = 0;

  uint32_t now = millis();
  float t = (float)(now % 1400UL) / 1400.0f;
  float s = sinf(t * 3.1415926f);
  float k = 0.10f + 0.90f * s;

  uint8_t r = (uint8_t)(baseR * k);
  uint8_t g = (uint8_t)(baseG * k);
  uint8_t b = (uint8_t)(baseB * k);
  uint8_t w = (uint8_t)(baseW * k);

  for (int i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, strip.Color(r, g, b, w));
  }
  strip.show();
}

// White spiral loader (W channel)
static void animateLoading(uint32_t elapsedMs)
{
  const int trail = 18;
  const uint32_t stepMs = 25;
  int head = (int)(elapsedMs / stepMs);

  strip.clear();

  for (int t = 0; t < trail; t++) {
    int idx = head - t;
    if (idx < 0) break;
    if (idx >= LED_COUNT) continue;

    float k = 1.0f - (float)t / (float)trail;
    uint8_t w = (uint8_t)(255.0f * k);          // white brightness
    strip.setPixelColor(idx, strip.Color(0, 0, 0, w));
  }

  strip.show();
}

static void renderTdsColorBar(float ppm)
{
  uint8_t r, g, b, w;
  ppmToColor(ppm, r, g, b, w);

  for (int i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, strip.Color(r, g, b, w));
  }
  strip.show();
}

static void updateTdsIfDue()
{
  uint32_t now = millis();
  if ((now - lastTdsMs) < TDS_PERIOD_MS) return;
  lastTdsMs = now;

  float v = readVoltageAveraged();
  float rawPpm = tdsFromVoltage(v, TEMP_C);
  if (rawPpm < 0) rawPpm = 0;

  float ppm = (rawPpm * scaleK) - offsetPpm;
  if (ppm < 0) ppm = 0;

  filteredPpm = lowpass(filteredPpm, ppm, LP_ALPHA);

  Serial.print("BTN=");
  Serial.print(btn.stablePressed ? "DOWN" : "UP");
  Serial.print("  V=");
  Serial.print(v, 3);
  Serial.print("  raw=");
  Serial.print(rawPpm, 1);
  Serial.print("  filtered=");
  Serial.println(filteredPpm, 1);

  renderTdsColorBar(filteredPpm);
}

void setup()
{
  Serial.begin(115200);
  delay(200);

  analogSetPinAttenuation(TDS_PIN, ADC_11db);

  btn.begin(BTN_PIN);

  strip.begin();
  strip.setBrightness(LED_BRIGHTNESS);
  clearStrip();

  clearValues();
  uiState = STATE_RESET_IDLE;
  stateStartMs = millis();

  Serial.println();
  Serial.println("ESP32 TDS started");
}

void loop()
{
  bool shortPress = false, longPress = false;
  btn.update(shortPress, longPress);

  if (shortPress) Serial.println("SHORT");
  if (longPress)  Serial.println("LONG");

  if (longPress) {
    clearValues();
    uiState = STATE_RESET_IDLE;
    stateStartMs = millis();
  }

  uint32_t now = millis();
  uint32_t elapsed = now - stateStartMs;

  if (uiState == STATE_RESET_IDLE) {
    animateResetIdlePulse();

    if (shortPress) {
      uiState = STATE_LOADING;
      stateStartMs = now;
      clearStrip();
    }
    return;
  }

  if (uiState == STATE_LOADING) {
    const uint32_t loadingTotalMs = 4000;  // changed to 4000 ms
    animateLoading(elapsed);

    if (elapsed >= loadingTotalMs) {
      uiState = STATE_SHOW_TDS;
      stateStartMs = now;
      clearStrip();
      lastTdsMs = 0; // force immediate reading
    }
    return;
  }

  // STATE_SHOW_TDS
  if (shortPress) {
    uiState = STATE_LOADING;
    stateStartMs = now;
    clearStrip();
    return;
  }

  updateTdsIfDue();
}