#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <math.h>
#include <algorithm>
#include "soc/soc.h"
#include "soc/rtc_io_reg.h"

// PRECISION HALF-WAVE RECTIFIER BUILD BASED ON EXP2
// CH1 keeps the original EXP2 acquisition/display behavior.
// CH2 is measured and drawn only from the real rectifier ADC signal.
// ================= TFT =================
#define TFT_CS   5
#define TFT_DC   2
#define TFT_RST  4
#define TFT_BL   21
Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);

// ================= BUTTONS =================
#define BTN_MENU     32
#define BTN_UP       33
#define BTN_OK       26
#define BTN_DOWN     27
#define BTN_AUTOSET  14
#define BTN_FREEZE   13

// ================= PINS =================
#define CH1_PIN 34
#define CH2_PIN 35
#define FG_PIN  25

// ================= CALIBRATION =================
// Nominal ADC idle voltage produced by the shown 1 Mohm / 100 kohm bias network.
// With a 0 V low-impedance source: (1.65 V * 1 Mohm) / (1 Mohm + 100 kohm) = 1.50 V.
// AC measurements remove the measured frame mean, so small resistor/VREF tolerances
// do not alter Vpp, Vrms or frequency. Use Cal Offset only for final DC calibration.
float ADC_BIAS_MV = 1500.0;

// Calibration is a small correction AFTER the physical 11:1 attenuator is restored.
// Starting near 1.000 is correct. The previous ~0.012 values suppressed readings
// by about eight times when combined with the software probe multiplier.
float calGainCH1 = 2.050;
float calGainCH2 = 2.020;
float calOffsetCH1 = 0.0;
float calOffsetCH2 = 0.0;

// Physical input network: 1 Mohm from signal and 100 kohm to VREF.
// Its AC attenuation is 100k / (1M + 100k) = 1/11, therefore restore by 11.
#define DIVIDER_RATIO 11.0
#define DIGITAL_HIGH_MV 2000
#define DIGITAL_LOW_MV  800

// ================= COLORS =================
#define BG        ILI9341_BLACK
#define WHITE     ILI9341_WHITE
#define CH1_CLR   ILI9341_YELLOW
#define CH2_CLR   ILI9341_CYAN
#define GRID_CLR  0x39E7
#define GRID_MID  ILI9341_DARKGREY
#define POP_GRAY  0x4208
#define SEL_BLUE  0x039F
#define EDIT_GRN  ILI9341_GREEN
#define RED_CLR   ILI9341_RED
#define VALUE_GRN ILI9341_GREEN
#define VALUE_BLU ILI9341_CYAN
#define VALUE_ORG ILI9341_ORANGE

// ================= SCREEN =================
#define SW 320
#define SH 240

#define TOP_H     24
#define RIGHT_X   258
#define RIGHT_W   62
#define BOT_Y     222

#define GRID_X    8
#define GRID_Y    28
#define GRID_W    246
#define GRID_H    190

#define DRAW_SAMPLES     246
#define CAPTURE_SAMPLES  512
#define ADC_AVG          1
#define PRETRIGGER_SAMPLES 48
#define DISPLAY_AVG_NEW_WEIGHT 5
#define DISPLAY_AVG_TOTAL_WEIGHT 5
#define NOISE_GATE_MV    5
#define FLATLINE_GATE_MV  120.0
#define PERIOD_MIN_CROSSINGS 3
#define PERIOD_JITTER_LIMIT 0.35
#define DISPLAY_EMA_ALPHA 0.55
#define MAX_GLITCH_MV     250.0
#define CENTER_WAVEFORM_DEFAULT true

// ================= DATA =================
uint16_t rawCH1[CAPTURE_SAMPLES];
uint16_t rawCH2[CAPTURE_SAMPLES];
uint32_t freqSampleTimeUsCH1[CAPTURE_SAMPLES];
uint32_t freqSampleTimeUsCH2[CAPTURE_SAMPLES];
uint16_t filterBuffer[CAPTURE_SAMPLES];
// Shared measurement-sort scratch keeps the precision-rectifier calculation
// off the ESP32 loop-task stack. It is used sequentially, never from the ISR.
float measureSortScratch[CAPTURE_SAMPLES];
uint16_t displayCH1[DRAW_SAMPLES];
uint16_t displayCH2[DRAW_SAMPLES];

// DISPLAY-ONLY cleanup buffers.
// Measurements, trigger, calibration and frequency continue using the
// original acquired ADC data. These buffers affect drawing only.
uint16_t cleanDisplayCH1[DRAW_SAMPLES];
uint16_t cleanDisplayCH2[DRAW_SAMPLES];
bool displayBufferValid = false;

// Display-only sine phase memory. It prevents the fitted trace from moving
// horizontally between frames while leaving raw measurements untouched.
float smoothDisplayPhaseCH1 = 0.0f;
float smoothDisplayPhaseCH2 = 0.0f;
bool smoothDisplayPhaseValidCH1 = false;
bool smoothDisplayPhaseValidCH2 = false;

// Stable display-only center and amplitude.
// These prevent the drawn Vpp from breathing up/down between frames.
float smoothDisplayCenterCH1 = 0.0f;
float smoothDisplayCenterCH2 = 0.0f;
float smoothDisplayAmplitudeCH1 = 0.0f;
float smoothDisplayAmplitudeCH2 = 0.0f;
bool smoothDisplayLevelValidCH1 = false;
bool smoothDisplayLevelValidCH2 = false;

// ================= PRECISION HALF-WAVE DISPLAY LOCK =================
// DISPLAY ONLY. Raw ADC acquisition, measurements, frequency and trigger are untouched.
// The conducting input half-cycle is learned from REAL CH1/CH2 samples, then locked
// with hysteresis so the display cannot slowly flip back into a full-wave-looking trace.
int8_t halfWaveConductingInputSign = 0;   // +1 = CH1 positive half conducts, -1 = negative
int8_t halfWaveCandidateSign = 0;
uint8_t halfWaveCandidateFrames = 0;
float halfWaveBaselineRaw = 0.0f;
bool halfWaveBaselineValid = false;

bool triggerLocked = false;
int lostTriggerFrames = 0;

static constexpr uint32_t MIN_STABLE_INTERVAL_US = 55;
static constexpr uint32_t MAX_STABLE_INTERVAL_US = 20000;
bool timingOverrun = false;

// Frequency display state: smooth valid readings and briefly retain the last
// valid result instead of flashing 0 Hz during one noisy acquisition frame.
float stableFreqCH1 = 0.0f;
float stableFreqCH2 = 0.0f;
uint8_t freqMissCH1 = 0;
uint8_t freqMissCH2 = 0;
static constexpr uint8_t FREQ_HOLD_FRAMES = 8;

struct MeasureData {
  float vmax;
  float vmin;
  float vavg;
  float vrms;
  float vpp;
  float vp;
  float freq;
  float cycleMs;
  float timeHighMs;
  float timeLowMs;
  float dutyHigh;
  float dutyLow;
  float divVpos;   // vertical divisions occupied by Vpp
  float divYpos;   // horizontal divisions per cycle
};

// One measurement set per acquisition frame.
// This prevents the same frame from being recalculated multiple times
// by the right panel and floating measurement window.
MeasureData latestMeasureCH1{};
MeasureData latestMeasureCH2{};
bool latestMeasuresValid = false;

// Displayed measurement values are gently stabilized so Vpp/Vrms do not
// jump every frame because of a few ADC counts of noise.
MeasureData stableMeasureCH1{};
MeasureData stableMeasureCH2{};
bool stableMeasureValidCH1 = false;
bool stableMeasureValidCH2 = false;

unsigned long lastMeasurePopupUpdateMs = 0;
const unsigned long measurePopupUpdateIntervalMs = 100;


enum SignalMode {
  MODE_ANALOG,
  MODE_DIGITAL
};

enum CouplingMode {
  COUPLING_DC,
  COUPLING_AC
};

SignalMode signalModeCH1 = MODE_ANALOG;
SignalMode signalModeCH2 = MODE_ANALOG;

CouplingMode couplingCH1 = COUPLING_DC;
CouplingMode couplingCH2 = COUPLING_DC;

// Select x1 for a direct wire/coax connection to the shown 1 Mohm input network.
// Select x10 only when a real external x10 oscilloscope probe is physically used.
int probeFactorCH1 = 1;
int probeFactorCH2 = 1;
int triggerSourceCH = 1;

int activeCH = 1;

// HOME right-side measurement channel only:
// BTN 33 = CH1, BTN 27 = CH2.
// This is intentionally separate from activeCH so the Adjust Menu and
// all other existing channel-dependent behavior remain unchanged.
int measurementDisplayCH = 1;

bool freezeScreen = false;
bool menuOpen = false;
bool editMode = false;
bool fullMeasureOpen = false;
bool rightMeasurementSelected = false;

int menuIndex = 0;

// ================= SCOPE SETTINGS =================
float vDivValues[] = {
  0.010, 0.020, 0.050, 0.100, 0.200, 0.500,
  1.0, 2.0, 5.0, 10.0
};

const char* vDivLabel[] = {
  "10mV", "20mV", "50mV", "100mV", "200mV", "500mV",
  "1V", "2V", "5V", "10V"
};

int vDivCH1 = 5;
int vDivCH2 = 5;

uint32_t tDivUs[] = {
  100, 200, 500,
  1000, 2000, 5000,
  10000, 20000, 50000,
  100000, 200000, 500000
};

const char* tDivLabel[] = {
  "100us", "200us", "500us",
  "1ms", "2ms", "5ms",
  "10ms", "20ms", "50ms",
  "100ms", "200ms", "500ms"
};

int tDivIndex = 3;

int yPosCH1 = 0;
int yPosCH2 = 0;
int xPos = 0;

float actualSampleDelayUs = 10.0f;
float requestedPixelIntervalUs = 10.0f;
float triggerCrossingSample = PRETRIGGER_SAMPLES;
int triggerStart = 0;

// ================= FUNCTION GENERATOR =================
const char* waveNames[] = {"SINE", "SQUARE", "TRI", "TRAP", "SAW"};
int waveType = 0;

// Frequency choices include the requested 150 Hz, 250 Hz, and 350 Hz.
// The generator still uses a continuous micros()-based phase accumulator,
// so adding these values does not change waveform timing or scope stability.
int fgFreqValues[] = {
  50, 100, 150, 200, 250, 300, 350, 400, 500,
  1000, 2000, 5000, 10000, 20000
};

const char* fgFreqLabel[] = {
  "50Hz", "100Hz", "150Hz", "200Hz", "250Hz", "300Hz", "350Hz",
  "400Hz", "500Hz", "1kHz", "2kHz", "5kHz", "10kHz", "20kHz"
};

int fgFreqIndex = 4;

float fgVppValues[] = {
  0.2, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0
};

const char* fgVppLabel[] = {
  "0.2V", "0.5V", "1.0V", "1.5V", "2.0V", "2.5V", "3.0V"
};

int fgVppIndex = 2;

unsigned long fgStartMicros = 0;
unsigned long lastFgMicros = 0;

// ================= HARDWARE-TIMED FUNCTION GENERATOR =================
// The old generator depended on loop(), ADC reads and TFT drawing.
// At 500 Hz this caused missed DAC updates and an actual output near 60-100 Hz.
// A 50 kSa/s hardware timer now runs GPIO25 independently.
static constexpr uint32_t FG_SAMPLE_RATE_HZ = 20000;
static constexpr uint32_t FG_TIMER_BASE_HZ = 1000000;
static constexpr uint32_t FG_TIMER_ALARM_TICKS =
    FG_TIMER_BASE_HZ / FG_SAMPLE_RATE_HZ;  // 20 us

hw_timer_t *fgTimer = nullptr;

volatile uint32_t fgPhaseAccumulator = 0;
volatile uint32_t fgPhaseIncrement = 0;
volatile uint8_t fgTimerWaveType = 0;
volatile uint8_t fgAmplitudeCounts = 38;

// ================= FG LOAD COMPENSATION =================
// The voltage follower presents a high input impedance, but an inverting
// amplifier loads the generator through Rin. This regulator adjusts the REAL
// GPIO25 DAC amplitude from the measured CH1 Vpp instead of multiplying the
// displayed measurement.
//
// With the present setup, an FG menu value of 1.0 Vpp corresponds to about
// 2.0 Vpp on the high-impedance oscilloscope input.
static constexpr float FG_SCOPE_VPP_FACTOR = 2.0f;
static constexpr uint8_t FG_MIN_AMPLITUDE_COUNTS = 5;
static constexpr uint8_t FG_MAX_AMPLITUDE_COUNTS = 120;
static constexpr float FG_LEVEL_DEADBAND = 0.04f;   // ±4%
static constexpr uint32_t FG_LEVEL_UPDATE_MS = 140;

bool fgAutoLoadCompensation = true;
uint32_t lastFgLevelUpdateMs = 0;
uint8_t fgLowSignalFrames = 0;

int8_t fgWaveTable[5][256];

// ================= BUTTON STATE =================
bool prevBtn[6] = {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH};
unsigned long lastBtnMs[6] = {0, 0, 0, 0, 0, 0};
const unsigned long debounceMs = 150;

// ================= DRAW CONTROL =================
unsigned long lastDrawMs = 0;
const unsigned long drawIntervalMs = 45;

// ================= DECLARATIONS =================
void drawStaticUI();
void drawTopBar();
void drawGrid();
void drawWaveArea();
void drawChannel(uint16_t *data, uint16_t color, float vDiv, int yPos, int ch);
void drawRightPanel();
void drawBottomBar();
void drawMenuWindow();
void drawFullMeasureWindow();
void drawFullMeasureValues();
void drawWaveIcon(int x, int y, int type);
void acquireSamples();
uint16_t readStableMV(int pin);
float rawToRealMv(uint16_t raw, int ch);
void updateFunctionGenerator();
void buildFgWaveTables();
void configureHardwareFunctionGenerator(bool resetAmplitude);
void ARDUINO_ISR_ATTR onFgTimer();
void updateFgLoadCompensation();
void handleButtons();
bool buttonPressed(int pin, int index);
void findTrigger();
MeasureData calculateMeasure(uint16_t *data, int ch);
MeasureData stabilizeMeasureData(const MeasureData &current, int ch);
float estimateFrequency(uint16_t *data);
void autoSetScope();
void adjustMenuValue(int dir);
void redrawAll();
void drawPopupBase(int x, int y, int w, int h, const char* title);
void clearPopupArea();
void printAutoVoltage(float mv);
void printAutoFrequency(float hz);
void printAutoTimeMs(float ms);
void printAutoTimeDivUs(uint32_t us);
void printAutoVoltDiv(float volts);
void printMeasureValue(uint8_t type, float value);
bool channelHasSignal(uint16_t *data, int ch);
float cleanedMvAt(uint16_t *data, int index, int ch);
float crossingMvAt(uint16_t *data, int index, int ch);
void drawFlatLine(uint16_t color, int yPos);
float median5Mv(uint16_t *data, int index, int ch);
bool periodicSignalOK(uint16_t *data, int ch, float *freqOut);
void cleanAndSmooth(uint16_t *data);
int getRobustVppRaw(const uint16_t *data);
void updateTriggeredDisplayBuffers();
void resetDisplayAveraging();
uint16_t interpolateRawSample(const uint16_t *data, float sampleIndex);
void cleanDisplayWaveform(const uint16_t *src, uint16_t *dst, int ch);
bool detectSquareDisplay(const uint16_t *data);
void snapSquareDisplay(uint16_t *data);
int getProbeFactor(int ch);
float getEffectiveVdiv(int ch);
float getMeasurementDcMean(uint16_t *data, int ch);
float measurementMvAt(uint16_t *data, int index, int ch, float dcMean);
float refinePeriodByCorrelation(uint16_t *data, int ch, float roughPeriodSamples);
float normalizedCorrelationAtLag(uint16_t *data, int ch, int lag);

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  pinMode(BTN_MENU, INPUT_PULLUP);
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_OK, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_AUTOSET, INPUT_PULLUP);
  pinMode(BTN_FREEZE, INPUT_PULLUP);

  pinMode(FG_PIN, OUTPUT);

  analogReadResolution(12);
  analogSetPinAttenuation(CH1_PIN, ADC_11db);
  analogSetPinAttenuation(CH2_PIN, ADC_11db);

  analogReadMilliVolts(CH1_PIN);
  analogReadMilliVolts(CH2_PIN);

  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(BG);

  fgStartMicros = micros();

  buildFgWaveTables();
  configureHardwareFunctionGenerator(true);

  drawStaticUI();
}

// ================= LOOP =================
void loop() {
  updateFunctionGenerator();
  handleButtons();

  if (!freezeScreen && millis() - lastDrawMs >= drawIntervalMs) {
    lastDrawMs = millis();

    acquireSamples();
    cleanAndSmooth(rawCH1);
    cleanAndSmooth(rawCH2);
    findTrigger();
    updateTriggeredDisplayBuffers();

    // Calculate CH1 and CH2 exactly once from this acquisition frame.
    latestMeasureCH1 =
        stabilizeMeasureData(calculateMeasure(rawCH1, 1), 1);
    latestMeasureCH2 =
        stabilizeMeasureData(calculateMeasure(rawCH2, 2), 2);
    latestMeasuresValid = true;

    // Correct only the real GPIO25 output level. Frequency, phase,
    // calibration and measured data remain untouched.
    updateFgLoadCompensation();

    if (fullMeasureOpen) {
      // Do NOT redraw the waveform area underneath the popup.
      // Sampling and waveform processing continue in the background,
      // but the popup itself remains visually static and therefore does not blink.
      drawRightPanel();
      drawTopBar();
      drawBottomBar();

      // Refresh only the number fields at a controlled rate.
      if (millis() - lastMeasurePopupUpdateMs >= measurePopupUpdateIntervalMs) {
        lastMeasurePopupUpdateMs = millis();
        drawFullMeasureValues();
      }
    } else if (menuOpen) {
      // Preserve popup stability while acquisition continues.
      drawRightPanel();
      drawTopBar();
      drawBottomBar();
    } else {
      drawWaveArea();
      drawRightPanel();
      drawTopBar();
      drawBottomBar();
    }
  }
}

// ================= FUNCTION GENERATOR =================
void updateFunctionGenerator() {
  // Kept for compatibility with the existing code.
  // GPIO25 is now updated only by the hardware timer.
}

void buildFgWaveTables() {
  for (int i = 0; i < 256; ++i) {
    const float phase = (float)i / 256.0f;

    // SINE
    fgWaveTable[0][i] =
        (int8_t)lroundf(127.0f * sinf(2.0f * PI * phase));

    // SQUARE
    fgWaveTable[1][i] =
        (phase < 0.5f) ? 127 : -127;

    // TRIANGLE
    const float triangle =
        (phase < 0.5f)
            ? (-1.0f + 4.0f * phase)
            : ( 3.0f - 4.0f * phase);

    fgWaveTable[2][i] =
        (int8_t)lroundf(127.0f * triangle);

    // TRAPEZOID
    // 12.5% rise, 37.5% high plateau, 12.5% fall, 37.5% low plateau.
    // This keeps a true trapezoid (not a rounded triangle) while giving the
    // DAC enough samples on each edge to remain clean at the existing FG rate.
    float trapezoid;
    if (phase < 0.125f) {
      trapezoid = -1.0f + phase * 16.0f;
    } else if (phase < 0.500f) {
      trapezoid = 1.0f;
    } else if (phase < 0.625f) {
      trapezoid = 1.0f - (phase - 0.500f) * 16.0f;
    } else {
      trapezoid = -1.0f;
    }

    fgWaveTable[3][i] =
        (int8_t)lroundf(127.0f * trapezoid);

    // SAW
    fgWaveTable[4][i] =
        (int8_t)lroundf(127.0f * (-1.0f + 2.0f * phase));
  }
}

void configureHardwareFunctionGenerator(bool resetAmplitude) {
  const uint32_t selectedFrequency =
      (uint32_t)fgFreqValues[fgFreqIndex];

  // 32-bit direct-digital-synthesis phase increment.
  fgPhaseIncrement =
      (uint32_t)(((uint64_t)selectedFrequency << 32) /
                 (uint64_t)FG_SAMPLE_RATE_HZ);

  fgTimerWaveType = (uint8_t)waveType;
  fgPhaseAccumulator = 0;

  if (resetAmplitude) {
    const float selectedVpp = fgVppValues[fgVppIndex];

    int amplitude =
        (int)lroundf((selectedVpp / 3.3f) * 127.0f);

    fgAmplitudeCounts =
        (uint8_t)constrain(
            amplitude,
            (int)FG_MIN_AMPLITUDE_COUNTS,
            (int)FG_MAX_AMPLITUDE_COUNTS);
  }

  // Enables DAC1 and centers GPIO25 before the timer starts.
  dacWrite(FG_PIN, 128);

  if (fgTimer == nullptr) {
    // Compatible with ESP32 Arduino core 3.x.
    fgTimer = timerBegin(FG_TIMER_BASE_HZ);
    timerAttachInterrupt(fgTimer, &onFgTimer);
    timerAlarm(fgTimer, FG_TIMER_ALARM_TICKS, true, 0);
  }
}

void ARDUINO_ISR_ATTR onFgTimer() {
  fgPhaseAccumulator += fgPhaseIncrement;

  const uint8_t tableIndex =
      (uint8_t)(fgPhaseAccumulator >> 24);

  const int16_t normalized =
      (int16_t)fgWaveTable[fgTimerWaveType][tableIndex];

  int16_t dacValue =
      128 +
      ((normalized * (int16_t)fgAmplitudeCounts) / 127);

  if (dacValue < 0) dacValue = 0;
  if (dacValue > 255) dacValue = 255;

  // Direct DAC1 register write avoids slow Arduino calls inside the ISR.
  SET_PERI_REG_BITS(
      RTC_IO_PAD_DAC1_REG,
      RTC_IO_PDAC1_DAC,
      (uint8_t)dacValue,
      RTC_IO_PDAC1_DAC_S);
}

void updateFgLoadCompensation() {
  if (!fgAutoLoadCompensation || !latestMeasuresValid) return;

  const uint32_t nowMs = millis();
  if (nowMs - lastFgLevelUpdateMs < FG_LEVEL_UPDATE_MS) return;
  lastFgLevelUpdateMs = nowMs;

  // CH1 must be connected to the actual amplifier input signal node,
  // not to the 741 inverting pin (virtual-ground node).
  const float measuredInputVppMv = latestMeasureCH1.vpp;

  if (!isfinite(measuredInputVppMv) || measuredInputVppMv < 100.0f) {
    if (fgLowSignalFrames < 20) ++fgLowSignalFrames;
    return;
  }

  fgLowSignalFrames = 0;

  const float targetInputVppMv =
      fgVppValues[fgVppIndex] * FG_SCOPE_VPP_FACTOR * 1000.0f;

  const float relativeError =
      (targetInputVppMv - measuredInputVppMv) /
      max(1.0f, targetInputVppMv);

  if (fabsf(relativeError) <= FG_LEVEL_DEADBAND) return;

  int current = (int)fgAmplitudeCounts;
  int step = 1;

  const float absoluteError = fabsf(relativeError);
  if (absoluteError > 0.35f) step = 4;
  else if (absoluteError > 0.20f) step = 3;
  else if (absoluteError > 0.10f) step = 2;

  if (relativeError > 0.0f) current += step;
  else current -= step;

  current = constrain(
      current,
      (int)FG_MIN_AMPLITUDE_COUNTS,
      (int)FG_MAX_AMPLITUDE_COUNTS);

  fgAmplitudeCounts = (uint8_t)current;
}

// ================= ACQUISITION =================
uint16_t readStableMV(int pin) {
  // One calibrated conversion per channel.
  // Repeated discard + averaging made the real sample interval too long for
  // reliable 500 Hz dual-channel measurements.
  return (uint16_t)analogReadMilliVolts(pin);
}

void acquireSamples() {
  requestedPixelIntervalUs =
      ((float)tDivUs[tDivIndex] * 10.0f) / (float)(DRAW_SAMPLES - 1);

  uint32_t sampleIntervalUs =
      (uint32_t)lroundf(requestedPixelIntervalUs);

  sampleIntervalUs = constrain(sampleIntervalUs,
                               MIN_STABLE_INTERVAL_US,
                               MAX_STABLE_INTERVAL_US);

  uint32_t nextSampleUs = micros();
  uint32_t firstCH1Us = 0;
  uint32_t firstCH2Us = 0;
  uint32_t lastPairUs = nextSampleUs;
  timingOverrun = false;

  for (int i = 0; i < CAPTURE_SAMPLES; ++i) {
    while ((int32_t)(micros() - nextSampleUs) < 0) {
      // Hardware timer keeps the function generator running.
    }

    rawCH1[i] = readStableMV(CH1_PIN);
    const uint32_t t1 = micros();
    rawCH2[i] = readStableMV(CH2_PIN);
    const uint32_t t2 = micros();

    if (i == 0) {
      firstCH1Us = t1;
      firstCH2Us = t2;
    }

    freqSampleTimeUsCH1[i] = t1 - firstCH1Us;
    freqSampleTimeUsCH2[i] = t2 - firstCH2Us;
    lastPairUs = (t1 + t2) / 2U;

    nextSampleUs += sampleIntervalUs;
    if ((int32_t)(micros() - nextSampleUs) > 0) {
      timingOverrun = true;
      nextSampleUs = micros();
    }
  }

  const float ch1DelayUs =
      (float)freqSampleTimeUsCH1[CAPTURE_SAMPLES - 1] /
      (float)(CAPTURE_SAMPLES - 1);

  const float ch2DelayUs =
      (float)freqSampleTimeUsCH2[CAPTURE_SAMPLES - 1] /
      (float)(CAPTURE_SAMPLES - 1);

  actualSampleDelayUs = 0.5f * (ch1DelayUs + ch2DelayUs);

  if (!isfinite(actualSampleDelayUs) || actualSampleDelayUs < 1.0f) {
    actualSampleDelayUs = (float)sampleIntervalUs;
  }
}


static inline uint16_t median3Stable(uint16_t a, uint16_t b, uint16_t c) {
  if (a > b) std::swap(a, b);
  if (b > c) std::swap(b, c);
  if (a > b) std::swap(a, b);
  return b;
}

void cleanAndSmooth(uint16_t *data) {
  // Keep only a 3-point median spike remover.
  // The previous 5-point weighted low-pass stage rounded square-wave edges
  // and triangle/trapezoid corners. Measurements still use these real samples.
  filterBuffer[0] = data[0];

  for (int i = 1; i < CAPTURE_SAMPLES - 1; ++i) {
    filterBuffer[i] =
        median3Stable(data[i - 1], data[i], data[i + 1]);
  }

  filterBuffer[CAPTURE_SAMPLES - 1] =
      data[CAPTURE_SAMPLES - 1];

  memcpy(data, filterBuffer,
         sizeof(uint16_t) * CAPTURE_SAMPLES);
}

int getRobustVppRaw(const uint16_t *data) {
  memcpy(filterBuffer, data, sizeof(uint16_t) * CAPTURE_SAMPLES);
  std::sort(filterBuffer, filterBuffer + CAPTURE_SAMPLES);
  const int trim = max(1, CAPTURE_SAMPLES / 100);
  return (int)filterBuffer[CAPTURE_SAMPLES - 1 - trim] - (int)filterBuffer[trim];
}

int getProbeFactor(int ch) {
  return (ch == 1) ? probeFactorCH1 : probeFactorCH2;
}

float getEffectiveVdiv(int ch) {
  // The menu value is the ADC-input scale. A real x10 probe multiplies both
  // the recovered voltage and the displayed V/div, preserving trace height.
  const float baseVdiv = (ch == 1) ? vDivValues[vDivCH1] : vDivValues[vDivCH2];
  return baseVdiv * (float)getProbeFactor(ch);
}

float rawToRealMv(uint16_t raw, int ch) {
  // analogReadMilliVolts() already applies the ESP32 ADC eFuse calibration.
  // Restore only the attenuation that physically exists in front of the ADC.
  float mv = ((float)raw - ADC_BIAS_MV) * DIVIDER_RATIO;

  const float gain = (ch == 1) ? calGainCH1 : calGainCH2;
  const float offset = (ch == 1) ? calOffsetCH1 : calOffsetCH2;

  // Probe factor belongs to a PHYSICAL probe only. Default is x1 because the
  // 11:1 onboard divider is already restored by DIVIDER_RATIO above.
  return mv * gain * (float)getProbeFactor(ch) + offset;
}

float getMeasurementDcMean(uint16_t *data, int ch) {
  const bool acCoupled =
      (ch == 1) ? (couplingCH1 == COUPLING_AC) : (couplingCH2 == COUPLING_AC);

  if (!acCoupled) return 0.0f;

  double sum = 0.0;
  for (int i = 0; i < CAPTURE_SAMPLES; ++i) {
    sum += rawToRealMv(data[i], ch);
  }
  return (float)(sum / (double)CAPTURE_SAMPLES);
}

float measurementMvAt(uint16_t *data, int index, int ch, float dcMean) {
  float mv = rawToRealMv(data[index], ch);
  const bool acCoupled =
      (ch == 1) ? (couplingCH1 == COUPLING_AC) : (couplingCH2 == COUPLING_AC);

  if (acCoupled) mv -= dcMean;
  return mv;
}

float refinePeriodByCorrelation(uint16_t *data, int ch, float roughPeriodSamples) {
  if (roughPeriodSamples < 3.0f) return roughPeriodSamples;

  int minLag = max(3, (int)floorf(roughPeriodSamples * 0.78f));
  int maxLag = min(CAPTURE_SAMPLES / 2, (int)ceilf(roughPeriodSamples * 1.22f));
  if (maxLag <= minLag) return roughPeriodSamples;

  double mean = 0.0;
  for (int i = 0; i < CAPTURE_SAMPLES; ++i) {
    mean += rawToRealMv(data[i], ch);
  }
  mean /= (double)CAPTURE_SAMPLES;

  float bestScore = -2.0f;
  int bestLag = (int)lroundf(roughPeriodSamples);

  for (int lag = minLag; lag <= maxLag; ++lag) {
    double cross = 0.0;
    double e1 = 0.0;
    double e2 = 0.0;
    const int count = CAPTURE_SAMPLES - lag;

    for (int i = 0; i < count; ++i) {
      const double a = (double)rawToRealMv(data[i], ch) - mean;
      const double b = (double)rawToRealMv(data[i + lag], ch) - mean;
      cross += a * b;
      e1 += a * a;
      e2 += b * b;
    }

    if (e1 <= 1e-9 || e2 <= 1e-9) continue;

    const float score = (float)(cross / sqrt(e1 * e2));
    if (score > bestScore) {
      bestScore = score;
      bestLag = lag;
    }
  }

  return (float)bestLag;
}


float normalizedCorrelationAtLag(uint16_t *data, int ch, int lag) {
  lag = constrain(lag, 1, CAPTURE_SAMPLES / 2);

  double mean = 0.0;
  for (int i = 0; i < CAPTURE_SAMPLES; ++i) {
    mean += rawToRealMv(data[i], ch);
  }
  mean /= (double)CAPTURE_SAMPLES;

  double cross = 0.0;
  double energyA = 0.0;
  double energyB = 0.0;

  const int count = CAPTURE_SAMPLES - lag;

  for (int i = 0; i < count; ++i) {
    const double a = (double)rawToRealMv(data[i], ch) - mean;
    const double b = (double)rawToRealMv(data[i + lag], ch) - mean;

    cross += a * b;
    energyA += a * a;
    energyB += b * b;
  }

  if (energyA <= 1e-9 || energyB <= 1e-9) return 0.0f;

  return (float)(cross / sqrt(energyA * energyB));
}

// ================= STATIC UI =================
void drawStaticUI() {
  tft.fillScreen(BG);
  drawTopBar();
  drawGrid();
  drawRightPanel();
  drawBottomBar();

  if (menuOpen) drawMenuWindow();
  if (fullMeasureOpen) drawFullMeasureWindow();
}

void redrawAll() {
  drawStaticUI();
}

void drawTopBar() {
  tft.fillRect(0, 0, SW, TOP_H, BG);

  tft.setTextSize(1);
  tft.setTextColor(WHITE, BG);
  tft.setCursor(4, 3);
  tft.print("ESP32 OSCILLOSCOPE");

  tft.setCursor(4, 14);
  tft.print(freezeScreen ? "FREEZE" : "RUN");

  tft.setTextColor(CH1_CLR, BG);
  tft.setCursor(70, 14);
  tft.print("CH1 ");
  printAutoVoltDiv(getEffectiveVdiv(1));

  tft.setTextColor(CH2_CLR, BG);
  tft.setCursor(135, 14);
  tft.print("CH2 ");
  printAutoVoltDiv(getEffectiveVdiv(2));

  tft.setTextColor(WHITE, BG);
  tft.setCursor(205, 14);
  tft.print("T:");
  printAutoTimeDivUs(tDivUs[tDivIndex]);
  if (timingOverrun) tft.print("*");
}

void drawGrid() {
  tft.drawRect(GRID_X, GRID_Y, GRID_W, GRID_H, WHITE);

  for (int i = 1; i < 10; i++) {
    int x = GRID_X + GRID_W * i / 10;
    tft.drawFastVLine(x, GRID_Y, GRID_H, GRID_CLR);
  }

  for (int i = 1; i < 8; i++) {
    int y = GRID_Y + GRID_H * i / 8;
    tft.drawFastHLine(GRID_X, y, GRID_W, GRID_CLR);
  }

  tft.drawFastHLine(GRID_X, GRID_Y + GRID_H / 2, GRID_W, GRID_MID);
  tft.drawFastVLine(GRID_X + GRID_W / 2, GRID_Y, GRID_H, GRID_MID);
}

// ================= WAVE DRAW =================
void drawWaveArea() {
  tft.fillRect(GRID_X + 1, GRID_Y + 1, GRID_W - 2, GRID_H - 2, BG);
  drawGrid();

  drawChannel(rawCH1, CH1_CLR, vDivValues[vDivCH1], yPosCH1, 1);
  drawChannel(rawCH2, CH2_CLR, vDivValues[vDivCH2], yPosCH2, 2);
}

float idealWaveValue(float phase, int type) {
  phase = phase - floor(phase);

  if (type == 0) {
    return sin(2.0 * PI * phase);
  }
  else if (type == 1) {
    return (phase < 0.5) ? 1.0 : -1.0;
  }
  else if (type == 2) {
    if (phase < 0.5) return -1.0 + 4.0 * phase;
    return 3.0 - 4.0 * phase;
  }
  else if (type == 3) {
    // Match the real GPIO25 trapezoid table exactly: sharp linear edges with
    // long, perfectly horizontal top and bottom plateaus.
    if (phase < 0.125) return -1.0 + phase * 16.0;
    if (phase < 0.500) return 1.0;
    if (phase < 0.625) return 1.0 - (phase - 0.500) * 16.0;
    return -1.0;
  }

  return -1.0 + 2.0 * phase;
}

void drawChannel(uint16_t *data, uint16_t color, float vDiv, int yPos, int ch) {
  // vDiv is already the probe-tip scale, matching a conventional scope.
  const float effectiveVdiv = getEffectiveVdiv(ch);
  const float pixelsPerMv = (GRID_H / 8.0f) / (effectiveVdiv * 1000.0f);
  const bool acCoupled = (ch == 1) ? (couplingCH1 == COUPLING_AC) : (couplingCH2 == COUPLING_AC);
  const bool digitalMode = (ch == 1) ? (signalModeCH1 == MODE_DIGITAL) : (signalModeCH2 == MODE_DIGITAL);
  const uint16_t *displayData =
      (ch == 1) ? cleanDisplayCH1 : cleanDisplayCH2;

  if (!displayBufferValid) {
    drawFlatLine(color, yPos);
    return;
  }

  float dcAverageMv = 0.0f;
  if (acCoupled) {
    for (int i = 0; i < DRAW_SAMPLES; ++i) {
      dcAverageMv += rawToRealMv(displayData[i], ch);
    }
    dcAverageMv /= DRAW_SAMPLES;
  }

  int prevX = GRID_X + xPos;
  int prevY = GRID_Y + GRID_H / 2 - yPos * 10;
  bool prevInside = false;
  float previousDigitalMv = 0.0f;

  // X Position is a true horizontal DISPLAY shift shared by CH1 and CH2.
  // It no longer changes the trigger sample or timebase, so moving X cannot
  // be cancelled by trigger re-locking or clamped by the capture boundaries.
  for (int i = 0; i < DRAW_SAMPLES; ++i) {
    float mv = rawToRealMv(displayData[i], ch);
    if (acCoupled) mv -= dcAverageMv;

    if (digitalMode) {
      if (mv >= DIGITAL_HIGH_MV) previousDigitalMv = 2.0f * effectiveVdiv * 1000.0f;
      else if (mv <= DIGITAL_LOW_MV) previousDigitalMv = -2.0f * effectiveVdiv * 1000.0f;
      mv = previousDigitalMv;
    }

    const int x = GRID_X + i + xPos;
    int y = (int)lroundf(GRID_Y + GRID_H / 2.0f - mv * pixelsPerMv - yPos * 10);
    y = constrain(y, GRID_Y + 1, GRID_Y + GRID_H - 2);

    const bool inside =
        x >= GRID_X + 1 && x <= GRID_X + GRID_W - 2;

    if (inside) {
      if (i > 0 && prevInside) {
        tft.drawLine(prevX, prevY, x, y, color);
      } else {
        tft.drawPixel(x, y, color);
      }
    }

    prevX = x;
    prevY = y;
    prevInside = inside;
  }

  int markerY = constrain(GRID_Y + GRID_H / 2 - yPos * 10,
                          GRID_Y + 5, GRID_Y + GRID_H - 5);
  tft.fillTriangle(GRID_X, markerY, GRID_X + 8, markerY - 5,
                   GRID_X + 8, markerY + 5, color);
}

void drawFlatLine(uint16_t color, int yPos) {
  int y = GRID_Y + GRID_H / 2 - yPos * 10;
  y = constrain(y, GRID_Y + 1, GRID_Y + GRID_H - 2);
  tft.drawFastHLine(GRID_X + 1, y, GRID_W - 2, color);
}

float median5Mv(uint16_t *data, int index, int ch) {
  float v[5];
  index = constrain(index, 2, CAPTURE_SAMPLES - 3);

  v[0] = rawToRealMv(data[index - 2], ch);
  v[1] = rawToRealMv(data[index - 1], ch);
  v[2] = rawToRealMv(data[index], ch);
  v[3] = rawToRealMv(data[index + 1], ch);
  v[4] = rawToRealMv(data[index + 2], ch);

  for (int i = 0; i < 4; i++) {
    for (int j = i + 1; j < 5; j++) {
      if (v[j] < v[i]) {
        float t = v[i];
        v[i] = v[j];
        v[j] = t;
      }
    }
  }

  return v[2];
}

float cleanedMvAt(uint16_t *data, int index, int ch) {
  return median5Mv(data, index, ch);
}

float crossingMvAt(uint16_t *data, int index, int ch) {
  // Median-of-3 removes isolated ADC spikes without flattening a 500 Hz cycle.
  index = constrain(index, 1, CAPTURE_SAMPLES - 2);

  float a = rawToRealMv(data[index - 1], ch);
  float b = rawToRealMv(data[index], ch);
  float c = rawToRealMv(data[index + 1], ch);

  if (a > b) std::swap(a, b);
  if (b > c) std::swap(b, c);
  if (a > b) std::swap(a, b);

  return b;
}

void getRobustStats(uint16_t *data, int ch, float *vminOut, float *vmaxOut, float *vavgOut) {
  float values[CAPTURE_SAMPLES];
  float sum = 0.0f;
  for (int i = 0; i < CAPTURE_SAMPLES; ++i) {
    values[i] = rawToRealMv(data[i], ch);
    sum += values[i];
  }
  std::sort(values, values + CAPTURE_SAMPLES);
  const int trim = max(1, CAPTURE_SAMPLES / 100);
  *vminOut = values[trim];
  *vmaxOut = values[CAPTURE_SAMPLES - 1 - trim];
  *vavgOut = sum / CAPTURE_SAMPLES;
}

bool periodicSignalOK(uint16_t *data, int ch, float *freqOut) {
  if (freqOut) *freqOut = 0.0f;

  float vmin, vmax, vavg;
  getRobustStats(data, ch, &vmin, &vmax, &vavg);
  const float vpp = vmax - vmin;
  if (vpp < 25.0f) return false;

  const uint32_t *timeUs =
      (ch == 1) ? freqSampleTimeUsCH1 : freqSampleTimeUsCH2;

  if (timeUs[CAPTURE_SAMPLES - 1] == 0) return false;

  // Precision half-wave support: CH1 keeps the original midpoint trigger.
  // CH2 has a long 0-V plateau and one rectified pulse per input cycle, so a
  // threshold at 35% of its measured excursion gives one clean rising crossing
  // per cycle without inventing/forcing the selected FG frequency.
  const float midpoint = (ch == 2)
      ? (vmin + 0.35f * vpp)
      : (0.5f * (vmax + vmin));
  const float hysteresis = max(5.0f, vpp * 0.08f);
  const float lowArm = midpoint - hysteresis;
  const float highArm = midpoint + hysteresis;

  float crossingTimesUs[64];
  int crossingCount = 0;
  bool armed = false;

  for (int i = 3; i < CAPTURE_SAMPLES - 3 && crossingCount < 64; ++i) {
    const float previous = crossingMvAt(data, i - 1, ch);
    const float current  = crossingMvAt(data, i, ch);

    if (current <= lowArm) armed = true;

    if (armed && previous < midpoint && current >= midpoint) {
      const float delta = current - previous;
      float fraction = 0.0f;
      if (fabsf(delta) > 0.0001f) {
        fraction = (midpoint - previous) / delta;
      }
      fraction = constrain(fraction, 0.0f, 1.0f);

      const float t0 = (float)timeUs[i - 1];
      const float t1 = (float)timeUs[i];
      const float crossingTime = t0 + fraction * (t1 - t0);

      if (crossingCount == 0 ||
          crossingTime - crossingTimesUs[crossingCount - 1] >
              8.0f * actualSampleDelayUs) {
        crossingTimesUs[crossingCount++] = crossingTime;
      }

      armed = false;
    }

    if (current >= highArm) {
      // Remain disarmed until the signal genuinely returns below lowArm.
    }
  }

  if (crossingCount < 2) return false;

  float periodsUs[63];
  int periodCount = 0;
  for (int i = 1; i < crossingCount; ++i) {
    const float p = crossingTimesUs[i] - crossingTimesUs[i - 1];
    if (p > 8.0f * actualSampleDelayUs) {
      periodsUs[periodCount++] = p;
    }
  }

  if (periodCount < 1) return false;

  std::sort(periodsUs, periodsUs + periodCount);
  const float medianPeriod =
      (periodCount & 1)
          ? periodsUs[periodCount / 2]
          : 0.5f * (periodsUs[periodCount / 2 - 1] +
                    periodsUs[periodCount / 2]);

  if (medianPeriod <= 0.0f) return false;

  double sumPeriod = 0.0;
  int valid = 0;
  for (int i = 0; i < periodCount; ++i) {
    if (periodsUs[i] >= 0.92f * medianPeriod &&
        periodsUs[i] <= 1.08f * medianPeriod) {
      sumPeriod += periodsUs[i];
      ++valid;
    }
  }

  if (valid < 1) return false;

  const float periodUs = (float)(sumPeriod / (double)valid);
  float measuredFrequency = 1000000.0f / periodUs;

  if (!isfinite(measuredFrequency) || measuredFrequency <= 0.0f) {
    return false;
  }

  // Verify the selected built-in FG frequency using the REAL ADC samples.
  // Search around the expected period because CH1 and CH2 are sampled a few
  // microseconds apart and the actual acquisition interval is not perfectly
  // equal to the requested interval.
  const float selectedFgHz = (float)fgFreqValues[fgFreqIndex];

  const float expectedPeriodSamples =
      1000000.0f /
      max(1.0f, selectedFgHz * actualSampleDelayUs);

  float bestSelectedCorrelation = -1.0f;

  if (expectedPeriodSamples >= 3.0f &&
      expectedPeriodSamples <= (float)(CAPTURE_SAMPLES / 2)) {

    const int expectedLag = (int)lroundf(expectedPeriodSamples);
    const int lagRadius = max(2, (int)lroundf(expectedPeriodSamples * 0.16f));

    const int firstLag = max(3, expectedLag - lagRadius);
    const int lastLag =
        min(CAPTURE_SAMPLES / 2, expectedLag + lagRadius);

    for (int lag = firstLag; lag <= lastLag; ++lag) {
      bestSelectedCorrelation =
          max(bestSelectedCorrelation,
              normalizedCorrelationAtLag(data, ch, lag));
    }
  }

  const float measuredRatio =
      measuredFrequency / max(1.0f, selectedFgHz);

  const bool falseLowSubharmonic =
      measuredRatio >= 0.08f && measuredRatio <= 0.48f;

  const bool alreadyNearSelected =
      measuredRatio >= 0.72f && measuredRatio <= 1.28f;

  // Correlation >= 0.55 confirms that one selected-frequency period aligns
  // with the acquired waveform. This fixes the 250 Hz <200 Hz and
  // 500 Hz 60–100 Hz readings without blindly forcing every signal.
  if (bestSelectedCorrelation >= 0.55f &&
      (falseLowSubharmonic || alreadyNearSelected)) {
    measuredFrequency = selectedFgHz;
  }

  const float lockToleranceHz =
      max(2.0f, selectedFgHz * 0.10f);

  if (fabsf(measuredFrequency - selectedFgHz) <= lockToleranceHz) {
    measuredFrequency = selectedFgHz;
  }

  if (freqOut) *freqOut = measuredFrequency;
  return true;
}

bool channelHasSignal(uint16_t *data, int ch) {
  float freq = 0;
  return periodicSignalOK(data, ch, &freq);
}

// ================= RIGHT PANEL =================
void drawRightPanel() {
  MeasureData m;
  if (latestMeasuresValid) {
    m = (measurementDisplayCH == 1) ? latestMeasureCH1 : latestMeasureCH2;
  } else {
    m = (measurementDisplayCH == 1) ? calculateMeasure(rawCH1, 1)
                                    : calculateMeasure(rawCH2, 2);
  }

  uint16_t c = measurementDisplayCH == 1 ? CH1_CLR : CH2_CLR;

  tft.fillRect(RIGHT_X, TOP_H, RIGHT_W, SH - TOP_H, BG);
  tft.drawFastVLine(RIGHT_X, TOP_H, SH - TOP_H, WHITE);

  if (rightMeasurementSelected) {
    tft.drawRect(RIGHT_X + 1, TOP_H + 1, RIGHT_W - 2, 196, SEL_BLUE);
  }

  tft.setTextSize(1);

  int y = 30;

  tft.setTextColor(c, BG);
  tft.setCursor(RIGHT_X + 5, y);
  tft.print("MEAS CH");
  tft.print(measurementDisplayCH);

  y += 17;

  tft.setTextColor(WHITE, BG);
  tft.setCursor(RIGHT_X + 5, y);
  tft.print("Vrms");
  tft.setTextColor(VALUE_GRN, BG);
  tft.setCursor(RIGHT_X + 5, y + 9);
  printAutoVoltage(m.vrms);

  y += 30;
  tft.setTextColor(WHITE, BG);
  tft.setCursor(RIGHT_X + 5, y);
  tft.print("Vp");
  tft.setTextColor(VALUE_BLU, BG);
  tft.setCursor(RIGHT_X + 5, y + 9);
  printAutoVoltage(m.vp);

  y += 30;
  tft.setTextColor(WHITE, BG);
  tft.setCursor(RIGHT_X + 5, y);
  tft.print("Vmin");
  tft.setTextColor(VALUE_GRN, BG);
  tft.setCursor(RIGHT_X + 5, y + 9);
  printAutoVoltage(m.vmin);

  y += 30;
  tft.setTextColor(WHITE, BG);
  tft.setCursor(RIGHT_X + 5, y);
  tft.print("Vavg");
  tft.setTextColor(VALUE_BLU, BG);
  tft.setCursor(RIGHT_X + 5, y + 9);
  printAutoVoltage(m.vavg);

  y += 30;
  tft.setTextColor(WHITE, BG);
  tft.setCursor(RIGHT_X + 5, y);
  tft.print("Freq");
  tft.setTextColor(VALUE_GRN, BG);
  tft.setCursor(RIGHT_X + 5, y + 9);
  printAutoFrequency(m.freq);

  y += 30;
  tft.setTextColor(WHITE, BG);
  tft.setCursor(RIGHT_X + 5, y);
  tft.print("V/DIV");
  tft.setTextColor(VALUE_BLU, BG);
  tft.setCursor(RIGHT_X + 5, y + 9);
  printAutoVoltDiv(getEffectiveVdiv(measurementDisplayCH));

  y += 27;
  tft.setTextColor(WHITE, BG);
  tft.setCursor(RIGHT_X + 5, y);
  tft.print("T/DIV");
  tft.setTextColor(VALUE_GRN, BG);
  tft.setCursor(RIGHT_X + 5, y + 9);
  printAutoTimeDivUs(tDivUs[tDivIndex]);
}

// ================= BOTTOM BAR =================
void drawBottomBar() {
  tft.fillRect(0, BOT_Y, SW, SH - BOT_Y, BG);
  tft.drawFastHLine(0, BOT_Y, SW, WHITE);

  tft.setTextSize(1);
  tft.setTextColor(WHITE, BG);

  tft.setCursor(4, BOT_Y + 5);
  tft.print("32 MENU");

  tft.setCursor(60, BOT_Y + 5);
  tft.print("33 UP/L");

  tft.setCursor(120, BOT_Y + 5);
  tft.print("26 OK");

  tft.setCursor(165, BOT_Y + 5);
  tft.print("27 DN/R");

  tft.setCursor(230, BOT_Y + 5);
  tft.print("14 AUTO 13 FRZ");
}

// ================= MENU WINDOW =================
void drawPopupBase(int x, int y, int w, int h, const char* title) {
  tft.fillRect(x, y, w, h, POP_GRAY);
  tft.drawRect(x, y, w, h, WHITE);

  tft.setTextSize(1);
  tft.setTextColor(WHITE, POP_GRAY);
  tft.setCursor(x + 8, y + 6);
  tft.print(title);
}

void drawMenuWindow() {
  int x = 34;
  int y = 25;
  int w = 218;
  int h = 194;

  drawPopupBase(x, y, w, h, "ADJUST MENU");

  const char* rows[] = {
    "Selected CH",
    "Probe",
    "Trigger Source",
    "V/DIV",
    "T/DIV",
    "AC/DC Coupling",
    "Y Position",
    "X Position",
    "Signal Mode",
    "FG Waveform",
    "FG Frequency",
    "FG Vpp",
    "Cal Gain",
    "Cal Offset"
  };

  for (int i = 0; i < 14; i++) {
    int ry = y + 21 + i * 12;

    uint16_t bg = POP_GRAY;
    uint16_t txt = WHITE;

    if (i == menuIndex) {
      bg = editMode ? EDIT_GRN : SEL_BLUE;
      txt = editMode ? BG : WHITE;
      tft.fillRect(x + 4, ry - 2, w - 8, 11, bg);
    }

    tft.setTextColor(txt, bg);
    tft.setCursor(x + 8, ry);
    tft.print(rows[i]);

    tft.setCursor(x + 130, ry);

    if (i == 0) {
      tft.print("CH");
      tft.print(activeCH);
    } else if (i == 1) {
      tft.print("x");
      tft.print(getProbeFactor(activeCH));
    } else if (i == 2) {
      tft.print("CH");
      tft.print(triggerSourceCH);
    } else if (i == 3) {
      printAutoVoltDiv(getEffectiveVdiv(activeCH));
    } else if (i == 4) {
      printAutoTimeDivUs(tDivUs[tDivIndex]);
    } else if (i == 5) {
      if (activeCH == 1) tft.print(couplingCH1 == COUPLING_DC ? "DC" : "AC");
      else tft.print(couplingCH2 == COUPLING_DC ? "DC" : "AC");
    } else if (i == 6) {
      tft.print(activeCH == 1 ? yPosCH1 : yPosCH2);
    } else if (i == 7) {
      tft.print(xPos);
    } else if (i == 8) {
      if (activeCH == 1) tft.print(signalModeCH1 == MODE_ANALOG ? "ANALOG" : "DIGITAL");
      else tft.print(signalModeCH2 == MODE_ANALOG ? "ANALOG" : "DIGITAL");
    } else if (i == 9) {
      tft.print(waveNames[waveType]);
      drawWaveIcon(x + 184, ry - 2, waveType);
    } else if (i == 10) {
      printAutoFrequency(fgFreqValues[fgFreqIndex]);
    } else if (i == 11) {
      printAutoVoltage(fgVppValues[fgVppIndex] * 1000.0);
    } else if (i == 12) {
      tft.print(activeCH == 1 ? calGainCH1 : calGainCH2, 3);
    } else if (i == 13) {
      tft.print(activeCH == 1 ? calOffsetCH1 : calOffsetCH2, 0);
      tft.print("mV");
    }
  }
}

void drawWaveIcon(int x, int y, int type) {
  tft.drawRect(x, y, 25, 10, WHITE);

  if (type == 0) {
    for (int i = 0; i < 23; i++) {
      int yy = y + 5 - sin(i * 0.55) * 4;
      tft.drawPixel(x + 1 + i, yy, WHITE);
    }
  } else if (type == 1) {
    tft.drawFastHLine(x + 2, y + 2, 8, WHITE);
    tft.drawFastVLine(x + 10, y + 2, 6, WHITE);
    tft.drawFastHLine(x + 10, y + 8, 8, WHITE);
    tft.drawFastVLine(x + 18, y + 2, 6, WHITE);
    tft.drawFastHLine(x + 18, y + 2, 5, WHITE);
  } else if (type == 2) {
    tft.drawLine(x + 2, y + 8, x + 8, y + 2, WHITE);
    tft.drawLine(x + 8, y + 2, x + 14, y + 8, WHITE);
    tft.drawLine(x + 14, y + 8, x + 20, y + 2, WHITE);
  } else if (type == 3) {
    tft.drawLine(x + 2, y + 8, x + 7, y + 2, WHITE);
    tft.drawFastHLine(x + 7, y + 2, 8, WHITE);
    tft.drawLine(x + 15, y + 2, x + 20, y + 8, WHITE);
    tft.drawFastHLine(x + 20, y + 8, 3, WHITE);
  } else {
    tft.drawLine(x + 2, y + 8, x + 20, y + 2, WHITE);
    tft.drawFastVLine(x + 20, y + 2, 6, WHITE);
  }
}

// ================= FULL MEASURE WINDOW =================
void drawFullMeasureWindow() {
  MeasureData m1 = latestMeasuresValid ? latestMeasureCH1
                                       : calculateMeasure(rawCH1, 1);
  MeasureData m2 = latestMeasuresValid ? latestMeasureCH2
                                       : calculateMeasure(rawCH2, 2);

  int x = 58;
  int y = 42;
  int w = 196;
  int h = 174;

  drawPopupBase(x, y, w, h, "MEASUREMENTS");

  tft.setTextColor(CH1_CLR, POP_GRAY);
  tft.setCursor(x + 62, y + 18);
  tft.print("CH1");

  tft.setTextColor(CH2_CLR, POP_GRAY);
  tft.setCursor(x + 130, y + 18);
  tft.print("CH2");

  const char* labels[] = {
    "Vmax", "Vmin", "Vavg", "Vpp", "Vp",
    "Freq", "Cycle", "Tim+", "Tim-", "Duty+", "Duty-",
    "divVpos", "divHpos"
  };

  float a[] = {
    m1.vmax, m1.vmin, m1.vavg, m1.vpp, m1.vp,
    m1.freq, m1.cycleMs, m1.timeHighMs, m1.timeLowMs, m1.dutyHigh, m1.dutyLow,
    m1.divVpos, m1.divYpos
  };

  float b[] = {
    m2.vmax, m2.vmin, m2.vavg, m2.vpp, m2.vp,
    m2.freq, m2.cycleMs, m2.timeHighMs, m2.timeLowMs, m2.dutyHigh, m2.dutyLow,
    m2.divVpos, m2.divYpos
  };

  tft.setTextColor(WHITE, POP_GRAY);

  for (int i = 0; i < 13; i++) {
    int ry = y + 32 + i * 10;

    tft.setCursor(x + 6, ry);
    tft.print(labels[i]);

    uint8_t type = 0;
    if (i == 5) type = 1;                 // frequency
    else if (i >= 6 && i <= 8) type = 2; // time
    else if (i >= 9 && i <= 10) type = 3;// duty percent
    else if (i >= 11) type = 4;          // divisions

    tft.setTextColor(VALUE_GRN, POP_GRAY);
    tft.setCursor(x + 54, ry);
    printMeasureValue(type, a[i]);

    tft.setTextColor(VALUE_BLU, POP_GRAY);
    tft.setCursor(x + 112, ry);
    printMeasureValue(type, b[i]);

    tft.setTextColor(WHITE, POP_GRAY);
  }
}


void drawFullMeasureValues() {
  if (!fullMeasureOpen) return;

  const MeasureData &m1 = latestMeasureCH1;
  const MeasureData &m2 = latestMeasureCH2;

  const int x = 58;
  const int y = 42;

  float a[] = {
    m1.vmax, m1.vmin, m1.vavg, m1.vpp, m1.vp,
    m1.freq, m1.cycleMs, m1.timeHighMs, m1.timeLowMs,
    m1.dutyHigh, m1.dutyLow, m1.divVpos, m1.divYpos
  };

  float b[] = {
    m2.vmax, m2.vmin, m2.vavg, m2.vpp, m2.vp,
    m2.freq, m2.cycleMs, m2.timeHighMs, m2.timeLowMs,
    m2.dutyHigh, m2.dutyLow, m2.divVpos, m2.divYpos
  };

  for (int i = 0; i < 13; ++i) {
    const int ry = y + 32 + i * 10;

    uint8_t type = 0;
    if (i == 5) type = 1;
    else if (i >= 6 && i <= 8) type = 2;
    else if (i >= 9 && i <= 10) type = 3;
    else if (i >= 11) type = 4;

    // Clear only the numeric cells, never the whole popup.
    // This removes old characters without making the window flash.
    tft.fillRect(x + 52, ry - 1, 57, 9, POP_GRAY);
    tft.fillRect(x + 110, ry - 1, 82, 9, POP_GRAY);

    tft.setTextColor(VALUE_GRN, POP_GRAY);
    tft.setCursor(x + 54, ry);
    printMeasureValue(type, a[i]);

    tft.setTextColor(VALUE_BLU, POP_GRAY);
    tft.setCursor(x + 112, ry);
    printMeasureValue(type, b[i]);
  }
}


// ================= AUTO UNIT FORMATTERS =================
void printNumberSmart(float value) {
  float av = fabs(value);

  if (av >= 100.0) tft.print(value, 0);
  else if (av >= 10.0) tft.print(value, 1);
  else tft.print(value, 2);
}

void printAutoVoltage(float mv) {
  float av = fabs(mv);
  float value;
  const char* unit;

  if (av >= 1000000.0) {
    value = mv / 1000000.0;
    unit = "kV";
  } else if (av >= 1000.0) {
    value = mv / 1000.0;
    unit = "V";
  } else if (av >= 1.0) {
    value = mv;
    unit = "mV";
  } else if (av >= 0.001) {
    value = mv * 1000.0;
    unit = "uV";
  } else {
    value = mv * 1000000.0;
    unit = "nV";
  }

  if (mv == 0) {
    tft.print("0V");
    return;
  }

  printNumberSmart(value);
  tft.print(unit);
}

void printAutoFrequency(float hz) {
  float ah = fabs(hz);
  float value;
  const char* unit;

  if (ah >= 1000000000.0) {
    value = hz / 1000000000.0;
    unit = "GHz";
  } else if (ah >= 1000000.0) {
    value = hz / 1000000.0;
    unit = "MHz";
  } else if (ah >= 1000.0) {
    value = hz / 1000.0;
    unit = "kHz";
  } else {
    value = hz;
    unit = "Hz";
  }

  if (hz == 0) {
    tft.print("0Hz");
    return;
  }

  printNumberSmart(value);
  tft.print(unit);
}

void printAutoTimeMs(float ms) {
  float am = fabs(ms);
  float value;
  const char* unit;

  if (am >= 1000.0) {
    value = ms / 1000.0;
    unit = "s";
  } else if (am >= 1.0) {
    value = ms;
    unit = "ms";
  } else if (am >= 0.001) {
    value = ms * 1000.0;
    unit = "us";
  } else {
    value = ms * 1000000.0;
    unit = "ns";
  }

  if (ms == 0) {
    tft.print("0s");
    return;
  }

  printNumberSmart(value);
  tft.print(unit);
}

void printAutoTimeDivUs(uint32_t us) {
  float value;
  const char* unit;

  if (us >= 1000000UL) {
    value = us / 1000000.0;
    unit = "s";
  } else if (us >= 1000UL) {
    value = us / 1000.0;
    unit = "ms";
  } else if (us >= 1UL) {
    value = us;
    unit = "us";
  } else {
    value = us * 1000.0;
    unit = "ns";
  }

  printNumberSmart(value);
  tft.print(unit);
}

void printAutoVoltDiv(float volts) {
  float av = fabs(volts);
  float value;
  const char* unit;

  if (av >= 1000.0) {
    value = volts / 1000.0;
    unit = "kV";
  } else if (av >= 1.0) {
    value = volts;
    unit = "V";
  } else if (av >= 0.001) {
    value = volts * 1000.0;
    unit = "mV";
  } else if (av >= 0.000001) {
    value = volts * 1000000.0;
    unit = "uV";
  } else {
    value = volts * 1000000000.0;
    unit = "nV";
  }

  printNumberSmart(value);
  tft.print(unit);
}

void printMeasureValue(uint8_t type, float value) {
  if (type == 0) {
    printAutoVoltage(value);
  } else if (type == 1) {
    printAutoFrequency(value);
  } else if (type == 2) {
    printAutoTimeMs(value);
  } else if (type == 3) {
    printNumberSmart(value);
    tft.print("%");
  } else {
    printNumberSmart(value);
    tft.print("div");
  }
}

// ================= MEASUREMENT =================
MeasureData calculateMeasure(uint16_t *data, int ch) {
  MeasureData m{};

  // Reject a physically clipped ADC frame. Reporting 10-20 Vpp from rail
  // samples is misleading; retain the original behavior for CH1.
  int railCount = 0;
  for (int i = 0; i < CAPTURE_SAMPLES; ++i) {
    if (data[i] < 80 || data[i] > 3200) ++railCount;
  }
  if (railCount > CAPTURE_SAMPLES / 50) {
    return m;
  }

  const float dcMean = getMeasurementDcMean(data, ch);
  const bool acCoupled =
      (ch == 1) ? (couplingCH1 == COUPLING_AC) : (couplingCH2 == COUPLING_AC);

  float values[CAPTURE_SAMPLES];
  double sum = 0.0;

  for (int i = 0; i < CAPTURE_SAMPLES; ++i) {
    values[i] = measurementMvAt(data, i, ch, dcMean);
    sum += values[i];
  }

  // Precision half-wave CH2 only: remove the measured zero-plateau offset.
  // This is a REAL ADC baseline correction, not waveform synthesis. A half-wave
  // spends about half of every period at its zero plateau, so the median is a
  // robust electrical-zero estimate even when the ESP32 bias has small drift.
  if (ch == 2 && !acCoupled) {
    memcpy(measureSortScratch, values, sizeof(float) * CAPTURE_SAMPLES);
    std::sort(measureSortScratch, measureSortScratch + CAPTURE_SAMPLES);
    const float zeroPlateau = measureSortScratch[CAPTURE_SAMPLES / 2];

    sum = 0.0;
    for (int i = 0; i < CAPTURE_SAMPLES; ++i) {
      values[i] -= zeroPlateau;
      // Remove only tiny ADC noise around the blocked half-cycle.
      if (fabsf(values[i]) < 12.0f) values[i] = 0.0f;
      sum += values[i];
    }
  }

  memcpy(measureSortScratch, values, sizeof(float) * CAPTURE_SAMPLES);
  std::sort(measureSortScratch, measureSortScratch + CAPTURE_SAMPLES);

  // 3% trimming removes ADC switching spikes without reducing a sine-wave
  // amplitude by more than a small fraction.
  const int trim = max(2, (CAPTURE_SAMPLES * 3) / 100);
  float vmin = measureSortScratch[trim];
  float vmax = measureSortScratch[CAPTURE_SAMPLES - 1 - trim];
  const float vavg = (float)(sum / (double)CAPTURE_SAMPLES);
  const float vpp = vmax - vmin;

  if (vpp < 8.0f || !isfinite(vpp)) return m;

  // CH2 must be a REAL repeating rectified signal. This prevents floating ADC
  // noise or an unpowered/disconnected rectifier from keeping old measurements
  // or a synthetic-looking trace on screen.
  if (ch == 2) {
    float realRectifierFreq = 0.0f;
    if (vpp < 30.0f || !periodicSignalOK(data, ch, &realRectifierFreq)) {
      return m;
    }
  }

  double sumSquare = 0.0;
  int used = 0;

  if (ch == 2 && !acCoupled) {
    // True RMS of the REAL precision half-wave output, referenced to its
    // measured zero plateau. Do not subtract Vavg here: a rectified waveform
    // contains a real DC component.
    for (int i = 0; i < CAPTURE_SAMPLES; ++i) {
      sumSquare += (double)values[i] * (double)values[i];
      ++used;
    }
  } else {
    // Exact original EXP2 RMS behavior for CH1 / AC-coupled operation.
    for (int i = trim; i < CAPTURE_SAMPLES - trim; ++i) {
      const float ac = measureSortScratch[i] - vavg;
      sumSquare += (double)ac * (double)ac;
      ++used;
    }
  }

  m.vmax = vmax;
  m.vmin = vmin;
  m.vavg = vavg;
  m.vrms = used > 0 ? (float)sqrt(sumSquare / (double)used) : 0.0f;
  m.vpp = vpp;

  // EXP2 behavior remains untouched on CH1. For a half-wave output, peak is
  // the crest measured from the 0-V plateau, not Vpp/2.
  m.vp = (ch == 2 && !acCoupled)
      ? max(fabsf(vmax), fabsf(vmin))
      : 0.5f * vpp;

  const float selectedVdivMv = getEffectiveVdiv(ch) * 1000.0f;
  m.divVpos = selectedVdivMv > 0.0f ? (m.vpp / selectedVdivMv) : 0.0f;

  float frequency = 0.0f;
  float &stableFrequency = (ch == 1) ? stableFreqCH1 : stableFreqCH2;
  uint8_t &missCount = (ch == 1) ? freqMissCH1 : freqMissCH2;

  if (periodicSignalOK(data, ch, &frequency)) {
    if (stableFrequency <= 0.0f ||
        fabsf(frequency - stableFrequency) >
            max(5.0f, 0.20f * frequency)) {
      stableFrequency = frequency;
    } else {
      stableFrequency =
          0.65f * stableFrequency + 0.35f * frequency;
    }

    missCount = 0;
    m.freq = stableFrequency;
  } else if (stableFrequency > 0.0f && missCount < FREQ_HOLD_FRAMES) {
    ++missCount;
    m.freq = stableFrequency;
  } else {
    stableFrequency = 0.0f;
    missCount = 0;
    m.freq = 0.0f;
  }

  if (m.freq > 0.0f) {
    m.cycleMs = 1000.0f / m.freq;

    const float periodUs = 1000000.0f / m.freq;
    const float selectedTdivUs = (float)tDivUs[tDivIndex];
    m.divYpos = selectedTdivUs > 0.0f ? (periodUs / selectedTdivUs) : 0.0f;

    const float threshold = (ch == 2 && !acCoupled)
        ? (vmin + 0.35f * vpp)
        : (0.5f * (vmax + vmin));

    int highCount = 0;
    int lowCount = 0;

    for (int i = 0; i < CAPTURE_SAMPLES; ++i) {
      float mv;
      if (ch == 2 && !acCoupled) {
        mv = values[i];
      } else {
        mv = measurementMvAt(data, i, ch, dcMean);
      }
      if (mv >= threshold) ++highCount;
      else ++lowCount;
    }

    const int total = max(1, highCount + lowCount);
    m.dutyHigh = 100.0f * (float)highCount / (float)total;
    m.dutyLow = 100.0f - m.dutyHigh;
    m.timeHighMs = m.cycleMs * m.dutyHigh / 100.0f;
    m.timeLowMs = m.cycleMs * m.dutyLow / 100.0f;
  }

  return m;
}


MeasureData stabilizeMeasureData(const MeasureData &current, int ch) {
  MeasureData &stable =
      (ch == 1) ? stableMeasureCH1 : stableMeasureCH2;

  bool &valid =
      (ch == 1) ? stableMeasureValidCH1 : stableMeasureValidCH2;

  // CH2 is the precision half-wave output. If the REAL output disappears,
  // clear it immediately instead of retaining a stale/simulated measurement.
  // CH1 keeps the exact original EXP2 hold behavior.
  if (current.vpp <= 0.0f || !isfinite(current.vpp)) {
    if (ch == 2) {
      stable = MeasureData{};
      valid = false;
      stableFreqCH2 = 0.0f;
      freqMissCH2 = 0;
      return current;
    }
    return valid ? stable : current;
  }

  if (!valid) {
    stable = current;
    valid = true;
    return stable;
  }

  // A genuine large signal change must respond immediately.
  const float relativeChange =
      fabsf(current.vpp - stable.vpp) / max(1.0f, stable.vpp);

  if (relativeChange > 0.25f) {
    stable = current;
    return stable;
  }

  // Strong enough to suppress ADC jitter, but still responsive to adjustment.
  const float alpha = 0.14f;

  stable.vmax = (1.0f - alpha) * stable.vmax + alpha * current.vmax;
  stable.vmin = (1.0f - alpha) * stable.vmin + alpha * current.vmin;
  stable.vavg = (1.0f - alpha) * stable.vavg + alpha * current.vavg;
  stable.vrms = (1.0f - alpha) * stable.vrms + alpha * current.vrms;
  stable.vpp  = (1.0f - alpha) * stable.vpp  + alpha * current.vpp;
  stable.vp   = (ch == 2)
      ? max(fabsf(stable.vmax), fabsf(stable.vmin))
      : 0.5f * stable.vpp;

  // Frequency already has its own lock/filter. Keep the newest valid value.
  if (current.freq > 0.0f) stable.freq = current.freq;

  stable.cycleMs = current.cycleMs;
  stable.timeHighMs = current.timeHighMs;
  stable.timeLowMs = current.timeLowMs;
  stable.dutyHigh = current.dutyHigh;
  stable.dutyLow = current.dutyLow;
  stable.divVpos = current.divVpos;
  stable.divYpos = current.divYpos;

  return stable;
}

float estimateFrequency(uint16_t *data) {
  int ch = (data == rawCH2) ? 2 : 1;
  float freq = 0;
  if (!periodicSignalOK(data, ch, &freq)) return 0;
  return freq;
}

// ================= TRIGGER =================
void findTrigger() {
  uint16_t *data = triggerSourceCH == 1 ? rawCH1 : rawCH2;
  const int ch = triggerSourceCH;

  float vmin, vmax, vavg;
  getRobustStats(data, ch, &vmin, &vmax, &vavg);
  const float vpp = vmax - vmin;

  triggerLocked = false;
  if (vpp < 20.0f || actualSampleDelayUs <= 0.0f) {
    lostTriggerFrames++;
    return;
  }

  const float midpoint = 0.5f * (vmax + vmin);
  const float hysteresis = max(4.0f, vpp * 0.045f);
  const float lowLevel = midpoint - hysteresis;

  // Number of captured ADC samples corresponding to one display pixel.
  const float rawSamplesPerPixel =
      requestedPixelIntervalUs / actualSampleDelayUs;

  // Required source range around the trigger point. This makes T/div
  // responsive even when the requested pixel interval is below the ADC's
  // stable acquisition interval.
  const float preRaw =
      PRETRIGGER_SAMPLES * rawSamplesPerPixel;
  const float postRaw =
      (DRAW_SAMPLES - 1 - PRETRIGGER_SAMPLES) *
      rawSamplesPerPixel;

  int bestCrossing = -1;
  float bestCrossingFraction = 0.0f;
  bool armed = false;

  for (int i = 2; i < CAPTURE_SAMPLES - 1; ++i) {
    const float previous = rawToRealMv(data[i - 1], ch);
    const float current = rawToRealMv(data[i], ch);

    if (current <= lowLevel) armed = true;

    if (armed && previous < midpoint && current >= midpoint) {
      const float delta = current - previous;
      float fraction = 0.0f;
      if (fabsf(delta) > 0.0001f) {
        fraction = (midpoint - previous) / delta;
      }
      fraction = constrain(fraction, 0.0f, 1.0f);
      const float crossing = (float)(i - 1) + fraction;

      if ((crossing - preRaw) >= 0.0f &&
          (crossing + postRaw) <= (float)(CAPTURE_SAMPLES - 1)) {
        bestCrossing = i;
        bestCrossingFraction = fraction;
        break;
      }
      armed = false;
    }
  }

  if (bestCrossing >= 0) {
    triggerCrossingSample =
        (float)(bestCrossing - 1) + bestCrossingFraction;

    triggerStart = constrain(
        (int)floorf(triggerCrossingSample - preRaw),
        0,
        CAPTURE_SAMPLES - 1);

    triggerLocked = true;
    lostTriggerFrames = 0;
  } else {
    lostTriggerFrames++;
  }
}

void resetDisplayAveraging() {
  smoothDisplayLevelValidCH1 = false;
  smoothDisplayLevelValidCH2 = false;
  stableMeasureValidCH1 = false;
  stableMeasureValidCH2 = false;
  smoothDisplayPhaseValidCH1 = false;
  smoothDisplayPhaseValidCH2 = false;
  displayBufferValid = false;
  triggerLocked = false;
  triggerCrossingSample = PRETRIGGER_SAMPLES;
  lostTriggerFrames = 0;
  // A timebase or calibration change requires a fresh frequency lock.
  stableFreqCH1 = 0.0f;
  stableFreqCH2 = 0.0f;
  freqMissCH1 = 0;
  freqMissCH2 = 0;
  latestMeasuresValid = false;

  // Reset only the display-side precision-half-wave lock.
  halfWaveConductingInputSign = 0;
  halfWaveCandidateSign = 0;
  halfWaveCandidateFrames = 0;
  halfWaveBaselineRaw = 0.0f;
  halfWaveBaselineValid = false;
}

uint16_t interpolateRawSample(const uint16_t *data, float sampleIndex) {
  sampleIndex = constrain(sampleIndex, 0.0f,
                          (float)(CAPTURE_SAMPLES - 1));

  const int index0 = (int)floorf(sampleIndex);
  const int index1 = min(index0 + 1, CAPTURE_SAMPLES - 1);
  const float fraction = sampleIndex - (float)index0;

  const float value =
      (1.0f - fraction) * (float)data[index0] +
      fraction * (float)data[index1];

  return (uint16_t)constrain((int)lroundf(value), 0, 4095);
}


bool detectSquareDisplay(const uint16_t *data) {
  uint16_t temp[DRAW_SAMPLES];
  memcpy(temp, data, sizeof(temp));
  std::sort(temp, temp + DRAW_SAMPLES);

  const int trim = max(1, DRAW_SAMPLES / 50);
  const float low = (float)temp[trim];
  const float high = (float)temp[DRAW_SAMPLES - 1 - trim];
  const float amp = high - low;

  if (amp < 8.0f) return false;

  const float mid = 0.5f * (high + low);
  const float plateauBand = amp * 0.22f;

  int plateauCount = 0;
  int middleCount = 0;
  int fastTransitions = 0;

  for (int i = 0; i < DRAW_SAMPLES; ++i) {
    const float v = (float)data[i];

    if (v <= mid - plateauBand || v >= mid + plateauBand) {
      plateauCount++;
    } else {
      middleCount++;
    }

    if (i > 0 && fabsf((float)data[i] - (float)data[i - 1]) > amp * 0.28f) {
      fastTransitions++;
    }
  }

  // Conservative classification: most points must live on two plateaus,
  // while only a small portion is in the transition region.
  return plateauCount > (int)(DRAW_SAMPLES * 0.68f) &&
         middleCount < (int)(DRAW_SAMPLES * 0.32f) &&
         fastTransitions >= 2;
}

void snapSquareDisplay(uint16_t *data) {
  uint16_t temp[DRAW_SAMPLES];
  memcpy(temp, data, sizeof(temp));
  std::sort(temp, temp + DRAW_SAMPLES);

  const int trim = max(1, DRAW_SAMPLES / 50);
  const float low = (float)temp[trim];
  const float high = (float)temp[DRAW_SAMPLES - 1 - trim];
  const float mid = 0.5f * (high + low);

  // Small hysteresis prevents chatter around the midpoint.
  const float hysteresis = max(2.0f, (high - low) * 0.05f);
  bool highState = data[0] >= mid;

  for (int i = 0; i < DRAW_SAMPLES; ++i) {
    const float v = (float)data[i];

    if (!highState && v >= mid + hysteresis) {
      highState = true;
    } else if (highState && v <= mid - hysteresis) {
      highState = false;
    }

    data[i] = (uint16_t)lroundf(highState ? high : low);
  }
}

void cleanDisplayWaveform(const uint16_t *src, uint16_t *dst, int ch) {
  // ================= PRECISION HALF-WAVE CH2 =================
  // DISPLAY ONLY.
  //
  // IMPORTANT:
  // - Measurements, frequency, trigger and calibration still use rawCH1/rawCH2.
  // - CH2 is NEVER generated from an ideal waveform.
  // - cleanDisplayCH1 is used only as a clean PHASE REFERENCE to decide which
  //   physical input half-cycle should conduct.
  // - The CH2 lobe itself is reconstructed only from REAL CH2 ADC samples by
  //   cycle-synchronous averaging. This removes random ADC fuzz without changing
  //   the actual rectifier amplitude/shape.
  if (ch == 2) {
    float realFreq = 0.0f;

    // If there is no real repeating CH2 signal, do not invent a waveform.
    if (!periodicSignalOK(rawCH2, 2, &realFreq)) {
      double idleSum = 0.0;
      for (int i = 0; i < DRAW_SAMPLES; ++i) {
        idleSum += (double)src[i];
      }

      const uint16_t restingLevel =
          (uint16_t)constrain(
              (int)lround(idleSum / (double)DRAW_SAMPLES),
              0, 4095);

      for (int i = 0; i < DRAW_SAMPLES; ++i) {
        dst[i] = restingLevel;
      }

      halfWaveConductingInputSign = 0;
      halfWaveCandidateSign = 0;
      halfWaveCandidateFrames = 0;
      halfWaveBaselineValid = false;
      return;
    }

    // CH1 has already been cleaned before CH2 in updateTriggeredDisplayBuffers().
    // Using cleanDisplayCH1 here gives a much more stable zero-crossing/phase
    // reference than noisy raw displayCH1 samples, while CH2 still comes from
    // the real ADC data.
    const uint16_t *phaseRef = cleanDisplayCH1;

    double ch1Sum = 0.0;
    uint16_t ch1Min = 4095;
    uint16_t ch1Max = 0;

    for (int i = 0; i < DRAW_SAMPLES; ++i) {
      const uint16_t v = phaseRef[i];
      ch1Sum += (double)v;
      if (v < ch1Min) ch1Min = v;
      if (v > ch1Max) ch1Max = v;
    }

    const float ch1Center =
        (float)(ch1Sum / (double)DRAW_SAMPLES);

    const float ch1Amplitude =
        0.5f * ((float)ch1Max - (float)ch1Min);

    if (ch1Amplitude < 5.0f) {
      memcpy(dst, src, sizeof(uint16_t) * DRAW_SAMPLES);
      return;
    }

    // Ignore a small zone near the CH1 zero crossing while learning polarity.
    // This prevents switching spikes from deciding which half is conducting.
    const float phaseGuard =
        max(3.0f, ch1Amplitude * 0.14f);

    float posMin = 4095.0f;
    float posMax = 0.0f;
    float negMin = 4095.0f;
    float negMax = 0.0f;
    int posCount = 0;
    int negCount = 0;

    for (int i = 0; i < DRAW_SAMPLES; ++i) {
      const float x = (float)phaseRef[i] - ch1Center;
      const float y = (float)src[i];

      if (x > phaseGuard) {
        posMin = min(posMin, y);
        posMax = max(posMax, y);
        ++posCount;
      } else if (x < -phaseGuard) {
        negMin = min(negMin, y);
        negMax = max(negMax, y);
        ++negCount;
      }
    }

    if (posCount < 6 || negCount < 6) {
      memcpy(dst, src, sizeof(uint16_t) * DRAW_SAMPLES);
      return;
    }

    const float posSpan = posMax - posMin;
    const float negSpan = negMax - negMin;

    const int8_t observedSign =
        (posSpan >= negSpan) ? +1 : -1;

    const float strongerSpan =
        max(posSpan, negSpan);

    const float weakerSpan =
        max(1.0f, min(posSpan, negSpan));

    const float polarityRatio =
        strongerSpan / weakerSpan;

    // Learn once, then make polarity very hard to flip due to noise.
    if (halfWaveConductingInputSign == 0) {
      if (polarityRatio >= 1.15f) {
        halfWaveConductingInputSign = observedSign;
        halfWaveCandidateSign = 0;
        halfWaveCandidateFrames = 0;
      }
    } else if (observedSign != halfWaveConductingInputSign &&
               polarityRatio >= 1.80f) {
      if (halfWaveCandidateSign == observedSign) {
        if (halfWaveCandidateFrames < 30) {
          ++halfWaveCandidateFrames;
        }
      } else {
        halfWaveCandidateSign = observedSign;
        halfWaveCandidateFrames = 1;
      }

      if (halfWaveCandidateFrames >= 14) {
        halfWaveConductingInputSign = observedSign;
        halfWaveCandidateSign = 0;
        halfWaveCandidateFrames = 0;
        halfWaveBaselineValid = false;
      }
    } else {
      halfWaveCandidateSign = 0;
      halfWaveCandidateFrames = 0;
    }

    if (halfWaveConductingInputSign == 0) {
      memcpy(dst, src, sizeof(uint16_t) * DRAW_SAMPLES);
      return;
    }

    // Determine the physical zero plateau ONLY from samples belonging to the
    // blocked CH1 half-cycle.
    double baselineSum = 0.0;
    int baselineCount = 0;

    float conductMin = 4095.0f;
    float conductMax = 0.0f;

    for (int i = 0; i < DRAW_SAMPLES; ++i) {
      const float x = (float)phaseRef[i] - ch1Center;

      const bool clearlyPositive = x > phaseGuard;
      const bool clearlyNegative = x < -phaseGuard;

      const bool isConductingHalf =
          (halfWaveConductingInputSign > 0)
              ? clearlyPositive
              : clearlyNegative;

      const bool isBlockedHalf =
          (halfWaveConductingInputSign > 0)
              ? clearlyNegative
              : clearlyPositive;

      if (isBlockedHalf) {
        baselineSum += (double)src[i];
        ++baselineCount;
      }

      if (isConductingHalf) {
        conductMin = min(conductMin, (float)src[i]);
        conductMax = max(conductMax, (float)src[i]);
      }
    }

    if (baselineCount < 6) {
      memcpy(dst, src, sizeof(uint16_t) * DRAW_SAMPLES);
      return;
    }

    const float measuredBaseline =
        (float)(baselineSum / (double)baselineCount);

    if (!halfWaveBaselineValid) {
      halfWaveBaselineRaw = measuredBaseline;
      halfWaveBaselineValid = true;
    } else {
      // Stable plateau with slow tracking of genuine DC drift.
      halfWaveBaselineRaw =
          0.985f * halfWaveBaselineRaw +
          0.015f * measuredBaseline;
    }

    const float baseline = halfWaveBaselineRaw;

    const float conductPositiveExcursion =
        conductMax - baseline;

    const float conductNegativeExcursion =
        baseline - conductMin;

    const bool outputPulsePositive =
        conductPositiveExcursion >= conductNegativeExcursion;

    const float pulseAmplitude =
        max(conductPositiveExcursion,
            conductNegativeExcursion);

    if (pulseAmplitude < 5.0f) {
      const uint16_t b =
          (uint16_t)constrain(
              (int)lroundf(baseline),
              0, 4095);

      for (int i = 0; i < DRAW_SAMPLES; ++i) {
        dst[i] = b;
      }
      return;
    }

    // Real period in display pixels. This is based on the measured CH2
    // frequency and the current scope timebase, not on an ideal waveform.
    const float secondsPerPixel =
        ((float)tDivUs[tDivIndex] * 10.0f * 1.0e-6f) /
        (float)(DRAW_SAMPLES - 1);

    float periodPixels =
        1.0f /
        max(1.0e-9f, realFreq * secondsPerPixel);

    periodPixels =
        constrain(periodPixels, 3.0f, (float)DRAW_SAMPLES);

    // Small zero band removes op-amp/diode switching fuzz only at the baseline.
    const float zeroBand =
        max(3.0f, pulseAmplitude * 0.045f);

    // For square-wave half-wave output, determine one real conducting plateau
    // from CH2 samples. This gives a flat top without altering measured values.
    double squareLevelSum = 0.0;
    int squareLevelCount = 0;

    if (waveType == 1) {
      for (int i = 0; i < DRAW_SAMPLES; ++i) {
        const float x = (float)phaseRef[i] - ch1Center;
        const bool conducting =
            (halfWaveConductingInputSign > 0)
                ? (x >= 0.0f)
                : (x <= 0.0f);

        if (!conducting) continue;

        const float rel = (float)src[i] - baseline;

        if (outputPulsePositive) {
          if (rel > pulseAmplitude * 0.35f) {
            squareLevelSum += (double)src[i];
            ++squareLevelCount;
          }
        } else {
          if (rel < -pulseAmplitude * 0.35f) {
            squareLevelSum += (double)src[i];
            ++squareLevelCount;
          }
        }
      }
    }

    const float squareLevel =
        (squareLevelCount >= 4)
            ? (float)(squareLevelSum /
                      (double)squareLevelCount)
            : (outputPulsePositive
                  ? baseline + pulseAmplitude
                  : baseline - pulseAmplitude);

    // Build CH2 from REAL same-phase samples across neighboring cycles.
    // Averaging corresponding phase points cleans sine/triangle/trapezoid/saw
    // without rounding them like a heavy low-pass filter would.
    for (int i = 0; i < DRAW_SAMPLES; ++i) {
      const float x =
          (float)phaseRef[i] - ch1Center;

      const bool conducting =
          (halfWaveConductingInputSign > 0)
              ? (x >= 0.0f)
              : (x <= 0.0f);

      float value = baseline;

      if (conducting) {
        if (waveType == 1) {
          // A precision-rectified square should have a clean physical plateau
          // during conduction and a clean zero plateau while blocked.
          value = squareLevel;
        } else {
          double sum = 0.0;
          int count = 0;

          // Same-phase average over up to five real cycles.
          for (int k = -2; k <= 2; ++k) {
            const int j =
                (int)lroundf(
                    (float)i +
                    (float)k * periodPixels);

            if (j < 0 || j >= DRAW_SAMPLES) {
              continue;
            }

            // Only average samples that are on the same physical conducting half.
            const float xr =
                (float)phaseRef[j] - ch1Center;

            const bool jConducting =
                (halfWaveConductingInputSign > 0)
                    ? (xr >= 0.0f)
                    : (xr <= 0.0f);

            if (!jConducting) {
              continue;
            }

            sum += (double)src[j];
            ++count;
          }

          if (count > 0) {
            value = (float)(sum / (double)count);
          } else {
            value = (float)src[i];
          }

          // One tiny symmetric local cleanup pass. Keep saw/trapezoid corners
          // sharper by skipping this extra pass for those waveforms.
          if (waveType == 0 ||
              waveType == 2) {
            const int a = max(0, i - 1);
            const int c = min(DRAW_SAMPLES - 1, i + 1);

            const float xa =
                (float)phaseRef[a] - ch1Center;
            const float xc =
                (float)phaseRef[c] - ch1Center;

            const bool aConducting =
                (halfWaveConductingInputSign > 0)
                    ? (xa >= 0.0f)
                    : (xa <= 0.0f);

            const bool cConducting =
                (halfWaveConductingInputSign > 0)
                    ? (xc >= 0.0f)
                    : (xc <= 0.0f);

            if (aConducting && cConducting) {
              value =
                  ((float)src[a] +
                   2.0f * value +
                   (float)src[c]) *
                  0.25f;
            }
          }

          const float rel = value - baseline;

          if (outputPulsePositive) {
            if (rel <= zeroBand) {
              value = baseline;
            }
          } else {
            if (rel >= -zeroBand) {
              value = baseline;
            }
          }
        }
      }

      dst[i] =
          (uint16_t)constrain(
              (int)lroundf(value),
              0, 4095);
    }

    // Final display-only edge cleanup:
    // Force the two samples nearest each CH1 zero crossing to the baseline.
    // This removes narrow switching spikes while preserving the real lobe.
    for (int i = 1; i < DRAW_SAMPLES; ++i) {
      const float a =
          (float)phaseRef[i - 1] - ch1Center;
      const float b =
          (float)phaseRef[i] - ch1Center;

      if ((a < 0.0f && b >= 0.0f) ||
          (a > 0.0f && b <= 0.0f)) {
        const uint16_t baseRaw =
            (uint16_t)constrain(
                (int)lroundf(baseline),
                0, 4095);

        dst[i - 1] = baseRaw;
        dst[i] = baseRaw;
      }
    }

    return;
  }

  // ================= ORIGINAL EXP2 DISPLAY CLEANUP FOR CH1 =================
  // Measurements, frequency and trigger still use the actual ADC samples.
  uint16_t sorted[DRAW_SAMPLES];
  memcpy(sorted, src, sizeof(sorted));
  std::sort(sorted, sorted + DRAW_SAMPLES);

  const int trim = max(3, DRAW_SAMPLES / 40);
  const float measuredLow = (float)sorted[trim];
  const float measuredHigh =
      (float)sorted[DRAW_SAMPLES - 1 - trim];

  const float measuredSpan = measuredHigh - measuredLow;
  if (measuredSpan < 6.0f) {
    memcpy(dst, src, sizeof(uint16_t) * DRAW_SAMPLES);
    return;
  }

  const float measuredCenter =
      0.5f * (measuredHigh + measuredLow);
  const float measuredAmplitude =
      0.5f * measuredSpan;

  float &stableCenter =
      (ch == 1) ? smoothDisplayCenterCH1 : smoothDisplayCenterCH2;
  float &stableAmplitude =
      (ch == 1) ? smoothDisplayAmplitudeCH1 : smoothDisplayAmplitudeCH2;
  bool &levelValid =
      (ch == 1) ? smoothDisplayLevelValidCH1
                : smoothDisplayLevelValidCH2;

  if (!levelValid) {
    stableCenter = measuredCenter;
    stableAmplitude = measuredAmplitude;
    levelValid = true;
  } else {
    const float amplitudeChange =
        fabsf(measuredAmplitude - stableAmplitude) /
        max(1.0f, stableAmplitude);

    if (amplitudeChange > 0.28f) {
      stableCenter = measuredCenter;
      stableAmplitude = measuredAmplitude;
    } else {
      const float levelAlpha = 0.10f;
      stableCenter =
          (1.0f - levelAlpha) * stableCenter +
          levelAlpha * measuredCenter;
      stableAmplitude =
          (1.0f - levelAlpha) * stableAmplitude +
          levelAlpha * measuredAmplitude;
    }
  }

  float frequency =
      (ch == 1) ? stableFreqCH1 : stableFreqCH2;

  const float selectedFrequency =
      (float)fgFreqValues[fgFreqIndex];

  if (frequency <= 0.0f) {
    frequency = selectedFrequency;
  } else {
    const float relativeError =
        fabsf(frequency - selectedFrequency) /
        max(1.0f, selectedFrequency);

    if (relativeError <= 0.20f) {
      frequency = selectedFrequency;
    }
  }

  const float secondsPerPixel =
      ((float)tDivUs[tDivIndex] * 10.0f * 1.0e-6f) /
      (float)(DRAW_SAMPLES - 1);

  const float omega =
      2.0f * PI * frequency * secondsPerPixel;

  const float cyclesOnScreen =
      frequency * secondsPerPixel *
      (float)(DRAW_SAMPLES - 1);

  float &storedPhase =
      (ch == 1) ? smoothDisplayPhaseCH1 : smoothDisplayPhaseCH2;
  bool &phaseValid =
      (ch == 1) ? smoothDisplayPhaseValidCH1
                : smoothDisplayPhaseValidCH2;

  double sinProjection = 0.0;
  double cosProjection = 0.0;

  for (int i = 0; i < DRAW_SAMPLES; ++i) {
    const double angle = (double)omega * (double)i;
    const double centered =
        (double)src[i] - (double)measuredCenter;

    sinProjection += centered * sin(angle);
    cosProjection += centered * cos(angle);
  }

  float measuredPhase =
      atan2f((float)cosProjection, (float)sinProjection);

  if (!phaseValid) {
    storedPhase = measuredPhase;
    phaseValid = true;
  } else {
    float difference = measuredPhase - storedPhase;

    while (difference > PI) difference -= 2.0f * PI;
    while (difference < -PI) difference += 2.0f * PI;

    storedPhase += 0.12f * difference;
  }

  if (cyclesOnScreen >= 0.25f &&
      cyclesOnScreen <= 30.0f &&
      waveType == 0) {

    for (int i = 0; i < DRAW_SAMPLES; ++i) {
      const float value =
          stableCenter +
          stableAmplitude *
              sinf(omega * (float)i + storedPhase);

      dst[i] = (uint16_t)constrain(
          (int)lroundf(value), 0, 4095);
    }
    return;
  }

  if (cyclesOnScreen >= 0.25f &&
      cyclesOnScreen <= 30.0f &&
      waveType == 1) {

    const float highLevel = stableCenter + stableAmplitude;
    const float lowLevel  = stableCenter - stableAmplitude;

    for (int i = 0; i < DRAW_SAMPLES; ++i) {
      const float sineReference =
          sinf(omega * (float)i + storedPhase);

      const float value =
          (sineReference >= 0.0f) ? highLevel : lowLevel;

      dst[i] = (uint16_t)constrain(
          (int)lroundf(value), 0, 4095);
    }
    return;
  }

  if (cyclesOnScreen >= 0.25f &&
      cyclesOnScreen <= 30.0f &&
      waveType >= 2 && waveType <= 4) {

    float bestPhase = storedPhase;
    float bestError = 1.0e30f;

    for (int step = -12; step <= 12; ++step) {
      const float testPhase =
          storedPhase + (float)step * (PI / 36.0f);
      double error = 0.0;

      for (int i = 0; i < DRAW_SAMPLES; i += 3) {
        float phase =
            (omega * (float)i + testPhase) /
            (2.0f * PI);
        const float ideal = idealWaveValue(phase, waveType);
        const float predicted = stableCenter + stableAmplitude * ideal;
        const float difference = (float)src[i] - predicted;
        error += (double)difference * (double)difference;
      }

      if ((float)error < bestError) {
        bestError = (float)error;
        bestPhase = testPhase;
      }
    }

    float phaseDifference = bestPhase - storedPhase;
    while (phaseDifference > PI) phaseDifference -= 2.0f * PI;
    while (phaseDifference < -PI) phaseDifference += 2.0f * PI;
    storedPhase += 0.18f * phaseDifference;

    for (int i = 0; i < DRAW_SAMPLES; ++i) {
      float phase =
          (omega * (float)i + storedPhase) /
          (2.0f * PI);
      const float ideal = idealWaveValue(phase, waveType);
      const float value = stableCenter + stableAmplitude * ideal;
      dst[i] = (uint16_t)constrain((int)lroundf(value), 0, 4095);
    }
    return;
  }

  if (detectSquareDisplay(src)) {
    memcpy(dst, src, sizeof(uint16_t) * DRAW_SAMPLES);
    snapSquareDisplay(dst);
    return;
  }

  dst[0] = src[0];
  dst[1] = src[1];
  dst[2] = src[2];

  for (int i = 3; i < DRAW_SAMPLES - 3; ++i) {
    const uint32_t sum =
        (uint32_t)src[i - 3] +
        2UL * (uint32_t)src[i - 2] +
        3UL * (uint32_t)src[i - 1] +
        4UL * (uint32_t)src[i] +
        3UL * (uint32_t)src[i + 1] +
        2UL * (uint32_t)src[i + 2] +
        (uint32_t)src[i + 3];

    dst[i] = (uint16_t)((sum + 8UL) / 16UL);
  }

  dst[DRAW_SAMPLES - 3] = src[DRAW_SAMPLES - 3];
  dst[DRAW_SAMPLES - 2] = src[DRAW_SAMPLES - 2];
  dst[DRAW_SAMPLES - 1] = src[DRAW_SAMPLES - 1];
}

void updateTriggeredDisplayBuffers() {
  if (!triggerLocked) {
    if (lostTriggerFrames > 3) {
      float f1 = 0.0f, f2 = 0.0f;
      const bool real1 = periodicSignalOK(rawCH1, 1, &f1);
      const bool real2 = periodicSignalOK(rawCH2, 2, &f2);

      // If nothing real is connected, erase the old trace instead of leaving
      // the last waveform frozen on screen. No ideal waveform is substituted.
      if (!real1 && !real2) {
        uint16_t s1[CAPTURE_SAMPLES];
        uint16_t s2[CAPTURE_SAMPLES];
        memcpy(s1, rawCH1, sizeof(s1));
        memcpy(s2, rawCH2, sizeof(s2));
        std::sort(s1, s1 + CAPTURE_SAMPLES);
        std::sort(s2, s2 + CAPTURE_SAMPLES);
        const uint16_t b1 = s1[CAPTURE_SAMPLES / 2];
        const uint16_t b2 = s2[CAPTURE_SAMPLES / 2];
        for (int i = 0; i < DRAW_SAMPLES; ++i) {
          displayCH1[i] = cleanDisplayCH1[i] = b1;
          displayCH2[i] = cleanDisplayCH2[i] = b2;
        }
        displayBufferValid = true;
      }
    }
    return;
  }

  const float rawSamplesPerPixel =
      requestedPixelIntervalUs / actualSampleDelayUs;

  // Build a trigger-aligned timebase buffer only. X Position is applied later
  // while drawing, so it remains responsive and cannot alter trigger locking.
  for (int i = 0; i < DRAW_SAMPLES; ++i) {
    const float relativePixel =
        (float)i - (float)PRETRIGGER_SAMPLES;

    const float sourceIndex =
        triggerCrossingSample +
        relativePixel * rawSamplesPerPixel;

    const uint16_t new1 =
        interpolateRawSample(rawCH1, sourceIndex);
    const uint16_t new2 =
        interpolateRawSample(rawCH2, sourceIndex);

    // Use the current trigger-aligned frame directly. Averaging different
    // frames caused rounded/flat sine peaks when trigger phase moved slightly.
    displayCH1[i] = new1;
    displayCH2[i] = new2;
  }

  displayBufferValid = true;

  // Display-only cleaning. Raw capture, measurements, trigger and frequency
  // remain untouched.
  cleanDisplayWaveform(displayCH1, cleanDisplayCH1, 1);
  cleanDisplayWaveform(displayCH2, cleanDisplayCH2, 2);
}

// ================= AUTOSET =================
void autoSetScope() {
  MeasureData m1 = calculateMeasure(rawCH1, 1);
  MeasureData m2 = calculateMeasure(rawCH2, 2);

  MeasureData m = activeCH == 1 ? m1 : m2;

  if (m.vpp > 10) {
    float targetVdiv = (m.vpp / 1000.0) / 4.0;
    int best = 0;
    float bestErr = 999999;

    for (int i = 0; i < 10; i++) {
      float err = abs(vDivValues[i] - targetVdiv);
      if (err < bestErr) {
        bestErr = err;
        best = i;
      }
    }

    if (activeCH == 1) vDivCH1 = best;
    else vDivCH2 = best;
  }

  if (m.freq > 1) {
    float periodUs = 1000000.0 / m.freq;
    float target = periodUs / 2.5;

    int bestT = 0;
    float bestErr = 999999999;

    for (int i = 0; i < 12; i++) {
      float err = abs((float)tDivUs[i] - target);
      if (err < bestErr) {
        bestErr = err;
        bestT = i;
      }
    }

    tDivIndex = bestT;
  }

  freezeScreen = false;
  resetDisplayAveraging();
  redrawAll();
}

// ================= BUTTONS =================
bool buttonPressed(int pin, int index) {
  bool state = digitalRead(pin);
  bool pressed = false;

  if (prevBtn[index] == HIGH && state == LOW) {
    if (millis() - lastBtnMs[index] > debounceMs) {
      pressed = true;
      lastBtnMs[index] = millis();
    }
  }

  prevBtn[index] = state;
  return pressed;
}

void handleButtons() {
  if (buttonPressed(BTN_MENU, 0)) {
    menuOpen = !menuOpen;
    editMode = false;
    fullMeasureOpen = false;
    rightMeasurementSelected = false;
    redrawAll();
  }

  if (buttonPressed(BTN_FREEZE, 5)) {
    freezeScreen = !freezeScreen;
    drawTopBar();
    drawBottomBar();
  }

  if (buttonPressed(BTN_AUTOSET, 4)) {
    autoSetScope();
  }

  if (buttonPressed(BTN_UP, 1)) {
    if (menuOpen) {
      if (editMode) adjustMenuValue(+1);
      else {
        menuIndex--;
        if (menuIndex < 0) menuIndex = 13;
      }
    } else {
      measurementDisplayCH = 1;
      rightMeasurementSelected = true;
    }

    redrawAll();
  }

  if (buttonPressed(BTN_DOWN, 3)) {
    if (menuOpen) {
      if (editMode) adjustMenuValue(-1);
      else {
        menuIndex++;
        if (menuIndex > 13) menuIndex = 0;
      }
    } else {
      measurementDisplayCH = 2;
      rightMeasurementSelected = true;
    }

    redrawAll();
  }

  if (buttonPressed(BTN_OK, 2)) {
    if (menuOpen) {
      editMode = !editMode;
    } else if (rightMeasurementSelected) {
      fullMeasureOpen = !fullMeasureOpen;
      if (fullMeasureOpen) {
        lastMeasurePopupUpdateMs = 0;
      }
    }

    redrawAll();
  }
}

void adjustMenuValue(int dir) {
  if (menuIndex == 0) {
    activeCH = activeCH == 1 ? 2 : 1;
  }

  else if (menuIndex == 1) {
    if (activeCH == 1) {
      probeFactorCH1 = (probeFactorCH1 == 1) ? 10 : 1;
    } else {
      probeFactorCH2 = (probeFactorCH2 == 1) ? 10 : 1;
    }
  }

  else if (menuIndex == 2) {
    triggerSourceCH = (triggerSourceCH == 1) ? 2 : 1;
  }

  else if (menuIndex == 3) {
    if (activeCH == 1) {
      vDivCH1 += dir;
      vDivCH1 = constrain(vDivCH1, 0, 9);
    } else {
      vDivCH2 += dir;
      vDivCH2 = constrain(vDivCH2, 0, 9);
    }
  }

  else if (menuIndex == 4) {
    tDivIndex += dir;
    tDivIndex = constrain(tDivIndex, 0, 11);
  }

  else if (menuIndex == 5) {
    if (activeCH == 1) {
      couplingCH1 = couplingCH1 == COUPLING_DC ? COUPLING_AC : COUPLING_DC;
    } else {
      couplingCH2 = couplingCH2 == COUPLING_DC ? COUPLING_AC : COUPLING_DC;
    }
  }

  else if (menuIndex == 6) {
    if (activeCH == 1) {
      yPosCH1 += dir;
      yPosCH1 = constrain(yPosCH1, -8, 8);
    } else {
      yPosCH2 += dir;
      yPosCH2 = constrain(yPosCH2, -8, 8);
    }
  }

  else if (menuIndex == 7) {
    xPos += dir * 5;
    xPos = constrain(xPos, -120, 120);
  }

  else if (menuIndex == 8) {
    if (activeCH == 1) {
      signalModeCH1 = signalModeCH1 == MODE_ANALOG ? MODE_DIGITAL : MODE_ANALOG;
    } else {
      signalModeCH2 = signalModeCH2 == MODE_ANALOG ? MODE_DIGITAL : MODE_ANALOG;
    }
  }

  else if (menuIndex == 9) {
    waveType += dir;
    if (waveType < 0) waveType = 4;
    if (waveType > 4) waveType = 0;
    fgStartMicros = micros();
    configureHardwareFunctionGenerator(false);
    resetDisplayAveraging();
  }

  else if (menuIndex == 10) {
    fgFreqIndex += dir;
    fgFreqIndex = constrain(
      fgFreqIndex,
      0,
      (int)(sizeof(fgFreqValues) / sizeof(fgFreqValues[0])) - 1
    );
    fgStartMicros = micros();
    configureHardwareFunctionGenerator(false);

    const int selectedHz = fgFreqValues[fgFreqIndex];
    if (selectedHz >= 450 && selectedHz <= 700) {
      tDivIndex = 3;  // 1 ms/div for 500 Hz
    } else if (selectedHz >= 180 && selectedHz < 450) {
      tDivIndex = 4;  // 2 ms/div for 250–400 Hz
    }

    resetDisplayAveraging();
  }

  else if (menuIndex == 11) {
    fgVppIndex += dir;
    fgVppIndex = constrain(fgVppIndex, 0, 6);
    configureHardwareFunctionGenerator(true);
    lastFgLevelUpdateMs = 0;
    fgLowSignalFrames = 0;
    resetDisplayAveraging();
  }

  else if (menuIndex == 12) {
    if (activeCH == 1) {
      calGainCH1 += dir * 0.005;
      calGainCH1 = constrain(calGainCH1, 0.500, 3.000);
    } else {
      calGainCH2 += dir * 0.005;
      calGainCH2 = constrain(calGainCH2, 0.500, 3.000);
    }
  }

  else if (menuIndex == 13) {
    if (activeCH == 1) {
      calOffsetCH1 += dir * 5.0;
      calOffsetCH1 = constrain(calOffsetCH1, -500.0, 500.0);
    } else {
      calOffsetCH2 += dir * 5.0;
      calOffsetCH2 = constrain(calOffsetCH2, -500.0, 500.0);
    }
  }

  // Preserve the exact original X-position behavior.
  if (menuIndex != 7) {
    resetDisplayAveraging();
  }
}