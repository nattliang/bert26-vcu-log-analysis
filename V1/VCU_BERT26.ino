#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <SPI.h>
#include <IntervalTimer.h>
#include <ST7796_t3.h>
#include <XPT2046_Touchscreen.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

/*
  BERT26 VCU integrated sketch

  Functions:
    - Drives the ST7796/XPT2046 touchscreen dashboard.
    - Plays a buzzer chirp when BMS status becomes active.
    - Plays a different buzzer pattern when the RTD button on GPIO 23 is pressed.
    - Reads the dual analog accelerator pedal sensor.
    - Sends Cascadia PM100DX torque commands at 100 Hz over CAN.
*/

// ---------------- Display / touch pins ----------------
static const int TFT_MISO = 12;
static const int TFT_MOSI = 11;
static const int TFT_SCK  = 13;
static const int TFT_DC   = 8;
static const int TFT_CS   = 10;
static const int TFT_RST  = 9;
static const int TFT_LED  = 29;

static const int TOUCH_CS   = 7;
static const int TOUCH_IRQ  = 6;

ST7796_t3 tft(TFT_CS, TFT_DC, TFT_RST);
XPT2046_Touchscreen ts(TOUCH_CS, TOUCH_IRQ);

// ---------------- Vehicle IO ----------------
static const int READY_BUTTON_PIN = 23;
static const int BUZZER_PIN       = 22;    // Change here if the buzzer is wired elsewhere.
static const int BRAKE_LIGHT_PIN  = 5;     // HIGH = brake light on
static const int FAN_1_PIN        = 1;
static const int FAN_2_PIN        = 2;

// Shutdown-circuit status inputs: HIGH = signal OK (green), LOW = tripped (red).
static const int BSPD_STATUS_PIN  = 24;
static const int IMD_STATUS_PIN   = 25;
static const int BMS_STATUS_PIN   = 28;

static constexpr uint32_t FAN_PWM_FREQUENCY_HZ = 20000;
static constexpr uint8_t FAN_PWM_WRITE_MAX = 255;

// Fan duty tracks the hotter of motor/inverter temp: 20% at room temp,
// ramping linearly to 100% by 40 C (system max is 50 C).
static const float FAN_TEMP_MIN_C = 25.0f;
static const float FAN_TEMP_MAX_C = 40.0f;
static const int FAN_DUTY_MIN_PERCENT = 20;
static const int FAN_DUTY_MAX_PERCENT = 100;

// EURO-XPD analog input pins from VCU_Pedal_Read_Test.
static const int APPS_1_PIN = 38;         // Inverted channel
static const int APPS_2_PIN = 41;         // Normal channel
static const int BSE_1_PIN  = 17;
static const int BSE_2_PIN  = 16;

static const int ADC_RESOLUTION_BITS = 12;
static const int ADC_MAX_COUNTS      = (1 << ADC_RESOLUTION_BITS) - 1;
static const float ADC_REFERENCE_VOLTAGE = 3.3f;

// Raw ADC calibration ranges from the BERT26 pedal test sketch.
static const int APPS_1_RAW_MIN = 325;
static const int APPS_1_RAW_MAX = 2400;
static const int APPS_2_RAW_MIN = 2130;
static const int APPS_2_RAW_MAX = 4013;

static const float APPS_EMA_ALPHA          = 0.35f;
static const float APPS_IMPLAUSIBLE_PCT    = 10.0f;
// T.4.2.5 allows up to 100 ms of >10% channel deviation before it counts as
// an implausibility, so brief sensor/connector noise no longer drops RTD.
static const uint32_t APPS_IMPLAUSIBILITY_PERSIST_MS = 100;
// A raw count this far outside a channel's calibrated span means a wiring or
// sensor fault (short/open), not pedal travel.
static const int APPS_RAW_FAULT_MARGIN = 150;
static const float PEDAL_ZERO_DEADBAND_PCT = 7.0f;
static const float MAX_TORQUE_NM           = 120.0f;
static const float BSPC_APPS_THRESHOLD_PCT = 25.0f;

// Calculated from voltage divider at input. R1 = 5.6k, R2 = 15k, Gain = 0.728
// This comment was written by a human, me
static const float BSE_SENSOR_MIN_VOLTAGE  = 0.364f;
static const float BSE_SENSOR_MAX_VOLTAGE  = 3.277f;
static const int BSE_1_BRAKING_RAW_THRESHOLD = 600;
static const int BSE_2_BRAKING_RAW_THRESHOLD = 800;
static const int BSE_1_HARD_BRAKING_RAW_THRESHOLD = 2000;
static const int BSE_2_HARD_BRAKING_RAW_THRESHOLD = 2100;

// ---------------- CAN ----------------
// GPIO 23 is used by the RTD button, so this sketch uses the Teensy 4.1 CAN3
// peripheral on pins 30=CAN RX and 31=CAN TX. These pins connect to the TXD/RXD
// side of an external CAN transceiver module; the module's CANH/CANL side
// connects to the vehicle CAN bus.
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> VehicleCan;

static constexpr uint32_t CAN_BAUD = 500000;
static constexpr bool CAN_LOG_SERIAL_ENABLED = true;
static constexpr bool CAN_TRANSLATION_SERIAL_ENABLED = true;
static constexpr bool APPS_SERIAL_LOG_ENABLED = true;

// Set false to bypass brake-press requirement when bench testing without pedals.
static constexpr bool REQUIRE_BRAKE_FOR_RTD = true;
static constexpr uint32_t APPS_SERIAL_LOG_PERIOD_MS = 100;

// Cascadia PM100DX command frame from BERT25.
static constexpr uint32_t PM100_CMD_ID       = 0x0C0;
static constexpr uint32_t PM100_TEMPERATURE_1_FRAME_ID = 0x0A0;
static constexpr uint32_t PM100_TEMPERATURE_3_FRAME_ID = 0x0A2;
static constexpr uint32_t PM100_RPM_FRAME_ID = 0x0A5;
static constexpr uint32_t PM100_CURRENT_INFO_FRAME_ID = 0x0A6;
static constexpr uint32_t PM100_VOLTAGE_INFO_FRAME_ID = 0x0A7;
static constexpr uint32_t PM100_INTERNAL_VOLTAGES_FRAME_ID = 0x0A9;
static constexpr uint32_t PM100_FAULT_CODES_FRAME_ID = 0x0AB;
static constexpr uint32_t PM100_TORQUE_CAPABILITY_FRAME_ID = 0x0B1;
static constexpr uint32_t PM100_PARAM_COMMAND_ID  = 0x0C1;
static constexpr uint32_t PM100_PARAM_RESPONSE_ID = 0x0C2;
// EEPROM parameter 148 = "CAN Active Messages Lo Word"; bit 0x0200 enables the
// 0x0A9 Internal Voltages broadcast that carries the GLV (12V) voltage.
static constexpr uint16_t PM100_PARAM_CAN_ACTIVE_MSGS = 148;
static constexpr uint32_t PM100_BROADCAST_ENABLE_DELAY_MS = 5000;
static constexpr uint32_t PM100_BROADCAST_ENABLE_RETRY_MS = 5000;
static constexpr uint8_t PM100_BROADCAST_ENABLE_MAX_ATTEMPTS = 3;
static constexpr uint16_t TORQUE_SCALE       = 10;      // 0.1 Nm per LSB
static constexpr uint32_t CMD_PERIOD_MS      = 10;      // 100 Hz
static constexpr uint32_t PM100_CMD_PERIOD_US = CMD_PERIOD_MS * 1000u;
static constexpr float MPH_PER_RPM           = 0.014005f;

static constexpr uint32_t BMS_TIMEOUT_MS = 10000;
static constexpr uint32_t SCREEN_PERIOD_MS = 100;
static constexpr uint32_t STARTUP_BMS_IMD_FAULT_DELAY_MS = 12000;
static constexpr uint32_t FAULT_DISPLAY_CYCLE_MS = 1500;
static constexpr uint32_t FAULT_BANNER_DURATION_MS = 5000;
// GLV battery sits around 12-13 V; show the readout red once it sags below this.
static constexpr float GLV_LOW_VOLTAGE_WARN_V = 11.5f;
static constexpr uint8_t DASHBOARD_MAX_FAULT_TEXTS = 48;
static constexpr uint32_t RTD_BLOCKED_LOG_PERIOD_MS = 1000;

// BERT26 vehicle CAN definitions for human-readable log translations.
static constexpr uint32_t BMS_STATUS_2026_ID        = 0x310;
static constexpr uint32_t BMS_TEMPERATURE_2026_ID   = 0x311;
static constexpr uint32_t BMS_CELL_REQUEST_2026_ID  = 0x312;
static constexpr uint32_t BMS_CELL_RESPONSE_2026_ID = 0x313;
static constexpr uint8_t CELL_DATA_REQUEST_UNKNOWN      = 0x00;
static constexpr uint8_t CELL_DATA_REQUEST_VOLTAGES     = 0x01;
static constexpr uint8_t CELL_DATA_REQUEST_TEMPERATURES = 0x02;
static constexpr int16_t CELL_TEMP_UNKNOWN_C = -1000;

// Bender iso175 IMD/TSSI frames from VCU_TSSI_CODE_5_31_26.ino.
static constexpr uint32_t IMD_INFO_GENERAL_ID = 0x37;
static constexpr uint32_t IMD_INFO_ISOLATION_DETAIL_ID = 0x38;
static constexpr uint32_t IMD_INFO_VOLTAGE_ID = 0x39;
static constexpr uint32_t IMD_INFO_IT_SYSTEM_ID = 0x3A;
static constexpr uint32_t IMD_RESPONSE_ID     = 0x23;
static constexpr uint32_t IMD_REQUEST_ID      = 0x22;
static constexpr uint32_t IMD_FALLBACK_REQUEST_ID = 0x0FF;
static constexpr uint8_t IDX_WARN_ALARMS      = 0x6C;
static constexpr uint16_t IMD_SNV_WORD        = 0xFFFF;
static constexpr uint8_t IMD_SNV_BYTE         = 0xFF;
static constexpr int32_t IMD_VOLTAGE_OFFSET   = 32128;
static constexpr float IMD_VOLTAGE_SCALE      = 0.05f;

// TSSI LED pins: low-side switches, HIGH = transistor on = LED on.
static const int TSSI_LED_RED   = 3;
static const int TSSI_LED_GREEN = 4;

static constexpr uint32_t TSSI_BLINK_PERIOD_MS    = 500;  // 2 Hz
static constexpr uint32_t TSSI_BLINK_ON_MS        = 250;
static constexpr uint32_t IMD_HEARTBEAT_TIMEOUT_MS = 5000;
static constexpr uint32_t IMD_GET_REQ_PERIOD_MS    = 500;
 
static constexpr uint16_t IMD_FAULT_BITS_MASK = (1u << 0) | (1u << 1) | (1u << 2) |
                                                (1u << 3) | (1u << 4) | (1u << 7) |
                                                (1u << 8) | (1u << 9) | (1u << 10);

enum RtdState {
  RTD_STATE_FAULT_BLOCKED,
  RTD_STATE_READY_FOR_BUTTON,
  RTD_STATE_READY_TO_DRIVE
};

// ---------------- Runtime state ----------------
char driveMode = 'N';  // RTD state machine sets D only while ready to drive.

float apps1Pct = 0.0f;
float apps2Pct = 0.0f;
float pedalPct = 0.0f;
float desiredTorqueNm = 0.0f;
// Torque actually loaded into the PM100 command frame this cycle; this is
// what the dashboard shows so the display always matches the bus.
float commandedTorqueNm = 0.0f;
bool appsPlausible = false;
bool appsDisagreeing = false;
uint32_t appsDisagreeStartMs = 0;
// Names the kind of APPS problem (channel disagreement vs. out-of-range
// sensor) for the dash fault texts; only meaningful while not plausible.
const char *appsFaultText = "APPS Disagree";
// True while brake + accelerator >25% travel are applied together; commands
// zero torque only for as long as the condition holds (no latch).
bool bspcActive = false;
bool appsFilterInitialized = false;
int rawApps1Last = 0;
int rawApps2Last = 0;
int rawBse1Last = 0;
int rawBse2Last = 0;
float bse1Voltage = 0.0f;
float bse2Voltage = 0.0f;
float bse1Pct = 0.0f;
float bse2Pct = 0.0f;
bool braking = false;
bool hardBraking = false;

// Duty cycles are set automatically from motor/inverter temps unless a
// manual override is entered over serial (digits = override, 'a' = auto).
int fan1DutyPercent = FAN_DUTY_MIN_PERCENT;
int fan2DutyPercent = FAN_DUTY_MIN_PERCENT;
int fan1LastDutyPercent = -1;
int fan2LastDutyPercent = -1;
bool fanManualOverride = false;

float packVoltage = 0.0f;
float packCurrent = 0.0f;
float inverterDcBusVoltage = 0.0f;
float inverterDcBusCurrent = 0.0f;
bool haveInverterVoltageInfo = false;
bool haveInverterCurrentInfo = false;
// GLV (12V system) voltage reported by the PM100 Internal Voltages frame.
float glvVoltage = 0.0f;
bool haveGlvVoltage = false;
// If the inverter is talking but 0x0A9 never arrives, its broadcast mask has
// Internal Voltages disabled; we write parameter 148 once to turn it back on.
bool havePm100Traffic = false;
uint32_t firstPm100FrameMs = 0;
uint8_t pm100BroadcastEnableAttempts = 0;
uint32_t pm100BroadcastEnableLastTxMs = 0;
float powerKw = 0.0f;
uint8_t stateOfChargePct = 0;
uint8_t batteryTempC = 0;
uint16_t minCellVoltageMv = 0;
uint16_t maxCellVoltageMv = 0;
uint16_t avgCellVoltageMv = 0;
bool haveCellVoltageStats = false;
int16_t minCellTempC = CELL_TEMP_UNKNOWN_C;
int16_t maxCellTempC = CELL_TEMP_UNKNOWN_C;
int16_t avgCellTempC = CELL_TEMP_UNKNOWN_C;
bool haveCellTempStats = false;
int16_t inverterTempTenthsC = 0;
int16_t pm100MotorTempTenthsC = 0;
bool haveInverterTemp = false;
bool havePm100MotorTemp = false;
uint32_t pm100PostFaults = 0;
uint32_t pm100RunFaults = 0;
bool havePm100FaultStatus = false;
bool pm100Fault = false;
int16_t motorRpm = 0;
int speedMph = 0;

bool bmsActive = false;
bool bmsFault = true;
bool bmsTimedOut = true;
bool haveBmsStatus = false;
uint8_t bmsFlags = 0;
uint8_t bms2026FaultCode = 0;
bool haveBms2026FaultCode = false;
uint32_t lastBmsRxMs = 0;
uint8_t lastCellDataRequestType = CELL_DATA_REQUEST_UNKNOWN;

bool imdFault = true;
uint16_t imdWarnAlarms = 0;
uint16_t imdRIsoKohm = 0xFFFF;
uint8_t imdRIsoStatus = 0;
uint8_t imdDeviceActivity = 0;
uint32_t lastImdRxMs = 0;
bool imdCanTimedOut = true;

uint32_t tssiBlinkTimerMs = 0;
bool tssiRedOn = false;
uint32_t lastImdGetReqMs = 0;

RtdState rtdState = RTD_STATE_FAULT_BLOCKED;
// Compatibility flag used by the display and PM100 gate; kept in sync with rtdState.
bool readyToDriveLatched = false;

bool readyButtonLast = false;

// Fault banner: a red strip over the drive-button row naming the most recent
// fault, shown for FAULT_BANNER_DURATION_MS after each fault event.
char faultBannerText[48] = "";
uint32_t faultBannerUntilMs = 0;
bool faultBannerVisible = false;
bool faultBannerDirty = false;

int backlightBrightness = 255;
bool screenDirty = true;
bool dashboardNeedsFullRedraw = true;
uint32_t startupFaultDelayStartMs = 0;

IntervalTimer pm100CommandTimer;
volatile int16_t pm100CommandTorqueRaw = 0;
volatile uint8_t pm100CommandDirection = 0x00;
volatile uint8_t pm100CommandEnable = 0x00;

static constexpr uint8_t PM100_TX_LOG_QUEUE_SIZE = 192;
CAN_message_t pm100TxLogQueue[PM100_TX_LOG_QUEUE_SIZE];
volatile uint8_t pm100TxLogHead = 0;
volatile uint8_t pm100TxLogTail = 0;
volatile uint32_t pm100TxLogDropped = 0;

// ---------------- Buzzer sequencer ----------------
struct ToneStep {
  uint16_t frequencyHz;
  uint16_t onMs;
  uint16_t offMs;
};

static const ToneStep TS_READY_CHIME_PATTERN[] = {
  {1568, 200, 0},
  {1047, 100, 0},
  {1319, 300, 1500}
};

static const ToneStep RTD_PATTERN[] = {
  {2400, 90, 80},
  {2400, 90, 80},
  {2400, 90, 0}
};

const ToneStep *activeTonePattern = nullptr;
uint8_t activeToneLength = 0;
uint8_t activeToneIndex = 0;
bool activeToneInGap = false;
bool activeToneRepeats = false;
uint32_t activeToneDeadlineMs = 0;

typedef void (*CanTranslationPrinter)(const CAN_message_t &msg);

struct CanFrameTranslation {
  uint32_t id;
  const char *name;
  CanTranslationPrinter print;
};

struct DashboardFaultList {
  const char *texts[DASHBOARD_MAX_FAULT_TEXTS];
  uint8_t count;
};

// ---------------- Function declarations ----------------
void setupDisplay();
void resetDisplay();
void handleSerialFanDutyInput();
void handleTouch();
char checkIfTouchedRaw(int x, int y);
void drawDashboard();
void drawDriveButtons(bool force);
void drawDashboardLabel(int x, int y, const char *label);
void drawCachedDashboardText(int x, int y, int w, int h, uint8_t textSize,
                             const char *value, uint16_t valueColor,
                             char *lastValue, size_t lastValueSize,
                             uint16_t &lastColor, bool force);
void formatCellVoltageText(char *buffer, size_t bufferSize, uint16_t millivolts, bool valid);
void formatCellTempText(char *buffer, size_t bufferSize, int16_t tempC, bool valid);
void formatTemperatureTenthsText(char *buffer, size_t bufferSize, int16_t tempTenthsC, bool valid);
const char *dashboardFaultText();
void collectDashboardFaults(DashboardFaultList &faults);
void appendDashboardFault(DashboardFaultList &faults, const char *text);
void appendTssiFaultNames(DashboardFaultList &faults, uint16_t bits);
void appendPm100FaultNames(DashboardFaultList &faults, uint32_t postFaults, uint32_t runFaults);
const char *firstTssiFaultName(uint16_t bits);
const char *firstPm100FaultName(uint32_t postFaults, uint32_t runFaults);
const char *bmsTemperatureFaultCodeName(uint8_t faultCode);
const char *pm100PostFaultName(uint8_t bit);
const char *pm100RunFaultName(uint8_t bit);
void drawStatusText(int x, int y, const char *label, const char *value, uint16_t valueColor);
void drawStatusText(int x, int y, const char *label, int value, const char *units, uint16_t valueColor);
void drawShutdownStatusIndicators(bool force);
void drawShutdownStatusIndicator(int x, int y, const char *label, bool high,
                                 int &lastState, bool force);

void setupFans();
void updateFanDutyFromTemps();
void serviceFans();
uint8_t dutyPercentToFanPwmWrite(int dutyPercent);

void setupVehicleCan();
void receiveCanFrames();
void dispatchReceivedCanFrame(const CAN_message_t &msg);
void sendCanFrame(CAN_message_t &msg);
void logCanFrame(const char *direction, const CAN_message_t &msg);
void logCanFrameToSerial(const char *direction, const CAN_message_t &msg);
void setupPm100CommandTimer();
void pm100CommandTimerIsr();
void updatePm100CommandState();
void queuePm100TxLogFromIsr(const CAN_message_t &msg);
bool dequeuePm100TxLog(CAN_message_t &msg);
void servicePm100TxLogging();
void updateCanTranslationState(const char *direction, const CAN_message_t &msg);
void logCanTranslationToSerial(const char *direction, const CAN_message_t &msg);
void printBmsStatus2026Translation(const CAN_message_t &msg);
void printBmsTemperature2026Translation(const CAN_message_t &msg);
void printCellDataRequest2026Translation(const CAN_message_t &msg);
void printCellDataResponse2026Translation(const CAN_message_t &msg);
void printImdRequestTranslation(const CAN_message_t &msg);
void printImdResponseTranslation(const CAN_message_t &msg);
void printImdInfoGeneralTranslation(const CAN_message_t &msg);
void printImdInfoIsolationDetailTranslation(const CAN_message_t &msg);
void printImdInfoVoltageTranslation(const CAN_message_t &msg);
void printImdInfoItSystemTranslation(const CAN_message_t &msg);
void printPm100CommandTranslation(const CAN_message_t &msg);
void printPm100Temperatures1Translation(const CAN_message_t &msg);
void printPm100Temperatures3Translation(const CAN_message_t &msg);
void printPm100MotorPositionTranslation(const CAN_message_t &msg);
void printPm100CurrentInfoTranslation(const CAN_message_t &msg);
void printPm100VoltageInfoTranslation(const CAN_message_t &msg);
void printPm100InternalVoltagesTranslation(const CAN_message_t &msg);
void printPm100FaultCodesTranslation(const CAN_message_t &msg);
void printPm100TorqueCapabilityTranslation(const CAN_message_t &msg);
void printRawPayloadFrom(const CAN_message_t &msg, uint8_t startIndex);
void printBmsTemperatureFaultCode(uint8_t faultCode);
void printCellDataRequestName(uint8_t requestType);
void printImdRequestIndexName(uint8_t requestIndex);
void printImdResponseErrorName(uint8_t errorCode);
void printImdInfoMessageSelectorName(uint8_t selector);
void printImdProfileName(uint8_t profile);
void printImdRIsoStatusName(uint8_t status);
void printImdDeviceActivityName(uint8_t activity);
void printImdLockStateName(uint8_t state);
void printImdEarthliftStateName(uint8_t state);
void printImdVoltageModeName(uint8_t mode);
void printImdBaudRateName(uint8_t baudRate);
void printImdAsciiPayloadFrom(const CAN_message_t &msg, uint8_t startIndex);
void printImdWordValue(uint16_t value);
void printImdResistanceValue(uint16_t kohms);
void printImdVoltageValue(uint16_t raw);
void printImdCapacityValue(uint16_t raw);
void printImdPercentValue(uint8_t value);
void printImdCounterValue(uint8_t value);
void printCellVoltageValue(uint16_t millivolts);
void printCanHexByte(uint8_t value);
bool tssiFaultFromWarnBits(uint16_t warnBits);
bool bmsImdFaultDetectionArmed();
bool anyFaultActive();
bool contactorsOpen();
void handleBms2026StatusFrame(const CAN_message_t &msg);
void handleBms2026TempFrame(const CAN_message_t &msg);
void handleImdFrame(const CAN_message_t &msg);
void handlePm100Temperatures1Frame(const CAN_message_t &msg);
void handlePm100Temperatures3Frame(const CAN_message_t &msg);
void handlePm100RpmFrame(const CAN_message_t &msg);
void handlePm100CurrentInfoFrame(const CAN_message_t &msg);
void handlePm100VoltageInfoFrame(const CAN_message_t &msg);
void handlePm100InternalVoltagesFrame(const CAN_message_t &msg);
void handlePm100FaultCodesFrame(const CAN_message_t &msg);
void handlePm100ParamResponseFrame(const CAN_message_t &msg);
void servicePm100BroadcastEnable();
void sendPm100BroadcastEnable();
void setupTssi();
void serviceTssi();
void requestImdWarnings();
void updateTssiLed(bool faultActive, uint32_t now);
void setTssiGreenOn();
void setTssiRedOn();
void setTssiRedOff();
void setTssiAllOff();
void printTssiWarnBits(uint16_t bits);

void updatePedal();
void updateBrakePressureSensors();
void logAppsTelemetry();
float scalePercentNormal(int rawValue, int rawMin, int rawMax);
float scalePercentInverted(int rawValue, int rawMin, int rawMax);
float adcCountsToVoltage(int rawValue);
float scaleBrakePressurePercent(float voltage);

void serviceReadyToDrive();
bool readReadyButtonPressed();
bool rtdInputsReady();
bool readyForRtdButton();
void refreshRtdStateFromInputs();
void logRtdBlockedReason();
void handleRtdButtonPressed();
void setRtdState(RtdState newState);
void dropReadyToDrive(const char *reason);
const char *rtdDropCauseText();
void showFaultBanner(const char *text);
void serviceFaultBanner();
void drawFaultBanner(bool force);
const char *rtdStateName(RtdState state);
void setDriveMode(char newDriveMode);
uint8_t pm100DirectionByteForMode(char mode);

void playTonePattern(const ToneStep *pattern, uint8_t length, bool repeats);
void serviceBuzzer();
void stopBuzzer();

static const CanFrameTranslation CAN_FRAME_TRANSLATIONS[] = {
  {BMS_STATUS_2026_ID,        "BMS Status",        printBmsStatus2026Translation},
  {BMS_TEMPERATURE_2026_ID,   "BMS Temperature",   printBmsTemperature2026Translation},
  {BMS_CELL_REQUEST_2026_ID,  "Cell Data Request", printCellDataRequest2026Translation},
  {BMS_CELL_RESPONSE_2026_ID, "Cell Data Response", printCellDataResponse2026Translation},
  {IMD_REQUEST_ID, "IMD Request", printImdRequestTranslation},
  {IMD_FALLBACK_REQUEST_ID, "IMD Fallback Request", printImdRequestTranslation},
  {IMD_RESPONSE_ID, "IMD Response", printImdResponseTranslation},
  {IMD_INFO_GENERAL_ID, "IMD Info General", printImdInfoGeneralTranslation},
  {IMD_INFO_ISOLATION_DETAIL_ID, "IMD Info Isolation Detail", printImdInfoIsolationDetailTranslation},
  {IMD_INFO_VOLTAGE_ID, "IMD Info Voltage", printImdInfoVoltageTranslation},
  {IMD_INFO_IT_SYSTEM_ID, "IMD Info IT-System", printImdInfoItSystemTranslation},
  {PM100_CMD_ID, "PM100 Command", printPm100CommandTranslation},
  {PM100_TEMPERATURE_1_FRAME_ID, "PM100 Temperatures #1", printPm100Temperatures1Translation},
  {PM100_TEMPERATURE_3_FRAME_ID, "PM100 Temperatures #3", printPm100Temperatures3Translation},
  {PM100_RPM_FRAME_ID, "PM100 Motor Position", printPm100MotorPositionTranslation},
  {PM100_CURRENT_INFO_FRAME_ID, "PM100 Current Information", printPm100CurrentInfoTranslation},
  {PM100_VOLTAGE_INFO_FRAME_ID, "PM100 Voltage Information", printPm100VoltageInfoTranslation},
  {PM100_INTERNAL_VOLTAGES_FRAME_ID, "PM100 Internal Voltages", printPm100InternalVoltagesTranslation},
  {PM100_FAULT_CODES_FRAME_ID, "PM100 Fault Codes", printPm100FaultCodesTranslation},
  {PM100_TORQUE_CAPABILITY_FRAME_ID, "PM100 Torque Capability", printPm100TorqueCapabilityTranslation}
};

static constexpr uint8_t CAN_FRAME_TRANSLATION_COUNT =
    sizeof(CAN_FRAME_TRANSLATIONS) / sizeof(CAN_FRAME_TRANSLATIONS[0]);

static inline uint16_t u16Le(uint8_t lo, uint8_t hi) {
  return (uint16_t)lo | ((uint16_t)hi << 8);
}

static inline uint16_t u16Be(uint8_t hi, uint8_t lo) {
  return ((uint16_t)hi << 8) | (uint16_t)lo;
}

static inline uint32_t u32Le(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {
  return (uint32_t)b0 | ((uint32_t)b1 << 8) |
         ((uint32_t)b2 << 16) | ((uint32_t)b3 << 24);
}

static inline int16_t s16Le(uint8_t lo, uint8_t hi) {
  return (int16_t)u16Le(lo, hi);
}

// ---------------- Setup ----------------
void setup() {
  startupFaultDelayStartMs = millis();

  Serial.begin(115200);

  pinMode(BUZZER_PIN, OUTPUT);
  noTone(BUZZER_PIN);

  setupFans();

  pinMode(READY_BUTTON_PIN, INPUT);
  readyButtonLast = readReadyButtonPressed();
  refreshRtdStateFromInputs();

  analogReadResolution(ADC_RESOLUTION_BITS);
  pinMode(APPS_1_PIN, INPUT);
  pinMode(APPS_2_PIN, INPUT);
  pinMode(BSE_1_PIN, INPUT);
  pinMode(BSE_2_PIN, INPUT);

  pinMode(BRAKE_LIGHT_PIN, OUTPUT);
  digitalWrite(BRAKE_LIGHT_PIN, LOW);

  // Pulldowns so a disconnected status line reads LOW and shows red.
  pinMode(BSPD_STATUS_PIN, INPUT_PULLDOWN);
  pinMode(IMD_STATUS_PIN, INPUT_PULLDOWN);
  pinMode(BMS_STATUS_PIN, INPUT_PULLDOWN);

  setupTssi();
  setupVehicleCan();
  setupPm100CommandTimer();

  while (!Serial && millis() < 1500) {}

  Serial.println("BERT26 VCU starting");
  Serial.print("[Startup] BMS/IMD fault detection delayed for ");
  Serial.print(STARTUP_BMS_IMD_FAULT_DELAY_MS);
  Serial.println(" ms");

  setupDisplay();

  drawDashboard();
}

// ---------------- Main loop ----------------
void loop() {
  handleSerialFanDutyInput();
  updateFanDutyFromTemps();
  serviceFans();
  receiveCanFrames();
  updatePedal();
  updateBrakePressureSensors();
  digitalWrite(BRAKE_LIGHT_PIN, braking ? HIGH : LOW);
  logAppsTelemetry();
  serviceReadyToDrive();
  serviceBuzzer();
  serviceTssi();
  serviceFaultBanner();
  handleTouch();

  uint32_t now = millis();
  if (bmsImdFaultDetectionArmed() && haveBmsStatus && (now - lastBmsRxMs > BMS_TIMEOUT_MS)) {
    if (!bmsTimedOut) {
      Serial.println("[BMS] Status timeout"); //,disabling torque
      showFaultBanner("BMS Timeout");
      screenDirty = true;
    }
    bmsTimedOut = true;
    bmsActive = false;
    bmsFault = true;
    dropReadyToDrive("BMS Timeout");
  }

  if (haveInverterVoltageInfo && haveInverterCurrentInfo) {
    powerKw = (inverterDcBusVoltage * inverterDcBusCurrent) / 1000.0f;
  } else {
    powerKw = 0.0f;
  }
  speedMph = (int)roundf((float)motorRpm * MPH_PER_RPM);
  updatePm100CommandState();
  servicePm100TxLogging();
  servicePm100BroadcastEnable();

  static uint32_t lastDrawMs = 0;
  if (screenDirty || now - lastDrawMs >= SCREEN_PERIOD_MS) {
    lastDrawMs = now;
    screenDirty = false;
    drawDashboard();
  }
}

// ---------------- Fans ----------------
void setupFans() {
  pinMode(FAN_1_PIN, OUTPUT);
  pinMode(FAN_2_PIN, OUTPUT);
  analogWriteFrequency(FAN_1_PIN, FAN_PWM_FREQUENCY_HZ);
  analogWriteFrequency(FAN_2_PIN, FAN_PWM_FREQUENCY_HZ);
  serviceFans();
}

void updateFanDutyFromTemps() {
  if (fanManualOverride) {
    return;
  }

  // Drive the fans from the hotter of the PM100 motor and inverter temps.
  bool haveTemp = false;
  float tempC = 0.0f;
  if (haveInverterTemp) {
    tempC = (float)inverterTempTenthsC / 10.0f;
    haveTemp = true;
  }
  if (havePm100MotorTemp) {
    float motorTempC = (float)pm100MotorTempTenthsC / 10.0f;
    if (!haveTemp || motorTempC > tempC) {
      tempC = motorTempC;
    }
    haveTemp = true;
  }

  int duty;
  if (!haveTemp || tempC <= FAN_TEMP_MIN_C) {
    duty = FAN_DUTY_MIN_PERCENT;
  } else if (tempC >= FAN_TEMP_MAX_C) {
    duty = FAN_DUTY_MAX_PERCENT;
  } else {
    float span = FAN_TEMP_MAX_C - FAN_TEMP_MIN_C;
    float fraction = (tempC - FAN_TEMP_MIN_C) / span;
    duty = FAN_DUTY_MIN_PERCENT +
           (int)(fraction * (float)(FAN_DUTY_MAX_PERCENT - FAN_DUTY_MIN_PERCENT) + 0.5f);
  }

  fan1DutyPercent = duty;
  fan2DutyPercent = duty;
}

void serviceFans() {
  int duty1 = constrain(fan1DutyPercent, 0, 100);
  int duty2 = constrain(fan2DutyPercent, 0, 100);

  if (duty1 != fan1LastDutyPercent) {
    fan1LastDutyPercent = duty1;
    analogWrite(FAN_1_PIN, dutyPercentToFanPwmWrite(duty1));
  }

  if (duty2 != fan2LastDutyPercent) {
    fan2LastDutyPercent = duty2;
    analogWrite(FAN_2_PIN, dutyPercentToFanPwmWrite(duty2));
  }
}

uint8_t dutyPercentToFanPwmWrite(int dutyPercent) {
  int duty = constrain(dutyPercent, 0, 100);
  return (uint8_t)((duty * FAN_PWM_WRITE_MAX + 50) / 100);
}

// ---------------- Display / touch ----------------
void setupDisplay() {
  SPI.setMOSI(TFT_MOSI);
  SPI.setMISO(TFT_MISO);
  SPI.setSCK(TFT_SCK);

  pinMode(TFT_RST, OUTPUT);
  digitalWrite(TFT_RST, HIGH);

  pinMode(TFT_LED, OUTPUT);
  analogWriteFrequency(TFT_LED, 20000);
  analogWrite(TFT_LED, backlightBrightness);

  resetDisplay();

  ts.begin();
  ts.setRotation(1);

  tft.init(320, 480);
  tft.setRotation(135);
  tft.fillScreen(ST7735_BLACK);
  dashboardNeedsFullRedraw = true;
  screenDirty = true;
}

void resetDisplay() {
  digitalWrite(TFT_RST, HIGH);
  delay(20);
  digitalWrite(TFT_RST, LOW);
  delay(20);
  digitalWrite(TFT_RST, HIGH);
  delay(150);
}

void handleSerialFanDutyInput() {
  static char input[8];
  static uint8_t index = 0;

  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c >= '0' && c <= '9') {
      if (index < sizeof(input) - 1) {
        input[index++] = c;
      }
    } else if (c == 'a' || c == 'A') {
      // Return fan control to automatic temperature-based duty.
      fanManualOverride = false;
      Serial.println("[Fans] Automatic temperature control resumed");
      index = 0;
    } else if (c == '\n' || c == '\r') {
      if (index > 0) {
        input[index] = '\0';
        int value = atoi(input);
        // Manual override: enter 0-100 to force both fan PWM duties.
        // Send 'a' to return to automatic temperature control.
        if (value >= 0 && value <= 100) {
          fanManualOverride = true;
          fan1DutyPercent = value;
          fan2DutyPercent = value;
          Serial.print("[Fans] Manual override duty set to ");
          Serial.print(value);
          Serial.println("% (send 'a' for auto)");
        } else {
          Serial.println("[Fans] Enter a duty percent from 0 to 100, or 'a' for auto");
        }
        index = 0;
      }
    } else {
      index = 0;
    }
  }
}

void handleTouch() {
  // RTD owns driveMode: D while ready to drive, N in every other RTD state.
  return;

  if (!ts.touched()) {
    return;
  }

  TS_Point p = ts.getPoint();
  char newMode = checkIfTouchedRaw(p.x, p.y);
  if (newMode && newMode != driveMode) {
    setDriveMode(newMode);

    Serial.print("[Touch] Drive mode ");
    Serial.println(driveMode);
  }
}

char checkIfTouchedRaw(int x, int y) {
  const int yMin = 2650;
  const int yMax = 3500;

  if (x > 950 && x < 1550 && y > yMin && y < yMax) return 'D';
  if (x > 1900 && x < 2500 && y > yMin && y < yMax) return 'N';
  if (x > 2900 && x < 3500 && y > yMin && y < yMax) return 'R';

  return 0;
}

void drawDashboard() {
  static char lastSpeed[12] = "";
  static char lastPedal[16] = "";
  static char lastTorque[16] = "";
  static char lastPower[16] = "";
  static char lastSoc[16] = "";
  static char lastBatteryTemp[16] = "";
  static char lastBms[16] = "";
  static char lastRtd[16] = "";
  static char lastFault[16] = "";
  static char lastRpm[16] = "";
  static char lastMinCellVoltage[16] = "";
  static char lastMaxCellVoltage[16] = "";
  static char lastAvgCellVoltage[16] = "";
  static char lastMinCellTemp[16] = "";
  static char lastMaxCellTemp[16] = "";
  static char lastAvgCellTemp[16] = "";
  static char lastInverterTemp[16] = "";
  static char lastPm100MotorTemp[16] = "";
  static char lastFanDuty[16] = "";
  static uint16_t lastSpeedColor = ST7735_WHITE;
  static uint16_t lastPedalColor = ST7735_WHITE;
  static uint16_t lastTorqueColor = ST7735_WHITE;
  static uint16_t lastPowerColor = ST7735_WHITE;
  static uint16_t lastSocColor = ST7735_WHITE;
  static uint16_t lastBatteryTempColor = ST7735_WHITE;
  static uint16_t lastBmsColor = ST7735_WHITE;
  static uint16_t lastRtdColor = ST7735_WHITE;
  static uint16_t lastFaultColor = ST7735_WHITE;
  static uint16_t lastRpmColor = ST7735_WHITE;
  static uint16_t lastMinCellVoltageColor = ST7735_WHITE;
  static uint16_t lastMaxCellVoltageColor = ST7735_WHITE;
  static uint16_t lastAvgCellVoltageColor = ST7735_WHITE;
  static uint16_t lastMinCellTempColor = ST7735_WHITE;
  static uint16_t lastMaxCellTempColor = ST7735_WHITE;
  static uint16_t lastAvgCellTempColor = ST7735_WHITE;
  static uint16_t lastInverterTempColor = ST7735_WHITE;
  static uint16_t lastPm100MotorTempColor = ST7735_WHITE;
  static uint16_t lastFanDutyColor = ST7735_WHITE;
  static char lastContactor[8] = "";
  static uint16_t lastContactorColor = ST7735_WHITE;
  static char lastGlv[16] = "";
  static uint16_t lastGlvColor = ST7735_WHITE;
  static char lastBusVoltage[16] = "";
  static uint16_t lastBusVoltageColor = ST7735_WHITE;

  bool force = dashboardNeedsFullRedraw;

  if (force) {
    tft.fillScreen(ST7735_BLACK);
    tft.setTextColor(ST7735_WHITE);
    tft.setTextSize(2);
    tft.setCursor(14, 10);
    tft.print("BERT26 VCU");

    tft.setCursor(225, 77);
    tft.print("mph");

    drawDashboardLabel(18, 125, "Pedal");
    drawDashboardLabel(18, 150, "Cmd Nm");
    drawDashboardLabel(18, 175, "Power");
    drawDashboardLabel(18, 200, "SOC");
    drawDashboardLabel(18, 225, "Batt T");

    drawDashboardLabel(170, 125, "BMS");
    drawDashboardLabel(170, 150, "INV");
    tft.setTextSize(1);
    tft.setTextColor(ST7735_WHITE);
    tft.setCursor(170, 179);
    tft.print("FAULT:");
    drawDashboardLabel(170, 200, "RPM");

    tft.setTextSize(1);
    tft.setTextColor(ST7735_WHITE);
    tft.setCursor(335, 42);
    tft.print("CELL STATS");
    tft.setCursor(335, 62);
    tft.print("V MAX:");
    tft.setCursor(335, 78);
    tft.print("V MIN:");
    tft.setCursor(335, 94);
    tft.print("V AVG:");
    tft.setCursor(335, 118);
    tft.print("T MAX:");
    tft.setCursor(335, 134);
    tft.print("T MIN:");
    tft.setCursor(335, 150);
    tft.print("T AVG:");
    tft.setCursor(335, 174);
    tft.print("INV T:");
    tft.setCursor(335, 190);
    tft.print("MTR T:");
    tft.setCursor(335, 206);
    tft.print("FAN:");
    tft.setCursor(335, 222);
    tft.print("CTR:");
    tft.setCursor(335, 238);
    tft.print("GLV:");
    tft.setCursor(335, 254);
    tft.print("BUS:");

    dashboardNeedsFullRedraw = false;
  }

  char value[16];

  snprintf(value, sizeof(value), "%d", speedMph);
  drawCachedDashboardText(78, 43, 140, 64, 6, value, ST7735_WHITE,
                          lastSpeed, sizeof(lastSpeed), lastSpeedColor, force);

  snprintf(value, sizeof(value), "%.0f%%", pedalPct);
  drawCachedDashboardText(102, 125, 64, 18, 2, value, appsPlausible ? ST7735_GREEN : ST7735_RED,
                          lastPedal, sizeof(lastPedal), lastPedalColor, force);

  // Shows the torque actually loaded into the PM100 command frame; red while
  // RTD is latched but torque is being suppressed (BSPC / APPS fault).
  {
    uint16_t torqueColor;
    if (!readyToDriveLatched) {
      torqueColor = ST7735_YELLOW;
    } else if (bspcActive || !appsPlausible) {
      torqueColor = ST7735_RED;
    } else {
      torqueColor = ST7735_GREEN;
    }
    snprintf(value, sizeof(value), "%.1f", commandedTorqueNm);
    drawCachedDashboardText(114, 150, 64, 18, 2, value, torqueColor,
                            lastTorque, sizeof(lastTorque), lastTorqueColor, force);
  }

  bool haveInverterPower = haveInverterVoltageInfo && haveInverterCurrentInfo;
  if (haveInverterPower) {
    snprintf(value, sizeof(value), "%d kW", (int)roundf(powerKw));
  } else {
    snprintf(value, sizeof(value), "-- kW");
  }
  drawCachedDashboardText(102, 175, 64, 18, 2, value,
                          haveInverterPower ? ST7735_WHITE : ST7735_YELLOW,
                          lastPower, sizeof(lastPower), lastPowerColor, force);

  snprintf(value, sizeof(value), "%d%%", stateOfChargePct);
  drawCachedDashboardText(78, 200, 88, 18, 2, value, ST7735_GREEN,
                          lastSoc, sizeof(lastSoc), lastSocColor, force);

  snprintf(value, sizeof(value), "%d C", batteryTempC);
  drawCachedDashboardText(114, 225, 54, 18, 2, value, ST7735_WHITE,
                          lastBatteryTemp, sizeof(lastBatteryTemp), lastBatteryTempColor, force);

  const char *bmsText = "WAIT";
  uint16_t bmsColor = ST7735_YELLOW;
  bool faultDetectionArmed = bmsImdFaultDetectionArmed();
  bool bms2026DisplayFault = haveBms2026FaultCode && bms2026FaultCode != 0;
  if (!faultDetectionArmed) {
    bmsText = "STARTUP";
    bmsColor = ST7735_YELLOW;
  } else if (bmsTimedOut) {
    bmsText = "TIMEOUT";
    bmsColor = ST7735_RED;
  } else if (bmsFault || bms2026DisplayFault) {
    bmsText = "FAULT";
    bmsColor = ST7735_RED;
  } else if (bmsActive) {
    bmsText = "ACTIVE";
    bmsColor = ST7735_GREEN;
  }
  drawCachedDashboardText(230, 125, 90, 18, 2, bmsText, bmsColor,
                          lastBms, sizeof(lastBms), lastBmsColor, force);

  {
    const char *invText;
    uint16_t invColor;
    if (!havePm100FaultStatus) {
      invText = "--";
      invColor = ST7735_YELLOW;
    } else if (pm100Fault) {
      invText = firstPm100FaultName(pm100PostFaults, pm100RunFaults);
      invColor = ST7735_RED;
    } else {
      invText = "No Fault";
      invColor = ST7735_GREEN;
    }
    drawCachedDashboardText(230, 150, 90, 18, 1, invText, invColor,
                            lastRtd, sizeof(lastRtd), lastRtdColor, force);
  }

  const char *faultText = dashboardFaultText();
  bool faultActive = pm100Fault ||
                     (faultDetectionArmed &&
                      (anyFaultActive() || bmsTimedOut || imdCanTimedOut || bms2026DisplayFault));
  uint16_t faultColor = faultActive ? ST7735_RED : (faultDetectionArmed ? ST7735_GREEN : ST7735_YELLOW);
  drawCachedDashboardText(214, 179, 106, 18, 1, faultText,
                          faultColor,
                          lastFault, sizeof(lastFault), lastFaultColor, force);

  snprintf(value, sizeof(value), "%d", motorRpm);
  drawCachedDashboardText(230, 200, 90, 18, 2, value, ST7735_WHITE,
                          lastRpm, sizeof(lastRpm), lastRpmColor, force);

  formatCellVoltageText(value, sizeof(value), maxCellVoltageMv, haveCellVoltageStats);
  drawCachedDashboardText(382, 62, 78, 10, 1, value, haveCellVoltageStats ? ST7735_WHITE : ST7735_YELLOW,
                          lastMaxCellVoltage, sizeof(lastMaxCellVoltage), lastMaxCellVoltageColor, force);

  formatCellVoltageText(value, sizeof(value), minCellVoltageMv, haveCellVoltageStats);
  drawCachedDashboardText(382, 78, 78, 10, 1, value, haveCellVoltageStats ? ST7735_WHITE : ST7735_YELLOW,
                          lastMinCellVoltage, sizeof(lastMinCellVoltage), lastMinCellVoltageColor, force);

  formatCellVoltageText(value, sizeof(value), avgCellVoltageMv, haveCellVoltageStats);
  drawCachedDashboardText(382, 94, 78, 10, 1, value, haveCellVoltageStats ? ST7735_WHITE : ST7735_YELLOW,
                          lastAvgCellVoltage, sizeof(lastAvgCellVoltage), lastAvgCellVoltageColor, force);

  formatCellTempText(value, sizeof(value), maxCellTempC, haveCellTempStats);
  drawCachedDashboardText(382, 118, 78, 10, 1, value, haveCellTempStats ? ST7735_WHITE : ST7735_YELLOW,
                          lastMaxCellTemp, sizeof(lastMaxCellTemp), lastMaxCellTempColor, force);

  formatCellTempText(value, sizeof(value), minCellTempC, haveCellTempStats);
  drawCachedDashboardText(382, 134, 78, 10, 1, value, haveCellTempStats ? ST7735_WHITE : ST7735_YELLOW,
                          lastMinCellTemp, sizeof(lastMinCellTemp), lastMinCellTempColor, force);

  formatCellTempText(value, sizeof(value), avgCellTempC, haveCellTempStats);
  drawCachedDashboardText(382, 150, 78, 10, 1, value, haveCellTempStats ? ST7735_WHITE : ST7735_YELLOW,
                          lastAvgCellTemp, sizeof(lastAvgCellTemp), lastAvgCellTempColor, force);

  formatTemperatureTenthsText(value, sizeof(value), inverterTempTenthsC, haveInverterTemp);
  drawCachedDashboardText(382, 174, 78, 10, 1, value, haveInverterTemp ? ST7735_WHITE : ST7735_YELLOW,
                          lastInverterTemp, sizeof(lastInverterTemp), lastInverterTempColor, force);

  formatTemperatureTenthsText(value, sizeof(value), pm100MotorTempTenthsC, havePm100MotorTemp);
  drawCachedDashboardText(382, 190, 78, 10, 1, value, havePm100MotorTemp ? ST7735_WHITE : ST7735_YELLOW,
                          lastPm100MotorTemp, sizeof(lastPm100MotorTemp), lastPm100MotorTempColor, force);

  {
    int fan1Duty = constrain(fan1DutyPercent, 0, 100);
    int fan2Duty = constrain(fan2DutyPercent, 0, 100);
    snprintf(value, sizeof(value), "%d/%d%%", fan1Duty, fan2Duty);
    drawCachedDashboardText(382, 206, 78, 10, 1, value, ST7735_WHITE,
                            lastFanDuty, sizeof(lastFanDuty), lastFanDutyColor, force);
  }

  {
    bool ctrClosed = haveBmsStatus && (bmsFlags & 0x02) != 0;
    const char *ctrText = !haveBmsStatus ? "--" : (ctrClosed ? "CLOSED" : "OPEN");
    uint16_t ctrColor = !haveBmsStatus ? ST7735_YELLOW : (ctrClosed ? ST7735_GREEN : ST7735_RED);
    drawCachedDashboardText(382, 222, 78, 10, 1, ctrText, ctrColor,
                            lastContactor, sizeof(lastContactor), lastContactorColor, force);
  }

  {
    uint16_t glvColor;
    if (!haveGlvVoltage) {
      snprintf(value, sizeof(value), "--");
      glvColor = ST7735_YELLOW;
    } else {
      snprintf(value, sizeof(value), "%.1fV", glvVoltage);
      glvColor = (glvVoltage < GLV_LOW_VOLTAGE_WARN_V) ? ST7735_RED : ST7735_WHITE;
    }
    drawCachedDashboardText(382, 238, 78, 10, 1, value, glvColor,
                            lastGlv, sizeof(lastGlv), lastGlvColor, force);
  }

  {
    uint16_t busColor;
    if (!haveInverterVoltageInfo) {
      snprintf(value, sizeof(value), "--");
      busColor = ST7735_YELLOW;
    } else {
      snprintf(value, sizeof(value), "%.1fV", inverterDcBusVoltage);
      busColor = ST7735_WHITE;
    }
    drawCachedDashboardText(382, 254, 78, 10, 1, value, busColor,
                            lastBusVoltage, sizeof(lastBusVoltage), lastBusVoltageColor, force);
  }

  drawShutdownStatusIndicators(force);

  if (faultBannerVisible) {
    drawFaultBanner(force);
  } else {
    drawDriveButtons(force);
  }
}

void formatCellVoltageText(char *buffer, size_t bufferSize, uint16_t millivolts, bool valid) {
  if (!valid) {
    snprintf(buffer, bufferSize, "--");
    return;
  }

  snprintf(buffer, bufferSize, "%.3fV", (float)millivolts / 1000.0f);
}

void formatCellTempText(char *buffer, size_t bufferSize, int16_t tempC, bool valid) {
  if (!valid || tempC == CELL_TEMP_UNKNOWN_C) {
    snprintf(buffer, bufferSize, "--");
    return;
  }

  snprintf(buffer, bufferSize, "%dC", (int)tempC);
}

void formatTemperatureTenthsText(char *buffer, size_t bufferSize, int16_t tempTenthsC, bool valid) {
  if (!valid) {
    snprintf(buffer, bufferSize, "--");
    return;
  }

  snprintf(buffer, bufferSize, "%.1fC", (float)tempTenthsC / 10.0f);
}

const char *dashboardFaultText() {
  DashboardFaultList faults;
  collectDashboardFaults(faults);

  if (faults.count == 0) {
    if (!bmsImdFaultDetectionArmed()) {
      return "Startup Wait";
    }
    return "No Fault";
  }

  if (faults.count == 1) {
    return faults.texts[0];
  }

  uint8_t faultIndex = (millis() / FAULT_DISPLAY_CYCLE_MS) % faults.count;
  return faults.texts[faultIndex];
}

void collectDashboardFaults(DashboardFaultList &faults) {
  faults.count = 0;
  bool faultDetectionArmed = bmsImdFaultDetectionArmed();

  if (faultDetectionArmed && bmsTimedOut) {
    appendDashboardFault(faults, "BMS Timeout");
  }

  if (faultDetectionArmed && imdCanTimedOut) {
    appendDashboardFault(faults, "IMD Timeout");
  }

  if (faultDetectionArmed) {
    if (haveBms2026FaultCode && bms2026FaultCode != 0) {
      appendDashboardFault(faults, bmsTemperatureFaultCodeName(bms2026FaultCode));
    } else if (bmsFault && haveBmsStatus && !haveBms2026FaultCode) {
      appendDashboardFault(faults, "BMS Fault");
    }
  }

  if (faultDetectionArmed && imdFault && (imdWarnAlarms != 0 || !imdCanTimedOut)) {
    uint8_t beforeCount = faults.count;
    appendTssiFaultNames(faults, imdWarnAlarms);
    if (faults.count == beforeCount) {
      appendDashboardFault(faults, "IMD Fault");
    }
  }

  if (havePm100FaultStatus && pm100Fault) {
    uint8_t beforeCount = faults.count;
    appendPm100FaultNames(faults, pm100PostFaults, pm100RunFaults);
    if (faults.count == beforeCount) {
      appendDashboardFault(faults, "Inv Fault");
    }
  }

  if (!appsPlausible) {
    appendDashboardFault(faults, appsFaultText);
  }

  if (bspcActive) {
    appendDashboardFault(faults, "BSPC Zero Trq");
  }
}

void appendDashboardFault(DashboardFaultList &faults, const char *text) {
  if (faults.count >= DASHBOARD_MAX_FAULT_TEXTS || text == nullptr || text[0] == '\0') {
    return;
  }

  faults.texts[faults.count++] = text;
}

void appendTssiFaultNames(DashboardFaultList &faults, uint16_t bits) {
  if (bits & (1u << 0)) appendDashboardFault(faults, "DeviceError");
  if (bits & (1u << 1)) appendDashboardFault(faults, "HV+ConnFail");
  if (bits & (1u << 2)) appendDashboardFault(faults, "HV-ConnFail");
  if (bits & (1u << 3)) appendDashboardFault(faults, "EarthConnFail");
  if (bits & (1u << 4)) appendDashboardFault(faults, "IsoAlarm");
  if (bits & (1u << 7)) appendDashboardFault(faults, "Unbalance");
  if (bits & (1u << 8)) appendDashboardFault(faults, "Undervolt");
  if (bits & (1u << 9)) appendDashboardFault(faults, "UnsafeStart");
  if (bits & (1u << 10)) appendDashboardFault(faults, "EarthliftOpen");
  if (bits & (1u << 5)) appendDashboardFault(faults, "IsoWarning");
  if (bits & (1u << 6)) appendDashboardFault(faults, "IsoOutdated");
}

void appendPm100FaultNames(DashboardFaultList &faults, uint32_t postFaults, uint32_t runFaults) {
  for (uint8_t bit = 0; bit < 32; bit++) {
    if (postFaults & (1UL << bit)) {
      appendDashboardFault(faults, pm100PostFaultName(bit));
    }
  }

  for (uint8_t bit = 0; bit < 32; bit++) {
    if (runFaults & (1UL << bit)) {
      appendDashboardFault(faults, pm100RunFaultName(bit));
    }
  }
}

const char *firstTssiFaultName(uint16_t bits) {
  if (bits & (1u << 0)) return "DeviceError";
  if (bits & (1u << 1)) return "HV+ConnFail";
  if (bits & (1u << 2)) return "HV-ConnFail";
  if (bits & (1u << 3)) return "EarthConnFail";
  if (bits & (1u << 4)) return "IsoAlarm";
  if (bits & (1u << 7)) return "Unbalance";
  if (bits & (1u << 8)) return "Undervolt";
  if (bits & (1u << 9)) return "UnsafeStart";
  if (bits & (1u << 10)) return "EarthliftOpen";
  if (bits & (1u << 5)) return "IsoWarning";
  if (bits & (1u << 6)) return "IsoOutdated";
  return "Fault";
}

const char *firstPm100FaultName(uint32_t postFaults, uint32_t runFaults) {
  for (uint8_t bit = 0; bit < 32; bit++) {
    if (postFaults & (1UL << bit)) return pm100PostFaultName(bit);
  }
  for (uint8_t bit = 0; bit < 32; bit++) {
    if (runFaults & (1UL << bit)) return pm100RunFaultName(bit);
  }
  return "No Fault";
}

const char *bmsTemperatureFaultCodeName(uint8_t faultCode) {
  switch (faultCode) {
    case 0:  return "No Fault";
    case 1:  return "Cell UV";
    case 2:  return "Cell OV";
    case 3:  return "Pack UV";
    case 4:  return "Pack OV";
    case 5:  return "Over Temp";
    case 6:  return "Cell Spread";
    case 7:  return "Prchg Timeout";
    case 8:  return "Prchg Fast";
    case 9:  return "Pack V Sense";
    case 10: return "Chgr Comms";
    case 11: return "BQ Init Fail";
    case 12: return "BQ Comms Fail";
    default: return "BMS Fault";
  }
}

const char *pm100PostFaultName(uint8_t bit) {
  switch (bit) {
    case 0: return "POST GateDesat";
    case 1: return "POST OverCur";
    case 2: return "POST AccelShrt";
    case 3: return "POST AccelOpen";
    case 4: return "POST CurSensLo";
    case 5: return "POST CurSensHi";
    case 6: return "POST ModTempLo";
    case 7: return "POST ModTempHi";
    case 8: return "POST CtrlTmpLo";
    case 9: return "POST CtrlTmpHi";
    case 10: return "POST GateTmpLo";
    case 11: return "POST GateTmpHi";
    case 12: return "POST 5V Low";
    case 13: return "POST 5V High";
    case 14: return "POST 12V Low";
    case 15: return "POST 12V High";
    case 16: return "POST 2.5V Lo";
    case 17: return "POST 2.5V Hi";
    case 18: return "POST 1.5V Lo";
    case 19: return "POST 1.5V Hi";
    case 20: return "POST DCBusHi";
    case 21: return "POST DCBusLo";
    case 22: return "POST PrechTmo";
    case 23: return "POST PrechVolt";
    case 24: return "POST EEPROMSum";
    case 25: return "POST EEPROMRg";
    case 26: return "POST EEPROMUpd";
    case 27: return "POST HWBusOV";
    case 28: return "POST GateInit";
    case 29: return "POST Reserved";
    case 30: return "POST BrakeShrt";
    case 31: return "POST BrakeOpen";
    default: return "POST Fault";
  }
}

const char *pm100RunFaultName(uint8_t bit) {
  switch (bit) {
    case 0: return "Mtr Overspeed";
    case 1: return "Inv Overcur";
    case 2: return "Inv Overvolt";
    case 3: return "Inv Overtemp";
    case 4: return "Accel Short";
    case 5: return "Accel Open";
    case 6: return "Dir Cmd Fault";
    case 7: return "Inv Resp Tmo";
    case 8: return "Gate/Desat";
    case 9: return "HW Overcur";
    case 10: return "Undervolt";
    case 11: return "CAN Cmd Lost";
    case 12: return "Mtr Overtemp";
    case 13: return "Run Reserved45";
    case 14: return "Run Reserved46";
    case 15: return "Run Reserved47";
    case 16: return "Brake Short";
    case 17: return "Brake Open";
    case 18: return "ModA Overtemp";
    case 19: return "ModB Overtemp";
    case 20: return "ModC Overtemp";
    case 21: return "PCB Overtemp";
    case 22: return "Gate1 Overtemp";
    case 23: return "Gate2 Overtemp";
    case 24: return "Gate3 Overtemp";
    case 25: return "Cur Sensor";
    case 26: return "Gate Overvolt";
    case 27: return "HWBusOV Gen3";
    case 28: return "HWBusOV Gen5";
    case 29: return "Run Reserved61";
    case 30: return "Resolver Disc";
    case 31: return "Run Reserved63";
    default: return "Inv Fault";
  }
}

void drawDashboardLabel(int x, int y, const char *label) {
  tft.setTextSize(2);
  tft.setTextColor(ST7735_WHITE);
  tft.setCursor(x, y);
  tft.print(label);
  tft.print(": ");
}

void drawCachedDashboardText(int x, int y, int w, int h, uint8_t textSize,
                             const char *value, uint16_t valueColor,
                             char *lastValue, size_t lastValueSize,
                             uint16_t &lastColor, bool force) {
  if (!force && strcmp(value, lastValue) == 0 && valueColor == lastColor) {
    return;
  }

  tft.fillRect(x, y, w, h, ST7735_BLACK);
  tft.setTextSize(textSize);
  tft.setTextColor(valueColor);
  tft.setCursor(x, y);
  tft.print(value);
  tft.setTextColor(ST7735_WHITE);

  strncpy(lastValue, value, lastValueSize - 1);
  lastValue[lastValueSize - 1] = '\0';
  lastColor = valueColor;
}

void drawDriveButtons(bool force) {
  static char lastDriveMode = 0;

  if (!force && driveMode == lastDriveMode) {
    return;
  }

  int y = (int)(tft.height() * 0.72f);
  int buttonW = 72;
  int buttonH = 54;
  int startX = 42;
  int gap = 24;

  // Clear only the button band; the cell-stats column at x>=335 shares this row.
  tft.fillRect(0, y - 8, 320, buttonH + 16, ST7735_BLACK);

  const char modes[2] = {'D', 'N'};
  for (int i = 0; i < 2; i++) {
    int x = startX + i * (buttonW + gap);
    bool selected = (driveMode == modes[i]);
    uint16_t color = selected ? ST7735_RED : ST7735_WHITE;

    tft.drawRoundRect(x, y, buttonW, buttonH, 6, color);
    tft.setTextSize(4);
    tft.setTextColor(color);
    tft.setCursor(x + 24, y + 12);
    tft.print(modes[i]);
  }

  tft.setTextColor(ST7735_WHITE);
  lastDriveMode = driveMode;
}

void drawStatusText(int x, int y, const char *label, const char *value, uint16_t valueColor) {
  tft.setTextSize(2);
  tft.setTextColor(ST7735_WHITE);
  tft.setCursor(x, y);
  tft.print(label);
  tft.print(": ");
  tft.setTextColor(valueColor);
  tft.print(value);
  tft.setTextColor(ST7735_WHITE);
}

void drawStatusText(int x, int y, const char *label, int value, const char *units, uint16_t valueColor) {
  char text[16];
  snprintf(text, sizeof(text), "%d%s", value, units);
  drawStatusText(x, y, label, text, valueColor);
}

// Shutdown-circuit status lights: GPIO HIGH = green, LOW = red.
void drawShutdownStatusIndicators(bool force) {
  static int lastBspd = -1;
  static int lastImd = -1;
  static int lastBms = -1;

  drawShutdownStatusIndicator(335, 268, "BSPD", digitalRead(BSPD_STATUS_PIN) == HIGH,
                              lastBspd, force);
  drawShutdownStatusIndicator(384, 268, "IMD", digitalRead(IMD_STATUS_PIN) == HIGH,
                              lastImd, force);
  drawShutdownStatusIndicator(433, 268, "BMS", digitalRead(BMS_STATUS_PIN) == HIGH,
                              lastBms, force);
}

void drawShutdownStatusIndicator(int x, int y, const char *label, bool high,
                                 int &lastState, bool force) {
  if (!force && (int)high == lastState) {
    return;
  }
  lastState = (int)high;

  const int w = 44;
  const int h = 18;
  uint16_t color = high ? ST7735_GREEN : ST7735_RED;
  tft.fillRoundRect(x, y, w, h, 4, color);
  tft.setTextSize(1);
  tft.setTextColor(ST7735_BLACK);
  int textW = (int)strlen(label) * 6;
  tft.setCursor(x + (w - textW) / 2, y + 5);
  tft.print(label);
  tft.setTextColor(ST7735_WHITE);
}

// ---------------- CAN transport ----------------
void setupVehicleCan() {
  VehicleCan.begin();
  VehicleCan.setBaudRate(CAN_BAUD);
  VehicleCan.setMaxMB(16);
  VehicleCan.enableFIFO();

  Serial.println("[CAN] CAN3 started at 500 kbit/s on pins 30=RX, 31=TX");
}

void sendCanFrame(CAN_message_t &msg) {
  noInterrupts();
  VehicleCan.write(msg);
  interrupts();
  logCanFrame("TX", msg);
}

void receiveCanFrames() {
  CAN_message_t rx;
  while (VehicleCan.read(rx)) {
    logCanFrame("RX", rx);
    dispatchReceivedCanFrame(rx);
  }
}

void dispatchReceivedCanFrame(const CAN_message_t &msg) {
  if (!havePm100Traffic && msg.id >= PM100_TEMPERATURE_1_FRAME_ID &&
      msg.id <= PM100_TORQUE_CAPABILITY_FRAME_ID) {
    havePm100Traffic = true;
    firstPm100FrameMs = millis();
  }

  switch (msg.id) {
    case IMD_INFO_GENERAL_ID:
    case IMD_RESPONSE_ID:
      handleImdFrame(msg);
      break;

    case PM100_TEMPERATURE_1_FRAME_ID:
      handlePm100Temperatures1Frame(msg);
      break;

    case PM100_TEMPERATURE_3_FRAME_ID:
      handlePm100Temperatures3Frame(msg);
      break;

    case PM100_RPM_FRAME_ID:
      handlePm100RpmFrame(msg);
      break;

    case PM100_CURRENT_INFO_FRAME_ID:
      handlePm100CurrentInfoFrame(msg);
      break;

    case PM100_VOLTAGE_INFO_FRAME_ID:
      handlePm100VoltageInfoFrame(msg);
      break;

    case PM100_INTERNAL_VOLTAGES_FRAME_ID:
      handlePm100InternalVoltagesFrame(msg);
      break;

    case PM100_FAULT_CODES_FRAME_ID:
      handlePm100FaultCodesFrame(msg);
      break;

    case PM100_PARAM_RESPONSE_ID:
      handlePm100ParamResponseFrame(msg);
      break;

    case BMS_STATUS_2026_ID:
      handleBms2026StatusFrame(msg);
      break;

    case BMS_TEMPERATURE_2026_ID:
      handleBms2026TempFrame(msg);
      break;

    default:
      break;
  }
}

// ---------------- CAN logging ----------------
void logCanFrame(const char *direction, const CAN_message_t &msg) {
  updateCanTranslationState(direction, msg);

  // Future SD-card logging should mirror this raw record and the decoded record here.
  if (CAN_LOG_SERIAL_ENABLED) {
    logCanFrameToSerial(direction, msg);
    logCanTranslationToSerial(direction, msg);
  }
}

void logCanFrameToSerial(const char *direction, const CAN_message_t &msg) {
  Serial.print("[CAN ");
  Serial.print(direction);
  Serial.print("] t=");
  Serial.print(millis());
  Serial.print(" id=0x");
  Serial.print(msg.id, HEX);
  Serial.print(msg.flags.extended ? " ext" : " std");
  Serial.print(" len=");
  Serial.print(msg.len);
  Serial.print(" data=");

  for (uint8_t i = 0; i < msg.len; i++) {
    printCanHexByte(msg.buf[i]);
    if (i + 1 < msg.len) {
      Serial.print(' ');
    }
  }

  Serial.println();
}

// ---------------- CAN decoded logging ----------------
void updateCanTranslationState(const char *direction, const CAN_message_t &msg) {
  (void)direction;

  if (msg.flags.extended || msg.id != BMS_CELL_REQUEST_2026_ID || msg.len < 1) {
    return;
  }

  lastCellDataRequestType = msg.buf[0];
}

void logCanTranslationToSerial(const char *direction, const CAN_message_t &msg) {
  if (!CAN_TRANSLATION_SERIAL_ENABLED) {
    return;
  }

  for (uint8_t i = 0; i < CAN_FRAME_TRANSLATION_COUNT; i++) {
    const CanFrameTranslation &translation = CAN_FRAME_TRANSLATIONS[i];
    if (msg.id != translation.id) {
      continue;
    }

    Serial.print("[CAN ");
    Serial.print(direction);
    Serial.print(" DECODED] ");
    Serial.print(translation.name);
    Serial.print(": ");
    translation.print(msg);
    Serial.println();
    return;
  }
}

void printBmsStatus2026Translation(const CAN_message_t &msg) {
  if (msg.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  uint8_t flags = msg.buf[0];
  uint16_t minCellMv = u16Be(msg.buf[1], msg.buf[2]);
  uint16_t maxCellMv = u16Be(msg.buf[3], msg.buf[4]);
  uint16_t avgCellMv = u16Be(msg.buf[5], msg.buf[6]);

  Serial.print("flags=0x");
  printCanHexByte(flags);
  Serial.print(" fault=");
  Serial.print((flags & (1u << 0)) ? "YES" : "NO");
  Serial.print(" posContactorClosed=");
  Serial.print((flags & (1u << 1)) ? "YES" : "NO");
  Serial.print(" chargeMode=");
  Serial.print((flags & (1u << 2)) ? "YES" : "NO");
  Serial.print(" minCell=");
  Serial.print(minCellMv);
  Serial.print("mV maxCell=");
  Serial.print(maxCellMv);
  Serial.print("mV avgCell=");
  Serial.print(avgCellMv);
  Serial.print("mV soc=");
  if (msg.buf[7] == 0xFF) {
    Serial.print("unknown");
  } else {
    Serial.print(msg.buf[7]);
    Serial.print('%');
  }
}

void printBmsTemperature2026Translation(const CAN_message_t &msg) {
  if (msg.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  uint8_t faultCode = msg.buf[3];
  uint16_t packCurrentRaw = u16Be(msg.buf[4], msg.buf[5]);
  float packCurrentA = ((int32_t)packCurrentRaw - 3000) * 0.1f;

  Serial.print("minCellTemp=");
  Serial.print((int)msg.buf[0] - 40);
  Serial.print("C maxCellTemp=");
  Serial.print((int)msg.buf[1] - 40);
  Serial.print("C avgCellTemp=");
  Serial.print((int)msg.buf[2] - 40);
  Serial.print("C faultCode=");
  Serial.print(faultCode);
  Serial.print('(');
  printBmsTemperatureFaultCode(faultCode);
  Serial.print(") packCurrent=");
  Serial.print(packCurrentA, 1);
  Serial.print("A reserved=");
  printCanHexByte(msg.buf[6]);
  Serial.print(' ');
  printCanHexByte(msg.buf[7]);
}

void printCellDataRequest2026Translation(const CAN_message_t &msg) {
  if (msg.len < 1) {
    Serial.print("short frame, expected request byte");
    return;
  }

  Serial.print("request=0x");
  printCanHexByte(msg.buf[0]);
  Serial.print(" (");
  printCellDataRequestName(msg.buf[0]);
  Serial.print(')');
}

void printCellDataResponse2026Translation(const CAN_message_t &msg) {
  if (msg.len < 1) {
    Serial.print("short frame, expected frame index");
    return;
  }

  uint8_t frameIndex = msg.buf[0];

  Serial.print("frameIndex=");
  Serial.print(frameIndex);

  if (lastCellDataRequestType == CELL_DATA_REQUEST_VOLTAGES) {
    uint8_t availablePairs = (msg.len > 1) ? ((msg.len - 1) / 2) : 0;
    if (availablePairs > 3) {
      availablePairs = 3;
    }

    Serial.print(" mode=voltages cells=");
    for (uint8_t i = 0; i < availablePairs; i++) {
      if (i > 0) {
        Serial.print(", ");
      }
      Serial.print('#');
      Serial.print((uint16_t)frameIndex * 3u + i + 1u);
      Serial.print('=');
      printCellVoltageValue(u16Be(msg.buf[1 + (i * 2)], msg.buf[2 + (i * 2)]));
    }
    if (availablePairs == 0) {
      Serial.print("none");
    }
    if (msg.len > 7) {
      Serial.print(" extra=");
      printCanHexByte(msg.buf[7]);
    }
  } else if (lastCellDataRequestType == CELL_DATA_REQUEST_TEMPERATURES) {
    uint8_t availableTemps = (msg.len > 1) ? (msg.len - 1) : 0;
    if (availableTemps > 7) {
      availableTemps = 7;
    }

    Serial.print(" mode=temperatures cells=");
    for (uint8_t i = 0; i < availableTemps; i++) {
      if (i > 0) {
        Serial.print(", ");
      }
      Serial.print('#');
      Serial.print((uint16_t)frameIndex * 7u + i + 1u);
      Serial.print('=');
      Serial.print((int)msg.buf[1 + i] - 40);
      Serial.print('C');
    }
    if (availableTemps == 0) {
      Serial.print("none");
    }
  } else {
    Serial.print(" mode=unknown lastRequest=0x");
    printCanHexByte(lastCellDataRequestType);
    Serial.print(" payload=");
    printRawPayloadFrom(msg, 1);
  }
}

void printImdRequestTranslation(const CAN_message_t &msg) {
  if (msg.len < 1) {
    Serial.print("short frame, expected request index");
    return;
  }

  Serial.print("index=0x");
  printCanHexByte(msg.buf[0]);
  Serial.print(" (");
  printImdRequestIndexName(msg.buf[0]);
  Serial.print(") data=");
  printRawPayloadFrom(msg, 1);
}

void printImdResponseTranslation(const CAN_message_t &msg) {
  if (msg.len < 1) {
    Serial.print("short frame, expected response index");
    return;
  }

  uint8_t index = msg.buf[0];

  if (index == 0xFF) {
    Serial.print("error");
    if (msg.len >= 2) {
      Serial.print(" code=0x");
      printCanHexByte(msg.buf[1]);
      Serial.print('(');
      printImdResponseErrorName(msg.buf[1]);
      Serial.print(')');
    }
    if (msg.len >= 3) {
      Serial.print(" requestedIndex=0x");
      printCanHexByte(msg.buf[2]);
      Serial.print('(');
      printImdRequestIndexName(msg.buf[2]);
      Serial.print(')');
    }
    return;
  }

  Serial.print("index=0x");
  printCanHexByte(index);
  Serial.print('(');
  printImdRequestIndexName(index);
  Serial.print(") ");

  if (msg.len < 2) {
    Serial.print("missing response data");
    return;
  }

  uint16_t word = (msg.len >= 3) ? u16Le(msg.buf[1], msg.buf[2]) : msg.buf[1];

  switch (index) {
    case 0x0A:
    case 0x0C:
    case 0x0E:
    case 0x1E:
    case 0x20:
    case 0x22:
      Serial.print("value=");
      printImdWordValue(word);
      break;

    case 0x10:
    case 0x12:
    case 0x14:
    case 0x16:
    case 0x18:
    case 0x1A:
    case 0x1C:
      Serial.print("ascii=");
      printImdAsciiPayloadFrom(msg, 1);
      break;

    case 0x2A:
    case 0x2E:
      Serial.print("percent=");
      printImdPercentValue(msg.buf[1]);
      break;

    case 0x2C:
    case 0x36:
    case 0x54:
    case 0x5C:
      Serial.print("counter=");
      printImdCounterValue(msg.buf[1]);
      break;

    case 0x30:
      Serial.print("selfHoldingAlarm=0x");
      printCanHexByte(msg.buf[1]);
      Serial.print('(');
      switch (msg.buf[1]) {
        case 0xFC: Serial.print("automatic alarm reset"); break;
        case 0xFD: Serial.print("self-holding alarm"); break;
        case 0xFE: Serial.print("reserved"); break;
        case IMD_SNV_BYTE: Serial.print("SNV"); break;
        default: Serial.print("unknown"); break;
      }
      Serial.print(')');
      break;

    case 0x38:
    case 0x3A:
      Serial.print("profile=");
      Serial.print(msg.buf[1]);
      Serial.print('(');
      printImdProfileName(msg.buf[1]);
      Serial.print(')');
      break;

    case 0x3E:
      Serial.print("quality=");
      printImdPercentValue(msg.buf[1]);
      break;

    case 0x40:
    case 0x42:
    case 0x46:
    case 0x4A:
    case 0x4C:
    case 0x4E:
    case 0x7E:
      Serial.print("resistance=");
      printImdResistanceValue(word);
      break;

    case 0x44:
      Serial.print("rIsoStatus=0x");
      printCanHexByte(msg.buf[1]);
      Serial.print('(');
      printImdRIsoStatusName(msg.buf[1]);
      Serial.print(')');
      break;

    case 0x48:
    case 0x50:
      Serial.print("seconds=");
      printImdWordValue(word);
      break;

    case 0x52:
      Serial.print("capacity=");
      printImdCapacityValue(word);
      break;

    case 0x58:
      Serial.print("selfTestPeriod=");
      if (word == 0) {
        Serial.print("disabled");
      } else if (word == IMD_SNV_WORD) {
        Serial.print("SNV");
      } else {
        Serial.print((uint32_t)word * 10u);
        Serial.print("s");
      }
      break;

    case 0x5E:
    case 0x60:
    case 0x62:
      Serial.print("voltage=");
      printImdVoltageValue(word);
      break;

    case 0x64:
      Serial.print("mode=0x");
      printCanHexByte(msg.buf[1]);
      Serial.print('(');
      printImdVoltageModeName(msg.buf[1]);
      Serial.print(')');
      break;

    case 0x66:
    case 0x72:
      Serial.print("voltage=");
      printImdWordValue(word);
      Serial.print("V");
      break;

    case 0x68:
      Serial.print("deviceActivity=0x");
      printCanHexByte(msg.buf[1]);
      Serial.print('(');
      printImdDeviceActivityName(msg.buf[1]);
      Serial.print(')');
      break;

    case 0x6A:
    case 0x6B:
      Serial.print("lock=0x");
      printCanHexByte(msg.buf[1]);
      Serial.print('(');
      printImdLockStateName(msg.buf[1]);
      Serial.print(')');
      break;

    case 0x6C:
      Serial.print("warningsAndAlarms=0x");
      Serial.print(word, HEX);
      printTssiWarnBits(word);
      break;

    case 0x70:
    case 0x71:
      Serial.print("earthlift=0x");
      printCanHexByte(msg.buf[1]);
      Serial.print('(');
      printImdEarthliftStateName(msg.buf[1]);
      Serial.print(')');
      break;

    case 0x74:
      Serial.print("maxVoltageDifference=");
      if (word == IMD_SNV_WORD) {
        Serial.print("SNV");
      } else {
        Serial.print((float)word * 0.01f, 2);
        Serial.print("V");
      }
      break;

    case 0x76:
    case 0x77:
      Serial.print("message=");
      if (msg.len >= 2) {
        printImdInfoMessageSelectorName(msg.buf[1]);
      } else {
        Serial.print("missing");
      }
      Serial.print(" canId=");
      if (msg.len >= 3) {
        Serial.print("0x");
        printCanHexByte(msg.buf[2]);
      } else {
        Serial.print("missing");
      }
      break;

    case 0x78:
    case 0x79:
      Serial.print("message=");
      if (msg.len >= 2) {
        printImdInfoMessageSelectorName(msg.buf[1]);
      } else {
        Serial.print("missing");
      }
      Serial.print(" period=");
      if (msg.len >= 3) {
        if (msg.buf[2] == 0) {
          Serial.print("deactivated");
        } else if (msg.buf[2] == IMD_SNV_BYTE) {
          Serial.print("SNV");
        } else {
          Serial.print((uint16_t)msg.buf[2] * 100u);
          Serial.print("ms");
        }
      } else {
        Serial.print("missing");
      }
      break;

    case 0x7B:
      Serial.print("baudRate=");
      printImdBaudRateName(msg.buf[1]);
      break;

    default:
      Serial.print("payload=");
      printRawPayloadFrom(msg, 1);
      break;
  }
}

void printImdInfoGeneralTranslation(const CAN_message_t &msg) {
  if (msg.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  uint16_t rIsoCorrected = u16Le(msg.buf[0], msg.buf[1]);
  uint8_t rIsoStatus = msg.buf[2];
  uint8_t isolationCounter = msg.buf[3];
  uint8_t warningsLowByte = msg.buf[4];
  uint8_t reservedByte = msg.buf[5];
  uint8_t deviceActivity = msg.buf[6];

  Serial.print("rIsoCorrected=");
  printImdResistanceValue(rIsoCorrected);
  Serial.print(" rIsoStatus=0x");
  printCanHexByte(rIsoStatus);
  Serial.print('(');
  printImdRIsoStatusName(rIsoStatus);
  Serial.print(") isolationCounter=");
  printImdCounterValue(isolationCounter);
  Serial.print(" warningsLow=0x");
  printCanHexByte(warningsLowByte);
  printTssiWarnBits(warningsLowByte);
  Serial.print(" reserved5=0x");
  printCanHexByte(reservedByte);
  Serial.print(" deviceActivity=0x");
  printCanHexByte(deviceActivity);
  Serial.print('(');
  printImdDeviceActivityName(deviceActivity);
  Serial.print(") byte7=0x");
  printCanHexByte(msg.buf[7]);
}

void printImdInfoIsolationDetailTranslation(const CAN_message_t &msg) {
  if (msg.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  Serial.print("rIsoNeg=");
  printImdResistanceValue(u16Le(msg.buf[0], msg.buf[1]));
  Serial.print(" rIsoPos=");
  printImdResistanceValue(u16Le(msg.buf[2], msg.buf[3]));
  Serial.print(" rIsoOriginal=");
  printImdResistanceValue(u16Le(msg.buf[4], msg.buf[5]));
  Serial.print(" isolationCounter=");
  printImdCounterValue(msg.buf[6]);
  Serial.print(" quality=");
  printImdPercentValue(msg.buf[7]);
}

void printImdInfoVoltageTranslation(const CAN_message_t &msg) {
  if (msg.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  Serial.print("hvSystem=");
  printImdVoltageValue(u16Le(msg.buf[0], msg.buf[1]));
  Serial.print(" hvNegToEarth=");
  printImdVoltageValue(u16Le(msg.buf[2], msg.buf[3]));
  Serial.print(" hvPosToEarth=");
  printImdVoltageValue(u16Le(msg.buf[4], msg.buf[5]));
  Serial.print(" voltageCounter=");
  printImdCounterValue(msg.buf[6]);
  Serial.print(" byte7=0x");
  printCanHexByte(msg.buf[7]);
}

void printImdInfoItSystemTranslation(const CAN_message_t &msg) {
  if (msg.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  Serial.print("capacity=");
  printImdCapacityValue(u16Le(msg.buf[0], msg.buf[1]));
  Serial.print(" capacityCounter=");
  printImdCounterValue(msg.buf[2]);
  Serial.print(" unbalance=");
  printImdPercentValue(msg.buf[3]);
  Serial.print(" unbalanceCounter=");
  printImdCounterValue(msg.buf[4]);
  Serial.print(" byte5=0x");
  printCanHexByte(msg.buf[5]);
  Serial.print(" byte6=0x");
  printCanHexByte(msg.buf[6]);
  Serial.print(" byte7=0x");
  printCanHexByte(msg.buf[7]);
}

void printPm100CommandTranslation(const CAN_message_t &msg) {
  if (msg.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  int16_t torqueRaw = s16Le(msg.buf[0], msg.buf[1]);
  int16_t speedCommandRpm = s16Le(msg.buf[2], msg.buf[3]);
  uint8_t directionCommand = msg.buf[4];
  uint8_t inverterEnable = msg.buf[5];
  uint8_t inverterDischarge = msg.buf[6];
  uint8_t speedModeEnable = msg.buf[7];

  Serial.print("torqueCommand=");
  Serial.print((float)torqueRaw / 10.0f, 1);
  Serial.print("Nm speedCommand=");
  Serial.print(speedCommandRpm);
  Serial.print("rpm direction=0x");
  printCanHexByte(directionCommand);
  Serial.print('(');
  if (directionCommand == 0x00) {
    Serial.print("reverse/clockwise");
  } else if (directionCommand == 0x01) {
    Serial.print("forward/counterclockwise");
  } else {
    Serial.print("unknown");
  }
  Serial.print(") inverterEnable=");
  Serial.print(inverterEnable ? "ON" : "OFF");
  Serial.print(" inverterEnableRaw=0x");
  printCanHexByte(inverterEnable);
  Serial.print(" inverterDischarge=0x");
  printCanHexByte(inverterDischarge);
  Serial.print(" speedModeEnable=0x");
  printCanHexByte(speedModeEnable);
}

void printPm100Temperatures1Translation(const CAN_message_t &msg) {
  if (msg.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  int16_t moduleATempRaw = s16Le(msg.buf[0], msg.buf[1]);
  int16_t moduleBTempRaw = s16Le(msg.buf[2], msg.buf[3]);
  int16_t moduleCTempRaw = s16Le(msg.buf[4], msg.buf[5]);
  int16_t gateDriverTempRaw = s16Le(msg.buf[6], msg.buf[7]);

  Serial.print("moduleA=");
  Serial.print((float)moduleATempRaw / 10.0f, 1);
  Serial.print("C moduleB=");
  Serial.print((float)moduleBTempRaw / 10.0f, 1);
  Serial.print("C moduleC=");
  Serial.print((float)moduleCTempRaw / 10.0f, 1);
  Serial.print("C gateDriver=");
  Serial.print((float)gateDriverTempRaw / 10.0f, 1);
  Serial.print("C");
}

void printPm100Temperatures3Translation(const CAN_message_t &msg) {
  if (msg.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  int16_t coolantTempRaw = s16Le(msg.buf[0], msg.buf[1]);
  int16_t hotSpotTempRaw = s16Le(msg.buf[2], msg.buf[3]);
  int16_t motorTempRaw = s16Le(msg.buf[4], msg.buf[5]);
  int16_t torqueShudderRaw = s16Le(msg.buf[6], msg.buf[7]);

  Serial.print("coolant=");
  Serial.print((float)coolantTempRaw / 10.0f, 1);
  Serial.print("C hotSpot=");
  Serial.print((float)hotSpotTempRaw / 10.0f, 1);
  Serial.print("C motor=");
  Serial.print((float)motorTempRaw / 10.0f, 1);
  Serial.print("C torqueShudder=");
  Serial.print((float)torqueShudderRaw / 10.0f, 1);
  Serial.print("Nm");
}

void printPm100MotorPositionTranslation(const CAN_message_t &msg) {
  if (msg.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  int16_t motorAngleRaw = s16Le(msg.buf[0], msg.buf[1]);
  int16_t motorSpeedRpm = s16Le(msg.buf[2], msg.buf[3]);
  int16_t electricalFrequencyRaw = s16Le(msg.buf[4], msg.buf[5]);
  int16_t deltaResolverRaw = s16Le(msg.buf[6], msg.buf[7]);

  Serial.print("motorAngle=");
  Serial.print((float)motorAngleRaw / 10.0f, 1);
  Serial.print("deg motorSpeed=");
  Serial.print(motorSpeedRpm);
  Serial.print("rpm electricalFrequency=");
  Serial.print((float)electricalFrequencyRaw / 10.0f, 1);
  Serial.print("Hz deltaResolver=");
  Serial.print((float)deltaResolverRaw / 10.0f, 1);
  Serial.print("deg");
}

void printPm100CurrentInfoTranslation(const CAN_message_t &msg) {
  if (msg.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  int16_t phaseACurrentRaw = s16Le(msg.buf[0], msg.buf[1]);
  int16_t phaseBCurrentRaw = s16Le(msg.buf[2], msg.buf[3]);
  int16_t phaseCCurrentRaw = s16Le(msg.buf[4], msg.buf[5]);
  int16_t dcBusCurrentRaw = s16Le(msg.buf[6], msg.buf[7]);

  Serial.print("phaseA=");
  Serial.print((float)phaseACurrentRaw / 10.0f, 1);
  Serial.print("A phaseB=");
  Serial.print((float)phaseBCurrentRaw / 10.0f, 1);
  Serial.print("A phaseC=");
  Serial.print((float)phaseCCurrentRaw / 10.0f, 1);
  Serial.print("A dcBus=");
  Serial.print((float)dcBusCurrentRaw / 10.0f, 1);
  Serial.print("A");
}

void printPm100VoltageInfoTranslation(const CAN_message_t &msg) {
  if (msg.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  int16_t dcBusVoltageRaw = s16Le(msg.buf[0], msg.buf[1]);
  int16_t outputVoltageRaw = s16Le(msg.buf[2], msg.buf[3]);
  int16_t vabVdVoltageRaw = s16Le(msg.buf[4], msg.buf[5]);
  int16_t vbcVqVoltageRaw = s16Le(msg.buf[6], msg.buf[7]);

  Serial.print("dcBus=");
  Serial.print((float)dcBusVoltageRaw / 10.0f, 1);
  Serial.print("V output=");
  Serial.print((float)outputVoltageRaw / 10.0f, 1);
  Serial.print("V vabOrVd=");
  Serial.print((float)vabVdVoltageRaw / 10.0f, 1);
  Serial.print("V vbcOrVq=");
  Serial.print((float)vbcVqVoltageRaw / 10.0f, 1);
  Serial.print("V");
}

void printPm100InternalVoltagesTranslation(const CAN_message_t &msg) {
  if (msg.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  int16_t ref1V5Raw = s16Le(msg.buf[0], msg.buf[1]);
  int16_t ref2V5Raw = s16Le(msg.buf[2], msg.buf[3]);
  int16_t ref5V0Raw = s16Le(msg.buf[4], msg.buf[5]);
  int16_t glvRaw = s16Le(msg.buf[6], msg.buf[7]);

  Serial.print("ref1V5=");
  Serial.print((float)ref1V5Raw / 100.0f, 2);
  Serial.print("V ref2V5=");
  Serial.print((float)ref2V5Raw / 100.0f, 2);
  Serial.print("V ref5V0=");
  Serial.print((float)ref5V0Raw / 100.0f, 2);
  Serial.print("V glv12V=");
  Serial.print((float)glvRaw / 100.0f, 2);
  Serial.print("V");
}

void printPm100FaultCodesTranslation(const CAN_message_t &msg) {
  if (msg.len < 8) {
    Serial.print("short frame, expected 8 bytes");
    return;
  }

  uint32_t postFaults = u32Le(msg.buf[0], msg.buf[1], msg.buf[2], msg.buf[3]);
  uint32_t runFaults = u32Le(msg.buf[4], msg.buf[5], msg.buf[6], msg.buf[7]);

  Serial.print("post=0x");
  Serial.print((unsigned long)postFaults, HEX);
  Serial.print(" run=0x");
  Serial.print((unsigned long)runFaults, HEX);
  Serial.print(" active=");

  bool printedFault = false;
  for (uint8_t bit = 0; bit < 32; bit++) {
    if (postFaults & (1UL << bit)) {
      if (printedFault) Serial.print(',');
      Serial.print(pm100PostFaultName(bit));
      printedFault = true;
    }
  }

  for (uint8_t bit = 0; bit < 32; bit++) {
    if (runFaults & (1UL << bit)) {
      if (printedFault) Serial.print(',');
      Serial.print(pm100RunFaultName(bit));
      printedFault = true;
    }
  }

  if (!printedFault) {
    Serial.print("none");
  }
}

void printPm100TorqueCapabilityTranslation(const CAN_message_t &msg) {
  if (msg.len < 2) {
    Serial.print("short frame, expected at least 2 bytes");
    return;
  }

  int16_t positiveTorqueRaw = s16Le(msg.buf[0], msg.buf[1]);

  Serial.print("positiveTorqueCapability=");
  Serial.print((float)positiveTorqueRaw / 10.0f, 1);
  Serial.print("Nm");

  if (msg.len >= 4) {
    int16_t negativeTorqueRaw = s16Le(msg.buf[2], msg.buf[3]);
    Serial.print(" possibleNegativeTorqueCapability=");
    Serial.print((float)negativeTorqueRaw / 10.0f, 1);
    Serial.print("Nm");
  }

  if (msg.len > 4) {
    Serial.print(" reserved=");
    printRawPayloadFrom(msg, 4);
  }
}

void printRawPayloadFrom(const CAN_message_t &msg, uint8_t startIndex) {
  if (startIndex >= msg.len) {
    Serial.print("none");
    return;
  }

  for (uint8_t i = startIndex; i < msg.len; i++) {
    if (i > startIndex) {
      Serial.print(' ');
    }
    printCanHexByte(msg.buf[i]);
  }
}

void printBmsTemperatureFaultCode(uint8_t faultCode) {
  Serial.print(bmsTemperatureFaultCodeName(faultCode));
}

void printCellDataRequestName(uint8_t requestType) {
  switch (requestType) {
    case CELL_DATA_REQUEST_VOLTAGES: Serial.print("cell voltages"); break;
    case CELL_DATA_REQUEST_TEMPERATURES: Serial.print("cell temperatures"); break;
    default: Serial.print("unknown"); break;
  }
}

void printImdRequestIndexName(uint8_t requestIndex) {
  switch (requestIndex) {
    case 0x0A: Serial.print("bootloader build number"); break;
    case 0x0C: Serial.print("bootloader D-number"); break;
    case 0x0E: Serial.print("bootloader version"); break;
    case 0x10: Serial.print("hardware AH history"); break;
    case 0x12: Serial.print("hardware AH number"); break;
    case 0x14: Serial.print("hardware AH number part B"); break;
    case 0x16: Serial.print("hardware item number"); break;
    case 0x18: Serial.print("hardware article number part B"); break;
    case 0x1A: Serial.print("serial number"); break;
    case 0x1C: Serial.print("serial number part B"); break;
    case 0x1E: Serial.print("software build number"); break;
    case 0x20: Serial.print("software D-number"); break;
    case 0x22: Serial.print("software version"); break;
    case 0x2A: Serial.print("unbalance measured value"); break;
    case 0x2C: Serial.print("unbalance measurement counter"); break;
    case 0x2E:
    case 0x2F: Serial.print("unbalance threshold"); break;
    case 0x30:
    case 0x31: Serial.print("self-holding alarm activation"); break;
    case 0x33: Serial.print("reset alarm"); break;
    case 0x36: Serial.print("isolation measurement counter"); break;
    case 0x38:
    case 0x39: Serial.print("isolation active profile"); break;
    case 0x3A:
    case 0x3B: Serial.print("isolation power-on profile"); break;
    case 0x3E: Serial.print("isolation quality"); break;
    case 0x40: Serial.print("R_iso_neg"); break;
    case 0x42: Serial.print("R_iso_pos"); break;
    case 0x44: Serial.print("R_iso_status"); break;
    case 0x46:
    case 0x47: Serial.print("isolation threshold error"); break;
    case 0x48:
    case 0x49: Serial.print("isolation measurement timeout threshold"); break;
    case 0x4A:
    case 0x4B: Serial.print("isolation threshold warning"); break;
    case 0x4C: Serial.print("R_iso_corrected"); break;
    case 0x4E: Serial.print("R_iso_original"); break;
    case 0x50: Serial.print("time since last isolation measurement"); break;
    case 0x52: Serial.print("capacity measured value"); break;
    case 0x54: Serial.print("capacity measurement counter"); break;
    case 0x57: Serial.print("trigger self test"); break;
    case 0x58:
    case 0x59: Serial.print("self test period"); break;
    case 0x5C: Serial.print("voltage measurement counter"); break;
    case 0x5E: Serial.print("HV system voltage"); break;
    case 0x60: Serial.print("HV negative to earth voltage"); break;
    case 0x62: Serial.print("HV positive to earth voltage"); break;
    case 0x64:
    case 0x65: Serial.print("voltage mode"); break;
    case 0x66:
    case 0x67: Serial.print("undervoltage threshold"); break;
    case 0x68: Serial.print("device activity"); break;
    case 0x6A:
    case 0x6B: Serial.print("parameter lock"); break;
    case IDX_WARN_ALARMS: Serial.print("warnings and alarms"); break;
    case 0x6F: Serial.print("factory reset"); break;
    case 0x70:
    case 0x71: Serial.print("earthlift status"); break;
    case 0x72:
    case 0x73: Serial.print("threshold first reference estimation"); break;
    case 0x74:
    case 0x75: Serial.print("pre-estimation max difference"); break;
    case 0x76:
    case 0x77: Serial.print("interface CAN-ID"); break;
    case 0x78:
    case 0x79: Serial.print("interface periodic cycle time"); break;
    case 0x7B: Serial.print("interface baudrate"); break;
    case 0x7E:
    case 0x7F: Serial.print("isolation initial value"); break;
    default: Serial.print("unknown"); break;
  }
}

void printImdResponseErrorName(uint8_t errorCode) {
  switch (errorCode) {
    case 0x23: Serial.print("unknown or invalid request"); break;
    case 0x24: Serial.print("set command failed, parameter locked"); break;
    case 0x25: Serial.print("data1 range overflow"); break;
    case 0x26: Serial.print("data2 range overflow"); break;
    case 0x27: Serial.print("CAN-ID already in use"); break;
    case 0x28: Serial.print("write failed"); break;
    case 0x29: Serial.print("read failed"); break;
    default: Serial.print("unknown"); break;
  }
}

void printImdInfoMessageSelectorName(uint8_t selector) {
  switch (selector) {
    case 0x00: Serial.print("Request"); break;
    case 0x01: Serial.print("Response"); break;
    case 0x02: Serial.print("IMD_Info_General"); break;
    case 0x03: Serial.print("IMD_Info_IsolationDetail"); break;
    case 0x04: Serial.print("IMD_Info_Voltage"); break;
    case 0x05: Serial.print("IMD_Info_IT-System"); break;
    default: Serial.print("unknown"); break;
  }
}

void printImdProfileName(uint8_t profile) {
  switch (profile) {
    case 1: Serial.print("standard fast startup"); break;
    case 2: Serial.print("standard"); break;
    case 3: Serial.print("high capacity fast startup"); break;
    case 4: Serial.print("high capacity"); break;
    case 5: Serial.print("disturbed"); break;
    case 6: Serial.print("service"); break;
    case IMD_SNV_BYTE: Serial.print("SNV"); break;
    default: Serial.print("unknown"); break;
  }
}

void printImdRIsoStatusName(uint8_t status) {
  switch (status) {
    case 0xFC: Serial.print("startup estimated"); break;
    case 0xFD: Serial.print("startup first measurement"); break;
    case 0xFE: Serial.print("normal operation"); break;
    case IMD_SNV_BYTE: Serial.print("SNV"); break;
    default: Serial.print("unknown"); break;
  }
}

void printImdDeviceActivityName(uint8_t activity) {
  switch (activity) {
    case 0: Serial.print("initialization"); break;
    case 1: Serial.print("normal operation"); break;
    case 2: Serial.print("self test"); break;
    case IMD_SNV_BYTE: Serial.print("SNV"); break;
    default: Serial.print("unknown"); break;
  }
}

void printImdLockStateName(uint8_t state) {
  switch (state) {
    case 0xFC: Serial.print("parameter write enable"); break;
    case 0xFD: Serial.print("parameter write disable"); break;
    case 0xFE: Serial.print("reserved"); break;
    case IMD_SNV_BYTE: Serial.print("SNV"); break;
    default: Serial.print("unknown"); break;
  }
}

void printImdEarthliftStateName(uint8_t state) {
  switch (state) {
    case 0:
    case 0xFC: Serial.print("earth connection closed"); break;
    case 1:
    case 0xFD: Serial.print("earth connection open"); break;
    case IMD_SNV_BYTE: Serial.print("SNV"); break;
    default: Serial.print("unknown"); break;
  }
}

void printImdVoltageModeName(uint8_t mode) {
  switch (mode) {
    case 0xFD: Serial.print("AC"); break;
    case 0xFE: Serial.print("DC"); break;
    case IMD_SNV_BYTE: Serial.print("SNV"); break;
    default: Serial.print("unknown"); break;
  }
}

void printImdBaudRateName(uint8_t baudRate) {
  switch (baudRate) {
    case 0x01: Serial.print("1 Mbit/s"); break;
    case 0x02: Serial.print("800 kbit/s"); break;
    case 0x03: Serial.print("666 kbit/s"); break;
    case 0x04: Serial.print("500 kbit/s"); break;
    case 0x05: Serial.print("250 kbit/s"); break;
    case 0x06: Serial.print("125 kbit/s"); break;
    case IMD_SNV_BYTE: Serial.print("SNV"); break;
    default: Serial.print("unknown"); break;
  }
}

void printImdAsciiPayloadFrom(const CAN_message_t &msg, uint8_t startIndex) {
  Serial.print('"');
  for (uint8_t i = startIndex; i < msg.len; i++) {
    uint8_t value = msg.buf[i];
    if (value == IMD_SNV_BYTE) {
      break;
    }
    if (value >= 32 && value <= 126) {
      Serial.write(value);
    } else {
      Serial.print('.');
    }
  }
  Serial.print('"');
}

void printImdWordValue(uint16_t value) {
  if (value == IMD_SNV_WORD) {
    Serial.print("SNV");
  } else {
    Serial.print(value);
  }
}

void printImdResistanceValue(uint16_t kohms) {
  if (kohms == IMD_SNV_WORD) {
    Serial.print("SNV");
  } else {
    Serial.print(kohms);
    Serial.print("kOhm");
  }
}

void printImdVoltageValue(uint16_t raw) {
  if (raw == IMD_SNV_WORD) {
    Serial.print("SNV");
  } else {
    float volts = ((int32_t)raw - IMD_VOLTAGE_OFFSET) * IMD_VOLTAGE_SCALE;
    Serial.print(volts, 2);
    Serial.print("V");
  }
}

void printImdCapacityValue(uint16_t raw) {
  if (raw == IMD_SNV_WORD) {
    Serial.print("SNV");
  } else {
    Serial.print((float)raw * 0.1f, 1);
    Serial.print("uF");
  }
}

void printImdPercentValue(uint8_t value) {
  if (value == IMD_SNV_BYTE) {
    Serial.print("SNV");
  } else {
    Serial.print(value);
    Serial.print('%');
  }
}

void printImdCounterValue(uint8_t value) {
  Serial.print(value);
}

void printCellVoltageValue(uint16_t millivolts) {
  if (millivolts == 0xFFFF) {
    Serial.print("pad");
  } else {
    Serial.print(millivolts);
    Serial.print("mV");
  }
}

void printCanHexByte(uint8_t value) {
  if (value < 0x10) {
    Serial.print('0');
  }
  Serial.print(value, HEX);
}

bool tssiFaultFromWarnBits(uint16_t warnBits) {
  return (warnBits & IMD_FAULT_BITS_MASK) != 0;
}

bool bmsImdFaultDetectionArmed() {
  return (millis() - startupFaultDelayStartMs) >= STARTUP_BMS_IMD_FAULT_DELAY_MS;
}

bool anyFaultActive() {
  if (!bmsImdFaultDetectionArmed()) {
    return false;
  }

  return imdFault || bmsFault;
}

// Contactors open is a normal pre-RTD condition, not a fault: it should drop/
// block RTD (back to neutral) without lighting the fault indicator.
bool contactorsOpen() {
  return haveBmsStatus && !(bmsFlags & 0x02);
}

// ---------------- CAN receive handlers ----------------
void handleBms2026StatusFrame(const CAN_message_t &msg) {
  if (msg.len < 8) {
    return;
  }

  bool wasActive = bmsActive;
  bool previousFault = bmsFault;
  bool faultDetectionArmed = bmsImdFaultDetectionArmed();

  uint8_t flags = msg.buf[0];
  minCellVoltageMv = u16Be(msg.buf[1], msg.buf[2]);
  maxCellVoltageMv = u16Be(msg.buf[3], msg.buf[4]);
  avgCellVoltageMv = u16Be(msg.buf[5], msg.buf[6]);
  haveCellVoltageStats = true;

  haveBmsStatus    = true;
  bmsTimedOut      = false;
  lastBmsRxMs      = millis();
  bmsFlags         = flags;
  bmsFault         = (flags & 0x01) != 0;
  bmsActive        = !bmsFault;
  stateOfChargePct = msg.buf[7];
  packVoltage      = (avgCellVoltageMv / 1000.0f) * 96.0f;  // 96 cells in series

  if (bmsActive && !wasActive) {
    Serial.println("[BMS] Active");
  }

  if (faultDetectionArmed && bmsFault != previousFault) {
    Serial.print("[BMS] 2026 flags=0x");
    Serial.print(flags, HEX);
    Serial.print(" fault=");
    Serial.println(bmsFault ? "YES" : "NO");
    if (bmsFault) {
      showFaultBanner((haveBms2026FaultCode && bms2026FaultCode != 0)
                          ? bmsTemperatureFaultCodeName(bms2026FaultCode)
                          : "BMS Fault");
    }
  }

  if (anyFaultActive() || contactorsOpen()) {
    dropReadyToDrive(rtdDropCauseText());
  }

  screenDirty = true;
}

void handleBms2026TempFrame(const CAN_message_t &msg) {
  if (msg.len < 6) {
    return;
  }

  // byte 1 = max cell temp encoded as (°C + 40); 0xFF means no valid reading
  uint8_t previousFaultCode = haveBms2026FaultCode ? bms2026FaultCode : 0;
  bms2026FaultCode = msg.buf[3];
  haveBms2026FaultCode = true;
  if (bmsImdFaultDetectionArmed() && bms2026FaultCode != 0 &&
      bms2026FaultCode != previousFaultCode) {
    showFaultBanner(bmsTemperatureFaultCodeName(bms2026FaultCode));
  }

  haveCellTempStats = (msg.buf[0] != 0xFF && msg.buf[1] != 0xFF && msg.buf[2] != 0xFF);
  if (haveCellTempStats) {
    minCellTempC = (int16_t)msg.buf[0] - 40;
    maxCellTempC = (int16_t)msg.buf[1] - 40;
    avgCellTempC = (int16_t)msg.buf[2] - 40;
  } else {
    minCellTempC = CELL_TEMP_UNKNOWN_C;
    maxCellTempC = CELL_TEMP_UNKNOWN_C;
    avgCellTempC = CELL_TEMP_UNKNOWN_C;
  }

  if (msg.buf[1] != 0xFF) {
    batteryTempC = (uint8_t)constrain((int)msg.buf[1] - 40, 0, 255);
  }

  // bytes 4-5 = pack current, big-endian, raw = A * 10 + 3000
  int16_t curRaw = (int16_t)(((uint16_t)msg.buf[4] << 8) | msg.buf[5]);
  packCurrent = ((float)curRaw - 3000.0f) / 10.0f;

  screenDirty = true;
}

void handleImdFrame(const CAN_message_t &msg) {
  bool wasTimedOut = imdCanTimedOut;
  uint16_t previousWarnings = imdWarnAlarms;
  bool previousFault = imdFault;
  bool faultDetectionArmed = bmsImdFaultDetectionArmed();

  if (msg.id == IMD_INFO_GENERAL_ID && msg.len >= 5) {
    imdRIsoKohm = u16Le(msg.buf[0], msg.buf[1]);
    imdRIsoStatus = msg.buf[2];
    imdWarnAlarms = (imdWarnAlarms & 0xFF00u) | msg.buf[4];
    imdDeviceActivity = (msg.len >= 7) ? msg.buf[6] : IMD_SNV_BYTE;
    lastImdRxMs = millis();
    imdCanTimedOut = false;
    imdFault = tssiFaultFromWarnBits(imdWarnAlarms);
  } else if (msg.id == IMD_RESPONSE_ID && msg.len >= 3 && msg.buf[0] == IDX_WARN_ALARMS) {
    imdWarnAlarms = u16Le(msg.buf[1], msg.buf[2]);
    lastImdRxMs = millis();
    imdCanTimedOut = false;
    imdFault = tssiFaultFromWarnBits(imdWarnAlarms);
  } else {
    return;
  }

  if (wasTimedOut) {
    Serial.println("[TSSI] IMD CAN restored");
  }

  if (faultDetectionArmed && (imdWarnAlarms != previousWarnings || imdFault != previousFault)) {
    Serial.print("[TSSI] IMD warn=0x");
    Serial.print(imdWarnAlarms, HEX);
    printTssiWarnBits(imdWarnAlarms);
    Serial.print(" => ");
    Serial.println(imdFault ? "FAULT, red blink" : "OK, green on");
    if (imdFault && !previousFault) {
      showFaultBanner(firstTssiFaultName(imdWarnAlarms));
    }
  }
}

void handlePm100Temperatures1Frame(const CAN_message_t &msg) {
  if (msg.len < 8) {
    return;
  }

  int16_t moduleATempRaw = s16Le(msg.buf[0], msg.buf[1]);
  int16_t moduleBTempRaw = s16Le(msg.buf[2], msg.buf[3]);
  int16_t moduleCTempRaw = s16Le(msg.buf[4], msg.buf[5]);
  int16_t gateDriverTempRaw = s16Le(msg.buf[6], msg.buf[7]);

  int16_t hottestTempRaw = moduleATempRaw;
  if (moduleBTempRaw > hottestTempRaw) hottestTempRaw = moduleBTempRaw;
  if (moduleCTempRaw > hottestTempRaw) hottestTempRaw = moduleCTempRaw;
  if (gateDriverTempRaw > hottestTempRaw) hottestTempRaw = gateDriverTempRaw;

  inverterTempTenthsC = hottestTempRaw;
  haveInverterTemp = true;
  screenDirty = true;
}

void handlePm100Temperatures3Frame(const CAN_message_t &msg) {
  if (msg.len < 6) {
    return;
  }

  pm100MotorTempTenthsC = s16Le(msg.buf[4], msg.buf[5]);
  havePm100MotorTemp = true;
  screenDirty = true;
}

void handlePm100RpmFrame(const CAN_message_t &msg) {
  if (msg.len < 4) {
    return;
  }

  motorRpm = s16Le(msg.buf[2], msg.buf[3]);
  screenDirty = true;
}

void handlePm100CurrentInfoFrame(const CAN_message_t &msg) {
  if (msg.len < 8) {
    return;
  }

  inverterDcBusCurrent = (float)s16Le(msg.buf[6], msg.buf[7]) / 10.0f;
  haveInverterCurrentInfo = true;
  screenDirty = true;
}

void handlePm100VoltageInfoFrame(const CAN_message_t &msg) {
  if (msg.len < 2) {
    return;
  }

  inverterDcBusVoltage = (float)s16Le(msg.buf[0], msg.buf[1]) / 10.0f;
  haveInverterVoltageInfo = true;
  screenDirty = true;
}

void handlePm100InternalVoltagesFrame(const CAN_message_t &msg) {
  if (msg.len < 8) {
    return;
  }

  // Bytes 6-7 = 12V System voltage, "Low Voltage" format (volts * 100).
  glvVoltage = (float)s16Le(msg.buf[6], msg.buf[7]) / 100.0f;
  haveGlvVoltage = true;
  screenDirty = true;
}

void handlePm100ParamResponseFrame(const CAN_message_t &msg) {
  if (msg.len < 3) {
    return;
  }

  uint16_t address = u16Le(msg.buf[0], msg.buf[1]);
  if (address != PM100_PARAM_CAN_ACTIVE_MSGS) {
    return;
  }

  Serial.print("[PM100] CAN Active Messages parameter write ");
  Serial.println(msg.buf[2] ? "succeeded" : "FAILED");
}

// The 0x0A9 Internal Voltages broadcast (GLV voltage source) can be disabled
// in the inverter's EEPROM broadcast mask. If the inverter is alive but 0x0A9
// stays silent, write parameter 148 to enable all broadcast messages.
// EEPROM parameters are only writable while the motor is not running.
void servicePm100BroadcastEnable() {
  if (haveGlvVoltage || !havePm100Traffic || readyToDriveLatched) {
    return;
  }

  if (pm100BroadcastEnableAttempts >= PM100_BROADCAST_ENABLE_MAX_ATTEMPTS) {
    return;
  }

  uint32_t now = millis();
  if (now - firstPm100FrameMs < PM100_BROADCAST_ENABLE_DELAY_MS) {
    return;
  }

  if (pm100BroadcastEnableAttempts > 0 &&
      now - pm100BroadcastEnableLastTxMs < PM100_BROADCAST_ENABLE_RETRY_MS) {
    return;
  }

  pm100BroadcastEnableAttempts++;
  pm100BroadcastEnableLastTxMs = now;

  Serial.print("[PM100] No 0x0A9 Internal Voltages frames; enabling broadcasts, attempt ");
  Serial.println(pm100BroadcastEnableAttempts);
  sendPm100BroadcastEnable();
}

void sendPm100BroadcastEnable() {
  CAN_message_t tx = {};
  tx.id = PM100_PARAM_COMMAND_ID;
  tx.len = 8;
  tx.flags.extended = 0;
  tx.buf[0] = (uint8_t)(PM100_PARAM_CAN_ACTIVE_MSGS & 0xFF);
  tx.buf[1] = (uint8_t)(PM100_PARAM_CAN_ACTIVE_MSGS >> 8);
  tx.buf[2] = 1;     // write
  tx.buf[3] = 0;     // reserved
  tx.buf[4] = 0xFF;  // CAN Active Messages Lo Word: enable all broadcasts
  tx.buf[5] = 0xFF;
  tx.buf[6] = 0xFF;  // Hi Word: keep command/BMS/OBD2 mailboxes enabled
  tx.buf[7] = 0xFF;
  sendCanFrame(tx);
}

void handlePm100FaultCodesFrame(const CAN_message_t &msg) {
  if (msg.len < 8) {
    return;
  }

  bool previousFault = havePm100FaultStatus && pm100Fault;
  pm100PostFaults = u32Le(msg.buf[0], msg.buf[1], msg.buf[2], msg.buf[3]);
  pm100RunFaults = u32Le(msg.buf[4], msg.buf[5], msg.buf[6], msg.buf[7]);
  havePm100FaultStatus = true;
  pm100Fault = (pm100PostFaults != 0 || pm100RunFaults != 0);
  if (pm100Fault && !previousFault) {
    showFaultBanner(firstPm100FaultName(pm100PostFaults, pm100RunFaults));
  }
  screenDirty = true;
}

void setupTssi() {
  pinMode(TSSI_LED_RED, OUTPUT);
  pinMode(TSSI_LED_GREEN, OUTPUT);
  setTssiAllOff();

  imdFault = true;
  imdCanTimedOut = true;
  lastImdRxMs = millis();
  tssiBlinkTimerMs = millis();
  lastImdGetReqMs = 0;
}

void serviceTssi() {
  uint32_t now = millis();
  bool faultDetectionArmed = bmsImdFaultDetectionArmed();

  if (faultDetectionArmed && now - lastImdRxMs > IMD_HEARTBEAT_TIMEOUT_MS) {
    if (!imdCanTimedOut) {
      Serial.println("[TSSI] IMD heartbeat lost, forcing FAULT");
      showFaultBanner("IMD Timeout");
    }
    imdCanTimedOut = true;
    imdFault = true;
  }

  if ((anyFaultActive() || contactorsOpen()) && readyToDriveLatched) {
    dropReadyToDrive(rtdDropCauseText());
  }

  if (now - lastImdGetReqMs >= IMD_GET_REQ_PERIOD_MS) {
    lastImdGetReqMs = now;
    requestImdWarnings();
  }

  if (!faultDetectionArmed) {
    setTssiGreenOn();
  } else {
    updateTssiLed(anyFaultActive(), now);
  }
}

void requestImdWarnings() {
  CAN_message_t tx = {};
  tx.id = IMD_REQUEST_ID;
  tx.len = 8;
  tx.flags.extended = 0;
  tx.buf[0] = IDX_WARN_ALARMS;
  for (int i = 1; i < 8; i++) {
    tx.buf[i] = 0xFF;
  }
  sendCanFrame(tx);
}

void updateTssiLed(bool faultActive, uint32_t now) {
  if (faultActive) {
    uint32_t elapsed = now - tssiBlinkTimerMs;
    if (elapsed >= TSSI_BLINK_PERIOD_MS) {
      tssiBlinkTimerMs += TSSI_BLINK_PERIOD_MS;
      elapsed = now - tssiBlinkTimerMs;
    }

    if (elapsed < TSSI_BLINK_ON_MS) {
      if (!tssiRedOn) {
        setTssiRedOn();
      }
    } else if (tssiRedOn) {
      setTssiRedOff();
    }
  } else {
    setTssiGreenOn();
  }
}

void setTssiGreenOn() {
  digitalWrite(TSSI_LED_RED, LOW);
  digitalWrite(TSSI_LED_GREEN, HIGH);
  tssiRedOn = false;
}

void setTssiRedOn() {
  digitalWrite(TSSI_LED_GREEN, LOW);
  digitalWrite(TSSI_LED_RED, HIGH);
  tssiRedOn = true;
}

void setTssiRedOff() {
  digitalWrite(TSSI_LED_RED, LOW);
  tssiRedOn = false;
}

void setTssiAllOff() {
  digitalWrite(TSSI_LED_GREEN, LOW);
  digitalWrite(TSSI_LED_RED, LOW);
  tssiRedOn = false;
}

void printTssiWarnBits(uint16_t bits) {
  if (bits & (1u << 0)) Serial.print(" [DeviceError]");
  if (bits & (1u << 1)) Serial.print(" [HV+ConnFail]");
  if (bits & (1u << 2)) Serial.print(" [HV-ConnFail]");
  if (bits & (1u << 3)) Serial.print(" [EarthConnFail]");
  if (bits & (1u << 4)) Serial.print(" [IsoAlarm]");
  if (bits & (1u << 5)) Serial.print(" [IsoWarning]");
  if (bits & (1u << 6)) Serial.print(" [IsoOutdated]");
  if (bits & (1u << 7)) Serial.print(" [UnbalanceAlarm]");
  if (bits & (1u << 8)) Serial.print(" [Undervoltage]");
  if (bits & (1u << 9)) Serial.print(" [UnsafeToStart]");
  if (bits & (1u << 10)) Serial.print(" [EarthliftOpen]");
}

// ---------------- PM100 CAN command output ----------------
void setupPm100CommandTimer() {
  updatePm100CommandState();
  pm100CommandTimerIsr();
  pm100CommandTimer.begin(pm100CommandTimerIsr, PM100_CMD_PERIOD_US);
  Serial.println("[PM100] 100 Hz command timer started");
}

void pm100CommandTimerIsr() {
  CAN_message_t tx = {};
  tx.id = PM100_CMD_ID;
  tx.len = 8;
  tx.flags.extended = 0;

  int16_t torqueRaw = pm100CommandTorqueRaw;
  tx.buf[0] = (uint8_t)(torqueRaw & 0xFF);
  tx.buf[1] = (uint8_t)((torqueRaw >> 8) & 0xFF);
  tx.buf[4] = pm100CommandDirection;
  tx.buf[5] = pm100CommandEnable;

  VehicleCan.write(tx);
  queuePm100TxLogFromIsr(tx);
}

void updatePm100CommandState() {
  int16_t torqueRaw = 0;
  uint8_t direction = 0x00;
  uint8_t inverterEnable = 0x00;

  bool driveSelected = readyToDriveLatched && (driveMode == 'D');
  // APPS implausibility and an active BSPC keep the inverter enabled but command
  // zero torque: updatePedal already forces desiredTorqueNm to 0 for both.
  bool torqueAllowed = driveSelected && !anyFaultActive() && !contactorsOpen();

  if (torqueAllowed) {
    torqueRaw = (int16_t)roundf(desiredTorqueNm * (float)TORQUE_SCALE);
    direction = pm100DirectionByteForMode(driveMode);
    inverterEnable = 0x01;
  }

  noInterrupts();
  pm100CommandTorqueRaw = torqueRaw;
  pm100CommandDirection = direction;
  pm100CommandEnable = inverterEnable;
  interrupts();

  commandedTorqueNm = (float)torqueRaw / (float)TORQUE_SCALE;
}

void queuePm100TxLogFromIsr(const CAN_message_t &msg) {
  uint8_t nextHead = (uint8_t)((pm100TxLogHead + 1u) % PM100_TX_LOG_QUEUE_SIZE);
  if (nextHead == pm100TxLogTail) {
    pm100TxLogDropped++;
    return;
  }

  pm100TxLogQueue[pm100TxLogHead] = msg;
  pm100TxLogHead = nextHead;
}

bool dequeuePm100TxLog(CAN_message_t &msg) {
  bool haveMessage = false;

  noInterrupts();
  if (pm100TxLogTail != pm100TxLogHead) {
    msg = pm100TxLogQueue[pm100TxLogTail];
    pm100TxLogTail = (uint8_t)((pm100TxLogTail + 1u) % PM100_TX_LOG_QUEUE_SIZE);
    haveMessage = true;
  }
  interrupts();

  return haveMessage;
}

void servicePm100TxLogging() {
  CAN_message_t tx;
  while (dequeuePm100TxLog(tx)) {
    logCanFrame("TX", tx);
  }

  static uint32_t lastDroppedCount = 0;
  uint32_t droppedCount;

  noInterrupts();
  droppedCount = pm100TxLogDropped;
  interrupts();

  if (droppedCount != lastDroppedCount) {
    Serial.print("[CAN TX LOG] PM100 command log dropped=");
    Serial.println(droppedCount);
    lastDroppedCount = droppedCount;
  }
}

// ---------------- Pedal ----------------
void updatePedal() {
  int rawApps1 = analogRead(APPS_1_PIN);
  int rawApps2 = analogRead(APPS_2_PIN);
  rawApps1Last = rawApps1;
  rawApps2Last = rawApps2;

  float newApps1Pct = scalePercentInverted(rawApps1, APPS_1_RAW_MIN, APPS_1_RAW_MAX);
  float newApps2Pct = scalePercentNormal(rawApps2, APPS_2_RAW_MIN, APPS_2_RAW_MAX);

  if (!appsFilterInitialized) {
    apps1Pct = newApps1Pct;
    apps2Pct = newApps2Pct;
    appsFilterInitialized = true;
  } else {
    apps1Pct = APPS_EMA_ALPHA * newApps1Pct + (1.0f - APPS_EMA_ALPHA) * apps1Pct;
    apps2Pct = APPS_EMA_ALPHA * newApps2Pct + (1.0f - APPS_EMA_ALPHA) * apps2Pct;
  }

  bool apps1InRange = rawApps1 >= APPS_1_RAW_MIN - APPS_RAW_FAULT_MARGIN &&
                      rawApps1 <= APPS_1_RAW_MAX + APPS_RAW_FAULT_MARGIN;
  bool apps2InRange = rawApps2 >= APPS_2_RAW_MIN - APPS_RAW_FAULT_MARGIN &&
                      rawApps2 <= APPS_2_RAW_MAX + APPS_RAW_FAULT_MARGIN;

  float percentError = fabsf(apps1Pct - apps2Pct);
  bool wasPlausible = appsPlausible;
  bool appsAbnormal = !apps1InRange || !apps2InRange ||
                      percentError > APPS_IMPLAUSIBLE_PCT;

  if (appsAbnormal) {
    if (!apps1InRange && !apps2InRange) {
      appsFaultText = "APPS1+2 Range";
    } else if (!apps1InRange) {
      appsFaultText = "APPS1 Range";
    } else if (!apps2InRange) {
      appsFaultText = "APPS2 Range";
    } else {
      appsFaultText = "APPS Disagree";
    }
  }

  if (!appsAbnormal) {
    appsDisagreeing = false;
    appsPlausible = true;
  } else {
    uint32_t now = millis();
    if (!appsDisagreeing) {
      appsDisagreeing = true;
      appsDisagreeStartMs = now;
    }
    if (now - appsDisagreeStartMs > APPS_IMPLAUSIBILITY_PERSIST_MS) {
      appsPlausible = false;
    }
  }

  // BSPC thresholds compare against true pedal travel (pre-deadband) so the
  // 25% latch and 5% reset points match actual pedal position per EV.4.7.
  float pedalTravelPct = (apps1Pct + apps2Pct) * 0.5f;

  pedalPct = pedalTravelPct;
  if (pedalPct <= PEDAL_ZERO_DEADBAND_PCT) {
    pedalPct = 0.0f;
  } else {
    // Rescale the remaining travel back up to 0-100% so output ramps
    // smoothly from zero instead of jumping past the deadband.
    pedalPct = (pedalPct - PEDAL_ZERO_DEADBAND_PCT) * 100.0f / (100.0f - PEDAL_ZERO_DEADBAND_PCT);
  }

  // BSPC: zero torque while the brake (light-braking threshold) and
  // accelerator >25% travel are applied together. Torque returns as soon as
  // either pedal is released.
  bool bspcWasActive = bspcActive;
  bspcActive = braking && pedalTravelPct > BSPC_APPS_THRESHOLD_PCT;
  if (bspcActive != bspcWasActive) {
    Serial.println(bspcActive ? "[BSPC] Active; commanding zero torque"
                              : "[BSPC] Cleared");
  }

  if (appsPlausible) {
    desiredTorqueNm = bspcActive ? 0.0f : (pedalPct / 100.0f) * MAX_TORQUE_NM;
  } else {
    desiredTorqueNm = 0.0f;
    if (wasPlausible) {
      Serial.print("[APPS] Implausible (");
      Serial.print(appsFaultText);
      Serial.print("): raw1=");
      Serial.print(rawApps1);
      Serial.print(" raw2=");
      Serial.print(rawApps2);
      Serial.print(" deviation=");
      Serial.print(percentError, 1);
      Serial.print("% for >");
      Serial.print(APPS_IMPLAUSIBILITY_PERSIST_MS);
      Serial.println(" ms");
      showFaultBanner(appsFaultText);
    }
    // T.4.2.10 only requires zero torque on implausibility; the car stays in
    // drive and torque returns once the channels agree again.
  }
}

void logAppsTelemetry() {
  if (!APPS_SERIAL_LOG_ENABLED) {
    return;
  }

  static uint32_t lastPrintMs = 0;
  uint32_t now = millis();
  if (now - lastPrintMs < APPS_SERIAL_LOG_PERIOD_MS) {
    return;
  }
  lastPrintMs = now;

  Serial.print("[APPS] raw1=");
  Serial.print(rawApps1Last);
  Serial.print(" raw2=");
  Serial.print(rawApps2Last);
  Serial.print(" apps1=");
  Serial.print(apps1Pct, 1);
  Serial.print("% apps2=");
  Serial.print(apps2Pct, 1);
  Serial.print("% pedal=");
  Serial.print(pedalPct, 1);
  Serial.print("% torque=");
  Serial.print(desiredTorqueNm, 1);
  Serial.print("Nm cmd=");
  Serial.print(commandedTorqueNm, 1);
  Serial.print("Nm bspc=");
  Serial.print(bspcActive ? "ACTIVE" : "clear");
  Serial.print(" plausible=");
  Serial.println(appsPlausible ? "YES" : "NO");
}

void updateBrakePressureSensors() {
  rawBse1Last = analogRead(BSE_1_PIN);
  rawBse2Last = analogRead(BSE_2_PIN);

  braking = (rawBse1Last > BSE_1_BRAKING_RAW_THRESHOLD) ||
            (rawBse2Last > BSE_2_BRAKING_RAW_THRESHOLD);
  hardBraking = (rawBse1Last > BSE_1_HARD_BRAKING_RAW_THRESHOLD) ||
                (rawBse2Last > BSE_2_HARD_BRAKING_RAW_THRESHOLD);

  bse1Voltage = adcCountsToVoltage(rawBse1Last);
  bse2Voltage = adcCountsToVoltage(rawBse2Last);

  bse1Pct = scaleBrakePressurePercent(bse1Voltage);
  bse2Pct = scaleBrakePressurePercent(bse2Voltage);
}

float scalePercentNormal(int rawValue, int rawMin, int rawMax) {
  float scaled = (float)(rawValue - rawMin) * 100.0f / (float)(rawMax - rawMin);
  return constrain(scaled, 0.0f, 100.0f);
}

float scalePercentInverted(int rawValue, int rawMin, int rawMax) {
  float scaled = 100.0f - ((float)(rawValue - rawMin) * 100.0f / (float)(rawMax - rawMin));
  return constrain(scaled, 0.0f, 100.0f);
}

float adcCountsToVoltage(int rawValue) {
  return ((float)rawValue * ADC_REFERENCE_VOLTAGE) / (float)ADC_MAX_COUNTS;
}

float scaleBrakePressurePercent(float voltage) {
  float scaled = (voltage - BSE_SENSOR_MIN_VOLTAGE) * 100.0f /
                 (BSE_SENSOR_MAX_VOLTAGE - BSE_SENSOR_MIN_VOLTAGE);
  scaled = constrain(scaled, 0.0f, 100.0f);
  return roundf(scaled * 100.0f) / 100.0f;
}

// ---------------- Ready to drive state machine ----------------
void serviceReadyToDrive() {
  refreshRtdStateFromInputs();

  bool pressed = readReadyButtonPressed();

  if (pressed && !readyButtonLast) {
    handleRtdButtonPressed();
    screenDirty = true;
  }

  readyButtonLast = pressed;
}

bool readReadyButtonPressed() {
  return digitalRead(READY_BUTTON_PIN) == HIGH;
}

bool rtdInputsReady() {
  if (!bmsImdFaultDetectionArmed()) {
    return false;
  }

  // APPS implausibility intentionally does not gate RTD; it zeroes the torque
  // command instead (see updatePedal/updatePm100CommandState).
  return !anyFaultActive() && !contactorsOpen();
}

bool readyForRtdButton() {
  return rtdState == RTD_STATE_READY_FOR_BUTTON;
}

void refreshRtdStateFromInputs() {
  if (rtdState == RTD_STATE_READY_TO_DRIVE) {
    if (!rtdInputsReady()) {
      dropReadyToDrive(rtdDropCauseText());
    }
    return;
  }

  bool inputsReady = rtdInputsReady();
  if (!inputsReady) {
    logRtdBlockedReason();
  }

  setRtdState(inputsReady ? RTD_STATE_READY_FOR_BUTTON : RTD_STATE_FAULT_BLOCKED);
}

// Periodically prints why RTD_STATE_FAULT_BLOCKED hasn't cleared, e.g. so a
// stuck contactor/fault/IMD flag after an HV power cycle shows up in the
// serial log instead of just leaving the dashboard stuck on "N".
void logRtdBlockedReason() {
  static uint32_t lastLogMs = 0;
  uint32_t now = millis();
  if (now - lastLogMs < RTD_BLOCKED_LOG_PERIOD_MS) {
    return;
  }
  lastLogMs = now;

  if (!bmsImdFaultDetectionArmed()) {
    Serial.println("[RTD] Blocked: startup fault-detection delay active");
    return;
  }

  Serial.print("[RTD] Blocked:");
  if (imdFault)            Serial.print(" imdFault");
  if (bmsFault)            Serial.print(" bmsFault");
  if (contactorsOpen())    Serial.print(" contactorsOpen");
  if (imdCanTimedOut)      Serial.print(" imdCanTimedOut");
  if (bmsTimedOut)         Serial.print(" bmsTimedOut");
  Serial.print(" bmsFlags=0x");
  printCanHexByte(bmsFlags);
  Serial.print(" imdWarnAlarms=0x");
  Serial.print(imdWarnAlarms, HEX);
  Serial.println();
}

void handleRtdButtonPressed() {
  Serial.print("[RTD] Button pressed, state=");
  Serial.println(rtdStateName(rtdState));

  switch (rtdState) {
    case RTD_STATE_READY_FOR_BUTTON:
      if (REQUIRE_BRAKE_FOR_RTD && !braking) {
        Serial.println("[RTD] Not latched; brake pedal must be pressed");
        break;
      }
      setRtdState(RTD_STATE_READY_TO_DRIVE);
      playTonePattern(RTD_PATTERN, sizeof(RTD_PATTERN) / sizeof(RTD_PATTERN[0]), false);
      Serial.println("[RTD] Ready to drive latched");
      break;

    case RTD_STATE_READY_TO_DRIVE:
      dropReadyToDrive("RTD button pressed");
      break;

    case RTD_STATE_FAULT_BLOCKED:
    default:
      setRtdState(RTD_STATE_FAULT_BLOCKED);
      Serial.println(anyFaultActive() ? "[RTD] Not latched; anyFault is active"
                                       : "[RTD] Not latched; contactors open");
      break;
  }
}

void setRtdState(RtdState newState) {
  char newDriveMode = (newState == RTD_STATE_READY_TO_DRIVE) ? 'D' : 'N';

  if (rtdState == newState) {
    readyToDriveLatched = (newState == RTD_STATE_READY_TO_DRIVE);
    if (driveMode != newDriveMode) {
      setDriveMode(newDriveMode);
      screenDirty = true;
    }
    return;
  }

  rtdState = newState;
  readyToDriveLatched = (newState == RTD_STATE_READY_TO_DRIVE);
  setDriveMode(newDriveMode);
  screenDirty = true;
}

void dropReadyToDrive(const char *reason) {
  bool wasLatched = (rtdState == RTD_STATE_READY_TO_DRIVE) || readyToDriveLatched;

  setRtdState(rtdInputsReady() ? RTD_STATE_READY_FOR_BUTTON : RTD_STATE_FAULT_BLOCKED);

  if (wasLatched && reason != nullptr) {
    Serial.print("[RTD] Dropped; ");
    Serial.println(reason);
    if (strcmp(reason, "RTD button pressed") != 0) {
      showFaultBanner(reason);
    }
  }
}

// Names the specific condition currently blocking RTD, for serial logs and
// the dashboard fault banner.
const char *rtdDropCauseText() {
  if (bmsTimedOut && haveBmsStatus) return "BMS Timeout";
  if (imdCanTimedOut) return "IMD Timeout";
  if (bmsFault) {
    return (haveBms2026FaultCode && bms2026FaultCode != 0)
               ? bmsTemperatureFaultCodeName(bms2026FaultCode)
               : "BMS Fault";
  }
  if (imdFault) return firstTssiFaultName(imdWarnAlarms);
  if (contactorsOpen()) return "Contactors Open";
  return "Fault";
}

void showFaultBanner(const char *text) {
  if (text == nullptr || text[0] == '\0') {
    return;
  }

  if (!faultBannerVisible || strcmp(text, faultBannerText) != 0) {
    strncpy(faultBannerText, text, sizeof(faultBannerText) - 1);
    faultBannerText[sizeof(faultBannerText) - 1] = '\0';
    faultBannerDirty = true;
  }
  faultBannerVisible = true;
  faultBannerUntilMs = millis() + FAULT_BANNER_DURATION_MS;
  screenDirty = true;
}

void serviceFaultBanner() {
  if (!faultBannerVisible) {
    return;
  }

  if ((int32_t)(millis() - faultBannerUntilMs) >= 0) {
    faultBannerVisible = false;
    faultBannerText[0] = '\0';
    dashboardNeedsFullRedraw = true;
    screenDirty = true;
  }
}

// Draws the fault banner over the drive-button row; the row is repainted via
// a full redraw when the banner expires.
void drawFaultBanner(bool force) {
  if (!force && !faultBannerDirty) {
    return;
  }
  faultBannerDirty = false;

  int y = (int)(tft.height() * 0.72f) - 8;
  int h = 54 + 16;

  tft.fillRect(0, y, tft.width(), h, ST7735_RED);
  tft.setTextSize(2);
  tft.setTextColor(ST7735_WHITE);
  tft.setCursor(12, y + 8);
  tft.print("FAULT");
  tft.setTextSize(3);
  tft.setCursor(12, y + 32);
  tft.print(faultBannerText);
  tft.setTextColor(ST7735_WHITE);
}

const char *rtdStateName(RtdState state) {
  switch (state) {
    case RTD_STATE_FAULT_BLOCKED:
      return "FAULT_BLOCKED";
    case RTD_STATE_READY_FOR_BUTTON:
      return "READY_FOR_BUTTON";
    case RTD_STATE_READY_TO_DRIVE:
      return "READY_TO_DRIVE";
    default:
      return "UNKNOWN";
  }
}

void setDriveMode(char newDriveMode) {
  if (newDriveMode != 'D' && newDriveMode != 'N') {
    return;
  }

  if (driveMode == newDriveMode) {
    return;
  }

  driveMode = newDriveMode;
  screenDirty = true;
}

uint8_t pm100DirectionByteForMode(char mode) {
  (void)mode;
  return 0x00;
}

// ---------------- Buzzer ----------------
void playTonePattern(const ToneStep *pattern, uint8_t length, bool repeats) {
  if (pattern == nullptr || length == 0) {
    return;
  }

  activeTonePattern = pattern;
  activeToneLength = length;
  activeToneIndex = 0;
  activeToneInGap = false;
  activeToneRepeats = repeats;
  activeToneDeadlineMs = millis() + activeTonePattern[0].onMs;
  tone(BUZZER_PIN, activeTonePattern[0].frequencyHz);
}

void serviceBuzzer() {
  // 3-tone RTD-ready chime disabled for testing.
  // bool shouldChimeForRtd = readyForRtdButton();
  //
  // if (activeTonePattern != RTD_PATTERN) {
  //   if (shouldChimeForRtd) {
  //     if (activeTonePattern != TS_READY_CHIME_PATTERN) {
  //       playTonePattern(TS_READY_CHIME_PATTERN,
  //                       sizeof(TS_READY_CHIME_PATTERN) / sizeof(TS_READY_CHIME_PATTERN[0]),
  //                       true);
  //     }
  //   } else if (activeTonePattern == TS_READY_CHIME_PATTERN) {
  //     stopBuzzer();
  //     return;
  //   }
  // }

  if (activeTonePattern == nullptr) {
    return;
  }

  uint32_t now = millis();
  if ((int32_t)(now - activeToneDeadlineMs) < 0) {
    return;
  }

  const ToneStep &step = activeTonePattern[activeToneIndex];

  if (!activeToneInGap && step.offMs > 0) {
    noTone(BUZZER_PIN);
    activeToneInGap = true;
    activeToneDeadlineMs = now + step.offMs;
    return;
  }

  activeToneIndex++;
  activeToneInGap = false;

  if (activeToneIndex >= activeToneLength) {
    if (activeToneRepeats) {
      activeToneIndex = 0;
      activeToneInGap = false;
      tone(BUZZER_PIN, activeTonePattern[activeToneIndex].frequencyHz);
      activeToneDeadlineMs = now + activeTonePattern[activeToneIndex].onMs;
    } else {
      stopBuzzer();
    }
    return;
  }

  tone(BUZZER_PIN, activeTonePattern[activeToneIndex].frequencyHz);
  activeToneDeadlineMs = now + activeTonePattern[activeToneIndex].onMs;
}

void stopBuzzer() {
  noTone(BUZZER_PIN);
  activeTonePattern = nullptr;
  activeToneLength = 0;
  activeToneIndex = 0;
  activeToneInGap = false;
  activeToneRepeats = false;
  activeToneDeadlineMs = 0;
}