/**
 * stackchan-atama - アタマだけスタックチャン
 *
 * M5Stack単体で動く顔+音声スタックチャン（サーボ不要）
 * USB シリアル or WiFi HTTP API で外部から制御可能
 *
 * Based on the work of:
 *   - robo8080 (AI_StackChan2, M5Unified_StackChan)
 *   - meganetaaan / lovyan03 (m5stack-avatar, M5Unified)
 *   - stack-chan community
 */

#include <M5Unified.h>
#include <Avatar.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <esp_wpa2.h>

Preferences preferences;

// Camera support (CoreS3 only)
#if defined(ENABLE_CAMERA)
#include <esp_camera.h>
#include "mbedtls/base64.h"
static bool camera_initialized = false;
// Forward declarations
bool setupCamera();
void handleCapture();
#endif

// M5Unified 0.2.7 has no pin table row for the official StackChan board, which
// M5GFX reports as board id 27, so _setup_pinmap falls through to the
// board_unknown row and points the internal bus at the AtomS3 pins (38/39).
// Every device behind the real internal bus - PMIC, touch, RTC and the servo
// power expander - is unreachable until it is re-pointed.
static void fixInternalI2C() {
  if (M5.getBoard() != m5::board_t::board_M5StackChan) return;
  if (M5.In_I2C.getSDA() == GPIO_NUM_12 && M5.In_I2C.getSCL() == GPIO_NUM_11) return;
  M5.In_I2C.release();
  M5.In_I2C.begin(I2C_NUM_1, GPIO_NUM_12, GPIO_NUM_11);
}

// Servo support (M5 official StackChan body: Feetech SCS bus)
#if defined(ENABLE_SERVO)
#include <SCSCL.h>

// Wiring and calibration follow the M5Stack StackChan firmware
// (firmware/main/hal/hal_servo.cpp). One step is 0.3125 deg.
static constexpr int SERVO_TX_PIN = 6;
static constexpr int SERVO_RX_PIN = 7;
static constexpr uint32_t SERVO_BAUD = 1000000;
static constexpr uint8_t SERVO_ID_YAW = 1;
static constexpr uint8_t SERVO_ID_PITCH = 2;
static constexpr int SERVO_ZERO_YAW = 460;
static constexpr int SERVO_ZERO_PITCH = 620;
// Travel limits are per servo: pitch runs into the body well before the bus
// limit, so clamping both axes to 0-1000 would drive it into a mechanical stop.
static constexpr int SERVO_RAW_MIN_YAW = 0;
static constexpr int SERVO_RAW_MAX_YAW = 1000;
static constexpr int SERVO_RAW_MIN_PITCH = 30;
static constexpr int SERVO_RAW_MAX_PITCH = 870;
static constexpr int SERVO_MOVE_TIME = 20;
// These servos hold their position by default and keep drawing current while
// stationary, so torque is dropped once a move has settled. The robot only
// nods occasionally, so there is nothing to hold between gestures.
static constexpr uint32_t SERVO_SETTLE_MS = 800;

// Servo power (VM EN) sits behind the PY32 IO expander on the StackChan base.
// The SCS bus stays completely silent until it is switched on.
static constexpr uint8_t IOE_ADDR = 0x6F;
static constexpr uint8_t IOE_REG_VERSION = 0x02;
static constexpr uint8_t IOE_REG_MODE = 0x03;
static constexpr uint8_t IOE_REG_OUTPUT = 0x05;
static constexpr uint8_t IOE_REG_PULLUP = 0x09;
static constexpr uint8_t IOE_PIN_VM_EN = 0;
// The expander is an MCU-based slave; the official firmware clocks it at 100kHz
// and it returns garbage at 400kHz.
static constexpr uint32_t IOE_FREQ = 100000;

static SCSCL scs_bus;
static bool servo_initialized = false;
static uint16_t servo_move_time = SERVO_MOVE_TIME;

static bool ioeBitOn(uint8_t reg, uint8_t bit) {
  uint8_t value = 0;
  if (!M5.In_I2C.readRegister(IOE_ADDR, reg, &value, 1, IOE_FREQ)) return false;
  value |= (1 << bit);
  return M5.In_I2C.writeRegister8(IOE_ADDR, reg, value, IOE_FREQ);
}

// Mirrors the official firmware's sequence in hal_io_expander.cpp:
// setDirection(0, true) -> setPullMode(0, true) -> setServoPowerEnabled(true).
static bool enableServoPower() {
  fixInternalI2C();
  bool ok = ioeBitOn(IOE_REG_MODE, IOE_PIN_VM_EN);
  ok &= ioeBitOn(IOE_REG_PULLUP, IOE_PIN_VM_EN);
  ok &= ioeBitOn(IOE_REG_OUTPUT, IOE_PIN_VM_EN);
  delay(300);
  return ok;
}

static int servoAngleToRaw(float deg, int zero, int raw_min, int raw_max) {
  return constrain(zero + (int)lroundf(deg * 16.0f / 5.0f), raw_min, raw_max);
}

static float servoRawToAngle(int raw, int zero) {
  return (raw - zero) * 5.0f / 16.0f;
}

// A servo that never answers is treated as absent, so a bare CoreS3 built with
// ENABLE_SERVO still boots into the face-only behaviour.
static bool setupServo() {
  enableServoPower();
  Serial1.end();
  Serial1.begin(SERVO_BAUD, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);
  scs_bus.pSerial = &Serial1;

  // Servos need time to boot after the VM rail comes up. Ping is used rather
  // than a position read because it answers as soon as the bus is up.
  for (int i = 0; i < 10; i++) {
    delay(200);
    if (scs_bus.Ping(SERVO_ID_YAW) != -1 && scs_bus.Ping(SERVO_ID_PITCH) != -1) {
      return true;
    }
  }
  return false;
}

// Retried on demand: the expander is not reachable during early boot.
static bool ensureServo() {
  if (!servo_initialized) servo_initialized = setupServo();
  return servo_initialized;
}

static void servoTorque(bool enabled) {
  scs_bus.EnableTorque(SERVO_ID_YAW, enabled ? 1 : 0);
  scs_bus.EnableTorque(SERVO_ID_PITCH, enabled ? 1 : 0);
}

// 0 = nothing scheduled. Compared with a signed difference so millis() rollover
// does not postpone a release by 49 days.
static uint32_t servo_release_at = 0;

static void servoMove(float yaw_deg, float pitch_deg) {
  servoTorque(true);
  scs_bus.WritePos(SERVO_ID_YAW,
                   servoAngleToRaw(yaw_deg, SERVO_ZERO_YAW, SERVO_RAW_MIN_YAW, SERVO_RAW_MAX_YAW),
                   servo_move_time, 0);
  scs_bus.WritePos(SERVO_ID_PITCH,
                   servoAngleToRaw(pitch_deg, SERVO_ZERO_PITCH, SERVO_RAW_MIN_PITCH, SERVO_RAW_MAX_PITCH),
                   servo_move_time, 0);
  servo_release_at = millis() + servo_move_time + SERVO_SETTLE_MS;
}

// Called from loop(); cheap enough to run every iteration.
static void servoReleaseTick() {
  if (servo_release_at == 0 || (int32_t)(millis() - servo_release_at) < 0) return;
  servo_release_at = 0;
  servoTorque(false);
}

static String servoStateJson() {
  int yaw_raw = scs_bus.ReadPos(SERVO_ID_YAW);
  int pitch_raw = scs_bus.ReadPos(SERVO_ID_PITCH);
  if (yaw_raw < 0 || pitch_raw < 0) {
    return String("{\"status\":\"error\",\"error\":\"servo read failed\"}");
  }
  char buf[96];
  snprintf(buf, sizeof(buf), "{\"status\":\"ok\",\"yaw\":%.1f,\"pitch\":%.1f}",
           servoRawToAngle(yaw_raw, SERVO_ZERO_YAW), servoRawToAngle(pitch_raw, SERVO_ZERO_PITCH));
  return String(buf);
}
#endif

// ---- Configuration ----
// WiFi credentials: set via Serial, NVS, or SD card
String wifi_ssid = "";
String wifi_pass = "";
// Both empty selects the PSK path; either one set selects PEAP/MSCHAPv2.
String wifi_identity = "";
String wifi_username = "";

// Forward declarations for WiFi credential management
void saveWiFiCredentials();
void clearWiFiCredentials();
void beginWiFiConnect();
void connectAndReport();

// ---- Audio ----
static constexpr uint8_t m5spk_virtual_channel = 0;

// Simple WAV player using M5.Speaker.playRaw directly (no ESP8266Audio dependency)
class SimpleWavPlayer {
public:
  SimpleWavPlayer(m5::Speaker_Class* speaker, uint8_t virtual_ch = 0)
    : _speaker(speaker), _virtual_ch(virtual_ch), _tri_index(0), _tri_filled(0), _sample_rate(24000) {}

  void setSampleRate(uint32_t rate) { _sample_rate = rate; }
  uint32_t getSampleRate() const { return _sample_rate; }

  // Feed a mono 16-bit sample
  void feedSample(int16_t sample) {
    _tri_buffer[_tri_index][_tri_filled++] = sample;
    if (_tri_filled >= TRI_BUF_SIZE) {
      flush();
    }
  }

  void flush() {
    if (_tri_filled > 0) {
      _speaker->playRaw(_tri_buffer[_tri_index], _tri_filled, _sample_rate, false, 1, _virtual_ch);
      _tri_index = (_tri_index + 1) % 3;
      _tri_filled = 0;
    }
  }

  void reset() {
    _tri_index = 0;
    _tri_filled = 0;
  }

  // Get current audio level for lip sync
  int16_t getLevel() {
    int idx = (_tri_index + 2) % 3;  // previous buffer
    int32_t sum = 0;
    for (int i = 0; i < TRI_BUF_SIZE; i++) {
      sum += abs(_tri_buffer[idx][i]);
    }
    return sum / TRI_BUF_SIZE;
  }

private:
  static constexpr int TRI_BUF_SIZE = 640;
  m5::Speaker_Class* _speaker;
  uint8_t _virtual_ch;
  int16_t _tri_buffer[3][TRI_BUF_SIZE] = {};
  uint8_t _tri_index;
  size_t _tri_filled;
  uint32_t _sample_rate;
};

// WAV header parser
struct WavHeader {
  uint16_t channels;
  uint32_t sample_rate;
  uint16_t bits_per_sample;
  uint32_t data_offset;
  uint32_t data_size;
  bool valid;
};

WavHeader parseWavHeader(const uint8_t* data, size_t len) {
  WavHeader h = {0, 0, 0, 0, 0, false};
  if (len < 44) return h;
  // RIFF header
  if (memcmp(data, "RIFF", 4) != 0 || memcmp(data + 8, "WAVE", 4) != 0) return h;

  // Find fmt chunk
  size_t pos = 12;
  while (pos + 8 <= len) {
    uint32_t chunk_size = data[pos+4] | (data[pos+5]<<8) | (data[pos+6]<<16) | (data[pos+7]<<24);
    if (memcmp(data + pos, "fmt ", 4) == 0) {
      if (pos + 8 + chunk_size > len || chunk_size < 16) return h;
      h.channels = data[pos+10] | (data[pos+11]<<8);
      h.sample_rate = data[pos+12] | (data[pos+13]<<8) | (data[pos+14]<<16) | (data[pos+15]<<24);
      h.bits_per_sample = data[pos+22] | (data[pos+23]<<8);
    } else if (memcmp(data + pos, "data", 4) == 0) {
      h.data_offset = pos + 8;
      h.data_size = chunk_size;
      h.valid = (h.sample_rate > 0 && h.bits_per_sample > 0 && h.channels > 0);
      return h;
    }
    pos += 8 + chunk_size;
    if (chunk_size & 1) pos++;  // Pad byte
  }
  return h;
}

// ---- Globals ----
using namespace m5avatar;
Avatar avatar;
AsyncWebServer server(80);
SimpleWavPlayer* wavPlayer = nullptr;
bool wifi_connected = false;

// WAV playback queue (ring buffer with 4 slots)
static constexpr int WAV_QUEUE_SIZE = 4;
struct WavSlot {
  uint8_t* data;
  size_t len;
};
WavSlot wav_queue[WAV_QUEUE_SIZE] = {};
volatile int wav_queue_head = 0;  // Next slot to play
volatile int wav_queue_tail = 0;  // Next slot to write
volatile bool wav_playing = false;

// Serial command buffer
String serial_cmd_buf = "";

// ---- Side panel ----
static constexpr int PANEL_W_DEFAULT = 96;
static constexpr int PANEL_W_MIN = 64;
static constexpr int PANEL_W_MAX = 240;
static constexpr int PANEL_H = 240;
static constexpr int PANEL_ART_MAX = 200;
// y 200..240 is reserved for a button strip the host pushes with IMG.
static constexpr int PANEL_TEXT_BOTTOM = 200;
static constexpr int PANEL_LINE_H = 16;
static constexpr int PANEL_MAX_LINES = 4;
static constexpr int PANEL_ROWS_PER_LINE = 2;
static constexpr size_t PANEL_IMG_MAX = 65536;
static const char PANEL_ELLIPSIS[] = "\xE2\x80\xA6";  // U+2026, present in efontJA_12

struct PanelLayout {
  int w;
  int face_w;
  int art_x;
  int art_y;
  int art_size;
  int text_y;
  int text_rows;
};

static PanelLayout panelLayout(int w) {
  PanelLayout l;
  l.w = w;
  l.face_w = 320 - w;
  l.art_size = std::min(w, PANEL_ART_MAX);
  l.art_x = (w - l.art_size) / 2;
  l.art_y = (w >= 160) ? 0 : 8;
  l.text_y = l.art_y + l.art_size + 8;
  l.text_rows = std::max(0, (PANEL_TEXT_BOTTOM - l.text_y) / PANEL_LINE_H);
  return l;
}

M5Canvas panel_canvas(&M5.Display);
SemaphoreHandle_t panel_mutex = nullptr;
volatile bool panel_dirty = false;
bool panel_on = false;
bool panel_supported = false;
PanelLayout panel_layout = panelLayout(PANEL_W_DEFAULT);
Face* face_full = nullptr;
Face* face_panel = nullptr;
M5Canvas* face_panel_strip = nullptr;  // Face's tmpSprite, which ~Face does not free

// Runs inside the avatar draw task, which owns the display bus; pushing from
// loop() could land between Face::draw's startWrite/endWrite.
static void panelBlitIfDirty() {
  if (!panel_dirty) return;
  if (xSemaphoreTake(panel_mutex, 0) != pdTRUE) return;
  M5.Display.clearClipRect();
  M5.Display.startWrite();
  panel_canvas.pushSprite(&M5.Display, panel_layout.face_w, 0);
  M5.Display.endWrite();
  panel_dirty = false;
  xSemaphoreGive(panel_mutex);
}

// Wraps the stock mouth so the panel work happens in the draw task, before
// Face::draw pushes its strips. The clip rect keeps the face from painting
// over the panel.
class PanelHook : public Drawable {
public:
  explicit PanelHook(Drawable* inner) : _inner(inner) {}
  ~PanelHook() override { delete _inner; }

  void draw(M5Canvas* spi, BoundingRect rect, DrawContext* ctx) override {
    _inner->draw(spi, rect, ctx);
    if (panel_on) {
      panelBlitIfDirty();
      M5.Display.setClipRect(0, 0, panel_layout.face_w, M5.Display.height());
    } else {
      M5.Display.clearClipRect();
    }
  }

private:
  Drawable* _inner;
};

static void panelInit() {
  if (M5.Display.width() != 320 || M5.Display.height() != 240) return;
  panel_canvas.setPsram(true);
  panel_canvas.setColorDepth(16);
  if (!panel_canvas.createSprite(panel_layout.w, PANEL_H)) return;
  panel_mutex = xSemaphoreCreateMutex();
  if (!panel_mutex) {
    panel_canvas.deleteSprite();
    return;
  }
  panel_canvas.fillSprite(TFT_BLACK);
  face_full = avatar.getFace();
  face_full->setMouth(new PanelHook(face_full->getMouth()));
  panel_supported = true;
}

static uint16_t scaledSize(int v, float s) {
  long r = lroundf(v * s);
  return r < 1 ? 1 : (uint16_t)r;
}

// The stock layout scaled into a face fw wide. Face's constructor leaves its
// balloon/effect/battery pointers uninitialised and ~Face deletes them, so the
// object lives in zeroed memory to make those deletes no-ops.
static Face* makePanelFace(int fw) {
  float s = std::min(1.0f, fw / 160.0f);
  auto pos = [&](int top, int left) {
    return new BoundingRect((int16_t)lroundf((top - 120) * s + 120),
                            (int16_t)lroundf((left - 160) * s + fw / 2));
  };
  void* mem = calloc(1, sizeof(Face));
  if (!mem) return nullptr;
  face_panel_strip = new M5Canvas(&M5.Display);
  return new (mem) Face(
      new PanelHook(new Mouth(scaledSize(50, s), scaledSize(90, s), scaledSize(4, s), scaledSize(60, s))),
      pos(148, 163),
      new Eye(scaledSize(8, s), false), pos(93, 90),
      new Eye(scaledSize(8, s), true), pos(96, 230),
      new Eyeblow(scaledSize(32, s), 0, false), pos(67, 96),
      new Eyeblow(scaledSize(32, s), 0, true), pos(72, 230),
      new BoundingRect(0, 0, fw, PANEL_H),
      new M5Canvas(&M5.Display), face_panel_strip);
}

// Caller must have taken face_panel off the avatar and let a frame finish.
static void destroyPanelFace() {
  if (!face_panel) return;
  face_panel->~Face();
  free(face_panel);
  face_panel = nullptr;
  delete face_panel_strip;
  face_panel_strip = nullptr;
}

// Returns false when the canvas or face for width w cannot be allocated; the
// panel is then left off.
static bool panelSetOn(bool on, int w = PANEL_W_DEFAULT) {
  if (!on) {
    if (panel_on) {
      panel_on = false;
      avatar.setFace(face_full);
    }
    return true;
  }
  if (panel_on && w == panel_layout.w) return true;

  if (panel_on) {
    panel_on = false;
    avatar.setFace(face_full);
  }
  if (w != panel_layout.w) {
    if (face_panel) {
      delay(60);
      destroyPanelFace();
    }
    xSemaphoreTake(panel_mutex, portMAX_DELAY);
    panel_canvas.deleteSprite();
    bool ok = panel_canvas.createSprite(w, PANEL_H) != nullptr;
    if (!ok) panel_canvas.createSprite(panel_layout.w, PANEL_H);
    panel_canvas.fillSprite(TFT_BLACK);
    if (ok) panel_layout = panelLayout(w);
    xSemaphoreGive(panel_mutex);
    if (!ok) return false;
  }
  if (!face_panel) {
    face_panel = makePanelFace(panel_layout.face_w);
    if (!face_panel) return false;
  }
  panel_dirty = true;
  panel_on = true;
  avatar.setFace(face_panel);
  return true;
}

static String panelStateJson() {
  const PanelLayout& l = panel_layout;
  char buf[160];
  snprintf(buf, sizeof(buf),
           "{\"status\":\"ok\",\"panel\":%s,\"width\":%d,\"art\":[%d,%d,%d],\"text_y\":%d,\"text_rows\":%d}",
           panel_on ? "true" : "false", l.w, l.art_x, l.art_y, l.art_size, l.text_y, l.text_rows);
  return String(buf);
}

// len == 0 clears the box.
static bool panelSetImage(const uint8_t* buf, size_t len, int x, int y, int w, int h) {
  xSemaphoreTake(panel_mutex, portMAX_DELAY);
  panel_canvas.fillRect(x, y, w, h, TFT_BLACK);
  bool ok = (len == 0) || panel_canvas.drawJpg(buf, len, x, y, w, h);
  panel_dirty = true;
  xSemaphoreGive(panel_mutex);
  return ok;
}

static bool panelSetArt(const uint8_t* buf, size_t len) {
  const PanelLayout& l = panel_layout;
  return panelSetImage(buf, len, l.art_x, l.art_y, l.art_size, l.art_size);
}

static size_t utf8CharLen(uint8_t c) {
  if ((c & 0xE0) == 0xC0) return 2;
  if ((c & 0xF0) == 0xE0) return 3;
  if ((c & 0xF8) == 0xF0) return 4;
  return 1;
}

// Wraps one line by code point into at most max_rows rows of width text_w and
// returns the number of rows used.
static int panelDrawWrapped(const String& line, int y, int text_w, int max_rows) {
  String row;
  int rows = 0;
  size_t i = 0;
  while (i < line.length()) {
    size_t n = utf8CharLen(line[i]);
    String next = row + line.substring(i, i + n);
    if (panel_canvas.textWidth(next.c_str()) > text_w && row.length() > 0) {
      if (rows == max_rows - 1) break;
      panel_canvas.drawString(row.c_str(), 2, y + rows * PANEL_LINE_H);
      rows++;
      row = "";
      continue;
    }
    row = next;
    i += n;
  }
  if (i < line.length()) {
    while (row.length() > 0 && panel_canvas.textWidth((row + PANEL_ELLIPSIS).c_str()) > text_w) {
      size_t cut = row.length() - 1;
      while (cut > 0 && ((uint8_t)row[cut] & 0xC0) == 0x80) cut--;
      row.remove(cut);
    }
    row += PANEL_ELLIPSIS;
  }
  if (row.length() > 0) {
    panel_canvas.drawString(row.c_str(), 2, y + rows * PANEL_LINE_H);
    rows++;
  }
  return rows > 0 ? rows : 1;
}

// Lines are separated by '|'; an empty string clears the text area. At most
// text_rows rows are drawn, so with 0 rows the text is accepted but not shown.
static int panelSetText(const String& text) {
  String lines[PANEL_MAX_LINES];
  int count = 0;
  int start = 0;
  while (text.length() > 0 && count < PANEL_MAX_LINES) {
    int bar = text.indexOf('|', start);
    if (bar < 0) {
      lines[count++] = text.substring(start);
      break;
    }
    lines[count++] = text.substring(start, bar);
    start = bar + 1;
  }

  xSemaphoreTake(panel_mutex, portMAX_DELAY);
  const PanelLayout& l = panel_layout;
  if (l.text_rows > 0) {
    panel_canvas.fillRect(0, l.text_y, l.w, l.text_rows * PANEL_LINE_H, TFT_BLACK);
  }
  panel_canvas.setFont(&fonts::efontJA_12);
  panel_canvas.setTextColor(TFT_WHITE, TFT_BLACK);
  panel_canvas.setTextDatum(textdatum_t::top_left);
  int y = l.text_y;
  int rows_left = l.text_rows;
  for (int i = 0; i < count && rows_left > 0; i++) {
    int used = panelDrawWrapped(lines[i], y, l.w - 4, std::min(PANEL_ROWS_PER_LINE, rows_left));
    y += used * PANEL_LINE_H;
    rows_left -= used;
  }
  panel_dirty = true;
  xSemaphoreGive(panel_mutex);
  return count;
}

// ---- Touch ----
static constexpr uint32_t TOUCH_DEBOUNCE_MS = 250;
static uint32_t touch_last_ms = 0;

static void touchTick() {
  if (!M5.Touch.isEnabled() || M5.Touch.getCount() != 1) return;
  const auto& t = M5.Touch.getDetail(0);
  if (!t.wasClicked()) return;
  uint32_t now = millis();
  if (now - touch_last_ms < TOUCH_DEBOUNCE_MS) return;
  touch_last_ms = now;

  int face_w = panel_on ? panel_layout.face_w : M5.Display.width();
  const char* zone;
  if (t.x >= face_w) zone = "panel";
  else if (t.x < face_w / 3) zone = "left";
  else if (t.x < face_w * 2 / 3) zone = "center";
  else zone = "right";
  Serial.printf("{\"event\":\"touch\",\"zone\":\"%s\",\"x\":%d,\"y\":%d}\n", zone, t.x, t.y);
  Serial.flush();
}

// ---- Expression map ----
Expression getExpression(const String& name) {
  if (name == "happy" || name == "1") return Expression::Happy;
  if (name == "sleepy" || name == "2") return Expression::Sleepy;
  if (name == "doubt" || name == "3") return Expression::Doubt;
  if (name == "sad" || name == "4") return Expression::Sad;
  if (name == "angry" || name == "5") return Expression::Angry;
  return Expression::Neutral;
}

String getExpressionName(Expression expr) {
  switch (expr) {
    case Expression::Happy: return "happy";
    case Expression::Sleepy: return "sleepy";
    case Expression::Doubt: return "doubt";
    case Expression::Sad: return "sad";
    case Expression::Angry: return "angry";
    default: return "neutral";
  }
}

// WAV queue helpers
int wavQueueCount() {
  int count = wav_queue_tail - wav_queue_head;
  if (count < 0) count += WAV_QUEUE_SIZE;
  return count;
}

bool wavQueueFull() {
  return wavQueueCount() >= (WAV_QUEUE_SIZE - 1);
}

bool wavQueueEmpty() {
  return wav_queue_head == wav_queue_tail;
}

// Enqueue WAV data (returns false if queue full)
bool wavQueuePush(uint8_t* data, size_t len) {
  if (wavQueueFull()) return false;
  wav_queue[wav_queue_tail].data = data;
  wav_queue[wav_queue_tail].len = len;
  wav_queue_tail = (wav_queue_tail + 1) % WAV_QUEUE_SIZE;
  return true;
}

// ---- WAV playback task ----
void wavPlayTask(void* param) {
  while (true) {
    if (!wavQueueEmpty() && !wav_playing) {
      wav_playing = true;

      // Dequeue
      WavSlot& slot = wav_queue[wav_queue_head];
      uint8_t* data = slot.data;
      size_t len = slot.len;
      slot.data = nullptr;
      slot.len = 0;
      wav_queue_head = (wav_queue_head + 1) % WAV_QUEUE_SIZE;

      // Parse WAV header
      WavHeader hdr = parseWavHeader(data, len);
      if (hdr.valid && hdr.bits_per_sample == 16) {
        wavPlayer->setSampleRate(hdr.sample_rate);
        wavPlayer->reset();

        const int16_t* pcm = (const int16_t*)(data + hdr.data_offset);
        size_t total_samples = hdr.data_size / (hdr.bits_per_sample / 8);
        size_t step = hdr.channels;  // Skip interleaved channels (take first channel only)

        for (size_t i = 0; i < total_samples; i += step) {
          // Mono mix if stereo
          if (hdr.channels == 2 && i + 1 < total_samples) {
            int32_t mix = ((int32_t)pcm[i] + (int32_t)pcm[i+1]) / 2;
            wavPlayer->feedSample((int16_t)mix);
          } else {
            wavPlayer->feedSample(pcm[i]);
          }

          // Lip sync (every 640 samples)
          if ((i / step) % 640 == 639) {
            float level = (float)wavPlayer->getLevel() / 5000.0f;
            if (level > 1.0f) level = 1.0f;
            avatar.setMouthOpenRatio(level);
            vTaskDelay(1);
          }
        }
        wavPlayer->flush();
      }

      avatar.setMouthOpenRatio(0.0f);
      free(data);

      wav_playing = false;
    }
    vTaskDelay(10);
  }
}

// ---- HTTP Handlers (ESPAsyncWebServer) ----
void handleRoot(AsyncWebServerRequest *request) {
  request->send(200, "text/plain", "hello from stackchan-atama!");
}

void handleStatus(AsyncWebServerRequest *request) {
  JsonDocument doc;
  doc["status"] = "online";
  doc["ip"] = WiFi.localIP().toString();
  doc["playing"] = wav_playing;
  doc["queued"] = wavQueueCount();
  doc["psram"] = psramFound();

  String json;
  serializeJson(doc, json);
  request->send(200, "application/json", json);
}

void handleFace(AsyncWebServerRequest *request) {
  if (!request->hasParam("expression")) {
    request->send(400, "application/json", "{\"error\":\"expression parameter required\"}");
    return;
  }

  String expr_str = request->getParam("expression")->value();
  Expression expr = getExpression(expr_str);
  avatar.setExpression(expr);

  JsonDocument doc;
  doc["status"] = "ok";
  doc["expression"] = getExpressionName(expr);
  String json;
  serializeJson(doc, json);
  request->send(200, "application/json", json);
}

#if defined(ENABLE_SERVO)
void handleServo(AsyncWebServerRequest *request) {
  if (!servo_initialized) {
    request->send(503, "application/json", "{\"error\":\"servo not available\"}");
    return;
  }
  if (request->hasParam("yaw") && request->hasParam("pitch")) {
    servoMove(request->getParam("yaw")->value().toFloat(),
              request->getParam("pitch")->value().toFloat());
  }
  request->send(200, "application/json", servoStateJson());
}
#endif

void handleSetting(AsyncWebServerRequest *request) {
  if (request->hasParam("volume")) {
    int vol = request->getParam("volume")->value().toInt();
    if (vol >= 0 && vol <= 255) {
      M5.Speaker.setChannelVolume(m5spk_virtual_channel, vol);
    }
  }

  JsonDocument doc;
  doc["status"] = "ok";
  doc["volume"] = M5.Speaker.getChannelVolume(m5spk_virtual_channel);
  String json;
  serializeJson(doc, json);
  request->send(200, "application/json", json);
}

void handleText(AsyncWebServerRequest *request) {
  if (!panel_supported) {
    request->send(503, "application/json", "{\"error\":\"panel not supported\"}");
    return;
  }
  if (!request->hasParam("lines")) {
    request->send(400, "application/json", "{\"error\":\"lines parameter required\"}");
    return;
  }

  JsonDocument doc;
  doc["status"] = "ok";
  doc["lines"] = panelSetText(request->getParam("lines")->value());
  String json;
  serializeJson(doc, json);
  request->send(200, "application/json", json);
}

void handlePanel(AsyncWebServerRequest *request) {
  if (!panel_supported) {
    request->send(503, "application/json", "{\"error\":\"panel not supported\"}");
    return;
  }
  if (request->hasParam("on")) {
    bool on = request->getParam("on")->value() == "1";
    int w = PANEL_W_DEFAULT;
    if (request->hasParam("w")) {
      w = request->getParam("w")->value().toInt();
      if (w < PANEL_W_MIN || w > PANEL_W_MAX) {
        request->send(400, "application/json", "{\"error\":\"w must be 64..240\"}");
        return;
      }
    }
    if (!panelSetOn(on, w)) {
      request->send(500, "application/json", "{\"error\":\"panel alloc failed\"}");
      return;
    }
  }
  request->send(200, "application/json", panelStateJson());
}

void handleNotFound(AsyncWebServerRequest *request) {
  request->send(404, "application/json", "{\"error\":\"not found\"}");
}

// Reads a binary payload after the READY handshake. Returns the bytes received,
// which is short of len if the line stays idle for 30 s.
static size_t readSerialPayload(uint8_t* buf, size_t len) {
  // Drain any leftover data in serial buffer before signaling READY
  while (Serial.available()) Serial.read();

  Serial.println("READY");  // Signal PC to start sending binary
  Serial.flush();  // Ensure READY is sent before reading

  size_t received = 0;
  unsigned long start = millis();
  while (received < len && (millis() - start) < 30000) {
    int avail = Serial.available();
    if (avail > 0) {
      while (avail > 0 && received < len) {
        buf[received++] = Serial.read();
        avail--;
      }
      start = millis();
    } else {
      delay(1);
    }
  }
  return received;
}

// ---- Serial Command Handler ----
// Commands:
//   FACE:happy       - Change expression
//   WAV:12345        - Receive WAV binary (next 12345 bytes)
//   VOLUME:180       - Set volume (0-255)
//   IMG:12345[:x:y[:w:h]] - Receive JPEG for a side panel box (default: art box; IMG:0 clears)
//   TEXT:a|b         - Set side panel text (up to 4 lines, empty clears)
//   PANEL:ON[:w]|OFF|? - Show/hide the side panel, w = 64..240 px (default 96)
//   TOUCH:?          - Print touch controller state
//   STATUS           - Print status JSON
//   WIFI:ssid:pass   - Connect to WiFi
void handleSerialCommand(const String& cmd) {
  if (cmd.startsWith("FACE:")) {
    String expr_str = cmd.substring(5);
    expr_str.trim();
    Expression expr = getExpression(expr_str);
    avatar.setExpression(expr);
    Serial.printf("{\"status\":\"ok\",\"expression\":\"%s\"}\n", getExpressionName(expr).c_str());

  } else if (cmd.startsWith("WAV:")) {
    if (wavQueueFull()) {
      Serial.println("{\"status\":\"error\",\"error\":\"queue full\"}");
      return;
    }
    size_t len = cmd.substring(4).toInt();
    size_t max_wav = psramFound() ? 1000000 : 100000;  // 1MB with PSRAM, 100KB without
    if (len < 44 || len > max_wav) {
      Serial.println("{\"status\":\"error\",\"error\":\"invalid size\"}");
      return;
    }

    // Allocate memory (PSRAM if available, else heap)
    uint8_t* buf = (uint8_t*)(psramFound() ? ps_malloc(len) : malloc(len));
    if (buf == nullptr) {
      Serial.println("{\"status\":\"error\",\"error\":\"memory allocation failed\"}");
      return;
    }

    size_t received = readSerialPayload(buf, len);
    if (received == len) {
      wavQueuePush(buf, len);
      Serial.printf("{\"status\":\"ok\",\"size\":%d,\"queued\":%d}\n", len, wavQueueCount());
    } else {
      free(buf);
      Serial.printf("{\"status\":\"error\",\"error\":\"incomplete\",\"received\":%d,\"expected\":%d}\n", received, len);
    }

  } else if (cmd.startsWith("VOLUME:")) {
    int vol = cmd.substring(7).toInt();
    if (vol >= 0 && vol <= 255) {
      M5.Speaker.setChannelVolume(m5spk_virtual_channel, vol);
    }
    Serial.printf("{\"status\":\"ok\",\"volume\":%d}\n", M5.Speaker.getChannelVolume(m5spk_virtual_channel));

  } else if (cmd.startsWith("IMG:")) {
    if (!panel_supported) {
      Serial.println("{\"status\":\"error\",\"error\":\"panel not supported\"}");
      return;
    }
    // IMG:<bytes>[:<x>:<y>[:<w>:<h>]] places the JPEG in the given panel box
    // (default: the art box). The box is cleared first; IMG:0 only clears it.
    unsigned len = 0;
    const PanelLayout& l = panel_layout;
    int x = l.art_x, y = l.art_y, w = l.art_size, h = l.art_size;
    int n = sscanf(cmd.c_str() + 4, "%u:%d:%d:%d:%d", &len, &x, &y, &w, &h);
    if (n != 1 && n != 3 && n != 5) {
      Serial.println("{\"status\":\"error\",\"error\":\"usage: IMG:<bytes>[:<x>:<y>[:<w>:<h>]]\"}");
      return;
    }
    if (x < 0 || y < 0 || w <= 0 || h <= 0 || x + w > l.w || y + h > PANEL_H) {
      Serial.println("{\"status\":\"error\",\"error\":\"box out of range\"}");
      return;
    }
    if (len == 0) {
      panelSetImage(nullptr, 0, x, y, w, h);
      Serial.println("{\"status\":\"ok\",\"size\":0}");
      return;
    }
    if (len < 4 || len > PANEL_IMG_MAX) {
      Serial.println("{\"status\":\"error\",\"error\":\"invalid size\"}");
      return;
    }
    uint8_t* buf = (uint8_t*)(psramFound() ? ps_malloc(len) : malloc(len));
    if (buf == nullptr) {
      Serial.println("{\"status\":\"error\",\"error\":\"memory allocation failed\"}");
      return;
    }
    size_t received = readSerialPayload(buf, len);
    if (received != len) {
      Serial.printf("{\"status\":\"error\",\"error\":\"incomplete\",\"received\":%d,\"expected\":%d}\n", received, len);
    } else if (buf[0] != 0xFF || buf[1] != 0xD8 || !panelSetImage(buf, len, x, y, w, h)) {
      Serial.println("{\"status\":\"error\",\"error\":\"decode failed\"}");
    } else {
      Serial.printf("{\"status\":\"ok\",\"size\":%u}\n", len);
    }
    free(buf);

  } else if (cmd.startsWith("TEXT:")) {
    if (!panel_supported) {
      Serial.println("{\"status\":\"error\",\"error\":\"panel not supported\"}");
      return;
    }
    Serial.printf("{\"status\":\"ok\",\"lines\":%d}\n", panelSetText(cmd.substring(5)));

  } else if (cmd.startsWith("PANEL:")) {
    String arg = cmd.substring(6);
    arg.trim();
    if (!panel_supported) {
      Serial.println("{\"status\":\"error\",\"error\":\"panel not supported\"}");
    } else if (arg == "?" || arg == "OFF") {
      if (arg == "OFF") panelSetOn(false);
      Serial.println(panelStateJson());
    } else if (arg == "ON" || arg.startsWith("ON:")) {
      int w = PANEL_W_DEFAULT;
      if (arg.length() > 3) {
        String ws = arg.substring(3);
        bool digits = ws.length() <= 3;
        for (size_t i = 0; i < ws.length(); i++) digits = digits && isDigit(ws[i]);
        w = digits ? ws.toInt() : -1;
      }
      if (w < PANEL_W_MIN || w > PANEL_W_MAX) {
        Serial.println("{\"status\":\"error\",\"error\":\"width must be 64..240\"}");
      } else if (!panelSetOn(true, w)) {
        Serial.println("{\"status\":\"error\",\"error\":\"panel alloc failed\"}");
      } else {
        Serial.println(panelStateJson());
      }
    } else {
      Serial.println("{\"status\":\"error\",\"error\":\"expected PANEL:ON[:<w>]|OFF|?\"}");
    }

  } else if (cmd.startsWith("I2C:")) {
    // I2C:<addr>:<reg>[:<count>] (hex) reads registers on the internal bus.
    unsigned addr = 0, reg = 0, n = 1;
    int parsed = sscanf(cmd.c_str() + 4, "%x:%x:%u", &addr, &reg, &n);
    uint8_t buf[16];
    if (parsed < 2 || n < 1 || n > sizeof(buf)) {
      Serial.println("{\"status\":\"error\",\"error\":\"expected I2C:<addr>:<reg>[:<count>]\"}");
    } else if (!M5.In_I2C.readRegister(addr, reg, buf, n, 400000)) {
      Serial.println("{\"status\":\"error\",\"error\":\"nack\"}");
    } else {
      Serial.print("{\"status\":\"ok\",\"data\":[");
      for (unsigned i = 0; i < n; i++) Serial.printf("%s%u", i ? "," : "", buf[i]);
      Serial.println("]}");
    }

  } else if (cmd.startsWith("BRIGHT:")) {
    int level = cmd.substring(7).toInt();
    M5.Display.setBrightness(constrain(level, 0, 255));
    Serial.printf("{\"status\":\"ok\",\"brightness\":%d}\n", M5.Display.getBrightness());

  } else if (cmd == "TOUCH:?") {
    Serial.printf("{\"status\":\"ok\",\"touch\":%s,\"count\":%d}\n",
                  M5.Touch.isEnabled() ? "true" : "false", M5.Touch.getCount());

#if defined(ENABLE_SERVO)
  } else if (cmd.startsWith("SERVO:")) {
    String arg = cmd.substring(6);
    arg.trim();
    if (arg == "SCAN") {
      enableServoPower();
      Serial1.begin(SERVO_BAUD, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);
      scs_bus.pSerial = &Serial1;
      delay(1000);
      Serial.print("{\"ping\":[");
      for (int id = 0; id <= 20; id++) {
        if (scs_bus.Ping(id) != -1) Serial.printf("%d,", id);
        delay(20);
      }
      Serial.println("]}");
    } else if (arg == "IOE") {
      fixInternalI2C();
      uint8_t uid = 0, ver = 0;
      bool ok_uid = M5.In_I2C.readRegister(IOE_ADDR, 0x00, &uid, 1, IOE_FREQ);
      bool ok_ver = M5.In_I2C.readRegister(IOE_ADDR, IOE_REG_VERSION, &ver, 1, IOE_FREQ);
      Serial.printf("{\"uid_ok\":%s,\"uid\":\"0x%02X\",\"ver_ok\":%s,\"ver\":\"0x%02X\"}\n",
                    ok_uid ? "true" : "false", uid, ok_ver ? "true" : "false", ver);

    } else if (arg == "I2CSCAN") {
      // Probed at the expander's own clock: it NACKs at the 400kHz that
      // M5Unified's built-in scanID uses.
      const char *names[] = {"in", "ex"};
      m5::I2C_Class *buses[] = {&M5.In_I2C, &M5.Ex_I2C};
      for (int b = 0; b < 2; b++) {
        m5::I2C_Class *bus = buses[b];
        Serial.printf("{\"bus\":\"%s\",\"port\":%d,\"sda\":%d,\"scl\":%d,\"addr\":[",
                      names[b], (int)bus->getPort(), bus->getSDA(), bus->getSCL());
        for (int addr = 0x08; addr < 0x78; addr++) {
          bool ack = bus->start(addr, false, IOE_FREQ) && bus->stop();
          if (!ack) bus->release();
          if (ack) Serial.printf("\"0x%02X\",", addr);
          delay(2);
        }
        Serial.println("]}");
      }

    } else if (!ensureServo()) {
      Serial.println("{\"status\":\"error\",\"error\":\"servo not available\"}");
    } else if (arg == "?") {
      Serial.println(servoStateJson());
    } else if (arg == "PMIC") {
      // Registers per M5Unified's AXP2101_Class: 0xA4 battery level, 0x01
      // charge status, 0x34 battery mV, 0x38 VBUS mV (both 14-bit).
      fixInternalI2C();
      static constexpr uint8_t AXP_ADDR = 0x34;
      static constexpr uint32_t AXP_FREQ = 400000;
      auto r8 = [](uint8_t reg) -> int {
        uint8_t v = 0;
        return M5.In_I2C.readRegister(AXP_ADDR, reg, &v, 1, AXP_FREQ) ? v : -1;
      };
      auto r14 = [](uint8_t reg) -> int {
        uint8_t buf[2] = {0};
        if (!M5.In_I2C.readRegister(AXP_ADDR, reg, buf, 2, AXP_FREQ)) return -1;
        return ((buf[0] & 0x3F) << 8) | buf[1];
      };
      Serial.printf("{\"chip\":%d,\"st0\":%d,\"st1\":%d,\"batt_pct\":%d,\"vbat_mv\":%d,\"vbus_mv\":%d,\"reg14\":%d,\"vindpm\":%d,\"iin_lim\":%d,\"reg18\":%d,\"chg_cur\":%d}\n",
                    r8(0x03), r8(0x00), r8(0x01), r8(0xA4), r14(0x34), r14(0x38),
                    r8(0x14), r8(0x15), r8(0x16), r8(0x18), r8(0x62));

    } else if (arg.startsWith("CHARGE ")) {
      // Uses M5Unified's own AXP2101 setter rather than a hand-written register
      // write, so the current encoding stays in one place.
      fixInternalI2C();
      m5::AXP2101_Class axp;
      if (!axp.begin()) {
        Serial.println("{\"status\":\"error\",\"error\":\"axp2101 not found\"}");
      } else {
        uint16_t mA = (uint16_t)arg.substring(7).toInt();
        axp.setChargeCurrent(mA);
        Serial.printf("{\"status\":\"ok\",\"requested_ma\":%u}\n", mA);
      }

    } else if (arg.startsWith("TIME ")) {
      servo_move_time = (uint16_t)arg.substring(5).toInt();
      Serial.printf("{\"status\":\"ok\",\"move_time\":%u}\n", servo_move_time);

    } else if (arg == "DIAG") {
      for (uint8_t id : {SERVO_ID_YAW, SERVO_ID_PITCH}) {
        Serial.printf("{\"id\":%u,\"torque\":%d,\"volt_dv\":%d,\"current\":%d,\"load\":%d,\"temp_c\":%d,\"moving\":%d}\n",
                      id, scs_bus.readByte(id, SCSCL_TORQUE_ENABLE),
                      scs_bus.ReadVoltage(id), scs_bus.ReadCurrent(id),
                      scs_bus.ReadLoad(id), scs_bus.ReadTemper(id), scs_bus.ReadMove(id));
      }

    } else if (arg == "ON" || arg == "OFF") {
      bool on = (arg == "ON");
      servo_release_at = 0;
      servoTorque(on);
      Serial.printf("{\"status\":\"ok\",\"torque\":%s}\n", on ? "true" : "false");
    } else {
      int comma = arg.indexOf(',');
      if (comma < 0) {
        Serial.println("{\"status\":\"error\",\"error\":\"expected SERVO:yaw,pitch\"}");
      } else {
        float yaw = arg.substring(0, comma).toFloat();
        float pitch = arg.substring(comma + 1).toFloat();
        servoMove(yaw, pitch);
        Serial.printf("{\"status\":\"ok\",\"yaw\":%.1f,\"pitch\":%.1f}\n", yaw, pitch);
      }
    }

#endif
  } else if (cmd == "STATUS") {
#ifdef ARDUINO_M5STACK_ATOMS3R
#if defined(ENABLE_CAMERA)
    Serial.printf("{\"status\":\"online\",\"board\":\"AtomS3R+Echo\",\"wifi\":%s,\"ip\":\"%s\",\"playing\":%s,\"queued\":%d,\"camera\":%s,\"psram\":%s}\n",
      wifi_connected ? "true" : "false",
      wifi_connected ? WiFi.localIP().toString().c_str() : "none",
      wav_playing ? "true" : "false",
      wavQueueCount(),
      camera_initialized ? "true" : "false",
      psramFound() ? "true" : "false");
#else
    Serial.printf("{\"status\":\"online\",\"board\":\"AtomS3R+Echo\",\"wifi\":%s,\"ip\":\"%s\",\"playing\":%s,\"queued\":%d,\"camera\":false,\"psram\":%s}\n",
      wifi_connected ? "true" : "false",
      wifi_connected ? WiFi.localIP().toString().c_str() : "none",
      wav_playing ? "true" : "false",
      wavQueueCount(),
      psramFound() ? "true" : "false");
#endif
#else
#if defined(ENABLE_CAMERA)
    Serial.printf("{\"status\":\"online\",\"wifi\":%s,\"ip\":\"%s\",\"playing\":%s,\"queued\":%d,\"camera\":%s,\"psram\":%s}\n",
      wifi_connected ? "true" : "false",
      wifi_connected ? WiFi.localIP().toString().c_str() : "none",
      wav_playing ? "true" : "false",
      wavQueueCount(),
      camera_initialized ? "true" : "false",
      psramFound() ? "true" : "false");
#else
    Serial.printf("{\"status\":\"online\",\"wifi\":%s,\"ip\":\"%s\",\"playing\":%s,\"queued\":%d,\"camera\":false,\"psram\":%s}\n",
      wifi_connected ? "true" : "false",
      wifi_connected ? WiFi.localIP().toString().c_str() : "none",
      wav_playing ? "true" : "false",
      wavQueueCount(),
      psramFound() ? "true" : "false");
#endif
#endif

  } else if (cmd == "WIFI:STATUS") {
    Serial.printf("{\"ssid\":\"%s\",\"auth\":\"%s\",\"identity\":\"%s\",\"has_password\":%s,"
                  "\"connected\":%s,\"ip\":\"%s\"}\n",
                  wifi_ssid.c_str(),
                  (wifi_identity.isEmpty() && wifi_username.isEmpty()) ? "psk" : "eap",
                  wifi_username.isEmpty() ? wifi_identity.c_str() : wifi_username.c_str(),
                  wifi_pass.isEmpty() ? "false" : "true",
                  WiFi.status() == WL_CONNECTED ? "true" : "false",
                  WiFi.localIP().toString().c_str());

  } else if (cmd == "WIFI:CLEAR") {
    clearWiFiCredentials();
    Serial.println("{\"status\":\"ok\",\"message\":\"wifi credentials cleared\"}");

  } else if (cmd.startsWith("WIFI:EAP:")) {
    // WIFI:EAP:ssid:identity:username:password
    // Only the password may contain ':'; an empty identity reuses the username.
    int p1 = cmd.indexOf(':', 9);
    int p2 = p1 > 0 ? cmd.indexOf(':', p1 + 1) : -1;
    int p3 = p2 > 0 ? cmd.indexOf(':', p2 + 1) : -1;
    if (p3 > 0) {
      wifi_ssid = cmd.substring(9, p1);
      wifi_identity = cmd.substring(p1 + 1, p2);
      wifi_username = cmd.substring(p2 + 1, p3);
      wifi_pass = cmd.substring(p3 + 1);
      wifi_ssid.trim();
      wifi_identity.trim();
      wifi_username.trim();
      wifi_pass.trim();
      connectAndReport();
    } else {
      Serial.println("{\"status\":\"error\","
                     "\"error\":\"usage: WIFI:EAP:ssid:identity:username:password\"}");
    }

  } else if (cmd.startsWith("WIFI:")) {
    // WIFI:ssid:password
    int first_colon = cmd.indexOf(':', 5);
    if (first_colon > 0) {
      wifi_ssid = cmd.substring(5, first_colon);
      wifi_pass = cmd.substring(first_colon + 1);
      wifi_ssid.trim();
      wifi_pass.trim();
      wifi_identity = "";
      wifi_username = "";
      connectAndReport();
    }

  } else if (cmd == "CAPTURE") {
#if defined(ENABLE_CAMERA)
    handleCapture();
#else
    Serial.println("{\"status\":\"error\",\"error\":\"camera not supported on this device\"}");
#endif

  } else {
    Serial.printf("{\"status\":\"error\",\"error\":\"unknown command: %s\"}\n", cmd.c_str());
  }
}

// ---- Camera Setup (CoreS3 only) ----
#if defined(ENABLE_CAMERA)
static constexpr int CAMERA_FB_COUNT = 2;

bool setupCamera() {
  // Release M5Unified's internal I2C so esp_camera can use GPIO 11/12
  M5.In_I2C.release();

  camera_config_t config = {};
  config.pin_pwdn     = -1;
  config.pin_reset    = -1;
  config.pin_xclk     = 2;
  config.pin_sccb_sda = 12;
  config.pin_sccb_scl = 11;
  config.pin_d7       = 47;
  config.pin_d6       = 48;
  config.pin_d5       = 16;
  config.pin_d4       = 15;
  config.pin_d3       = 42;
  config.pin_d2       = 41;
  config.pin_d1       = 40;
  config.pin_d0       = 39;
  config.pin_vsync    = 46;
  config.pin_href     = 38;
  config.pin_pclk     = 45;

  config.xclk_freq_hz = 20000000;
  config.ledc_timer   = LEDC_TIMER_1;    // Avoid conflict with M5Speaker
  config.ledc_channel = LEDC_CHANNEL_1;  // Avoid conflict with M5Speaker
  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size   = FRAMESIZE_QVGA;  // 320x240
  config.jpeg_quality = 12;
  config.fb_count     = CAMERA_FB_COUNT;
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
  config.sccb_i2c_port = -1;  // Let esp_camera manage I2C

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[CAM] init failed: 0x%x\n", err);
    return false;
  }

  // Discard first few frames to let AWB/AEC stabilize
  for (int i = 0; i < 4; i++) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
    delay(50);
  }

  Serial.println("[CAM] initialized");
  return true;
}

// With CAMERA_GRAB_WHEN_EMPTY the driver returns the oldest queued frame, which is
// CAMERA_FB_COUNT captures stale for an occasional caller. Drain the queue first.
static camera_fb_t* cameraFreshFrame() {
  for (int i = 0; i < CAMERA_FB_COUNT; i++) {
    camera_fb_t* stale = esp_camera_fb_get();
    if (!stale) return NULL;
    esp_camera_fb_return(stale);
  }
  return esp_camera_fb_get();
}

void handleCapture() {
  if (!camera_initialized) {
    Serial.println("{\"status\":\"error\",\"error\":\"camera not available\"}");
    return;
  }

  camera_fb_t* fb = cameraFreshFrame();
  if (!fb) {
    Serial.println("{\"status\":\"error\",\"error\":\"capture failed\"}");
    return;
  }

  // Convert RGB565 to JPEG
  uint8_t* jpg_buf = NULL;
  size_t jpg_len = 0;
  bool ok = frame2jpg(fb, 80, &jpg_buf, &jpg_len);
  esp_camera_fb_return(fb);

  if (!ok || !jpg_buf) {
    Serial.println("{\"status\":\"error\",\"error\":\"jpeg conversion failed\"}");
    return;
  }

  // Base64 encode
  size_t b64_len = 0;
  mbedtls_base64_encode(NULL, 0, &b64_len, jpg_buf, jpg_len);
  uint8_t* b64_buf = (uint8_t*)ps_malloc(b64_len + 1);
  if (!b64_buf) {
    free(jpg_buf);
    Serial.println("{\"status\":\"error\",\"error\":\"base64 alloc failed\"}");
    return;
  }

  mbedtls_base64_encode(b64_buf, b64_len + 1, &b64_len, jpg_buf, jpg_len);
  b64_buf[b64_len] = '\0';
  free(jpg_buf);

  // Send: header line + base64 data + END marker
  Serial.printf("{\"status\":\"ok\",\"format\":\"jpeg\",\"size\":%d,\"b64_size\":%d}\n", jpg_len, b64_len);
  Serial.flush();

  // Send base64 in chunks to avoid serial buffer overflow
  size_t sent = 0;
  while (sent < b64_len) {
    size_t chunk = (b64_len - sent > 1024) ? 1024 : (b64_len - sent);
    Serial.write(b64_buf + sent, chunk);
    sent += chunk;
    Serial.flush();
  }
  Serial.println();  // newline after base64
  Serial.println("END_CAPTURE");
  Serial.flush();

  free(b64_buf);
}

void handleCaptureHTTP(AsyncWebServerRequest *request) {
  if (!camera_initialized) {
    request->send(503, "application/json", "{\"error\":\"camera not available\"}");
    return;
  }

  camera_fb_t* fb = cameraFreshFrame();
  if (!fb) {
    request->send(500, "application/json", "{\"error\":\"capture failed\"}");
    return;
  }

  // Convert RGB565 to JPEG
  uint8_t* jpg_buf = NULL;
  size_t jpg_len = 0;
  bool ok = frame2jpg(fb, 80, &jpg_buf, &jpg_len);
  esp_camera_fb_return(fb);

  if (!ok || !jpg_buf) {
    request->send(500, "application/json", "{\"error\":\"jpeg conversion failed\"}");
    return;
  }

  // Use chunked response with captured buffer pointer and length
  uint8_t* capture_buf = jpg_buf;
  size_t capture_len = jpg_len;
  AsyncWebServerResponse *response = request->beginChunkedResponse(
    "image/jpeg",
    [capture_buf, capture_len](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
      if (index >= capture_len) {
        free(capture_buf);
        return 0;  // Done
      }
      size_t toSend = capture_len - index;
      if (toSend > maxLen) toSend = maxLen;
      memcpy(buffer, capture_buf + index, toSend);
      return toSend;
    }
  );
  response->addHeader("Content-Disposition", "inline; filename=\"capture.jpg\"");
  request->send(response);
}
#endif

// ---- WiFi Setup ----
void saveWiFiCredentials() {
  preferences.begin("wifi", false);
  preferences.putString("ssid", wifi_ssid);
  preferences.putString("pass", wifi_pass);
  preferences.putString("identity", wifi_identity);
  preferences.putString("username", wifi_username);
  preferences.end();
  Serial.println("[NVS] WiFi credentials saved");
}

void beginWiFiConnect() {
  WiFi.mode(WIFI_STA);
  if (wifi_identity.isEmpty() && wifi_username.isEmpty()) {
    // Without this, a PSK network cannot follow an enterprise one.
    esp_wifi_sta_wpa2_ent_disable();
    WiFi.begin(wifi_ssid.c_str(), wifi_pass.c_str());
    return;
  }
  const String& identity = wifi_identity.isEmpty() ? wifi_username : wifi_identity;
  esp_wifi_sta_wpa2_ent_set_identity((const unsigned char*)identity.c_str(), identity.length());
  if (!wifi_username.isEmpty()) {
    esp_wifi_sta_wpa2_ent_set_username((const unsigned char*)wifi_username.c_str(), wifi_username.length());
  }
  esp_wifi_sta_wpa2_ent_set_password((const unsigned char*)wifi_pass.c_str(), wifi_pass.length());
  esp_wifi_sta_wpa2_ent_enable();
  WiFi.begin(wifi_ssid.c_str());
}

void connectAndReport() {
  Serial.println("{\"status\":\"connecting\"}");
  avatar.setSpeechText("WiFi接続中...");
  beginWiFiConnect();
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 20) {
    delay(500);
    retry++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    wifi_connected = true;
    WiFi.setSleep(false);  // Disable WiFi power saving for lower latency
    saveWiFiCredentials();
    server.begin();
    Serial.printf("{\"status\":\"ok\",\"ip\":\"%s\"}\n", WiFi.localIP().toString().c_str());
    avatar.setSpeechText(WiFi.localIP().toString().c_str());
    delay(2000);
    avatar.setSpeechText("");
  } else {
    Serial.println("{\"status\":\"error\",\"error\":\"wifi connection failed\"}");
    avatar.setSpeechText("WiFi失敗");
    delay(1000);
    avatar.setSpeechText("");
  }
}

void clearWiFiCredentials() {
  preferences.begin("wifi", false);
  preferences.clear();
  preferences.end();
  wifi_ssid = "";
  wifi_pass = "";
  wifi_identity = "";
  wifi_username = "";
  Serial.println("[NVS] WiFi credentials cleared");
}

void setupWiFi() {
  // Load credentials from NVS
  preferences.begin("wifi", true);  // read-only
  wifi_ssid = preferences.getString("ssid", "");
  wifi_pass = preferences.getString("pass", "");
  wifi_identity = preferences.getString("identity", "");
  wifi_username = preferences.getString("username", "");
  preferences.end();

  // The password is reported as present/absent rather than assumed: a stored
  // ssid with no password still reaches WiFi.begin() and fails on any secured
  // network, and "credentials loaded" alone hides that.
  if (!wifi_ssid.isEmpty()) {
    Serial.printf("[NVS] WiFi credentials loaded: ssid=\"%s\" password=%s auth=%s\n",
                  wifi_ssid.c_str(), wifi_pass.isEmpty() ? "MISSING" : "set",
                  (wifi_identity.isEmpty() && wifi_username.isEmpty()) ? "psk" : "eap");
  }

  if (wifi_ssid.isEmpty()) {
    Serial.println("No WiFi credentials. Use serial command WIFI:ssid:password to connect.");
    avatar.setSpeechText("USB Ready");
    delay(1500);
    avatar.setSpeechText("");
    return;
  } else {
    beginWiFiConnect();
  }

  // Wait for connection
  avatar.setSpeechText("WiFi接続中...");
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 30) {
    delay(500);
    Serial.print(".");
    retry++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifi_connected = true;
    WiFi.setSleep(false);  // Disable WiFi power saving for lower latency
    Serial.printf("\nConnected! IP: %s\n", WiFi.localIP().toString().c_str());
    avatar.setSpeechText(WiFi.localIP().toString().c_str());
    delay(2000);
    avatar.setSpeechText("");
  } else {
    Serial.println("\nWiFi connection failed. Use serial commands instead.");
    avatar.setSpeechText("USB Ready");
    delay(1500);
    avatar.setSpeechText("");
  }
}

// ---- Setup ----
void setup() {
  auto cfg = M5.config();
#ifdef ARDUINO_M5STACK_ATOMS3R
  // AtomS3R with Atomic Echo Base configuration
  cfg.external_speaker.atomic_echo = true;
#else
  // Default configuration (CoreS3, Core2, etc.)
  cfg.external_spk = true;
#endif
  M5.begin(cfg);
  // The backlight is driven through the PMIC, so re-apply it once the bus is fixed.
  fixInternalI2C();
  M5.Display.setBrightness(M5.Display.getBrightness());

  // Re-init Serial after M5.begin() for USB CDC on ESP32-S3
#if defined(CONFIG_IDF_TARGET_ESP32S3)
  Serial.setRxBufferSize(32768);  // 32KB receive buffer (ESP32-S3 has more RAM)
#else
  Serial.setRxBufferSize(4096);   // 4KB receive buffer (ESP32 classic)
#endif

#ifdef ARDUINO_M5STACK_ATOMS3R
  Serial.begin(115200);  // Optimized for AtomS3R
#else
  Serial.begin(921600);  // Default for CoreS3/Core2
#endif
  delay(500);

  // Speaker config
  {
    auto spk_cfg = M5.Speaker.config();
    spk_cfg.sample_rate = 96000;
    spk_cfg.dma_buf_count = 20;
    spk_cfg.dma_buf_len = 512;
    M5.Speaker.config(spk_cfg);
#ifdef ARDUINO_M5STACK_ATOMS3R
    // Atomic Echo Base: use lower volume to prevent distortion
    M5.Speaker.setVolume(192);
    M5.Speaker.setChannelVolume(m5spk_virtual_channel, 192);
#else
    // Default: maximum volume
    M5.Speaker.setVolume(255);
    M5.Speaker.setChannelVolume(m5spk_virtual_channel, 255);
#endif
  }

  // Start avatar
#ifdef ARDUINO_M5STACK_ATOMS3R
  // AtomS3R: Initialize avatar for headless operation (no display)
  avatar.setScale(0.5);
  avatar.setPosition(-56, -96);
  avatar.init();  // Avatar library auto-detects no display and uses appropriate size
#else
  // M5Stack devices: Use default initialization with display
  avatar.init();
  panelInit();
#endif
  avatar.setExpression(Expression::Neutral);

  // Audio output (self-contained WAV player, no ESP8266Audio dependency)
  wavPlayer = new SimpleWavPlayer(&M5.Speaker, m5spk_virtual_channel);

  // Start WAV playback task on core 1 (same as Arduino loop, needed for M5Stack Core speaker)
  xTaskCreatePinnedToCore(wavPlayTask, "wavPlay", 8192, nullptr, 1, nullptr, APP_CPU_NUM);

  // WiFi (optional - will skip if no SD card credentials)
  setupWiFi();

  // HTTP server (ESPAsyncWebServer - register endpoints)
  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/face", HTTP_GET, handleFace);
#if defined(ENABLE_SERVO)
  server.on("/servo", HTTP_GET, handleServo);
#endif
  server.on("/setting", HTTP_GET, handleSetting);
#if defined(ENABLE_CAMERA)
  server.on("/capture", HTTP_GET, handleCaptureHTTP);
#endif

  // play_wav: POST binary WAV data
  // Usage: curl -X POST -H "Content-Type: application/octet-stream" --data-binary @audio.wav http://<ip>/play_wav
  server.on("/play_wav", HTTP_POST,
    [](AsyncWebServerRequest *request) {
      uint8_t* buf = (uint8_t*)request->_tempObject;
      if (!buf) {
        request->send(400, "application/json", "{\"error\":\"no data received\"}");
        return;
      }
      request->_tempObject = nullptr;
      size_t len = request->contentLength();
      if (wavQueueFull()) {
        free(buf);
        request->send(409, "application/json", "{\"error\":\"queue full\"}");
        return;
      }
      wavQueuePush(buf, len);
      JsonDocument doc;
      doc["status"] = "ok";
      doc["size"] = len;
      doc["queued"] = wavQueueCount();
      String json;
      serializeJson(doc, json);
      request->send(200, "application/json", json);
    },
    NULL,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      if (index == 0) {
        size_t max_wav = psramFound() ? 512000 : 80000;  // PSRAMなし: 80KB制限
        if (total < 44 || total > max_wav) return;
        uint8_t* buf = (uint8_t*)(psramFound() ? ps_malloc(total) : malloc(total));
        if (!buf) return;
        request->_tempObject = buf;
      }
      uint8_t* buf = (uint8_t*)request->_tempObject;
      if (buf) {
        memcpy(buf + index, data, len);
      }
    }
  );

  // img: POST a JPEG for the side panel art box
  // Usage: curl -X POST -H "Content-Type: application/octet-stream" --data-binary @art.jpg http://<ip>/img
  server.on("/img", HTTP_POST,
    [](AsyncWebServerRequest *request) {
      uint8_t* buf = (uint8_t*)request->_tempObject;
      request->_tempObject = nullptr;
      if (!panel_supported) {
        free(buf);
        request->send(503, "application/json", "{\"error\":\"panel not supported\"}");
        return;
      }
      if (!buf) {
        request->send(400, "application/json", "{\"error\":\"no data received\"}");
        return;
      }
      size_t len = request->contentLength();
      bool ok = buf[0] == 0xFF && buf[1] == 0xD8 && panelSetArt(buf, len);
      free(buf);
      if (!ok) {
        request->send(400, "application/json", "{\"error\":\"decode failed\"}");
        return;
      }
      JsonDocument doc;
      doc["status"] = "ok";
      doc["size"] = len;
      String json;
      serializeJson(doc, json);
      request->send(200, "application/json", json);
    },
    NULL,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      if (index == 0) {
        if (total < 4 || total > PANEL_IMG_MAX) return;
        uint8_t* buf = (uint8_t*)(psramFound() ? ps_malloc(total) : malloc(total));
        if (!buf) return;
        request->_tempObject = buf;
      }
      uint8_t* buf = (uint8_t*)request->_tempObject;
      if (buf) {
        memcpy(buf + index, data, len);
      }
    }
  );
  server.on("/text", HTTP_GET, handleText);
  server.on("/panel", HTTP_GET, handlePanel);

  server.onNotFound(handleNotFound);
  if (wifi_connected) {
    server.begin();
    Serial.println("HTTP server started on port 80");
  }

  // Servo first: the camera driver takes over the shared I2C pins, which leaves
  // the IO expander (servo power) unreachable if it is initialised afterwards.
#if defined(ENABLE_SERVO)
  servo_initialized = setupServo();
#endif

  // Camera init (CoreS3 only)
#if defined(ENABLE_CAMERA)
  camera_initialized = setupCamera();
#endif

#ifdef ARDUINO_M5STACK_ATOMS3R
  Serial.println("[AtomS3R+Echo] Serial commands ready: FACE:expr WAV:size VOLUME:vol IMG:size TEXT:a|b PANEL:ON[:w]|OFF|? TOUCH:? STATUS WIFI:ssid:pass CAPTURE");
#else
  Serial.println("Serial commands ready: FACE:expr WAV:size VOLUME:vol IMG:size TEXT:a|b PANEL:ON[:w]|OFF|? TOUCH:? STATUS WIFI:ssid:pass CAPTURE");
#endif
}

// ---- Loop ----
void loop() {
  M5.update();
  touchTick();

  // Note: ESPAsyncWebServer handles HTTP clients automatically (no handleClient() needed)

  // Handle serial commands
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serial_cmd_buf.length() > 0) {
        handleSerialCommand(serial_cmd_buf);
        serial_cmd_buf = "";
        // HWCDC writes are buffered; without this a reply can sit in the buffer
        // and be read by the host as the answer to its next command.
        Serial.flush();
      }
    } else {
      serial_cmd_buf += c;
    }
  }

#if defined(ENABLE_SERVO)
  if (servo_initialized) servoReleaseTick();
#endif

  delay(1);
}
