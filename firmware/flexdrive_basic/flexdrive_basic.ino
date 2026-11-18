// =============================================================================
//  flexDrive – osnovni firmware v eni datoteki
//  Plošča: ESP32 (TTGO LoRa32), Arduino IDE, jedro esp32 3.x
//
//  Enostavnejša različica firmwara 2.2.1 (mapa firmware/flexdrive/), vse v tej
//  datoteki. S spletno stranjo vaje VL4 (maketa.html) se pogovarja z istimi ukazi in
//  sporočili kot 2.2.1, zato vaja deluje enako. Opazovalnik hitrosti in regulator PID
//  računata enako kot v 2.2.1, zato veljajo isti pragi pri ocenah.
//
//  Zgradba:
//    prekinitve enkoderjev  vsak rob signala A ali B popravi štetje (zunanja prekinitev)
//    opravilo "zanka"       FreeRTOS, vsako 1 ms (vTaskDelayUntil), najvišja prioriteta:
//                             1. prebere enkoderja in oceni hitrost (opazovalnik)
//                             2. izračuna moč motorja: PID (način 1) ali ročni pwm (način 0)
//                             3. vzorec po potrebi shrani v zajem (ukaza step in run)
//    loop()                 bere ukaze in pošilja podatke (telemetrija, zajem); ker ima
//                           nižjo prioriteto, nikoli ne zamakne zanke
//    mutex "lock"           deljene spremenljivke spreminja vedno samo eden od obeh
//
//  Enote: hitrost [obr/s], moč u [% največje moči], pwm -4000..4000.
//
//  Razlike glede na 2.2.1: ni kotne regulacije (mode 2), ukazov zero, capture, cal,
//  obs_bw, obs_zeta; od varnostnih zaustavitev ostaneta prevelika hitrost in
//  watchdog povezave.
// =============================================================================
#include <Arduino.h>
#include "driver/gpio.h"

// ---- Identiteta (stran zahteva različico 2.2 ali novejšo) ----------------------
const char* FW_NAME    = "flexDrive-basic";
const char* FW_VERSION = "2.2.1-basic";

// ---- Serijska povezava -----------------------------------------------------------
const uint32_t SERIAL_BAUD = 921600;   // tako hitro zaradi prenosa zajema 1 kHz
const uint32_t TEL_MS      = 20;       // telemetrija 50 Hz

// ---- Enkoderja (štetje x4) ---------------------------------------------------------
// 1: izhodna os za elastično sklopko, 4096 imp./obr.
// 2: motor, 48 imp./obr. motorja x prenos 20,4 = imp. na obrat izhoda reduktorja.
//    GPIO36..39 nimajo notranjih pull-up uporov.
const uint8_t ENC_PIN_A[2] = {32, 36};
const uint8_t ENC_PIN_B[2] = {33, 37};
const float   ENC_CPR[2]   = {4096.0f, 48.0f * 20.4f};

// ---- Motor (H-mostič: PWM + smer) ------------------------------------------------
const uint8_t  MOTOR_PWM_PIN  = 26;
const uint8_t  MOTOR_DIR_PIN  = 25;
const uint32_t MOTOR_PWM_FREQ = 10000;   // [Hz]
const uint8_t  MOTOR_PWM_BITS = 12;      // 0..4095
const int      PWM_MAX        = 4000;    // največji ukaz (s predznakom: -4000..4000)

// ---- Opazovalnik hitrosti ------------------------------------------------------------
const float OBS_BW[2] = {70.0f, 40.0f};  // pasovna širina [Hz] za enkoder 1 in 2
const float OBS_ZETA  = 0.707f;          // dušenje

// ---- Regulator in omejitve ---------------------------------------------------------
const float    REF_MAX      = 10.0f;     // |ref| [obr/s]
const float    KP_MAX       = 200.0f;    // [%/(obr/s)]
const float    KI_MAX       = 2000.0f;   // [%/obr]
const float    KD_MAX       = 10.0f;     // [% s/obr]
const uint32_t TS_MAX_MS    = 50;        // najdaljša perioda regulatorja [ms]
const float    D_FILTER_S   = 0.005f;    // časovna konstanta filtra D-člena [s]
const float    V_LIMIT      = 7.5f;      // zaustavitev pri preveliki hitrosti [obr/s]
const uint32_t OVERSPEED_MS = 50;        //   ... če traja dlje od tega

// ---- Zajem in profil ---------------------------------------------------------------
const uint32_t CAP_MAX     = 5000;       // največ vzorcev (5 s pri 1 kHz)
const uint32_t DEC_MAX     = 50;         // zajem vsakega dec-tega vzorca
const uint32_t PROF_MAX    = 24;         // točk profila
const uint32_t PROF_MAX_MS = 120000;
const uint32_t STEP_PRE    = 200;        // privzeto število vzorcev pred skokom

// ---- FreeRTOS -------------------------------------------------------------------------
const uint8_t LOOP_CORE     = 1;         // isto jedro kot loop() (jedro 0 ostane za WiFi)
const uint8_t LOOP_PRIORITY = 5;         // višja od loop() (1)

SemaphoreHandle_t lock;                  // mutex za vse deljene spremenljivke

// =============================================================================
//  Nastavitve (spreminjajo jih ukazi s strani)
// =============================================================================
struct Settings {
  uint8_t  mode;    // 0 = odprta zanka (pwm), 1 = regulacija hitrosti
  float    ref;     // referenčna hitrost [obr/s]
  int      pwm;     // ročni ukaz v načinu 0
  float    kp, ki, kd;
  uint32_t ts;      // perioda regulatorja [ms]
  uint8_t  fb;      // povratna zveza: 1 = izhodna os, 2 = motor
  float    umax;    // omejitev moči [%]
  uint32_t wd;      // watchdog [ms], 0 = izklopljen
};
Settings cfg = {0, 0.0f, 0, 0.0f, 0.0f, 0.0f, 1, 1, 100.0f, 0};

// Odgovor na ukaz (glej handleCommand). Tipi so tu na vrhu, ker Arduino samodejne
// prototipe funkcij vstavi pred prvo funkcijo v datoteki.
enum ReplyKind : uint8_t { R_NONE, R_STATE, R_INFO, R_HELP, R_TEXT, R_STEP };
struct Reply {
  ReplyKind kind = R_NONE;
  char      text[160] = "";
  long      n = 0, pre = 0;     // R_STEP: izpiše se po odklepanju (float je počasen)
  float     val = 0.0f;
  uint8_t   mode = 0;
};

// =============================================================================
//  Enkoderja: vsaka sprememba signala A ali B sproži zunanjo prekinitev
// =============================================================================
struct Encoder {
  uint8_t pinA, pinB;
  volatile int32_t  count;    // položaj [imp.]
  volatile uint32_t errors;   // zgrešeni robovi (oba signala hkrati)
  volatile uint8_t  state;    // (A << 1) | B
};
Encoder enc[2];

// Sprememba štetja glede na prejšnje in novo stanje: indeks = (prej << 2) | zdaj
const int8_t QUAD[16] = {
   0, -1, +1,  0,
  +1,  0,  0, -1,
  -1,  0,  0, +1,
   0, +1, -1,  0,
};

// gpio_get_level je hitrejši od digitalRead (pri 6 obr/s je to ~25 000 prekinitev/s).
void IRAM_ATTR encoderEdge(void* arg) {
  Encoder* e = (Encoder*)arg;
  const uint8_t now  = (uint8_t)((gpio_get_level((gpio_num_t)e->pinA) << 1) |
                                 gpio_get_level((gpio_num_t)e->pinB));
  const int8_t  step = QUAD[(e->state << 2) | now];
  if (step != 0) e->count = e->count + step;
  else if ((e->state ^ now) == 0b11) e->errors = e->errors + 1;
  e->state = now;
}

void encodersBegin() {
  for (int i = 0; i < 2; ++i) {
    Encoder& e = enc[i];
    e.pinA = ENC_PIN_A[i]; e.pinB = ENC_PIN_B[i];
    e.count = 0; e.errors = 0;
    const uint8_t pm = (e.pinA >= 34) ? INPUT : INPUT_PULLUP;   // 34..39: samo INPUT
    pinMode(e.pinA, pm);
    pinMode(e.pinB, pm);
    e.state = (uint8_t)((digitalRead(e.pinA) << 1) | digitalRead(e.pinB));
    attachInterruptArg(digitalPinToInterrupt(e.pinA), encoderEdge, &e, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(e.pinB), encoderEdge, &e, CHANGE);
  }
}

// =============================================================================
//  Opazovalnik hitrosti (Luenbergerjev, model: konstantna hitrost)
//
//  Razlika dveh zaporednih štetij v 1 ms je pri 4096 imp./obr groba (1 imp. = 0,24
//  obr/s), zato hitrost ocenimo z opazovalnikom:
//    napoved:  kot += w * dt
//    popravek: y = izmerjeni kot - kot;  kot += alfa * y;  w += (beta / dt) * y
//  Ojačanji alfa in beta postavita pola na pasovno širino OBS_BW in dušenje OBS_ZETA.
// =============================================================================
struct Observer {
  int32_t last = 0;   // zadnje štetje
  float   e    = 0;   // ocenjeni kot - last [imp.]
  float   w    = 0;   // ocenjena hitrost [imp./s]
};
Observer obs[2];

// Prototip po strukturi: sicer ga Arduino samodejno vstavi pred definicijo Observer.
float observerUpdate(Observer& o, int32_t count, float dt, float bwHz);

float observerUpdate(Observer& o, int32_t count, float dt, float bwHz) {
  const float wn = 2.0f * PI * bwHz;
  const float x  = OBS_ZETA * wn * dt;
  const float a0 = expf(-2.0f * x);
  const float a1 = -2.0f * expf(-x) * cosf(wn * dt * sqrtf(1.0f - OBS_ZETA * OBS_ZETA));
  const float alpha = 1.0f - a0;
  const float beta  = 1.0f + a0 + a1;

  o.e += o.w * dt;                                          // napoved
  const float z = (float)(int32_t)((uint32_t)count - (uint32_t)o.last);
  const float y = z - o.e;                                  // popravek
  o.e += alpha * y;
  o.w += (beta / dt) * y;
  o.e -= z;                                                 // kot glede na novo štetje
  o.last = count;
  return o.w;
}

// =============================================================================
//  Motor
// =============================================================================
int pwmNow = 0;

void motorWrite(int pwm) {
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
  digitalWrite(MOTOR_DIR_PIN, pwm < 0 ? HIGH : LOW);
  ledcWrite(MOTOR_PWM_PIN, (uint32_t)abs(pwm));
  pwmNow = pwm;
}

// =============================================================================
//  REGULATOR PID
//
//  Kliče se vsakih ts ms (stran nastavi ts 10, torej 100 Hz).
//    e  = ref - v                    pogrešek hitrosti [obr/s]
//    uP = Kp * e                     proporcionalni del [%]
//    uI = uI + Ki * e * dt           integralni del [%], omejen na ±umax; ko je izhod v
//                                    nasičenju, ne integrira naprej (anti-windup)
//    uD = -Kd * dv/dt                diferencialni del na meritvi in ne na pogrešku, da skok
//                                    reference ne povzroči sunka; dv/dt je filtriran
//    u  = omejitev(uP + uI + uD, ±umax)   [%]
// =============================================================================
float uI       = 0.0f;    // integralni del [%]
float dFilt    = 0.0f;    // filtriran odvod hitrosti [obr/s^2]
float prevV    = 0.0f;
bool  firstRun = true;    // prvi izračun: ni še prejšnje hitrosti
bool  satOut   = false;   // izhod je bil omejen

float pid(float r, float v, float dt) {
  if (firstRun) { prevV = v; dFilt = 0.0f; firstRun = false; }

  const float e  = r - v;
  const float uP = cfg.kp * e;

  const float dv = (v - prevV) / dt;
  prevV  = v;
  dFilt += dt / (D_FILTER_S + dt) * (dv - dFilt);
  const float uD = -cfg.kd * dFilt;

  const float uINew  = uI + cfg.ki * e * dt;
  const float uRaw   = uP + uINew + uD;
  const bool  windup = (uRaw > cfg.umax && e > 0.0f) || (uRaw < -cfg.umax && e < 0.0f);
  if (!windup) uI = uINew;
  uI = constrain(uI, -cfg.umax, cfg.umax);

  const float u = uP + uI + uD;
  satOut = fabsf(u) > cfg.umax;
  return constrain(u, -cfg.umax, cfg.umax);
}

// Stanje regulatorja med izračuni (izhod se drži do naslednjega izračuna)
uint32_t tick      = 0;
uint32_t lastRunUs = 0;
float    uHold     = 0.0f;

void resetController() {
  uI = 0.0f; dFilt = 0.0f; prevV = 0.0f; firstRun = true; satOut = false;
  tick = 0; uHold = 0.0f;
}

// =============================================================================
//  Profil reference (prof add + run) in skok (step)
// =============================================================================
struct ProfPoint { uint32_t t; float v; };
ProfPoint prof[PROF_MAX];
uint32_t  profN      = 0;
bool      profArmed  = false;   // začne se v prvem vzorcu naslednjega zajema
bool      profActive = false;
uint32_t  profT      = 0;       // čas od začetka [ms]

// Odsekoma linearna referenca v času t [ms]
float profileAt(uint32_t t) {
  if (t <= prof[0].t) return prof[0].v;
  for (uint32_t i = 1; i < profN; ++i) {
    if (t <= prof[i].t) {
      const float f = (float)(t - prof[i - 1].t) / (float)(prof[i].t - prof[i - 1].t);
      return prof[i - 1].v + f * (prof[i].v - prof[i - 1].v);
    }
  }
  return prof[profN - 1].v;
}

bool     stepArmed = false;     // v vzorcu stepAt zajema nastavi stepVal
uint32_t stepAt    = 0;
float    stepVal   = 0.0f;      // način 0: pwm, način 1: ref
uint8_t  stepMode  = 0;

// =============================================================================
//  Zajem: zanka shranjuje vzorce, loop() jih po koncu pošlje kot CSV
// =============================================================================
struct CapSample {              // 20 bajtov, da gre 5000 vzorcev v RAM
  int32_t  count[2];
  int16_t  velMilli[2];         // hitrost [0,001 obr/s]
  uint16_t dtUs;                // čas od prejšnjega shranjenega vzorca [us]
  int16_t  pwm;
  int16_t  refMilli;            // ref [0,001 obr/s]
  int16_t  uiCenti;             // integralni del [0,01 %]
};
CapSample capBuf[CAP_MAX];

// IDLE -> (ukaz step/run) REC -> (zanka: zadnji vzorec) DUMP -> (loop: poslano) IDLE
enum CapState : uint8_t { CAP_IDLE, CAP_REC, CAP_DUMP };
volatile CapState capState = CAP_IDLE;
uint32_t capN = 0, capIdx = 0, capDec = 1, capSub = 0, capPrevUs = 0;

// Nastavitve ob začetku zajema (gredo v glavo cap_begin)
Settings capCfg;
bool     capIsStep = false, capIsRun = false;
uint32_t capStepPre = 0;
float    capStepVal = 0.0f;

// =============================================================================
//  Varnost
// =============================================================================
uint32_t    overspeedMs = 0;
uint32_t    lastCmdMs   = 0;      // zadnji prejeti ukaz (watchdog)
bool        wdTripped   = false;
const char* fault       = nullptr;

void stopAll() {                  // motor ugasni, ref 0, način 0
  cfg.mode = 0; cfg.ref = 0.0f; cfg.pwm = 0;
  stepArmed = false; profArmed = false; profActive = false;
  overspeedMs = 0;
  resetController();
}

// =============================================================================
//  ZANKA: opravilo FreeRTOS, vsako 1 ms
// =============================================================================
struct Latest {                   // zadnji vzorec za telemetrijo
  uint32_t tMs;
  int32_t  count[2];
  float    vel[2];
  float    uPct;
  int32_t  profMs;
} latest = {};

uint32_t prevSampleUs = 0;

// En vzorec. Kliče se z zaklenjenim mutexom.
void sample(uint32_t nowUs) {
  const float dt = (float)(nowUs - prevSampleUs) * 1e-6f;   // izmerjeni čas, ne nazivni
  prevSampleUs = nowUs;
  if (dt <= 0.0f) return;

  // 1. Enkoderja in hitrost
  int32_t count[2];
  float   vel[2];
  for (int i = 0; i < 2; ++i) {
    count[i] = enc[i].count;
    vel[i]   = observerUpdate(obs[i], count[i], dt, OBS_BW[i]) / ENC_CPR[i];
  }

  const int32_t capI  = (capState == CAP_REC) ? (int32_t)capIdx : -1;
  bool          force = false;   // regulator izračunaj takoj (skok ali začetek profila)

  // Skok v natanko določenem vzorcu zajema
  if (stepArmed && capI == (int32_t)stepAt) {
    stepArmed = false;
    if (stepMode == cfg.mode) {
      if (cfg.mode == 0) cfg.pwm = constrain((int)lroundf(stepVal), -PWM_MAX, PWM_MAX);
      else { cfg.ref = stepVal; force = true; }
    }
  }

  // Watchdog: brez ukazov s strani dlje od wd ms -> stop
  if (cfg.wd > 0 && !wdTripped && millis() - lastCmdMs > cfg.wd) {
    wdTripped = true; stopAll(); fault = "watchdog";
  }

  // Profil: začne v vzorcu 0 zajema, napreduje 1 ms na vzorec
  if (profArmed && capI == 0) { profArmed = false; profActive = true; profT = 0; force = true; }
  int32_t profMs = -1;
  if (profActive) {
    cfg.ref = profileAt(profT);
    profMs  = (int32_t)profT;
    if (profT >= prof[profN - 1].t) profActive = false;   // zadnja vrednost ostane
    else ++profT;
  }

  const float v = vel[cfg.fb - 1];

  // Prevelika hitrost (samo v zaprti zanki)
  if (cfg.mode == 1) {
    overspeedMs = (fabsf(v) > V_LIMIT) ? overspeedMs + 1 : 0;
    if (overspeedMs > OVERSPEED_MS) { stopAll(); fault = "overspeed"; }
  }

  // 2. Moč motorja
  int   pwm;
  float uPct;
  if (cfg.mode == 1) {
    ++tick;
    if (force || firstRun || tick >= cfg.ts) {
      tick = 0;
      const float dtc = firstRun ? cfg.ts * 1e-3f : (float)(nowUs - lastRunUs) * 1e-6f;
      lastRunUs = nowUs;
      uHold = pid(cfg.ref, v, dtc);
    }
    uPct = uHold;
    pwm  = (int)lroundf(uHold * PWM_MAX / 100.0f);
  } else {
    pwm  = cfg.pwm;
    uPct = pwm * 100.0f / PWM_MAX;
  }
  if (pwm != pwmNow) motorWrite(pwm);

  // 3. Zajem (vsak dec-ti vzorec)
  if (capI >= 0) {
    if (capSub == 0) {
      CapSample& s = capBuf[capIdx];
      s.dtUs = (capIdx == 0) ? 0 : (uint16_t)min<uint32_t>(nowUs - capPrevUs, 65535);
      capPrevUs = nowUs;
      for (int i = 0; i < 2; ++i) {
        s.count[i]    = count[i];
        s.velMilli[i] = (int16_t)constrain(lroundf(vel[i] * 1000.0f), -32767L, 32767L);
      }
      s.pwm      = (int16_t)pwm;
      s.refMilli = (int16_t)constrain(lroundf(cfg.ref * 1000.0f), -32767L, 32767L);
      s.uiCenti  = (int16_t)constrain(lroundf((cfg.mode == 1 ? uI : 0.0f) * 100.0f), -32767L, 32767L);
      ++capIdx;
      if (capIdx >= capN) capState = CAP_DUMP;
    }
    capSub = (capSub + 1 >= capDec) ? 0 : capSub + 1;
  }

  latest.tMs = millis();
  for (int i = 0; i < 2; ++i) { latest.count[i] = count[i]; latest.vel[i] = vel[i]; }
  latest.uPct   = uPct;
  latest.profMs = profMs;
}

void controlTask(void*) {
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(1));   // natanko vsako 1 ms
    const uint32_t now = micros();
    xSemaphoreTake(lock, portMAX_DELAY);
    sample(now);
    xSemaphoreGive(lock);
  }
}

// =============================================================================
//  Sporočila (vsaka vrstica je en objekt JSON); pošilja jih samo loop()
// =============================================================================
void printSettings(const Settings& s) {
  Serial.printf("\"mode\":%u,\"ref\":%.6g,\"pwm\":%d,\"kp\":%.6g,\"ki\":%.6g,\"kd\":%.6g,"
                "\"ts\":%lu,\"fb\":%u,\"umax\":%.6g,\"wd\":%lu",
                (unsigned)s.mode, (double)s.ref, s.pwm, (double)s.kp, (double)s.ki, (double)s.kd,
                (unsigned long)s.ts, (unsigned)s.fb, (double)s.umax, (unsigned long)s.wd);
}

void sendState(const Settings& s) {
  Serial.print("{\"type\":\"state\",");
  printSettings(s);
  Serial.print("}\n");
}

void sendInfo(const Settings& s) {
  const uint64_t mac = ESP.getEfuseMac();   // oznaka makete: zadnji trije bajti MAC
  Serial.printf("{\"type\":\"info\",\"id\":\"%02X%02X%02X\",\"cal\":null,"
                "\"fw\":\"%s\",\"ver\":\"%s\",\"sample_ms\":1,\"tel_ms\":%lu,"
                "\"cpr\":[%.1f,%.1f],\"obs_bw\":[%.6g,%.6g],\"obs_zeta\":%.6g,"
                "\"pwm_max\":%d,\"cap_max\":%lu,\"ref_max\":%.6g,\"prof_max\":%lu,"
                "\"dec_max\":%lu,\"v_limit\":%.6g,",
                (unsigned)((mac >> 24) & 0xFF), (unsigned)((mac >> 32) & 0xFF), (unsigned)((mac >> 40) & 0xFF),
                FW_NAME, FW_VERSION, (unsigned long)TEL_MS,
                ENC_CPR[0], ENC_CPR[1], (double)OBS_BW[0], (double)OBS_BW[1], (double)OBS_ZETA,
                PWM_MAX, (unsigned long)CAP_MAX, (double)REF_MAX, (unsigned long)PROF_MAX,
                (unsigned long)DEC_MAX, (double)V_LIMIT);
  printSettings(s);
  Serial.print("}\n");
}

void sendError(const char* msg) {
  Serial.printf("{\"type\":\"err\",\"msg\":\"%s\"}\n", msg);
}

void sendHelp() {
  Serial.print(
    "# flexDrive-basic, ukazi:\n"
    "#   info | state | help\n"
    "#   tel <0|1>          telemetrija izklop/vklop\n"
    "#   mode <0|1>         0 = odprta zanka (pwm), 1 = regulacija hitrosti\n"
    "#   pwm <n>            moc v nacinu 0, -4000..4000\n"
    "#   ref <x>            referencna hitrost [obr/s], -10..10\n"
    "#   kp|ki|kd <x>       ojacanja PID\n"
    "#   ts <ms>            perioda regulatorja 1..50 ms\n"
    "#   fb <1|2>           povratna zveza: 1 = izhodna os, 2 = motor\n"
    "#   umax <x>           omejitev moci [%], 1..100\n"
    "#   stop               motor ugasni, nacin 0\n"
    "#   step <v> <n> [pre] zajem n vzorcev, v vzorcu pre skok na v (pwm ali ref)\n"
    "#   prof clear | prof add <t_ms> <v>   profil reference\n"
    "#   run <n> [dec]      zazeni profil in zajem\n"
    "#   wd <ms>            stop, ce ms ni ukaza (0 = izklop) | ping\n");
}

bool telemetryOn = true;
uint32_t lastTelMs = 0;

void sendTelemetry() {
  // Kopija pod mutexom, oblikovanje besedila (počasno) brez njega
  xSemaphoreTake(lock, portMAX_DELAY);
  const Latest   l    = latest;
  const uint8_t  mode = cfg.mode;
  const float    ref  = cfg.ref;
  const float    ui   = (mode == 1) ? uI : 0.0f;
  const int      sat  = (mode == 1 && satOut) ? 1 : 0;
  const int      pwm  = pwmNow;
  const uint32_t e1 = enc[0].errors, e2 = enc[1].errors;
  xSemaphoreGive(lock);

  char buf[256];
  const int n = snprintf(buf, sizeof(buf),
      "{\"type\":\"tel\",\"t\":%lu,\"p1\":%ld,\"v1\":%.4f,\"p2\":%ld,\"v2\":%.4f,"
      "\"u\":%d,\"e1\":%lu,\"e2\":%lu,\"mode\":%u,\"ref\":%.6g,\"uc\":%.2f,\"ui\":%.2f,"
      "\"sat\":%d,\"pt\":%ld}\n",
      (unsigned long)l.tMs, (long)l.count[0], l.vel[0], (long)l.count[1], l.vel[1], pwm,
      (unsigned long)e1, (unsigned long)e2, (unsigned)mode, (double)ref, (double)l.uPct,
      (double)ui, sat, (long)l.profMs);
  if (n > 0 && n < (int)sizeof(buf)) Serial.write((const uint8_t*)buf, n);
}

// Pošiljanje končanega zajema. Zanka v stanju DUMP ne piše v capBuf, zato mutex ni potreben.
bool     dumpHeaderSent = false;
uint32_t dumpRow = 0, dumpTUs = 0;

void dumpSome() {
  if (!dumpHeaderSent) {
    if (Serial.availableForWrite() < 600) return;
    Serial.printf("{\"type\":\"cap_begin\",\"n\":%lu,\"fields\":\"t_us,c1,c2,v1,v2,pwm,ref,ui\","
                  "\"dec\":%lu,\"run\":%s,\"obs_bw\":[%.6g,%.6g],\"obs_zeta\":%.6g,"
                  "\"cpr\":[%.1f,%.1f],\"ctrl\":{",
                  (unsigned long)capN, (unsigned long)capDec, capIsRun ? "true" : "false",
                  (double)OBS_BW[0], (double)OBS_BW[1], (double)OBS_ZETA, ENC_CPR[0], ENC_CPR[1]);
    printSettings(capCfg);
    if (capIsStep) Serial.printf("},\"step\":{\"pre\":%lu,\"val\":%.6g,\"mode\":%u}}\n",
                                 (unsigned long)capStepPre, (double)capStepVal, (unsigned)capCfg.mode);
    else Serial.print("},\"step\":null}\n");
    dumpHeaderSent = true; dumpRow = 0; dumpTUs = 0;
  }
  char line[112];
  while (dumpRow < capN && Serial.availableForWrite() > 120) {
    const CapSample& s = capBuf[dumpRow];
    dumpTUs += s.dtUs;
    const int n = snprintf(line, sizeof(line), "%lu,%ld,%ld,%.3f,%.3f,%d,%.3f,%.2f\n",
                           (unsigned long)dumpTUs, (long)s.count[0], (long)s.count[1],
                           s.velMilli[0] * 1e-3f, s.velMilli[1] * 1e-3f, (int)s.pwm,
                           s.refMilli * 1e-3f, s.uiCenti * 1e-2f);
    if (n > 0 && n < (int)sizeof(line)) Serial.write((const uint8_t*)line, n);
    ++dumpRow;
  }
  if (dumpRow >= capN && Serial.availableForWrite() > 64) {
    Serial.printf("{\"type\":\"cap_end\",\"n\":%lu}\n", (unsigned long)capN);
    dumpHeaderSent = false;
    capState = CAP_IDLE;
  }
}

// =============================================================================
//  Ukazi (ena vrstica besedila, npr. "kp 20")
//
//  handleCommand() teče z zaklenjenim mutexom, zato samo spremeni nastavitve in vrne,
//  kaj je treba odgovoriti; besedilo odgovora loop() izpiše po odklepanju.
// =============================================================================
Reply replyKind(ReplyKind k) { Reply r; r.kind = k; return r; }
Reply replyErr(const char* msg) {
  Reply r; r.kind = R_TEXT;
  snprintf(r.text, sizeof(r.text), "{\"type\":\"err\",\"msg\":\"%s\"}\n", msg);
  return r;
}

// Število iz besedila v mejah [lo, hi]; false, če ga ni ali je zunaj meja
bool argFloat(const char* a, float lo, float hi, float& out) {
  char* end;
  const float v = strtof(a, &end);
  if (end == a || !isfinite(v) || v < lo || v > hi) return false;
  out = v;
  return true;
}
bool argInt(const char* a, long lo, long hi, long& out) {
  char* end;
  const long v = strtol(a, &end, 10);
  if (end == a || v < lo || v > hi) return false;
  out = v;
  return true;
}

// Ali se ukaz začne z name in presledkom; arg pokaže na argumente
bool isCmd(const char* cmd, const char* name, const char*& arg) {
  const size_t n = strlen(name);
  if (strncmp(cmd, name, n) != 0 || cmd[n] != ' ') return false;
  arg = cmd + n + 1;
  return true;
}

void startCapture(uint32_t n, uint32_t dec, bool isStep, bool isRun) {
  capCfg = cfg;
  capIsStep = isStep; capIsRun = isRun;
  capN = n; capDec = dec; capIdx = 0; capSub = 0;
  capState = CAP_REC;   // zanka začne v naslednjem vzorcu
}

// "step <v> <n> [pre]": zajem n vzorcev (1 kHz), v vzorcu pre nastavi v (način 0: pwm, 1: ref)
Reply cmdStep(const char* a) {
  char* end;
  const float val = strtof(a, &end);
  if (end == a || !isfinite(val)) return replyErr("step: missing value");
  const char* p = end;
  const long n = strtol(p, &end, 10);
  if (end == p) return replyErr("step: missing sample count");
  p = end;
  long pre = strtol(p, &end, 10);
  if (end == p) pre = STEP_PRE;
  if (n < 1 || n > (long)CAP_MAX) return replyErr("step: sample count out of range");
  if (pre < 0 || pre >= n)        return replyErr("step: pre must be 0..n-1");
  if (cfg.mode == 0 && fabsf(val) > (float)PWM_MAX) return replyErr("step: pwm out of range");
  if (cfg.mode == 1 && fabsf(val) > REF_MAX)        return replyErr("step: ref out of range");
  if (capState != CAP_IDLE) return replyErr("step: capture busy");
  stepArmed = true; stepAt = (uint32_t)pre; stepVal = val; stepMode = cfg.mode;
  capStepPre = (uint32_t)pre; capStepVal = val;
  startCapture((uint32_t)n, 1, true, false);
  Reply r; r.kind = R_STEP;
  r.n = n; r.pre = pre; r.val = val; r.mode = cfg.mode;
  return r;
}

// "run <n> [dec]": zažene profil in zajem n vzorcev (vsak dec-ti) v istem vzorcu
Reply cmdRun(const char* a) {
  char* end;
  const long n = strtol(a, &end, 10);
  if (end == a || n < 1 || n > (long)CAP_MAX) return replyErr("run: sample count out of range");
  const char* p = end;
  long dec = strtol(p, &end, 10);
  if (end == p) dec = 1;
  if (dec < 1 || dec > (long)DEC_MAX) return replyErr("run: dec must be 1..50");
  if (profN == 0) return replyErr("run: no profile (use prof add)");
  if (capState != CAP_IDLE) return replyErr("run: capture busy");
  profArmed = true; profActive = false;
  startCapture((uint32_t)n, (uint32_t)dec, false, true);
  Reply r; r.kind = R_TEXT;
  snprintf(r.text, sizeof(r.text), "{\"type\":\"cap_start\",\"n\":%ld,\"dec\":%ld,\"run\":true}\n", n, dec);
  return r;
}

// "prof clear" | "prof add <t_ms> <v>"
Reply cmdProf(const char* a) {
  if (strcmp(a, "clear") == 0) {
    profN = 0; profArmed = false; profActive = false;
  } else {
    if (strncmp(a, "add ", 4) != 0) return replyErr("prof: use 'prof clear' or 'prof add <t_ms> <val>'");
    char* end;
    const char* p = a + 4;
    const long t = strtol(p, &end, 10);
    if (end == p || t < 0 || t > (long)PROF_MAX_MS) return replyErr("prof: time out of range");
    p = end;
    const float v = strtof(p, &end);
    if (end == p || !isfinite(v) || fabsf(v) > REF_MAX) return replyErr("prof: value out of range");
    if (profActive || profArmed || profN >= PROF_MAX || (profN > 0 && (uint32_t)t <= prof[profN - 1].t))
      return replyErr("prof: full, time not increasing, or profile running");
    prof[profN++] = {(uint32_t)t, v};
  }
  Reply r; r.kind = R_TEXT;
  snprintf(r.text, sizeof(r.text), "{\"type\":\"prof\",\"n\":%lu}\n", (unsigned long)profN);
  return r;
}

Reply handleCommand(const char* c) {
  const char* a = nullptr;
  float f;
  long  n;
  if (strcmp(c, "ping") == 0)  return replyKind(R_NONE);   // samo za watchdog
  if (strcmp(c, "stop") == 0)  { stopAll(); return replyKind(R_STATE); }
  if (strcmp(c, "info") == 0)  return replyKind(R_INFO);
  if (strcmp(c, "state") == 0) return replyKind(R_STATE);
  if (strcmp(c, "help") == 0)  return replyKind(R_HELP);
  if (isCmd(c, "pwm", a)) {
    if (!argInt(a, -PWM_MAX, PWM_MAX, n)) return replyErr("pwm: value out of range");
    if (cfg.mode != 0) return replyErr("pwm: closed loop active (use mode 0)");
    cfg.pwm = (int)n;
    return replyKind(R_NONE);
  }
  if (isCmd(c, "mode", a)) {
    if (!argInt(a, 0, 1, n)) return replyErr("mode: use 0 or 1");
    if ((uint8_t)n != cfg.mode) {
      cfg.mode = (uint8_t)n; cfg.pwm = 0;
      stepArmed = false; profArmed = false; profActive = false;
      overspeedMs = 0;
      resetController();
    }
    return replyKind(R_STATE);
  }
  if (isCmd(c, "ref", a)) {
    if (!argFloat(a, -REF_MAX, REF_MAX, f)) return replyErr("ref: value out of range");
    cfg.ref = f; profArmed = false; profActive = false;
    return replyKind(R_STATE);
  }
  if (isCmd(c, "kp", a)) {
    if (!argFloat(a, 0, KP_MAX, f)) return replyErr("kp: value out of range");
    cfg.kp = f; return replyKind(R_STATE);
  }
  if (isCmd(c, "ki", a)) {
    if (!argFloat(a, 0, KI_MAX, f)) return replyErr("ki: value out of range");
    cfg.ki = f;
    if (f == 0.0f) uI = 0.0f;   // čisti P: brez zamrznjenega integrala
    return replyKind(R_STATE);
  }
  if (isCmd(c, "kd", a)) {
    if (!argFloat(a, 0, KD_MAX, f)) return replyErr("kd: value out of range");
    cfg.kd = f; return replyKind(R_STATE);
  }
  if (isCmd(c, "ts", a)) {
    if (!argInt(a, 1, TS_MAX_MS, n)) return replyErr("ts: value out of range");
    cfg.ts = (uint32_t)n; tick = 0; return replyKind(R_STATE);
  }
  if (isCmd(c, "fb", a)) {
    if (!argInt(a, 1, 2, n)) return replyErr("fb: use 1 or 2");
    if ((uint8_t)n != cfg.fb) { cfg.fb = (uint8_t)n; firstRun = true; overspeedMs = 0; }
    return replyKind(R_STATE);
  }
  if (isCmd(c, "umax", a)) {
    if (!argFloat(a, 1, 100, f)) return replyErr("umax: value out of range");
    cfg.umax = f; return replyKind(R_STATE);
  }
  if (isCmd(c, "wd", a)) {
    if (!argInt(a, 0, 10000, n) || (n != 0 && n < 200)) return replyErr("wd: use 0 or 200..10000");
    cfg.wd = (uint32_t)n; wdTripped = false; return replyKind(R_STATE);
  }
  if (isCmd(c, "tel", a)) {
    if (!argInt(a, 0, 1, n)) return replyErr("tel: use 0 or 1");
    telemetryOn = (n == 1); return replyKind(R_NONE);
  }
  if (isCmd(c, "step", a)) return cmdStep(a);
  if (isCmd(c, "run", a))  return cmdRun(a);
  if (isCmd(c, "prof", a)) return cmdProf(a);
  return replyErr("unknown command");
}

void sendReply(const Reply& r, const Settings& s) {
  switch (r.kind) {
    case R_STATE: sendState(s); break;
    case R_INFO:  sendInfo(s); break;
    case R_HELP:  sendHelp(); break;
    case R_TEXT:  Serial.print(r.text); break;
    case R_STEP:
      Serial.printf("{\"type\":\"cap_start\",\"n\":%ld,\"step\":{\"pre\":%ld,\"val\":%.6g,\"mode\":%u}}\n",
                    r.n, r.pre, (double)r.val, (unsigned)r.mode);
      break;
    default: break;
  }
}

// Bere znake brez čakanja; vsaka cela vrstica je en ukaz in hrani watchdog.
char   cmdBuf[321];
size_t cmdLen = 0;

void readCommands() {
  while (Serial.available() > 0) {
    const char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (cmdLen == 0) continue;
      cmdBuf[cmdLen] = '\0';
      cmdLen = 0;
      xSemaphoreTake(lock, portMAX_DELAY);
      lastCmdMs = millis();
      wdTripped = false;
      const Reply    r = handleCommand(cmdBuf);
      const Settings s = cfg;
      xSemaphoreGive(lock);
      sendReply(r, s);
    } else if (cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = ch;
    }
  }
}

// =============================================================================
//  setup / loop
// =============================================================================
void setup() {
  pinMode(MOTOR_DIR_PIN, OUTPUT);           // najprej: motor ugasnjen
  digitalWrite(MOTOR_DIR_PIN, LOW);
  ledcAttach(MOTOR_PWM_PIN, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
  ledcWrite(MOTOR_PWM_PIN, 0);

  Serial.setTxBufferSize(4096);             // pred begin(): prostor za vrstice zajema
  Serial.begin(SERIAL_BAUD);

  encodersBegin();
  for (int i = 0; i < 2; ++i) obs[i].last = enc[i].count;

  sendInfo(cfg);
  lastTelMs    = millis();
  prevSampleUs = micros();
  lock = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(controlTask, "zanka", 4096, nullptr, LOOP_PRIORITY, nullptr, LOOP_CORE);
}

void loop() {
  readCommands();

  // Varnostna zaustavitev v zanki: napaka in novo stanje
  xSemaphoreTake(lock, portMAX_DELAY);
  const char*    f = fault;
  fault = nullptr;
  const Settings s = cfg;
  xSemaphoreGive(lock);
  if (f) { sendError(f); sendState(s); }

  if (capState == CAP_DUMP) {
    dumpSome();                             // med pošiljanjem zajema ni telemetrije
  } else if (millis() - lastTelMs >= TEL_MS) {
    lastTelMs += TEL_MS;
    if (millis() - lastTelMs >= TEL_MS) lastTelMs = millis();
    if (telemetryOn && Serial.availableForWrite() > 300) sendTelemetry();
  }

  delay(1);                                 // procesor prepusti drugim opravilom
}
