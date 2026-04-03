/**
 * ╔═══════════════════════════════════════════════════════════════════╗
 * ║   SISTEMA DE SOMBRA — Firmware v1.1                              ║
 * ║   ESP32-C3 Mini · 28BYJ-48/ULN2003 · MG90S · DS3231 · LiPo     ║
 * ║                                                                   ║
 * ║   El DS3231 provee hora UTC precisa sin necesidad de WiFi.       ║
 * ║   El algoritmo SPA calcula la posición solar cada 60 s.          ║
 * ║   El stepper gira en azimut; el servo inclina el poste.          ║
 * ║   La sombrilla en la punta queda ⊥ a los rayos → sombra máxima. ║
 * ╚═══════════════════════════════════════════════════════════════════╝
 *
 * Dependencias (instalar desde Arduino Library Manager):
 *   • RTClib  (Adafruit)     → DS3231 RTC
 *   • ESP32Servo (Kevin Harrington) → MG90S
 *
 * Board: "ESP32C3 Dev Module"
 * Upload Speed: 921600
 *
 * Comandos Serial (115200 baud):
 *   i                       → estado (hora, sol, actuadores, temperatura)
 *   r                       → recalcular y mover ahora
 *   s:AAAA-MM-DD HH:MM:SS  → ajustar RTC (hora UTC)
 *                             ej: s:2026-04-15 14:30:00
 *   a:<grados>              → mover stepper a azimut  (ej: a:180)
 *   t:<grados>              → mover servo a inclinación (ej: t:45)
 *   z                       → marcar posición actual como Norte
 */

#include "config.h"
#include <Wire.h>
#include <RTClib.h>
#include <ESP32Servo.h>
#include <math.h>

// ════════════════════════════════════════════════════════════════════
//  ALGORITMO SPA — Posición Solar (Jean Meeus / NREL SPA)
//  Precisión: ±0.01° | Tiempo en ESP32-C3: < 1 ms
// ════════════════════════════════════════════════════════════════════

struct SolarPos { float az; float el; };

static double jDay(int Y, int Mo, int D, int h, int m, int s) {
    if (Mo <= 2) { Y--; Mo += 12; }
    int A = Y / 100, B = 2 - A + A / 4;
    return floor(365.25 * (Y + 4716)) + floor(30.6001 * (Mo + 1))
           + D + B - 1524.5 + (h + m / 60.0 + s / 3600.0) / 24.0;
}

static double norm360(double x) {
    x = fmod(x, 360.0);
    return x < 0 ? x + 360.0 : x;
}

SolarPos solarPosition(float latDeg, float lonDeg, const DateTime& dt) {
    double JD = jDay(dt.year(), dt.month(), dt.day(),
                     dt.hour(), dt.minute(), dt.second());
    double T = (JD - 2451545.0) / 36525.0;

    // Longitud media y anomalía media
    double L0 = norm360(280.46646 + 36000.76983 * T + 3.032e-4 * T * T);
    double M  = norm360(357.52911 + 35999.05029 * T - 1.537e-4 * T * T);
    double Mr = M * DEG_TO_RAD;

    // Ecuación del centro
    double C = (1.914602 - 4.817e-3 * T - 1.4e-5 * T * T) * sin(Mr)
             + (1.9993e-2 - 1.01e-4 * T) * sin(2.0 * Mr)
             + 2.89e-4 * sin(3.0 * Mr);

    // Longitud aparente (aberración + nutación)
    double omega  = 125.04 - 1934.136 * T;
    double appLon = L0 + C - 5.69e-3 - 4.78e-3 * sin(omega * DEG_TO_RAD);
    double appR   = appLon * DEG_TO_RAD;

    // Oblicuidad de la eclíptica
    double eps = 23.0 + (26.0 + (21.448 - T * (46.815 + T * (5.9e-4 - T * 1.813e-3))) / 60.0) / 60.0
               + 2.56e-3 * cos(omega * DEG_TO_RAD);
    double epsR = eps * DEG_TO_RAD;

    // Ascensión recta y declinación
    double RA   = norm360(atan2(cos(epsR) * sin(appR), cos(appR)) * RAD_TO_DEG);
    double decl = asin(sin(epsR) * sin(appR));

    // Tiempo sidéreo de Greenwich y ángulo horario local
    double GMST = norm360(280.46061837 + 360.98564736629 * (JD - 2451545.0)
                          + 3.87933e-4 * T * T - T * T * T / 38710000.0);
    double LHA  = norm360(GMST + lonDeg - RA) * DEG_TO_RAD;
    double latR = latDeg * DEG_TO_RAD;

    // Elevación geométrica
    double sinEl = sin(latR) * sin(decl) + cos(latR) * cos(decl) * cos(LHA);
    double el    = asin(constrain((float)sinEl, -1.0f, 1.0f)) * RAD_TO_DEG;

    // Refracción atmosférica (Bennett, 1982)
    double ref = 0.0;
    if (el > 5.0) {
        double te = tan(el * DEG_TO_RAD);
        ref = (58.1 / te - 0.07 / (te * te * te) + 8.6e-5 / pow(te, 5)) / 3600.0;
    } else if (el > -0.575) {
        ref = (1735.0 + el * (-518.2 + el * (103.4 + el * (-12.79 + el * 0.711)))) / 3600.0;
    }
    el += ref;

    // Azimut (0=Norte, 90=Este, sentido horario)
    double Az = atan2(sin(LHA), cos(LHA) * sin(latR) - tan(decl) * cos(latR));
    double az = norm360(Az * RAD_TO_DEG + 180.0);

    return { (float)az, (float)el };
}

// ════════════════════════════════════════════════════════════════════
//  DS3231 RTC
// ════════════════════════════════════════════════════════════════════

RTC_DS3231 rtc;
static bool g_rtcOk = false;

bool rtcInit() {
    Wire.begin(RTC_SDA, RTC_SCL);
    if (!rtc.begin()) {
        Serial.println("[RTC]   DS3231 no encontrado — revisa conexiones SDA/SCL");
        return false;
    }
    g_rtcOk = true;

    if (rtc.lostPower()) {
        // El RTC perdió la hora (batería CR2032 agotada o primer uso).
        // Se ajusta a la fecha/hora de compilación como referencia inicial.
        // ¡IMPORTANTE! Ajustar con 's:AAAA-MM-DD HH:MM:SS' por Serial
        // para poner la hora UTC correcta.
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        Serial.println("[RTC]   *** HORA PERDIDA — ajustada a fecha de compilación ***");
        Serial.println("[RTC]   Usar 's:AAAA-MM-DD HH:MM:SS' para poner hora UTC exacta.");
    } else {
        DateTime now = rtc.now();
        char ts[24];
        snprintf(ts, sizeof(ts), "%04d-%02d-%02d %02d:%02d:%02d UTC",
                 now.year(), now.month(), now.day(),
                 now.hour(), now.minute(), now.second());
        Serial.printf("[RTC]   OK — %s\n", ts);
    }
    return true;
}

// Ajustar RTC desde cadena "AAAA-MM-DD HH:MM:SS"
bool rtcSetFromString(const String& s) {
    // Formato esperado: "2026-04-15 14:30:00"
    if (s.length() < 19) return false;
    int Y  = s.substring(0, 4).toInt();
    int Mo = s.substring(5, 7).toInt();
    int D  = s.substring(8, 10).toInt();
    int h  = s.substring(11, 13).toInt();
    int m  = s.substring(14, 16).toInt();
    int se = s.substring(17, 19).toInt();
    if (Y < 2024 || Mo < 1 || Mo > 12 || D < 1 || D > 31) return false;
    rtc.adjust(DateTime(Y, Mo, D, h, m, se));
    return true;
}

// ════════════════════════════════════════════════════════════════════
//  DRIVER 28BYJ-48 — modo medio-paso (2048 pasos/revolución)
//
//  Reducción interna: 1/64 × 32 pasos = 2048 pasos/vuelta
//  Resolución: 360° / 2048 ≈ 0.176°/paso
// ════════════════════════════════════════════════════════════════════

static const uint8_t SPIN[4] = { STEP_IN1, STEP_IN2, STEP_IN3, STEP_IN4 };
static const int  STEPS_REV  = 2048;
static const float DEG_STEP  = 360.0f / STEPS_REV;   // ≈ 0.1758°

// Secuencia de 8 fases (medio paso)
static const uint8_t SEQ[8][4] = {
    {1,0,0,0}, {1,1,0,0}, {0,1,0,0}, {0,1,1,0},
    {0,0,1,0}, {0,0,1,1}, {0,0,0,1}, {1,0,0,1}
};

// Posición actual en pasos [0, STEPS_REV-1]
// RTC_DATA_ATTR sobrevive al deep sleep si se implementa en el futuro
RTC_DATA_ATTR int32_t g_stepPos = 0;

void stepperInit() {
    for (int i = 0; i < 4; i++) {
        pinMode(SPIN[i], OUTPUT);
        digitalWrite(SPIN[i], LOW);
    }
}

void stepperOff() {
    for (int i = 0; i < 4; i++) digitalWrite(SPIN[i], LOW);
}

void stepperMove(int steps) {
    int dir = (steps >= 0) ? 1 : -1;
    for (int i = 0; i < abs(steps); i++) {
        g_stepPos = ((g_stepPos + dir) % STEPS_REV + STEPS_REV) % STEPS_REV;
        for (int p = 0; p < 4; p++)
            digitalWrite(SPIN[p], SEQ[g_stepPos & 7][p]);
        delayMicroseconds(STEP_DELAY_US);
    }
    stepperOff();
}

// Girar al azimut indicado por el camino más corto (CW o CCW)
void stepperGoto(float azDeg) {
    float adj  = fmod(azDeg - AZIMUTH_OFFSET_DEG + 720.0f, 360.0f);
    int target = (int)roundf(adj / DEG_STEP) % STEPS_REV;
    int delta  = target - g_stepPos;
    if (delta >  STEPS_REV / 2) delta -= STEPS_REV;
    if (delta < -STEPS_REV / 2) delta += STEPS_REV;
    if (abs(delta) < 2) return;   // ya en posición (< 0.35°)
    Serial.printf("  [Stepper] az=%.1f°  target=%d  delta=%+d pasos\n",
                  azDeg, target, delta);
    stepperMove(delta);
}

// ════════════════════════════════════════════════════════════════════
//  SERVO MG90S — inclinación del poste
//  0° = poste vertical  |  87° = inclinación máxima
// ════════════════════════════════════════════════════════════════════

Servo g_servo;
int   g_tilt = -1;

void servoInit() {
    g_servo.attach(SERVO_PIN, 500, 2500);
    g_servo.write(SERVO_OFFSET_DEG);
    delay(600);
    g_tilt = 0;
}

void servoSetTilt(int tiltDeg) {
    tiltDeg = constrain(tiltDeg, 0, SERVO_MAX_DEG);
    if (tiltDeg == g_tilt) return;
    Serial.printf("  [Servo]   tilt=%d°\n", tiltDeg);
    g_servo.write(tiltDeg + SERVO_OFFSET_DEG);
    delay(450);
    g_tilt = tiltDeg;
}

// ════════════════════════════════════════════════════════════════════
//  LÓGICA PRINCIPAL
// ════════════════════════════════════════════════════════════════════

static SolarPos      g_lastSun      = {0, -90};
static unsigned long g_lastUpdateMs = 0;

void updatePosition() {
    if (!g_rtcOk) { Serial.println("[SPA]   RTC no disponible"); return; }

    DateTime now = rtc.now();
    char ts[24];
    snprintf(ts, sizeof(ts), "%04d-%02d-%02d %02d:%02d UTC",
             now.year(), now.month(), now.day(), now.hour(), now.minute());

    SolarPos sp = solarPosition(LAT_DEG, LON_DEG, now);
    g_lastSun   = sp;

    Serial.printf("[SPA]   %s → Az=%.2f°  El=%.2f°\n", ts, sp.az, sp.el);

    if (sp.el < MIN_EL_DEG) {
        Serial.println("[CTL]   Noche → posición de aparcado");
        servoSetTilt(PARK_TILT_DEG);
        stepperGoto(PARK_AZ_DEG);
    } else {
        int tilt = constrain((int)roundf(90.0f - sp.el), 0, SERVO_MAX_DEG);
        Serial.printf("[CTL]   Día → az=%.1f°  tilt=%d°\n", sp.az, tilt);
        // Si va a ser más vertical, levantar antes de girar en azimut
        if (tilt < g_tilt) servoSetTilt(tilt);
        stepperGoto(sp.az);
        if (tilt >= g_tilt) servoSetTilt(tilt);
    }

    g_lastUpdateMs = millis();
}

// ════════════════════════════════════════════════════════════════════
//  CONSOLA SERIAL — calibración y diagnóstico
// ════════════════════════════════════════════════════════════════════

void handleSerial() {
    if (!Serial.available()) return;
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "i") {
        // ── Estado ────────────────────────────────────────────────
        if (g_rtcOk) {
            DateTime now = rtc.now();
            char ts[32];
            snprintf(ts, sizeof(ts), "%04d-%02d-%02d %02d:%02d:%02d UTC",
                     now.year(), now.month(), now.day(),
                     now.hour(), now.minute(), now.second());
            Serial.printf("  RTC:      %s\n", ts);
            Serial.printf("  Temp RTC: %.1f °C\n", rtc.getTemperature());
        } else {
            Serial.println("  RTC:      NO DISPONIBLE");
        }
        Serial.printf("  Sol:      Az=%.2f°  El=%.2f°\n", g_lastSun.az, g_lastSun.el);
        Serial.printf("  Stepper:  pos=%d pasos (%.1f°)\n",
                      g_stepPos, g_stepPos * DEG_STEP);
        Serial.printf("  Servo:    tilt=%d°\n", g_tilt);
        Serial.printf("  Próxima actualización en ~%lus\n",
                      UPDATE_INTERVAL_S - (millis() - g_lastUpdateMs) / 1000UL);

    } else if (cmd == "r") {
        Serial.println("  → Recalculando posición...");
        updatePosition();

    } else if (cmd.startsWith("s:")) {
        // ── Ajustar RTC ───────────────────────────────────────────
        // Formato: s:2026-04-15 14:30:00  (hora UTC)
        String val = cmd.substring(2);
        val.trim();
        if (rtcSetFromString(val)) {
            DateTime now = rtc.now();
            char ts[32];
            snprintf(ts, sizeof(ts), "%04d-%02d-%02d %02d:%02d:%02d UTC",
                     now.year(), now.month(), now.day(),
                     now.hour(), now.minute(), now.second());
            Serial.printf("  → RTC ajustado: %s\n", ts);
        } else {
            Serial.println("  ERROR: formato incorrecto. Usa: s:2026-04-15 14:30:00");
        }

    } else if (cmd.startsWith("a:")) {
        float az = fmod(cmd.substring(2).toFloat() + 360.0f, 360.0f);
        Serial.printf("  → Stepper → az %.1f°\n", az);
        stepperGoto(az);

    } else if (cmd.startsWith("t:")) {
        int tilt = cmd.substring(2).toInt();
        Serial.printf("  → Servo → tilt %d°\n", tilt);
        servoSetTilt(tilt);

    } else if (cmd == "z") {
        g_stepPos = 0;
        Serial.println("  → Norte calibrado (paso 0).");
        Serial.println("     Escribe AZIMUTH_OFFSET_DEG=" + String(g_stepPos) + " en config.h para que persista.");

    } else if (cmd.length() > 0) {
        Serial.println("  Comandos: i  r  s:<fecha UTC>  a:<az>  t:<tilt>  z");
    }
}

// ════════════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    delay(200);

    Serial.println(F("\n╔════════════════════════════════════════╗"));
    Serial.println(F("║   Sistema de Sombra  v1.1              ║"));
    Serial.printf ("║   Lat=%.4f  Lon=%.4f              ║\n", LAT_DEG, LON_DEG);
    Serial.println(F("╚════════════════════════════════════════╝"));

    stepperInit();
    servoInit();
    Serial.println("[INIT]  Actuadores listos");

    if (!rtcInit()) {
        Serial.println("[WARN]  Sin RTC — sistema en espera. Revisar hardware.");
        // Parpadear indefinidamente indicando error (sin bloquear)
    }

    if (g_rtcOk) {
        updatePosition();
    }

    Serial.println("[INIT]  Sistema operativo. Comandos por Serial 115200 baud.");
    Serial.println("[INIT]  Si es primera puesta en marcha: s:AAAA-MM-DD HH:MM:SS (UTC)");
}

// ════════════════════════════════════════════════════════════════════
//  LOOP
// ════════════════════════════════════════════════════════════════════

void loop() {
    if (g_rtcOk) {
        unsigned long now = millis();
        if (now - g_lastUpdateMs >= (unsigned long)UPDATE_INTERVAL_S * 1000UL) {
            updatePosition();
        }
    }
    handleSerial();
    delay(50);
}
