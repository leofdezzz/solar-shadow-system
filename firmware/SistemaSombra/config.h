#pragma once
/**
 * ╔══════════════════════════════════════════════════════════════╗
 * ║  SISTEMA DE SOMBRA — config.h                               ║
 * ║  Edita este archivo antes de compilar.                      ║
 * ╚══════════════════════════════════════════════════════════════╝
 */

// ─────────────────────────────────────────────────────────────────────
//  Ubicación geográfica  (+ = Norte / Este)
// ─────────────────────────────────────────────────────────────────────
#define LAT_DEG     40.4168f    // Madrid
#define LON_DEG     -3.7038f   // Madrid

// ─────────────────────────────────────────────────────────────────────
//  Pines — ESP32-C3 Mini
//
//  ULN2003 ──► 28BYJ-48 (azimut)
//    IN1 → GPIO 1   (cable naranja del motor)
//    IN2 → GPIO 3   (cable amarillo)
//    IN3 → GPIO 4   (cable rosa)
//    IN4 → GPIO 5   (cable azul)
//    VCC → 5 V       (cable rojo → fuente 5 V externa, NO al ESP32)
//    GND → GND común
//
//  MG90S (inclinación)
//    Signal → GPIO 10  (cable naranja/amarillo)
//    VCC    → 5 V
//    GND    → GND común
//
//  DS3231 RTC (I2C)
//    SDA → GPIO 6
//    SCL → GPIO 7
//    VCC → 3.3 V  (el DS3231 funciona a 3.3 V)
//    GND → GND común
//    Batería CR2032 en zócalo del módulo (mantiene hora sin alimentación)
// ─────────────────────────────────────────────────────────────────────
#define STEP_IN1    1
#define STEP_IN2    3
#define STEP_IN3    4
#define STEP_IN4    5

#define SERVO_PIN   10

#define RTC_SDA     6
#define RTC_SCL     7

// ─────────────────────────────────────────────────────────────────────
//  Calibración mecánica
//
//  AZIMUTH_OFFSET_DEG:
//    Ángulo al que apunta físicamente el brazo cuando el stepper
//    está en su posición de inicio (paso 0 al arrancar).
//    Si el brazo apunta al Norte al encender → 0.0
//    Si apunta al Este                       → 90.0
//    Ajustar con el comando 'z' por Serial o modificando este valor.
//
//  SERVO_OFFSET_DEG:
//    Corrección en grados si el servo no queda exactamente vertical
//    con tilt = 0°. Ajustar en pasos de ±1°.
// ─────────────────────────────────────────────────────────────────────
#define AZIMUTH_OFFSET_DEG   0.0f
#define SERVO_OFFSET_DEG     0       // [grados, rango ±15°]

// ─────────────────────────────────────────────────────────────────────
//  Comportamiento
// ─────────────────────────────────────────────────────────────────────
// Intervalo de actualización de posición (segundos)
#define UPDATE_INTERVAL_S    60

// Elevación mínima para considerar que es de día
#define MIN_EL_DEG      1.0f

// Posición de aparcado nocturno
#define PARK_AZ_DEG     0.0f    // brazo apunta al Norte
#define PARK_TILT_DEG   0       // brazo vertical

// Límite mecánico de inclinación del servo
#define SERVO_MAX_DEG   87

// Velocidad del stepper (µs entre pasos en modo medio-paso)
// 2000 µs ≈ 4–5 RPM — silencioso y seguro
// 1500 µs ≈ 6–7 RPM — más rápido
#define STEP_DELAY_US   2000
