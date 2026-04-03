# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Descripción del Proyecto

**Sistema de Sombra** es un sistema autónomo de sombrilla solar que rastrea la posición del sol y orienta una sombrilla para maximizar la cobertura de sombra sobre un punto fijo. Combina:

- **Algoritmo SPA** (Solar Position Algorithm) basado en Jean Meeus / NREL, implementado en Python y JavaScript
- **Firmware para ESP32-C3** (Arduino framework) para control real del hardware
- **Visualización 3D interactiva** (Three.js) en el navegador
- **Simulación y validación** en Python con matplotlib

## Archivos del Proyecto

```
informe_tecnico_sombrilla_solar.md   ← Documentación técnica principal (en español)
simulacion_solar.py                  ← Simulación Python + validación del algoritmo SPA
simulacion_sombrilla_3D.html         ← Visualización 3D interactiva (Three.js)
solar_trajectory.png                 ← Generado por simulacion_solar.py
shadow_analysis.png                  ← Generado por simulacion_solar.py
seasonal_comparison.png              ← Generado por simulacion_solar.py
```

## Cómo Ejecutar

**Simulación Python** (genera las 3 imágenes PNG):
```bash
python simulacion_solar.py
```
Dependencias: `numpy`, `matplotlib` (no hay `requirements.txt`; instalar con `pip install numpy matplotlib`).

**Visualización 3D**: Abrir `simulacion_sombrilla_3D.html` directamente en el navegador. No requiere servidor.

**Firmware ESP32-C3**: Usar Arduino IDE con soporte de placa ESP32. El sketch aún no está escrito — ver la documentación para pseudocódigo y conexiones GPIO.

## Arquitectura del Sistema

### Mecanismo de Control (Opción B: Palo Inclinado)

El diseño elegido inclina todo el palo hacia el sol en lugar de usar una sombrilla grande fija. Esto garantiza que la sombra siempre cubra el punto objetivo (0,0).

- **Servo 1 (Azimut)**: SG90 360° continuo — rotación horizontal de la base (0–360°)
- **Servo 2 (Inclinación)**: MG90S metal gear — inclina el palo entero hacia el sol (0–87° desde vertical)

**Conversión de coordenadas solares a ángulos de servo:**
```
servo1_angle = azimut_solar        (0–360°)
servo2_angle = 90° - elevacion     (ángulo de inclinación del palo)
```

### Algoritmo SPA

El núcleo del sistema. Implementado en:
- `simulacion_solar.py` → función `solar_position(lat, lon, utc_datetime)`
- `simulacion_sombrilla_3D.html` → función JS `solarPosition(lat, lon, date)`

Entradas: latitud, longitud, fecha/hora UTC  
Salidas: azimut (0–360°) y elevación (-90°–90°) con precisión ±0.01°

Validado contra USNO Solar Calculator con resultados dentro de ±1.5°.

### Lógica del Firmware (ESP32-C3)

```
setup(): DS3231 init (I2C) → stepperInit → servoInit → updatePosition()
loop() [cada 60s]:
  1. rtc.now()                    → DateTime UTC del DS3231
  2. solarPosition(lat, lon, dt)  → (azimut, elevacion)
  3. servoSetTilt(90° − elevacion) → inclina el poste
  4. stepperGoto(azimut)          → gira hacia el sol (camino más corto)
```

**Puesta en marcha:** ajustar hora UTC por Serial con `s:AAAA-MM-DD HH:MM:SS`  
**Calibración de norte:** apuntar brazo al Norte físico → enviar `z` por Serial

## Principio Geométrico Clave

Un palo vertical de altura H desplaza la sombra `H / tan(elevacion)` del punto base. A bajas elevaciones solares (invierno), este desplazamiento puede superar metros, dejando el objetivo sin cobertura.

**Solución:** Al inclinar el palo en `90° - elevacion` hacia el sol, la sombrilla permanece siempre directamente sobre el punto objetivo, logrando 100% de cobertura independientemente de la posición solar.

## Estado del Proyecto

| Fase | Estado |
|------|--------|
| Algoritmo SPA | ✅ Completo y validado |
| Simulación Python | ✅ Completo |
| Visualización 3D | ✅ Completo |
| Diseño CAD mecánico | 📋 Pendiente |
| Prototipo físico | ⏳ Listo para iniciar |
| Firmware Arduino | 🔲 No iniciado |

## Archivos de Firmware

```
firmware/SistemaSombra/
├── SistemaSombra.ino   ← Sketch principal (SPA + stepper + servo + RTC)
└── config.h            ← Configuración de usuario (pines, ubicación, calibración)
```

**Dependencias Arduino:** `RTClib` (Adafruit), `ESP32Servo` (Kevin Harrington)  
**Board:** `ESP32C3 Dev Module`

## Hardware del Prototipo

- ESP32-C3 Mini (~3€), 28BYJ-48 + ULN2003 (~1.5€), servo MG90S (~3€), DS3231 RTC (~1.5€), batería LiPo + boost 5V (~5–8€)
- Costo total objetivo: <15€ | Radio sombrilla (prototipo): 20 cm
- **No requiere WiFi** — la hora UTC la provee el DS3231 (precisión ±2 ppm)
