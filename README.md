# Sistema de Sombra — Sombrilla Solar Autónoma

Sistema que rastrea la posición del sol y orienta una sombrilla para proyectar sombra constante sobre un punto fijo, sin intervención humana. Un algoritmo astronómico calcula dónde está el sol en cada momento; un motor paso a paso gira la base en azimut y un servo inclina el poste, de modo que la sombrilla siempre queda justo encima del objetivo.

---

## Tabla de contenidos

1. [Principio de funcionamiento](#1-principio-de-funcionamiento)
2. [Archivos del proyecto](#2-archivos-del-proyecto)
3. [Hardware](#3-hardware)
4. [Firmware — ESP32-C3](#4-firmware--esp32-c3)
5. [Simulación Python](#5-simulación-python)
6. [Visualización 3D interactiva](#6-visualización-3d-interactiva)
7. [Puesta en marcha paso a paso](#7-puesta-en-marcha-paso-a-paso)
8. [Comandos Serial](#8-comandos-serial)
9. [Calibración](#9-calibración)
10. [Estado del proyecto y escalabilidad](#10-estado-del-proyecto-y-escalabilidad)

---

## 1. Principio de funcionamiento

### El problema del poste fijo

Una sombrilla en un poste vertical de altura H proyecta su sombra desplazada del pie del poste una distancia `H / tan(elevación solar)`. Ese desplazamiento nunca es cero:

| Elevación solar | Desplazamiento (H = 1 m) | Cobertura (R = 0.5 m) |
|:---:|:---:|:---:|
| 90° (cénit) | 0.00 m | 100 % |
| 73° (Madrid, verano mediodía) | 0.31 m | 77 % |
| 54° (Madrid, abril mediodía) | 0.73 m | 0 % |
| 26° (Madrid, invierno mediodía) | 2.05 m | 0 % |

### La solución: poste inclinable

El sistema inclina el poste entero un ángulo `(90° − elevación)` hacia el sol. La geometría garantiza que la punta del poste —donde está la sombrilla— siempre queda exactamente en el rayo solar que pasa por el objetivo `(0, 0)`:

```
Punta del brazo:  T = L · (cos(el)·sin(az),  sin(el),  −cos(el)·cos(az))
Rayo solar (↓):   D = (−cos(el)·sin(az),     −sin(el),  cos(el)·cos(az))
Proyección en suelo (t = L):  T + L·D = (0, 0, 0)  ✓
```

Resultado: **100 % de cobertura en todas las estaciones**, independientemente de la elevación solar.

### Cinemática de los dos actuadores

```
stepper_azimut = azimut_solar          (0–360°, gira la base horizontalmente)
servo_tilt     = 90° − elevación_solar (0°=vertical → 87°=casi horizontal)
```

La sombrilla en la punta del poste es perpendicular al eje del poste. Al apuntar el poste al sol, la sombrilla queda perpendicular a los rayos → máxima área de sombra proyectada.

---

## 2. Archivos del proyecto

```
Sistema de Sombra/
├── README.md                          ← Este archivo
├── informe_tecnico_sombrilla_solar.md ← Documentación técnica completa
│
├── simulacion_solar.py                ← Simulación + validación del algoritmo SPA
├── simulacion_sombrilla_3D.html       ← Visualización 3D interactiva (Three.js)
│
├── solar_trajectory.png               ← Generado por simulacion_solar.py
├── shadow_analysis.png                ← Generado por simulacion_solar.py
├── seasonal_comparison.png            ← Generado por simulacion_solar.py
│
└── firmware/
    └── SistemaSombra/
        ├── SistemaSombra.ino          ← Sketch principal (Arduino / ESP32-C3)
        └── config.h                   ← Configuración de usuario
```

---

## 3. Hardware

### Lista de componentes (~15 €)

| Componente | Función | Precio aprox. |
|:---|:---|:---:|
| ESP32-C3 Mini | Microcontrolador principal (26 × 16 mm) | ~3 € |
| DS3231 RTC | Reloj en tiempo real, precisión ±2 ppm, batería CR2032 | ~1.5 € |
| 28BYJ-48 + ULN2003 | Motor paso a paso para rotación en azimut | ~1.5 € |
| MG90S | Servo metal gear para inclinación del poste | ~3 € |
| LiPo 3.7 V 1000 mAh | Batería recargable | ~4 € |
| Boost 5 V (MT3608 o similar) | Alimenta motor y servo desde la LiPo | ~1 € |

> El DS3231 mantiene la hora con una pila CR2032 cuando el sistema está apagado (~5 años de duración de la pila). No se necesita WiFi.

### Esquema de conexiones

```
ESP32-C3 Mini
┌──────────────────────┐
│ GPIO 1  ────────────►│ ULN2003 IN1 ──► 28BYJ-48 (cable naranja)
│ GPIO 3  ────────────►│ ULN2003 IN2 ──► 28BYJ-48 (cable amarillo)
│ GPIO 4  ────────────►│ ULN2003 IN3 ──► 28BYJ-48 (cable rosa)
│ GPIO 5  ────────────►│ ULN2003 IN4 ──► 28BYJ-48 (cable azul)
│                      │ ULN2003 VCC ──► 5 V (boost)
│                      │ 28BYJ-48 rojo ► 5 V (boost)
│                      │
│ GPIO 10 ────────────►│ MG90S Signal (naranja)
│                      │ MG90S VCC ────► 5 V (boost)
│                      │
│ GPIO 6 (SDA) ───────►│ DS3231 SDA
│ GPIO 7 (SCL) ───────►│ DS3231 SCL
│ 3V3 ────────────────►│ DS3231 VCC
│                      │
│ GND ────────────────►│ GND común (ULN2003 + MG90S + DS3231 + boost)
│ USB-C ◄─────────────── Programación y carga
└──────────────────────┘
```

> **Importante:** el 28BYJ-48 y el MG90S se alimentan a 5 V desde el boost converter. El DS3231 y el ESP32-C3 funcionan a 3.3 V. Los GPIOs del ESP32-C3 son 3.3 V y el ULN2003 acepta entradas de 3.3 V directamente.

### Motor paso a paso 28BYJ-48

- Reductor interno 1:64 → **2048 pasos por vuelta** en modo medio-paso
- Resolución angular: 360° / 2048 ≈ **0.176° por paso**
- La reducción retiene la posición sin corriente → bobinas apagadas en reposo
- A 2000 µs/paso (por defecto) → ~4–5 RPM, silencioso y seguro

---

## 4. Firmware — ESP32-C3

### Dependencias

Instalar desde **Arduino Library Manager** antes de compilar:

| Librería | Autor | Uso |
|:---|:---|:---|
| `RTClib` | Adafruit | Comunicación con el DS3231 |
| `ESP32Servo` | Kevin Harrington | Control del servo MG90S |

**Board:** `ESP32C3 Dev Module`  
**Upload Speed:** `921600`

### Estructura del código (`SistemaSombra.ino`)

El sketch está organizado en bloques independientes:

```
SistemaSombra.ino
│
├── [SPA] solarPosition(lat, lon, DateTime)
│         Algoritmo Jean Meeus / NREL → az + el con ±0.01°
│         Incluye corrección de refracción atmosférica (Bennett 1982)
│
├── [RTC] rtcInit() / rtcSetFromString()
│         Init I2C del DS3231, detecta pérdida de hora,
│         parsea el comando Serial 's:' para ajustar la hora UTC
│
├── [Stepper] stepperGoto(azDeg)
│         Calcula el camino más corto (CW/CCW) al azimut destino,
│         mueve en modo medio-paso, apaga bobinas al llegar
│
├── [Servo] servoSetTilt(tiltDeg)
│         Escribe el ángulo al MG90S con pulsos 500–2500 µs
│
├── updatePosition()
│         Lee RTC → calcula SPA → mueve actuadores
│         Si el sol está bajo el horizonte → posición de aparcado
│
├── handleSerial()
│         Procesa comandos de calibración y diagnóstico
│
├── setup()
│         Init actuadores → init RTC → primera posición
│
└── loop()
          Llama updatePosition() cada 60 s
          Procesa Serial continuamente
```

### Algoritmo SPA (Solar Position Algorithm)

El corazón del sistema. Convierte fecha/hora UTC + coordenadas geográficas en azimut y elevación solares con precisión astronómica:

```
Entradas:  latitud, longitud, DateTime (UTC del DS3231)
           ↓
Día Juliano (JD) → siglos julianos T desde J2000
           ↓
Longitud media L₀ → anomalía media M → ecuación del centro C
           ↓
Longitud aparente (aberración + nutación) → oblicuidad de la eclíptica
           ↓
Ascensión recta (RA) + declinación solar
           ↓
GMST → ángulo horario local (LHA)
           ↓
Elevación geométrica → +refracción atmosférica
Azimut (0=Norte, 90=Este, sentido horario)
           ↓
Salidas:   az [0–360°]   el [−90°–90°]    Tiempo: < 1 ms
```

Validado contra el calculador solar de la USNO con precisión **±1.5°** en todos los casos de prueba.

### Lógica de control en cada ciclo (60 s)

```
rtc.now() ──► solarPosition() ──► elevación < 1° ?
                                        │ Sí → aparcado (vertical, norte)
                                        │ No → Día:
                                        │   tilt = 90° − elevación
                                        │   si tilt < tilt_actual: servo primero, luego stepper
                                        │   si tilt ≥ tilt_actual: stepper primero, luego servo
                                        │   (evita que el poste inclinado rafe el suelo al girar)
```

### config.h — parámetros de usuario

Editar antes de compilar:

```c
// Ubicación geográfica
#define LAT_DEG     40.4168f    // + = Norte
#define LON_DEG     -3.7038f   // + = Este

// Pines (ver esquema de conexiones)
#define STEP_IN1    1
#define STEP_IN2    3
#define STEP_IN3    4
#define STEP_IN4    5
#define SERVO_PIN   10
#define RTC_SDA     6
#define RTC_SCL     7

// Calibración
#define AZIMUTH_OFFSET_DEG   0.0f   // ángulo físico al arrancar
#define SERVO_OFFSET_DEG     0      // corrección de cero del servo

// Comportamiento
#define UPDATE_INTERVAL_S    60     // actualización cada 60 s
#define SERVO_MAX_DEG        87     // límite mecánico de inclinación
#define STEP_DELAY_US        2000   // velocidad del stepper (µs/paso)
```

---

## 5. Simulación Python

`simulacion_solar.py` valida el algoritmo SPA y genera tres gráficas de análisis.

### Ejecución

```bash
pip install numpy matplotlib
python simulacion_solar.py
```

### Gráficas generadas

| Archivo | Contenido |
|:---|:---|
| `solar_trajectory.png` | Elevación, azimut, ángulos de servo y diagrama polar a lo largo del día |
| `shadow_analysis.png` | Cobertura de sombra vs hora, forma de la sombra al mediodía y a las 15h |
| `seasonal_comparison.png` | Curvas de elevación solar en solsticios y equinoccios |

### Módulos del script

```python
julian_day()          # Número de Día Juliano
solar_position()      # Algoritmo SPA completo (mismo que el firmware)
actuator_angles()     # az → stepper_az,  el → servo_tilt = 90° − el
compute_shadow_outline()  # contorno de sombra en el suelo
shadow_coverage_ratio()   # fracción del objetivo cubierto
simulate_day()            # simulación horaria de un día completo
```

---

## 6. Visualización 3D interactiva

`simulacion_sombrilla_3D.html` es una simulación en tiempo real del mecanismo. Se abre directamente en el navegador, sin servidor.

### Controles

| Control | Función |
|:---|:---|
| Fecha | Cambia el día simulado |
| Ubicación | Selecciona ciudad (Madrid, Londres, Tokio, Quito...) |
| Slider de hora | Posición temporal del día (0–24 h) |
| ▶ Play / ⏸ Pausa | Animación automática del día |
| Velocidad | Multiplicador 1×–20× |
| Ratón arrastrar | Orbitar la cámara |
| Scroll | Zoom |

### Qué muestra la escena 3D

- **Poste y sombrilla** inclinándose en tiempo real hacia el sol
- **Sombra elíptica** en el suelo: semieje menor = R, semieje mayor = R/sin(elevación). A sol bajo la sombra se alarga; a sol alto es casi circular
- **Rayo solar** (línea naranja) pasando por la punta de la sombrilla hasta el punto objetivo
- **Zona objetivo** (círculo rojo) en el origen — la sombra siempre lo cubre
- **Panel izquierdo**: azimut, elevación, ángulos de actuadores, cobertura en %

---

## 7. Puesta en marcha paso a paso

### Primera vez (hardware nuevo)

**1. Instalar librerías en Arduino IDE**
```
Library Manager → buscar "RTClib" (Adafruit) → instalar
Library Manager → buscar "ESP32Servo" (Kevin Harrington) → instalar
```

**2. Editar `config.h`**
- Cambiar `LAT_DEG` / `LON_DEG` a tu ubicación real
- Ajustar pines si los has cableado diferente
- Dejar `AZIMUTH_OFFSET_DEG = 0.0` de momento

**3. Compilar y subir**
```
Board: ESP32C3 Dev Module
Upload Speed: 921600
```

**4. Ajustar la hora UTC**

Abrir Serial Monitor a **115200 baud** y enviar:
```
s:2026-04-15 14:30:00
```
Usa la hora UTC actual (no la hora local). El DS3231 la retiene aunque desconectes la alimentación.

**5. Calibrar el norte** → ver sección [Calibración](#9-calibración)

**6. Verificar funcionamiento**
```
i          ← muestra estado completo
r          ← fuerza recálculo inmediato
```

---

## 8. Comandos Serial

Velocidad: **115200 baud**. Enviar texto seguido de Enter.

| Comando | Descripción | Ejemplo |
|:---|:---|:---|
| `i` | Estado completo: hora RTC, sol, stepper, servo, temperatura | `i` |
| `r` | Recalcular posición solar y mover actuadores ahora | `r` |
| `s:AAAA-MM-DD HH:MM:SS` | Ajustar RTC (hora UTC) | `s:2026-06-21 10:00:00` |
| `a:<grados>` | Mover stepper a azimut manualmente | `a:180` |
| `t:<grados>` | Mover servo a inclinación manualmente | `t:45` |
| `z` | Marcar posición actual del stepper como Norte (0°) | `z` |

### Ejemplo de salida del comando `i`

```
  RTC:      2026-04-15 14:30:00 UTC
  Temp RTC: 24.5 °C
  Sol:      Az=215.34°  El=48.72°
  Stepper:  pos=1220 pasos (209.2°)
  Servo:    tilt=41°
  Próxima actualización en ~38s
```

---

## 9. Calibración

### Calibrar el norte (azimut cero)

El 28BYJ-48 no tiene sensor de posición. Al arrancar asume que está en el ángulo definido por `AZIMUTH_OFFSET_DEG` en `config.h`.

**Procedimiento:**
1. Con el sistema encendido y el poste en posición vertical, gira físicamente la base hasta que el brazo apunte exactamente al **Norte geográfico** (no al norte magnético).
2. Envía `z` por Serial → el sistema registra esa posición como 0°.
3. Para que el valor persista tras reinicios, anota el ángulo que muestra `i` antes de hacer `z` y escríbelo en `config.h`:
   ```c
   #define AZIMUTH_OFFSET_DEG   <valor>
   ```

### Calibrar el servo (inclinación cero)

Si al enviar `t:0` el poste no queda perfectamente vertical:
1. Mide visualmente o con un nivel cuántos grados se desvía
2. Ajusta en `config.h`:
   ```c
   #define SERVO_OFFSET_DEG   <corrección en grados>
   ```

### Velocidad del stepper

Si el motor pierde pasos (ruido seco, saltos) con carga pesada, aumentar el retardo:
```c
#define STEP_DELAY_US   3000   // más lento pero más fiable
```

---

## 10. Estado del proyecto y escalabilidad

### Estado actual

| Fase | Estado |
|:---|:---:|
| Algoritmo SPA | ✅ Completo y validado (±1.5° vs USNO) |
| Simulación Python | ✅ Completo |
| Visualización 3D | ✅ Completo |
| Firmware ESP32-C3 | ✅ Completo |
| Diseño CAD mecánico | 📋 Pendiente |
| Prototipo físico | ⏳ Listo para construir |

### Escalar a tamaño real

El algoritmo SPA y la lógica de control son idénticos en cualquier escala. Solo cambian los actuadores:

| Parámetro | Maqueta (actual) | Versión real |
|:---|:---:|:---:|
| Radio sombrilla | 20 cm | 150–200 cm |
| Altura poste | 30 cm | 200–250 cm |
| Actuador azimut | 28BYJ-48 (9 g) | Motor DC + encoder / corona dentada |
| Actuador inclinación | MG90S (13 g) | Actuador lineal / servo 25 kg·cm |
| Alimentación | LiPo 1000 mAh | Panel solar + batería 12 V 7 Ah |
| Estructura | PLA / varillas | Aluminio / acero inoxidable |
| Protección IP | — | IP65 (exterior) |

---

## Referencias

- Jean Meeus, *Astronomical Algorithms*, 2nd ed. (1998)
- NREL Solar Position Algorithm (SPA) — Ibrahim Reda & Afshin Andreas
- Validación con [USNO Solar Calculator](https://aa.usno.navy.mil/data/RS_OneYear)
- Three.js r128 — visualización 3D
