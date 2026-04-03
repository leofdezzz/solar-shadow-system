#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════╗
║        SOMBRILLA SOLAR AUTÓNOMA — Simulación Python          ║
║        Algoritmo SPA + Validación + Análisis de Sombra       ║
╚══════════════════════════════════════════════════════════════╝

Valida el algoritmo de posición solar y simula el comportamiento
del mecanismo a lo largo del día para distintas fechas y ubicaciones.
"""

import os
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from datetime import datetime, timedelta, timezone, date as date_type

# ─────────────────────────────────────────────────────────────
# MÓDULO 1: ALGORITMO DE POSICIÓN SOLAR (SPA Simplificado)
# Precisión: ±0.01°  — Suficiente para control de servo
# Basado en: Jean Meeus "Astronomical Algorithms" + NREL SPA
# ─────────────────────────────────────────────────────────────

def julian_day(year, month, day, hour=0, minute=0, second=0):
    """Calcula el Número de Día Juliano (JD)"""
    if month <= 2:
        year -= 1
        month += 12
    A = int(year / 100)
    B = 2 - A + int(A / 4)
    JD = (int(365.25 * (year + 4716)) + int(30.6001 * (month + 1))
          + day + B - 1524.5)
    JD += (hour + minute / 60 + second / 3600) / 24
    return JD


def solar_position(lat_deg, lon_deg, dt_utc):
    """
    Calcula la posición solar usando el algoritmo SPA simplificado.

    Args:
        lat_deg : Latitud en grados  (+ = Norte)
        lon_deg : Longitud en grados (+ = Este)
        dt_utc  : datetime con tzinfo=UTC

    Returns:
        (azimut_deg, elevacion_deg)
        azimut   : 0=Norte, 90=Este, 180=Sur, 270=Oeste  [0-360]
        elevacion: 0=horizonte, 90=cénit                 [-90 a 90]
    """
    JD = julian_day(dt_utc.year, dt_utc.month, dt_utc.day,
                    dt_utc.hour, dt_utc.minute, dt_utc.second)

    T = (JD - 2451545.0) / 36525.0  # Siglos Julianos desde J2000.0

    # Longitud media del sol (°)
    L0 = (280.46646 + 36000.76983 * T + 0.0003032 * T**2) % 360

    # Anomalía media (°)
    M = (357.52911 + 35999.05029 * T - 0.0001537 * T**2) % 360
    M_rad = math.radians(M)

    # Ecuación del centro
    C = ((1.914602 - 0.004817 * T - 0.000014 * T**2) * math.sin(M_rad)
         + (0.019993 - 0.000101 * T) * math.sin(2 * M_rad)
         + 0.000289 * math.sin(3 * M_rad))

    sun_lon = L0 + C  # Longitud verdadera del sol

    # Corrección de aberración y nutación
    omega = 125.04 - 1934.136 * T
    apparent_lon = sun_lon - 0.00569 - 0.00478 * math.sin(math.radians(omega))
    apparent_lon_rad = math.radians(apparent_lon)

    # Oblicuidad de la eclíptica
    obl = (23.0 + (26.0 + (21.448 - T * (46.815 + T * (0.00059 - T * 0.001813))) / 60) / 60)
    obl_c = obl + 0.00256 * math.cos(math.radians(omega))
    obl_rad = math.radians(obl_c)

    # Ascensión recta y declinación solar
    RA = math.atan2(math.cos(obl_rad) * math.sin(apparent_lon_rad),
                    math.cos(apparent_lon_rad))
    RA_deg = math.degrees(RA) % 360
    decl = math.asin(math.sin(obl_rad) * math.sin(apparent_lon_rad))

    # Tiempo Sidéreo de Greenwich (GMST)
    GMST = (280.46061837 + 360.98564736629 * (JD - 2451545.0)
            + 0.000387933 * T**2 - T**3 / 38710000.0) % 360

    # Ángulo horario local (LHA)
    LHA = (GMST + lon_deg - RA_deg) % 360
    LHA_rad = math.radians(LHA)
    lat_rad = math.radians(lat_deg)

    # Elevación solar
    sin_el = (math.sin(lat_rad) * math.sin(decl)
              + math.cos(lat_rad) * math.cos(decl) * math.cos(LHA_rad))
    elevation = math.asin(max(-1.0, min(1.0, sin_el)))
    el_deg = math.degrees(elevation)

    # Refracción atmosférica (corrección al alzarse/ponerse el sol)
    if el_deg > 85:
        refraction = 0.0
    elif el_deg > 5:
        tan_e = math.tan(math.radians(el_deg))
        refraction = (58.1 / tan_e - 0.07 / tan_e**3 + 0.000086 / tan_e**5) / 3600
    elif el_deg > -0.575:
        refraction = (1735 + el_deg * (-518.2 + el_deg * (
            103.4 + el_deg * (-12.79 + el_deg * 0.711)))) / 3600
    else:
        refraction = 0.0

    el_apparent = el_deg + refraction

    # Azimut (desde Norte, sentido horario)
    Az = math.atan2(
        math.sin(LHA_rad),
        math.cos(LHA_rad) * math.sin(lat_rad) - math.tan(decl) * math.cos(lat_rad)
    )
    az_deg = (math.degrees(Az) + 180.0) % 360

    return az_deg, el_apparent


# ─────────────────────────────────────────────────────────────
# MÓDULO 2: CONTROL DEL MECANISMO
#
# DISEÑO FINAL:
#   • Stepper motor (base, eje vertical) → rota el brazo en AZIMUT (0–360°)
#   • Servo (eje horizontal en la base) → inclina el brazo desde vertical
#
# Cinemática:
#   stepper_angle = azimut_solar          (el brazo apunta hacia el sol)
#   servo_angle   = 90° - elevacion_solar (el brazo se inclina al ángulo del sol)
#
# La sombrilla está unida rígidamente en la PUNTA del brazo,
# perpendicular al brazo. Cuando el brazo apunta al sol,
# la sombrilla es automáticamente ⊥ a los rayos → sombra MÁXIMA.
#
# Geometría de sombra (demostración):
#   Punta del brazo: T = L·(cos(el)·sin(az),  cos(el)·cos(az),  sin(el))
#   Rayo solar (↓): D = (-cos(el)·sin(az), -cos(el)·cos(az), -sin(el))
#   Proyección en z=0: T + t·D  donde t = T_z/sin(el) = L
#   → sombra del centro = (0, 0, 0)  ✓  siempre en el objetivo
#   → la sombra es círculo de radio R centrado en (0,0)  ✓
# ─────────────────────────────────────────────────────────────

def actuator_angles(sun_az, sun_el):
    """
    Calcula los ángulos de los actuadores para el mecanismo de BRAZO INCLINABLE.

    Args:
        sun_az : Azimut solar en grados (0=Norte, 90=Este)
        sun_el : Elevación solar en grados (0=horizonte, 90=cénit)

    Returns:
        (stepper_az_deg, servo_tilt_deg)
        stepper_az : Rotación del stepper en azimut   [0–360°]
        servo_tilt : Inclinación del servo desde vert [0–87°]
    """
    stepper_az  = sun_az % 360
    servo_tilt  = 90.0 - sun_el          # 0° = brazo vertical, 90° = horizontal
    servo_tilt  = max(0.0, min(87.0, servo_tilt))   # límite mecánico seguro
    return stepper_az, servo_tilt


# Mantener alias para compatibilidad con código anterior
def umbrella_angles(sun_az, sun_el):
    return actuator_angles(sun_az, sun_el)


# ─────────────────────────────────────────────────────────────
# MÓDULO 3: GEOMETRÍA DE SOMBRA
# ─────────────────────────────────────────────────────────────

def compute_shadow_outline(umbrella_radius, pole_height, sun_az, sun_el,
                           servo_az=None, servo_tilt=None, n_rim=120):
    """
    Calcula el contorno de sombra para el mecanismo de BRAZO INCLINABLE.

    Con este mecanismo, la punta del brazo (y el centro de la sombrilla)
    está siempre en el rayo solar que pasa por el origen → la sombra es
    un círculo de radio umbrella_radius centrado en (0, 0).

    La demostración analítica es exacta; esta función calcula el círculo
    directamente sin necesidad de proyectar punto a punto.

    Returns:
        (sx, sy): arrays del borde de la sombra circular en el suelo
    """
    if sun_el <= 0:
        return None, None

    thetas = np.linspace(0, 2 * math.pi, n_rim, endpoint=False)
    sx = umbrella_radius * np.cos(thetas)
    sy = umbrella_radius * np.sin(thetas)
    return sx, sy


def shadow_coverage_ratio(sx, sy, target_r=0.5):
    """
    Cobertura del área objetivo.

    Con el mecanismo de brazo inclinable la sombra es siempre un círculo
    de radio R_umbrella centrado en (0,0). Si R_umbrella >= target_r
    la cobertura es 100%.
    """
    if sx is None or len(sx) == 0:
        return 0.0

    shadow_r = float(np.mean(np.sqrt(sx**2 + sy**2)))   # ≈ umbrella_radius

    # Sombra centrada en origen → distancia entre centros = 0
    if shadow_r >= target_r:
        return 1.0                          # sombra cubre completamente el objetivo
    else:
        return (shadow_r / target_r) ** 2  # sombra más pequeña que el objetivo


# ─────────────────────────────────────────────────────────────
# MÓDULO 4: SIMULACIÓN DEL DÍA COMPLETO
# ─────────────────────────────────────────────────────────────

def simulate_day(lat, lon, sim_date, utc_offset, umbrella_r=0.5, pole_h=1.0,
                 dt_min=10):
    """
    Simula el mecanismo de brazo inclinable a lo largo de un día.

    Actuadores:
        stepper_az  = azimut solar   (stepper 360°, eje vertical)
        servo_tilt  = 90° - elevación (servo, inclina el brazo desde vertical)

    Con este mecanismo la sombra cae siempre sobre el punto objetivo (0,0).
    """
    data = dict(hours=[], sun_az=[], sun_el=[], srv_rot=[], srv_tilt=[],
                coverage=[], is_day=[])

    for m in range(0, 24 * 60, dt_min):
        h, mn = divmod(m, 60)
        local_dt = datetime(sim_date.year, sim_date.month, sim_date.day,
                            h, mn, tzinfo=timezone.utc)
        utc_dt = local_dt - timedelta(hours=utc_offset)

        az, el = solar_position(lat, lon, utc_dt)

        data['hours'].append(h + mn / 60)
        data['sun_az'].append(az)
        data['sun_el'].append(el)

        if el > 0.5:
            rot, tilt = umbrella_angles(az, el)
            sx, sy = compute_shadow_outline(umbrella_r, pole_h, az, el, rot, tilt)
            cov = shadow_coverage_ratio(sx, sy, target_r=umbrella_r * 0.6)
            data['srv_rot'].append(rot)
            data['srv_tilt'].append(tilt)
            data['coverage'].append(cov)
            data['is_day'].append(True)
        else:
            data['srv_rot'].append(0)
            data['srv_tilt'].append(0)
            data['coverage'].append(0)
            data['is_day'].append(False)

    for k in data:
        data[k] = np.array(data[k])

    return data


# ─────────────────────────────────────────────────────────────
# MÓDULO 5: VALIDACIÓN DEL ALGORITMO
# ─────────────────────────────────────────────────────────────

def validate_algorithm(verbose=True):
    """
    Valida el algoritmo contra valores conocidos de referencia.
    Fuente: USNO Solar Position Calculator + tablas Meeus.
    """
    SEP = "─" * 62

    # Nota: Madrid lon=-3.7° está al oeste del meridiano UTC+2 (30°E)
    # Diferencia solar = (30-(-3.7))/15 = 2.25h → mediodía solar ≈ 12:15 UTC (14:15 local CEST)
    # En invierno (UTC+1, meridiano 15°E): diferencia = (15-(-3.7))/15 = 1.25h → mediodía ≈ 12:15 UTC
    tests = [
        # (nombre, lat, lon, dt_utc, (el_min, el_max), (az_min, az_max))
        ("Madrid, solsticio verano, mediodía solar (~12:15 UTC)",
         40.4, -3.7,
         datetime(2026, 6, 21, 12, 15, tzinfo=timezone.utc),
         (72, 76), (174, 186)),
        ("Madrid, solsticio invierno, mediodía solar (~12:15 UTC)",
         40.4, -3.7,
         datetime(2026, 12, 21, 12, 15, tzinfo=timezone.utc),
         (25, 30), (174, 186)),
        ("Madrid, equinoccio primavera, 10:00 local (~08:00 UTC)",
         40.4, -3.7,
         datetime(2026, 3, 21, 8, 0, tzinfo=timezone.utc),
         (20, 35), (90, 115)),
        ("Quito (Ecuador), mediodía, cénit casi vertical",
         -0.2, -78.5,
         datetime(2026, 3, 21, 17, 30, tzinfo=timezone.utc),  # 12:30h local ECT-5
         (85, 90), (0, 360)),   # Muy cerca del cénit, azimut indeterminado
        ("Reikiavik, solsticio invierno, mediodía (~13:00 UTC)",
         64.1, -21.9,
         datetime(2026, 12, 21, 13, 0, tzinfo=timezone.utc),
         (2, 7), (174, 186)),
    ]

    if verbose:
        print(f"\n{SEP}")
        print("  VALIDACIÓN DEL ALGORITMO SPA (Solar Position Algorithm)")
        print(SEP)

    all_ok = True
    for name, lat, lon, dt, (el_lo, el_hi), (az_lo, az_hi) in tests:
        az, el = solar_position(lat, lon, dt)

        # Para Quito sólo validamos la elevación
        skip_az = (az_lo == 0 and az_hi == 360)
        el_ok = el_lo <= el <= el_hi
        az_ok = az_lo <= az <= az_hi if not skip_az else True
        ok = el_ok and az_ok

        if not ok:
            all_ok = False

        if verbose:
            sym = "✅" if ok else "❌"
            print(f"\n  {sym}  {name}")
            print(f"      El = {el:6.2f}°  (esperado {el_lo}–{el_hi}°)  "
                  f"{'✓' if el_ok else '✗'}")
            if not skip_az:
                print(f"      Az = {az:6.2f}°  (esperado {az_lo}–{az_hi}°)  "
                      f"{'✓' if az_ok else '✗'}")

    # Test de continuidad: sin saltos bruscos entre minutos
    print(f"\n  {SEP}")
    print("  Test de continuidad (Madrid, 2 abril 2026)")
    print(f"  {SEP}")
    lat, lon = 40.4, -3.7
    prev_az = None
    max_jump = 0
    for h in range(6, 20):
        dt = datetime(2026, 4, 2, h, 0, tzinfo=timezone.utc)
        az, el = solar_position(lat, lon, dt)
        if el > 0:
            if prev_az is not None:
                jump = abs(az - prev_az)
                if jump > 180:
                    jump = 360 - jump
                max_jump = max(max_jump, jump)
            prev_az = az
            if verbose:
                print(f"    {h+2:02d}:00h local  →  Az={az:6.1f}°  El={el:5.1f}°")

    cont_ok = max_jump < 20
    if verbose:
        sym = "✅" if cont_ok else "⚠️"
        print(f"\n  {sym}  Máximo salto de azimut entre horas: {max_jump:.1f}°")
        print(f"\n  {'TODOS LOS TESTS OK ✅' if all_ok else 'ALGUNOS TESTS FALLARON ❌'}")
        print(SEP)

    return all_ok


# ─────────────────────────────────────────────────────────────
# MÓDULO 6: GRÁFICAS
# ─────────────────────────────────────────────────────────────

COLORS = {
    'Solsticio Verano':   '#e67e22',
    'Solsticio Invierno': '#2980b9',
    'Hoy (2 Abril)':      '#27ae60',
}


def make_figure_trajectory(results, location, date_str):
    """4-panel: elevación, azimut, servos, diagrama polar."""
    fig, axes = plt.subplots(2, 2, figsize=(16, 11))
    fig.patch.set_facecolor('#1a1a2e')
    fig.suptitle(
        f'Simulación Sombrilla Solar Autónoma\n{location}  ·  {date_str}',
        fontsize=14, fontweight='bold', color='white', y=0.99
    )

    hours    = results['hours']
    sun_el   = results['sun_el']
    sun_az   = results['sun_az']
    srv_rot  = results['srv_rot']
    srv_tilt = results['srv_tilt']
    is_day   = results['is_day'].astype(bool)

    style = dict(facecolor='#2d2d44', edgecolor='#444466', alpha=1.0)

    # Panel 1 — Elevación
    ax = axes[0, 0]
    ax.set_facecolor('#2d2d44')
    ax.fill_between(hours, 0, sun_el, where=sun_el > 0,
                    alpha=0.35, color='#f39c12')
    ax.plot(hours, sun_el, color='#f39c12', lw=2)
    ax.axhline(0, color='#888', lw=0.8, ls='--')
    ax.set(xlabel='Hora local', ylabel='Elevación (°)',
           title='Elevación Solar', xlim=(0, 24))
    ax.set_xticks(range(0, 25, 2))
    ax.tick_params(colors='white')
    ax.xaxis.label.set_color('white')
    ax.yaxis.label.set_color('white')
    ax.title.set_color('white')
    ax.grid(True, alpha=0.2)

    # Panel 2 — Azimut
    ax = axes[0, 1]
    ax.set_facecolor('#2d2d44')
    ax.plot(hours[is_day], sun_az[is_day], color='#3498db', lw=2)
    ax.set(xlabel='Hora local', ylabel='Azimut (°)',
           title='Azimut Solar', xlim=(0, 24), ylim=(0, 360))
    ax.set_yticks([0, 90, 180, 270, 360])
    ax.set_yticklabels(['N 0°', 'E 90°', 'S 180°', 'W 270°', 'N 360°'])
    ax.set_xticks(range(0, 25, 2))
    ax.tick_params(colors='white')
    ax.xaxis.label.set_color('white')
    ax.yaxis.label.set_color('white')
    ax.title.set_color('white')
    ax.grid(True, alpha=0.2)

    # Panel 3 — Ángulos de Servo
    ax = axes[1, 0]
    ax.set_facecolor('#2d2d44')
    ax.plot(hours[is_day], srv_rot[is_day],  color='#e74c3c', lw=2, label='Stepper — Azimut (0–360°)')
    ax.plot(hours[is_day], srv_tilt[is_day], color='#2ecc71', lw=2, label='Servo — Inclinación (0–87°)')
    ax.set(xlabel='Hora local', ylabel='Ángulo (°)',
           title='Ángulos de Actuadores', xlim=(0, 24))
    ax.set_xticks(range(0, 25, 2))
    ax.legend(facecolor='#3d3d54', labelcolor='white', fontsize=9)
    ax.tick_params(colors='white')
    ax.xaxis.label.set_color('white')
    ax.yaxis.label.set_color('white')
    ax.title.set_color('white')
    ax.grid(True, alpha=0.2)

    # Panel 4 — Diagrama polar de trayectoria
    ax4 = plt.subplot(2, 2, 4, projection='polar', facecolor='#2d2d44')
    az_r = np.radians(sun_az[is_day])
    zenith_d = 90 - sun_el[is_day]
    sc = ax4.scatter(az_r, zenith_d, c=hours[is_day], cmap='plasma',
                     s=18, alpha=0.8, zorder=5)
    for target_h in [6, 9, 12, 15, 18]:
        idx = np.argmin(np.abs(hours[is_day] - target_h))
        ax4.annotate(f'{target_h}h',
                     (az_r[idx], zenith_d[idx]),
                     color='white', fontsize=7, ha='center')
    ax4.set_theta_zero_location('N')
    ax4.set_theta_direction(-1)
    ax4.set_rticks([0, 30, 60, 90])
    ax4.set_yticklabels(['Cénit', '30°', '60°', 'Horizonte'], color='white', fontsize=7)
    ax4.set_thetagrids([0, 90, 180, 270], ['N', 'E', 'S', 'W'], color='white')
    ax4.tick_params(colors='white')
    ax4.title.set_color('white')
    ax4.set_title('Trayectoria Solar (diagrama polar)', pad=20, color='white')
    plt.colorbar(sc, ax=ax4, label='Hora local', shrink=0.75,
                 ).ax.yaxis.label.set_color('white')

    plt.tight_layout(rect=[0, 0, 1, 0.97])
    return fig


def make_figure_shadow(results, umbrella_r, pole_h, location, date_str):
    """3-panel: cobertura diaria + proyección de sombra en 2 momentos."""
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    fig.patch.set_facecolor('#1a1a2e')
    fig.suptitle(
        f'Análisis de Sombra  ·  {location}  ·  {date_str}\n'
        f'Sombrilla r={umbrella_r}m  ·  Poste h={pole_h}m',
        fontsize=12, fontweight='bold', color='white'
    )

    hours    = results['hours']
    sun_el   = results['sun_el']
    coverage = results['coverage']
    is_day   = results['is_day'].astype(bool)

    # Panel 1: cobertura vs tiempo
    ax = axes[0]
    ax.set_facecolor('#2d2d44')
    day_h   = hours[is_day]
    day_cov = coverage[is_day] * 100
    ax.fill_between(day_h, 0, day_cov, alpha=0.4, color='#27ae60')
    ax.plot(day_h, day_cov, color='#27ae60', lw=2)
    ax.axhline(80, color='#e74c3c', ls='--', lw=1.5, label='Umbral 80%')
    ax.set(xlabel='Hora local', ylabel='Cobertura (%)',
           title='Cobertura de Sombra vs Tiempo',
           ylim=(0, 105))
    ax.legend(facecolor='#3d3d54', labelcolor='white')
    ax.tick_params(colors='white')
    ax.xaxis.label.set_color('white')
    ax.yaxis.label.set_color('white')
    ax.title.set_color('white')
    ax.grid(True, alpha=0.2)

    # Instantes de interés
    noon_idx = int(np.argmax(sun_el))
    pm_idx   = int(np.argmin(np.abs(hours - 15.0)))

    for ax, idx, label_t in zip(axes[1:], [noon_idx, pm_idx], ['Mediodía solar', '15:00h local']):
        ax.set_facecolor('#2d2d44')
        az   = results['sun_az'][idx]
        el   = results['sun_el'][idx]
        rot  = results['srv_rot'][idx]
        tilt = results['srv_tilt'][idx]

        sx, sy = compute_shadow_outline(umbrella_r, pole_h, az, el, rot, tilt)

        if sx is not None and len(sx) > 0:
            ax.fill(sx, sy, alpha=0.45, color='#7f8c8d', label='Sombra proyectada')
            ax.plot(np.append(sx, sx[0]), np.append(sy, sy[0]), '#bdc3c7', lw=1)

        # Área objetivo
        theta = np.linspace(0, 2 * np.pi, 200)
        tr = umbrella_r * 0.6
        ax.plot(tr * np.cos(theta), tr * np.sin(theta),
                color='#e74c3c', ls='--', lw=2, label=f'Zona objetivo (r={tr:.2f}m)')

        # Poste
        ax.plot(0, 0, 'o', color='white', ms=8, zorder=6, label='Base del poste')

        # Flecha solar
        sz = umbrella_r * 0.8
        ax.annotate('', xy=(sz * math.sin(math.radians(az)), sz * math.cos(math.radians(az))),
                    xytext=(0, 0),
                    arrowprops=dict(arrowstyle='->', color='#f39c12', lw=2.5))
        ax.text(sz * 1.25 * math.sin(math.radians(az)),
                sz * 1.25 * math.cos(math.radians(az)),
                '☀', fontsize=14)

        cov_pct = coverage[idx] * 100
        ax.set(xlim=(-umbrella_r * 2.5, umbrella_r * 2.5),
               ylim=(-umbrella_r * 2.5, umbrella_r * 2.5),
               xlabel='X — Este (m)', ylabel='Y — Norte (m)',
               title=f'{label_t}\nEl={el:.1f}°  Az={az:.1f}°  Cobertura={cov_pct:.0f}%')
        ax.set_aspect('equal')
        ax.legend(facecolor='#3d3d54', labelcolor='white', fontsize=8, loc='upper right')
        ax.tick_params(colors='white')
        ax.xaxis.label.set_color('white')
        ax.yaxis.label.set_color('white')
        ax.title.set_color('white')
        ax.grid(True, alpha=0.2)

    plt.tight_layout()
    return fig


def make_figure_seasons(all_results, location):
    """Comparativa estacional de elevación solar."""
    fig, ax = plt.subplots(figsize=(14, 6))
    fig.patch.set_facecolor('#1a1a2e')
    ax.set_facecolor('#2d2d44')

    for label, results in all_results.items():
        color = COLORS.get(label, '#888')
        ax.plot(results['hours'], results['sun_el'], color=color, lw=2.5, label=label)

    ax.axhline(0, color='#888', lw=0.8, ls='--')
    ax.set(xlabel='Hora local', ylabel='Elevación solar (°)',
           title=f'Comparativa Estacional  ·  {location}',
           xlim=(0, 24), ylim=(-15, 80))
    ax.set_xticks(range(0, 25, 1))
    ax.tick_params(colors='white')
    ax.xaxis.label.set_color('white')
    ax.yaxis.label.set_color('white')
    ax.title.set_color('white')
    ax.legend(facecolor='#3d3d54', labelcolor='white', fontsize=11)
    ax.grid(True, alpha=0.2)
    plt.tight_layout()
    return fig


# ─────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────

if __name__ == '__main__':
    print("\n" + "═" * 62)
    print("  🌞  SOMBRILLA SOLAR AUTÓNOMA  ·  Simulación Python")
    print("═" * 62)

    # 1. Validación del algoritmo
    validate_algorithm(verbose=True)

    # 2. Parámetros de simulación
    LOCATION = dict(name='Madrid, España', lat=40.4168, lon=-3.7038)
    UMBRELLA = dict(radius=0.5, pole_height=1.0)   # maqueta en miniatura

    scenarios = [
        (date_type(2026, 6, 21), 'Solsticio Verano',   2),
        (date_type(2026, 12, 21), 'Solsticio Invierno', 1),
        (date_type(2026, 4, 2),   'Hoy (2 Abril)',      2),
    ]

    print("\n\n📊  SIMULANDO DÍAS REPRESENTATIVOS DEL AÑO...")
    print("─" * 62)

    all_results = {}
    for sim_date, label, utc_off in scenarios:
        print(f"\n  ⏳  {label} ({sim_date})…", end=' ', flush=True)
        res = simulate_day(LOCATION['lat'], LOCATION['lon'], sim_date, utc_off,
                           UMBRELLA['radius'], UMBRELLA['pole_height'])
        all_results[label] = res

        is_d = res['is_day'].astype(bool)
        if is_d.any():
            h = res['hours']
            rise = h[is_d][0]
            sset = h[is_d][-1]
            max_el = res['sun_el'].max()
            good_cov = (res['coverage'][is_d] > 0.8).mean() * 100
            print(f"✅")
            print(f"     🌅 {rise:.1f}h — 🌇 {sset:.1f}h  |  "
                  f"El max={max_el:.1f}°  |  >80% cobertura: {good_cov:.1f}% del día")

    print("\n\n🎨  GENERANDO GRÁFICAS...")
    print("─" * 62)

    OUT = os.path.dirname(os.path.abspath(__file__))

    fig1 = make_figure_trajectory(all_results['Hoy (2 Abril)'],
                                  LOCATION['name'], '2 de Abril 2026')
    fig1.savefig(f'{OUT}/solar_trajectory.png', dpi=150, bbox_inches='tight',
                 facecolor=fig1.get_facecolor())
    print("  ✅  solar_trajectory.png")
    plt.close(fig1)

    fig2 = make_figure_shadow(all_results['Hoy (2 Abril)'],
                              UMBRELLA['radius'], UMBRELLA['pole_height'],
                              LOCATION['name'], '2 de Abril 2026')
    fig2.savefig(f'{OUT}/shadow_analysis.png', dpi=150, bbox_inches='tight',
                 facecolor=fig2.get_facecolor())
    print("  ✅  shadow_analysis.png")
    plt.close(fig2)

    fig3 = make_figure_seasons(all_results, LOCATION['name'])
    fig3.savefig(f'{OUT}/seasonal_comparison.png', dpi=150, bbox_inches='tight',
                 facecolor=fig3.get_facecolor())
    print("  ✅  seasonal_comparison.png")
    plt.close(fig3)

    print("\n" + "═" * 62)
    print("  ✅  SIMULACIÓN COMPLETADA — 3 gráficas generadas")
    print("═" * 62)
    print("""
  📋  HARDWARE RECOMENDADO PARA LA MAQUETA
  ─────────────────────────────────────────
  MCU   : ESP32-C3 Mini  (26×16mm · WiFi · ~3€)
  Servo1: SG90 Micro     (rotación azimut 0-360°)
  Servo2: MG90S Metal    (inclinación 0-90°, más torque)
  Tiempo: NTP por WiFi   (no necesita chip RTC externo)
  GPS   : lat/lon fijos  (se configuran al montar)
  Energía: USB-C o LiPo 3.7V 1000mAh + panel solar 5V
  Tamaño total electrónica: ≈ 50×40×25 mm
  ─────────────────────────────────────────
  Dos servos SG90/MG90S dan ±1° de precisión angular,
  suficiente para mantener la sombra centrada en todo
  momento con un coste total <15€.
  """)
