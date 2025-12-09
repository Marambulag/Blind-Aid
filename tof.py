import RPi.GPIO as GPIO
import time
import sys
import numpy as np
import threading
import vl53l5cx_ctypes as vl53

# Intentar usar pyserial (para UART con la ESP32)
try:
    import serial
except ImportError:
    serial = None

# ==========================================
# 1. CONFIGURACIÓN DE PINES
# ==========================================
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# PINES ULTRASÓNICOS (HC-SR04)
US_PINS = [
    {"trig": 22, "echo": 23},  # Sensor 1
    {"trig": 24, "echo": 25},  # Sensor 2
    {"trig": 6,  "echo": 5}    # Sensor 3
]

# XSHUT ÚNICO DEL ToF
LP_XSHUT = 27

LECTURAS_US = [0.0, 0.0, 0.0]

GPIO.setup(LP_XSHUT, GPIO.OUT, initial=GPIO.HIGH)
for s in US_PINS:
    GPIO.setup(s["trig"], GPIO.OUT)
    GPIO.setup(s["echo"], GPIO.IN)
    GPIO.output(s["trig"], False)

# ==========================================
# 2. ULTRASÓNICOS EN THREAD
# ==========================================
def tarea_ultrasonicos():
    global LECTURAS_US

    # Timeout más relajado: hasta ~8–9 m de eco máximo (~50 ms)
    TIMEOUT_SYNC = 0.050
    PAUSA_ENTRE_SENSORES = 0.003

    def medir_flash(trig, echo):
        # Disparo
        GPIO.output(trig, True)
        time.sleep(0.00001)   # 10 us
        GPIO.output(trig, False)

        # Esperar a que ECHO suba (comienzo del pulso)
        t0 = time.time()
        while GPIO.input(echo) == 0:
            if time.time() - t0 > TIMEOUT_SYNC:
                return 0.0  # no hubo eco detectable

        start = time.time()

        # Esperar a que ECHO baje (fin del pulso)
        t0 = time.time()
        while GPIO.input(echo) == 1:
            if time.time() - t0 > TIMEOUT_SYNC:
                return 0.0  # pulso demasiado largo / raro

        stop = time.time()

        # Convertir a distancia en cm
        dist = ((stop - start) * 34300) / 2
        if dist > 250:
            return 250.0
        return dist

    while True:
        for i, s in enumerate(US_PINS):
            val = medir_flash(s["trig"], s["echo"])
            LECTURAS_US[i] = val
            time.sleep(PAUSA_ENTRE_SENSORES)


# ==========================================
# 3. ToF: FILTRO, POOLING, 3x3, NIVELES
# ==========================================

def median_filter_3x3_ignore_zeros(matrix_8):
    """Filtro de mediana 3x3 sobre 8x8, ignorando ceros (0 = sin dato)."""
    filtered = np.zeros_like(matrix_8)
    for i in range(8):
        for j in range(8):
            neighbors = matrix_8[max(0, i-1):min(8, i+2),
                                 max(0, j-1):min(8, j+2)]
            vals = neighbors[neighbors > 0]
            if vals.size == 0:
                filtered[i, j] = 0
            else:
                filtered[i, j] = int(np.median(vals))
    return filtered

def pool_8x8_to_4x4(matrix_8):
    """Pooling 2x2 por mediana → 4x4, ignorando ceros."""
    pooled = np.zeros((4, 4), dtype=matrix_8.dtype)
    for i in range(4):
        for j in range(4):
            block = matrix_8[2*i:2*i+2, 2*j:2*j+2]
            vals = block[block > 0]
            if vals.size == 0:
                pooled[i, j] = 0
            else:
                pooled[i, j] = int(np.median(vals))
    return pooled

def accumulate_frame_3x3(depth_4x4, d_min_mm=25, d_max_mm=4000):
    """
    depth_4x4: matriz 4x4 en mm (ya filtrada y pooleada).
    Devuelve grid_3x3 con la distancia mínima por zona (mm), np.nan si vacío.

    Mapeo fijo 4x4 → 3x3:
        filas  0 → fila 0 (arriba)
               1 → fila 1 (medio)
               2 → fila 1 (medio)
               3 → fila 2 (abajo)

        columnas igual (0→izq, 1–2→centro, 3→der).
    """
    ROW_MAP = [0, 1, 1, 2]
    COL_MAP = [0, 1, 1, 2]

    grid = np.full((3, 3), np.inf, dtype=float)

    for r in range(4):
        i = ROW_MAP[r]
        for c in range(4):
            j = COL_MAP[c]
            d = float(depth_4x4[r, c])

            if d <= 0:
                continue
            if d < d_min_mm or d > d_max_mm:
                continue

            if d < grid[i, j]:
                grid[i, j] = d

    grid[~np.isfinite(grid)] = np.nan
    return grid

# Distancia → nivel 0-9
NEAR_MM = 250.0   # <= esto es nivel 9
FAR_MM  = 3000.0  # >= esto es nivel 0

def distance_to_level(d_mm):
    """Convierte distancia en mm a nivel 0-9 (0 lejos, 9 muy cerca)."""
    if d_mm is None:
        return 0
    try:
        d = float(d_mm)
    except:
        return 0

    if not np.isfinite(d) or d <= 0:
        return 0

    if d <= NEAR_MM:
        return 9
    if d >= FAR_MM:
        return 0

    level = int(round(9.0 * (FAR_MM - d) / (FAR_MM - NEAR_MM)))
    if level < 0: level = 0
    if level > 9: level = 9
    return level

def grid3x3_to_levels(grid_3x3):
    """Aplica distance_to_level celda por celda a la 3x3."""
    levels = np.zeros((3, 3), dtype=int)
    for i in range(3):
        for j in range(3):
            val = grid_3x3[i, j]
            if np.isnan(val):
                levels[i, j] = 0
            else:
                levels[i, j] = distance_to_level(val)
    return levels

# ==========================================
# 4. BUCLE PRINCIPAL
# ==========================================
def main():
    # 1. Hilo de ultrasonidos
    t = threading.Thread(target=tarea_ultrasonicos)
    t.daemon = True
    t.start()
    
    # 2. Inicializar ToF único
    print(">> Inicializando ToF único (0x29, 8x8, 15 Hz)...")
    sensor = None

    try:
        sensor = vl53.VL53L5CX(i2c_addr=0x29)
        print("   ToF detectado en 0x29")
    except Exception as e:
        print("   No se pudo 0x29, intentando 0x30...", e)
        try:
            sensor = vl53.VL53L5CX(i2c_addr=0x30)
            print("   ToF detectado en 0x30")
        except Exception as e2:
            print("   ERROR: No se encontró ToF en 0x29 ni 0x30:", e2)
            GPIO.cleanup()
            return

    sensor.set_resolution(8*8)
    try:
        sensor.set_ranging_frequency_hz(15)
    except:
        pass

    sensor.start_ranging()
    
    # 3. Abrir UART hacia ESP32
    ser = None
    if serial is not None:
        try:
            ser = serial.Serial("/dev/serial0", 115200, timeout=0)
            print("UART abierto en /dev/serial0 a 115200.")
        except Exception as e:
            print("ADVERTENCIA: no se pudo abrir UART:", e)
            ser = None
    else:
        print("ADVERTENCIA: pyserial no está instalado; no se enviará por UART.")

    sys.stdout.write("\033[2J")
    sys.stdout.flush()

    VALID_STATUS = {5}  # ajusta si ves otros códigos válidos

    try:
        while True:
            if sensor.data_ready():
                data = sensor.get_data()

                # Distancias y target_status como 8x8
                dist_raw   = np.array(data.distance_mm,   dtype=int).reshape((8, 8))
                status_raw = np.array(data.target_status, dtype=int).reshape((8, 8))

                # CORRECCIÓN ORIENTACIÓN: sensor girado 90° -> usamos k=-1 (90° CW)
                dist_raw   = np.rot90(dist_raw,   k=-1)
                status_raw = np.rot90(status_raw, k=-1)

                # Zonas inválidas según target_status → distancia 0 (sin dato)
                mask_invalid = ~np.isin(status_raw, list(VALID_STATUS))
                dist_raw[mask_invalid] = 0

                # Filtro mediana ignorando ceros
                grid_8 = median_filter_3x3_ignore_zeros(dist_raw)

                # Pooling 8x8 → 4x4
                grid_4 = pool_8x8_to_4x4(grid_8)

                # Matriz 3x3 de distancias mínimas (d_min=25 mm)
                grid_3x3 = accumulate_frame_3x3(grid_4, d_min_mm=25, d_max_mm=4000)

                # Matriz 3x3 de niveles 0-9
                levels_3x3 = grid3x3_to_levels(grid_3x3)

                # Mensaje lineal para ESP32 (fila por fila)
                flat_levels = levels_3x3.flatten()
                msg = " ".join(str(x) for x in flat_levels)

                # Enviar por UART (formato: "0 1 2 3 4 5 6 7 8\n")
                if ser is not None:
                    try:
                        ser.write((msg + "\n").encode("ascii"))
                    except Exception as e:
                        # Si truena, solo lo avisamos una vez y seguimos
                        print("Error escribiendo por UART:", e)
                        ser = None

                # ---- Telemetría limpia ----
                sys.stdout.write("\033[H")
                sys.stdout.flush()

                print(">>> GRID 3x3 (dist mínima por zona, mm; --- = vacío):")
                for i in range(3):
                    row = ""
                    for j in range(3):
                        val = grid_3x3[i, j]
                        if np.isnan(val):
                            row += "   ---"
                        else:
                            row += f"{val:6.0f}"
                    print(row)

                print("-" * 40)
                print(">>> LEVELS 3x3 (0-9):")
                for i in range(3):
                    print(" ".join(str(levels_3x3[i, j]) for j in range(3)))

                print("-" * 40)
                print("Mensaje para ESP32:", msg)

                # ---- Telemetría de ultrasonidos ----
                print("-" * 40)
                print(">>> ULTRASÓNICOS (cm):")
                print(
                    f"S1 = {LECTURAS_US[0]:6.1f}   |   "
                    f"S2 = {LECTURAS_US[1]:6.1f}   |   "
                    f"S3 = {LECTURAS_US[2]:6.1f}"
                )

                print("-" * 40)
                print("Ctrl+C para salir.")

            # Opcional: bajar un poco CPU
            # time.sleep(0.001)

    except KeyboardInterrupt:
        print("\nApagando sistema...")
        try:
            sensor.stop_ranging()
        except:
            pass
        if ser is not None:
            ser.close()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
