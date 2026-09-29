"""
============================================================================
 FIRMWARE ESP32 (MicroPython) - CARRO AUTONOMO DIFERENCIAL PARA ROS
============================================================================
Maneja:
  - 3 sensores ultrasonicos HC-SR04 (TRIG compartido, ECHO independiente)
  - 1 IMU MPU6050 (I2C, registros directos)
  - 2 encoders de 1 canal (rueda de 20 ranuras, diametro 75 mm)
  - 4 motores DC independientes, controlados por 2 puentes H de doble canal
    (cada puente H mueve 2 motores: canal A y canal B)

Comunicacion con la Raspberry Pi por USB (puerto serial del REPL):
  Solo conecta el cable USB del ESP32 a la Raspberry. No hace falta
  cablear GPIOs ni configurar raspi-config.

  El ESP32 aparecera en la Raspberry como /dev/ttyUSB0 o /dev/ttyUSB1
  (segun el orden en que se conecten los dispositivos USB).

  IMPORTANTE - conflicto con el REPL:
  En MicroPython el puerto USB es COMPARTIDO con el REPL de Thonny.
  Esto significa:
    - NO puedes tener Thonny conectado mientras el nodo ROS lee datos.
      Se pelean por el mismo puerto y uno de los dos falla.
    - Para depurar: primero cierra el nodo ROS, luego abre Thonny.
    - Para operar: cierra Thonny, luego lanza el nodo ROS.

  IMPORTANTE - conflicto con el LIDAR:
  El LIDAR tambien usa /dev/ttyUSB*. El numero que le toca a cada
  dispositivo depende del orden de conexion al arrancar, asi que
  puede cambiar entre reinicios. Ver notas de udev en el nodo ROS
  para fijar un nombre estable por dispositivo.

Protocolo (115200 baud):
  ESP32 -> RPi (cada CONTROL_PERIOD_MS):
    D,us1_cm,us2_cm,us3_cm,ax,ay,az,gx,gy,gz,d_izq_mm,d_der_mm,t_ms\n
  RPi -> ESP32 (comando tipo cmd_vel):
    C,lineal_mm_s,angular_rad_s\n

Guarda este archivo como main.py en el ESP32 (arranca solo al energizar).
============================================================================
"""

from machine import Pin, PWM, I2C, time_pulse_us
import time
import sys
import select

# ---------------------------------------------------------------------------
# PINES - MOTORES (4 independientes, 2 puentes H de 2 canales c/u)
# ---------------------------------------------------------------------------
# NOTA: GPIO34 y GPIO35 son SOLO ENTRADA en el ESP32 (no tienen driver de
# salida), por lo que no pueden usarse para IN/ENA/ENB. Se reubicaron:
#   ENB delantero izquierdo: 34 -> 4
#   IN2 delantero derecho:   35 -> 16
# El resto de tus pines se mantiene exactamente igual.

# Motor Trasero Derecho (MTD) - canal B de un puente H
MTD_ENB = 25
MTD_IN3 = 26
MTD_IN4 = 27   # GPIO12 es pin de "strapping" (MTDI). Suele funcionar bien
               # como salida una vez arrancado; si tienes boot loops raros,
               # revisa este pin primero.

# Motor Trasero Izquierdo (MTI) - canal A de un puente H
MTI_ENA = 13
MTI_IN1 = 12
MTI_IN2 = 14

# Motor Delantero Derecho (MDD) - canal A de un puente H
MDD_ENA = 4
MDD_IN1 = 15
MDD_IN2 = 2   # reubicado (antes 35, input-only)

# Motor Delantero Izquierdo (MDI) - canal B de un puente H
MDI_ENB = 32    # reubicado (antes 34, input-only)
MDI_IN3 = 16   # strapping pin (MTDO), normalmente sin problema
MDI_IN4 = 33    # strapping pin, debe quedar en LOW/flotante al arrancar;
               # si el driver lo mantiene en un nivel definido, no hay drama

# ---------------------------------------------------------------------------
# PINES - ULTRASONICOS (TRIG compartido, 1 solo pin para los 3 sensores)
# ---------------------------------------------------------------------------
US_TRIG_PIN = 5
US_FRONT_ECHO_PIN = 34   # input-only: perfecto para ECHO (no necesita pull-up)
US_LEFT_ECHO_PIN = 19	    # input-only
US_RIGHT_ECHO_PIN = 35   # input-only

# ---------------------------------------------------------------------------
# PINES - I2C (MPU6050) y ENCODERS
# ---------------------------------------------------------------------------
# La comunicacion con la Raspberry ahora es por USB (sys.stdout/stdin),
# asi que los pines 19 y 23 que antes usaba el UART2 quedan LIBRES.
SDA_PIN, SCL_PIN = 21, 22
MPU_ADDR = 0x68

ENC_IZQ_PIN, ENC_DER_PIN = 17, 18

PWM_FREQ = 5000

# ---------------------------------------------------------------------------
# PARAMETROS FISICOS DEL ROBOT
# ---------------------------------------------------------------------------
DIAM_RUEDA_MM = 75.0
CIRCUNF_MM = 3.14159265 * DIAM_RUEDA_MM          # 235.62 mm

# Pulsos por vuelta CALIBRADOS con pruebas de linea recta.
#
# Historial de la calibracion (siempre sobre 32 cm reales):
#   1) A mano, 1 vuelta (antirrebote 15 ms): izq 23,   der 40
#   2) Con 23/40   -> la odometria reportaba 18 cm (corto)
#   3) Con 12.9/22.5 -> reportaba 40 cm (largo)
#   4) Con 16.1/28.1 -> valor intermedio, deberia acercarse a 32 cm
#
# La relacion entre lados se mantuvo constante en todos los ajustes
# (izq/der ~= 0.573), que es lo que mas importa: si esa proporcion esta
# bien, el robot va derecho aunque la escala absoluta tenga error.
#
# El paso de 18 a 40 cm al aplicar un factor de 1.78 muestra que la
# respuesta no es del todo lineal, asi que espera precision aproximada,
# no exacta. Si tras este ajuste queda cerca de 32 cm, es suficiente:
# el scan matching del LIDAR corrige el resto.
PULSOS_POR_REV_IZQ = 16.1
PULSOS_POR_REV_DER = 28.1

MM_POR_PULSO_IZQ = CIRCUNF_MM / PULSOS_POR_REV_IZQ   # ~10.24 mm
MM_POR_PULSO_DER = CIRCUNF_MM / PULSOS_POR_REV_DER   # ~5.89 mm

TRACK_MM = 370.0                                  # separacion entre ruedas

CONTROL_PERIOD_MS = 50    # 20 Hz
WATCHDOG_MS = 500         # sin cmd_vel -> detener por seguridad

VEL_MAX_MM_S = 400.0      # ajustar tras pruebas reales
KP_VEL = 0.6              # ganancia proporcional simple

# Control en lazo cerrado (realimentacion por encoders).
#
# Ponlo en False para operar SOLO con feedforward (PWM proporcional a la
# velocidad pedida, sin correccion por encoders) -- equivalente a mover
# los motores en lazo abierto, pero obedeciendo /cmd_vel.
#
# Por que puede convenir False: con 20 ranuras y 11.78mm por pulso, a
# velocidades bajas llega menos de un pulso por ciclo de control. La
# velocidad medida salta entre 0 y ~196 mm/s de un ciclo a otro, y el
# termino proporcional persigue ese ruido en vez de la senal. Ademas,
# si un encoder no cuenta (cableado, sensor danado), ese lado queda
# controlado a ciegas y se comporta de forma erratica.
USAR_LAZO_CERRADO = False

# PWM minimo para que los motores realmente giren (zona muerta).
#
# Los motores DC con reductora necesitan un minimo de corriente para vencer
# la friccion estatica. Por debajo de ese umbral solo zumban sin moverse.
#
# El firmware convierte velocidad a PWM de forma proporcional, asi que
# cuando Nav2 pide velocidades pequenas (al girar despacio o al acercarse
# a la meta) el PWM resultante queda por debajo del umbral y el robot no
# se mueve, aunque se oiga el zumbido.
#
# Este valor eleva cualquier PWM distinto de cero hasta el minimo util.
# CALIBRACION: si el robot sigue zumbando sin moverse, sube el valor de
# 50 en 50. Si arranca a tirones o demasiado brusco, bajalo.
#
# Subido a 800 (78% del maximo) porque con 450 los motores no lograban
# arrancar con los comandos que manda Nav2 (linear.x ~0.2 m/s, que en
# escala daba PWM 511). A 800 practicamente cualquier orden de
# movimiento entrega potencia suficiente para vencer la friccion.
#
# CONTRAPARTIDA: el robot pierde casi toda capacidad de moverse
# despacio. Cualquier comando pequeno lo mueve a ~78% de potencia, asi
# que se pasara de las metas y maniobrara con brusquedad. Es un parche
# para motores que no dan el par necesario; la solucion de fondo es
# mas voltaje, motores con mas reduccion, o menos peso.
# Rango del PWM en MicroPython: 0-1023.
PWM_MINIMO = 800

# Calibracion del acelerometro (obtenida con prueba multi-orientacion).
# corrected = (lectura_cruda_m/s2 - BIAS) / ESCALA
# El eje X quedo con calibracion aproximada (falto la 6ta posicion),
# si mas adelante notas que ax se ve raro, se puede refinar.
ACCEL_BIAS_X = 0.40
ACCEL_BIAS_Y = -4.75
ACCEL_BIAS_Z = 2.05
ACCEL_SCALE_X = 0.974
ACCEL_SCALE_Y = 0.984
ACCEL_SCALE_Z = 1.091

# Bias del giroscopio: se calcula AUTOMATICAMENTE al arrancar, promediando
# lecturas con el robot quieto (ver calibrar_giroscopio()).
#
# Por que hace falta: un MPU6050 en reposo no reporta 0 rad/s, sino un
# offset fijo (en tu sensor, gz ~= -0.025 rad/s = -1.4 grados/s). El EKF
# integra eso fielmente y cree que el robot gira solo, ~1.3 grados por
# segundo, lo que arruina la odometria y el mapa del SLAM.
GYRO_BIAS_X = 0.0
GYRO_BIAS_Y = 0.0
GYRO_BIAS_Z = 0.0

# ---------------------------------------------------------------------------
# INICIALIZACION DE PERIFERICOS
# ---------------------------------------------------------------------------
us_trig = Pin(US_TRIG_PIN, Pin.OUT)
us_front_echo = Pin(US_FRONT_ECHO_PIN, Pin.IN)
us_left_echo = Pin(US_LEFT_ECHO_PIN, Pin.IN)
us_right_echo = Pin(US_RIGHT_ECHO_PIN, Pin.IN)

# --- Motor Trasero Derecho ---
mtd_in3 = Pin(MTD_IN3, Pin.OUT)
mtd_in4 = Pin(MTD_IN4, Pin.OUT)
mtd_enb = PWM(Pin(MTD_ENB), freq=PWM_FREQ, duty=0)

# --- Motor Trasero Izquierdo ---
mti_in1 = Pin(MTI_IN1, Pin.OUT)
mti_in2 = Pin(MTI_IN2, Pin.OUT)
mti_ena = PWM(Pin(MTI_ENA), freq=PWM_FREQ, duty=0)

# --- Motor Delantero Derecho ---
mdd_in1 = Pin(MDD_IN1, Pin.OUT)
mdd_in2 = Pin(MDD_IN2, Pin.OUT)
mdd_ena = PWM(Pin(MDD_ENA), freq=PWM_FREQ, duty=0)

# --- Motor Delantero Izquierdo ---
mdi_in3 = Pin(MDI_IN3, Pin.OUT)
mdi_in4 = Pin(MDI_IN4, Pin.OUT)
mdi_enb = PWM(Pin(MDI_ENB), freq=PWM_FREQ, duty=0)
# Nota: duty() usa escala 0-1023 en la mayoria de builds ESP32.
# Si tu firmware solo tiene duty_u16(), cambia las llamadas duty(x)
# por duty_u16(x*64) dentro de _aplicar_motor().

i2c = I2C(0, scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=400000)

# Poller para leer stdin (USB) sin bloquear el bucle de control.
# select.poll() nos deja preguntar "hay datos?" antes de leer, igual
# que haciamos con uart.any().
_poller = select.poll()
_poller.register(sys.stdin, select.POLLIN)

# ---------------------------------------------------------------------------
# ENCODERS - CONTADORES POR INTERRUPCION
# ---------------------------------------------------------------------------
pulsos_izq = 0
pulsos_der = 0
dir_izq = 0   # -1, 0, 1  (ultima direccion comandada, lado izquierdo)
dir_der = 0   # -1, 0, 1  (ultima direccion comandada, lado derecho)

# Antirrebote de los encoders.
#
# Velocidad maxima real ~400 mm/s con 11.78 mm por pulso = ~34 pulsos/s,
# o sea ~29 ms entre pulsos legitimos. Cualquier flanco que llegue a
# menos de DEBOUNCE_US del anterior es fisicamente imposible y viene de
# rebote mecanico o ruido electrico en la senal del sensor.
#
# 5000 us (5 ms) permite hasta 200 pulsos/s: seis veces mas de lo que el
# robot puede generar, asi que no descarta nada real, pero corta el
# rebote observado (que llegaba a ~500 pulsos/s).
DEBOUNCE_US = 15000

# Inicializar con el tiempo actual, NO con 0: ticks_diff(ahora, 0)
# puede dar negativo y dejar el filtro descartando todos los
# pulsos para siempre.
_ultimo_pulso_izq = time.ticks_us()
_ultimo_pulso_der = time.ticks_us()


def _isr_enc_izq(pin):
    global pulsos_izq, _ultimo_pulso_izq
    ahora = time.ticks_us()
    if time.ticks_diff(ahora, _ultimo_pulso_izq) < DEBOUNCE_US:
        return
    _ultimo_pulso_izq = ahora
    pulsos_izq += 1


def _isr_enc_der(pin):
    global pulsos_der, _ultimo_pulso_der
    ahora = time.ticks_us()
    if time.ticks_diff(ahora, _ultimo_pulso_der) < DEBOUNCE_US:
        return
    _ultimo_pulso_der = ahora
    pulsos_der += 1


enc_izq = Pin(ENC_IZQ_PIN, Pin.IN, Pin.PULL_UP)
enc_der = Pin(ENC_DER_PIN, Pin.IN, Pin.PULL_UP)
enc_izq.irq(trigger=Pin.IRQ_RISING, handler=_isr_enc_izq)
enc_der.irq(trigger=Pin.IRQ_RISING, handler=_isr_enc_der)

# ---------------------------------------------------------------------------
# MPU6050 - LECTURA DIRECTA POR REGISTROS
# ---------------------------------------------------------------------------
def mpu_init():
    i2c.writeto_mem(MPU_ADDR, 0x6B, b'\x00')  # despertar sensor
    i2c.writeto_mem(MPU_ADDR, 0x1C, b'\x08')  # accel +-4g
    i2c.writeto_mem(MPU_ADDR, 0x1B, b'\x08')  # gyro +-500 dps


def _s16(hi, lo):
    val = (hi << 8) | lo
    if val >= 0x8000:
        val -= 0x10000
    return val


def mpu_leer():
    datos = i2c.readfrom_mem(MPU_ADDR, 0x3B, 14)

    raw_ax = _s16(datos[0], datos[1])
    raw_ay = _s16(datos[2], datos[3])
    raw_az = _s16(datos[4], datos[5])
    raw_gx = _s16(datos[8], datos[9])
    raw_gy = _s16(datos[10], datos[11])
    raw_gz = _s16(datos[12], datos[13])

    ACC_SENS = 8192.0
    GYRO_SENS = 65.5
    G_A_MS2 = 9.80665
    DEG_A_RAD = 3.14159265 / 180.0

    ax_raw = (raw_ax / ACC_SENS) * G_A_MS2
    ay_raw = (raw_ay / ACC_SENS) * G_A_MS2
    az_raw = (raw_az / ACC_SENS) * G_A_MS2
    gx = (raw_gx / GYRO_SENS) * DEG_A_RAD - GYRO_BIAS_X
    gy = (raw_gy / GYRO_SENS) * DEG_A_RAD - GYRO_BIAS_Y
    gz = (raw_gz / GYRO_SENS) * DEG_A_RAD - GYRO_BIAS_Z

    # Aplicar calibracion de bias/escala por eje (ver constantes arriba)
    ax = (ax_raw - ACCEL_BIAS_X) / ACCEL_SCALE_X
    ay = (ay_raw - ACCEL_BIAS_Y) / ACCEL_SCALE_Y
    az = (az_raw - ACCEL_BIAS_Z) / ACCEL_SCALE_Z

    return ax, ay, az, gx, gy, gz


def calibrar_giroscopio(n=200):
    """
    Promedia n lecturas del giroscopio con el robot QUIETO y guarda el
    resultado como bias, que luego se resta en cada lectura.

    IMPORTANTE: el robot NO se debe mover durante esta calibracion
    (dura ~2 segundos al arrancar). Si se mueve, el bias queda mal
    calculado y la odometria saldra peor que sin calibrar.

    El bias del giroscopio cambia con la temperatura, asi que se
    recalcula en cada arranque en vez de dejarlo fijo en el codigo.
    """
    global GYRO_BIAS_X, GYRO_BIAS_Y, GYRO_BIAS_Z

    # Asegurar que el bias este en cero mientras medimos
    GYRO_BIAS_X = 0.0
    GYRO_BIAS_Y = 0.0
    GYRO_BIAS_Z = 0.0

    sx = sy = sz = 0.0
    for _ in range(n):
        _, _, _, gx, gy, gz = mpu_leer()
        sx += gx
        sy += gy
        sz += gz
        time.sleep_ms(10)

    GYRO_BIAS_X = sx / n
    GYRO_BIAS_Y = sy / n
    GYRO_BIAS_Z = sz / n


# ---------------------------------------------------------------------------
# ULTRASONICOS (TRIG compartido)
# ---------------------------------------------------------------------------
def leer_ultrasonico(echo_pin):
    us_trig.value(0)
    time.sleep_us(2)
    us_trig.value(1)
    time.sleep_us(10)
    us_trig.value(0)

    duracion = time_pulse_us(echo_pin, 1, 25000)  # timeout 25 ms
    if duracion < 0:
        return -1.0
    return duracion * 0.0343 / 2.0  # cm


# ---------------------------------------------------------------------------
# MOTORES
# ---------------------------------------------------------------------------
def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _aplicar_motor(in_a, in_b, pwm_obj, valor):
    """valor: -1023..1023. Aplica direccion + PWM a un motor."""
    valor = int(_clamp(valor, -1023, 1023))
    if valor > 0:
        in_a.value(1); in_b.value(0)
    elif valor < 0:
        in_a.value(0); in_b.value(1)
    else:
        in_a.value(0); in_b.value(0)
    pwm_obj.duty(abs(valor))


def set_lado_izquierdo(valor):
    """Mueve los 2 motores izquierdos (delantero + trasero) igual."""
    global dir_izq
    dir_izq = 1 if valor > 0 else (-1 if valor < 0 else 0)
    _aplicar_motor(mti_in1, mti_in2, mti_ena, valor)   # trasero izquierdo
    _aplicar_motor(mdi_in3, mdi_in4, mdi_enb, valor)   # delantero izquierdo


def set_lado_derecho(valor):
    """Mueve los 2 motores derechos (delantero + trasero) igual."""
    global dir_der
    dir_der = 1 if valor > 0 else (-1 if valor < 0 else 0)
    _aplicar_motor(mdd_in1, mdd_in2, mdd_ena, valor)   # delantero derecho
    _aplicar_motor(mtd_in3, mtd_in4, mtd_enb, valor)   # trasero derecho


def detener_motores():
    global target_linear, target_angular
    set_lado_izquierdo(0)
    set_lado_derecho(0)
    target_linear = 0.0
    target_angular = 0.0


# ---------------------------------------------------------------------------
# PARSEO DE COMANDOS: "C,lineal_mm_s,angular_rad_s"
# ---------------------------------------------------------------------------
target_linear = 0.0
target_angular = 0.0
last_cmd_time = time.ticks_ms()


def procesar_linea(linea):
    global target_linear, target_angular, last_cmd_time
    try:
        linea = linea.strip()
        if not linea or linea[0] != 'C':
            return
        partes = linea.split(',')
        if len(partes) < 3:
            return
        target_linear = float(partes[1])
        target_angular = float(partes[2])
        last_cmd_time = time.ticks_ms()
    except (ValueError, IndexError):
        pass


# ---------------------------------------------------------------------------
# PROGRAMA PRINCIPAL
# ---------------------------------------------------------------------------
def main():
    global pulsos_izq, pulsos_der, target_linear, target_angular, last_cmd_time

    mpu_init()
    detener_motores()

    # Calibrar el giroscopio con el robot quieto (~2 segundos).
    # Los motores ya estan detenidos por la linea de arriba.
    calibrar_giroscopio()

    last_control_time = time.ticks_ms()
    rx_buffer = ""

    while True:
        # --- 1) leer USB (stdin) no bloqueante, linea por linea ---
        # poll(0) = "revisa si hay datos y regresa de inmediato".
        # Leemos de a un caracter porque sys.stdin.read(n) en MicroPython
        # bloquea hasta juntar n caracteres, y eso congelaria el control.
        while _poller.poll(0):
            ch = sys.stdin.read(1)
            if not ch:
                break
            if ch == '\n':
                procesar_linea(rx_buffer)
                rx_buffer = ""
            elif ch != '\r':
                rx_buffer += ch
                # proteccion contra basura sin salto de linea
                if len(rx_buffer) > 120:
                    rx_buffer = ""

        # --- 2) watchdog de seguridad ---
        if time.ticks_diff(time.ticks_ms(), last_cmd_time) > WATCHDOG_MS:
            target_linear = 0.0
            target_angular = 0.0

        # --- 3) ciclo de control periodico ---
        ahora = time.ticks_ms()
        dt_ms = time.ticks_diff(ahora, last_control_time)
        if dt_ms >= CONTROL_PERIOD_MS:
            dt_s = dt_ms / 1000.0
            last_control_time = ahora

            p_izq, pulsos_izq = pulsos_izq, 0
            p_der, pulsos_der = pulsos_der, 0

            d_izq_mm = p_izq * MM_POR_PULSO_IZQ * (1 if dir_izq >= 0 else -1)
            d_der_mm = p_der * MM_POR_PULSO_DER * (1 if dir_der >= 0 else -1)

            vel_izq_medida = d_izq_mm / dt_s
            vel_der_medida = d_der_mm / dt_s

            vel_izq_obj = target_linear - (target_angular * TRACK_MM / 2.0)
            vel_der_obj = target_linear + (target_angular * TRACK_MM / 2.0)

            pwm_ff_izq = (vel_izq_obj / VEL_MAX_MM_S) * 1023.0
            pwm_ff_der = (vel_der_obj / VEL_MAX_MM_S) * 1023.0

            if USAR_LAZO_CERRADO:
                error_izq = vel_izq_obj - vel_izq_medida
                error_der = vel_der_obj - vel_der_medida
                pwm_izq = pwm_ff_izq + KP_VEL * error_izq
                pwm_der = pwm_ff_der + KP_VEL * error_der
            else:
                # Solo feedforward: el PWM depende unicamente de la
                # velocidad pedida. Los encoders se siguen leyendo y
                # publicando para la odometria, pero no afectan el PWM.
                pwm_izq = pwm_ff_izq
                pwm_der = pwm_ff_der

            if abs(vel_izq_obj) < 1.0 and abs(vel_der_obj) < 1.0:
                pwm_izq = 0
                pwm_der = 0
            else:
                # Compensar la zona muerta: si se pide movimiento pero el
                # PWM calculado es muy bajo, elevarlo al minimo util.
                if 0 < abs(pwm_izq) < PWM_MINIMO:
                    pwm_izq = PWM_MINIMO if pwm_izq > 0 else -PWM_MINIMO
                if 0 < abs(pwm_der) < PWM_MINIMO:
                    pwm_der = PWM_MINIMO if pwm_der > 0 else -PWM_MINIMO

            set_lado_izquierdo(pwm_izq)
            set_lado_derecho(pwm_der)

            # --- leer sensores ---
            us1 = leer_ultrasonico(us_front_echo)
            us2 = leer_ultrasonico(us_left_echo)
            us3 = leer_ultrasonico(us_right_echo)
            ax, ay, az, gx, gy, gz = mpu_leer()

            # --- enviar trama por USB (stdout) ---
            trama = "D,{:.1f},{:.1f},{:.1f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.2f},{:.2f},{}\n".format(
                us1, us2, us3, ax, ay, az, gx, gy, gz, d_izq_mm, d_der_mm, ahora
            )
            sys.stdout.write(trama)

        time.sleep_ms(2)


if __name__ == "__main__":
    main()
