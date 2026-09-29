# ============================================================
#  Mecanismo de recoleccion y entrega - ESP32 / MicroPython
#  Protocolo serie (una linea por comando, terminada en \n):
#     REC:<n>   recoleccion de n productos (1..6)
#     ENT:<n>   entrega de n productos (1..6)
#     STOP      parada inmediata
#     EST?      consulta de estado
#     RESET     reinicia el ESP32
#  Respuestas:
#     ACK <cmd> <n> | EVT <detalle> | OK <cmd> <n> | ERR <motivo>
# ============================================================

from machine import Pin, PWM
import time, sys, select

MAX_PRODUCTOS = 6

# ---------------- ACTUADORES ----------------
BANDA_A = Pin(32, Pin.OUT)      # banda inicial
BANDA_B = Pin(33, Pin.OUT)
EST_A   = Pin(25, Pin.OUT)      # estiramiento de banda
EST_B   = Pin(26, Pin.OUT)
ACT_A   = Pin(19, Pin.OUT)      # actuador de despacho
ACT_B   = Pin(18, Pin.OUT)
ALM_A   = Pin(16, Pin.OUT)      # giro almacen
ALM_B   = Pin(17, Pin.OUT)

MOT_A = PWM(Pin(22), freq=1000, duty=0)   # plataforma: SUBIR
MOT_B = PWM(Pin(21), freq=1000, duty=0)   # plataforma: BAJAR

# ---------------- SERVOS ----------------
GARRA      = PWM(Pin(15), freq=50)
PLATAFORMA = PWM(Pin(2),  freq=50)

US_MIN = 600
US_MAX = 2400

GARRA_ABIERTA = 0
GARRA_CERRADA = 180
PLAT_ARRIBA   = 90
PLAT_ABAJO    = 180

PASO_SERVO   = 10               # grados por escalon
T_PASO_SERVO = 50               # ms entre escalones

# ---------------- SENSORES ----------------
FC_EST_A  = Pin(13, Pin.IN, Pin.PULL_UP)   # limite estiramiento lado A
FC_EST_A2 = Pin(35, Pin.IN)                # solo entrada: pull-up EXTERNO 10k
FC_EST_B  = Pin(23, Pin.IN, Pin.PULL_UP)   # limite estiramiento lado B
FC_ACT_A  = Pin(27, Pin.IN, Pin.PULL_UP)   # limite actuador lado A
FC_ACT_B  = Pin(14, Pin.IN, Pin.PULL_UP)   # limite actuador lado B
FC_ARRIBA = Pin(34, Pin.IN)                # solo entrada: pull-up EXTERNO 10k
FC_ABAJO  = Pin(0,  Pin.IN, Pin.PULL_UP)   # OJO: pin de arranque

IR1 = Pin(5, Pin.IN)
IR2 = Pin(4, Pin.IN)

NIVEL_PARO = 1                  # nivel de los IR que marca posicion valida

# ---------------- TIEMPOS (ms) ----------------
T_ESPERA_EXT      = 1000
T_BANDA_ATRAS     = 2000
T_ANTES_ALMACEN   = 1000
T_ENTRE_CICLOS    = 1000
T_ACT_EXTENDIDO   = 800
T_ENTRE_PRODUCTOS = 1000
T_SALIDA_IR       = 1300        # sale de la zona de deteccion antes de mirar

VELOCIDAD    = 800              # 0-1023
PWM_ARRANQUE = 1023             # patada inicial
T_ARRANQUE   = 200              # ms a maxima potencia
T_BAJADA     = 10000            # ms de bajada tras tocar el FC de arriba

TO_ESTIRAMIENTO = 50000
TO_ACTUADOR     = 50000
TO_ALMACEN      = 15000
TO_MOTOR        = 20000

# ---------------- ESTADO ----------------
productos = 0                   # productos actualmente en el almacen
ocupado = False
abortar = False

# ---------------- SERIE NO BLOQUEANTE ----------------
_poll = select.poll()
_poll.register(sys.stdin, select.POLLIN)
_buf = ""

def resp(txt):
    print(txt)

def leer_comando():
    """Devuelve una linea completa o None. Nunca bloquea."""
    global _buf
    while _poll.poll(0):
        c = sys.stdin.read(1)
        if c in ('\n', '\r'):
            if _buf:
                linea = _buf.strip()
                _buf = ""
                return linea
        else:
            _buf += c
            if len(_buf) > 32:
                _buf = ""
    return None

def revisar_stop():
    """Se llama dentro de los bucles de movimiento."""
    global abortar
    cmd = leer_comando()
    if cmd and cmd.upper() == "STOP":
        abortar = True
    return abortar

def espera(ms):
    """sleep interrumpible por STOP."""
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < ms:
        if revisar_stop():
            return False
        time.sleep_ms(10)
    return True

# ---------------- BASICAS ----------------
def mover(pin_a, pin_b, sentido):
    if sentido == 'A':
        pin_a.value(1); pin_b.value(0)
    elif sentido == 'B':
        pin_a.value(0); pin_b.value(1)
    else:
        pin_a.value(0); pin_b.value(0)

def parar_motor():
    MOT_A.duty(0)
    MOT_B.duty(0)

def parar_todo():
    mover(BANDA_A, BANDA_B, 0)
    mover(EST_A, EST_B, 0)
    mover(ACT_A, ACT_B, 0)
    mover(ALM_A, ALM_B, 0)
    parar_motor()

def presionado(fc):
    return fc.value() == 0

def almacen_en_posicion():
    return IR1.value() == NIVEL_PARO and IR2.value() == NIVEL_PARO

# ---------------- MOVIMIENTOS CON FINAL DE CARRERA ----------------
def hasta_final(pin_a, pin_b, sentido, fc, timeout_ms, nombre):
    if presionado(fc):
        return True
    mover(pin_a, pin_b, sentido)
    t0 = time.ticks_ms()
    while not presionado(fc):
        if revisar_stop():
            mover(pin_a, pin_b, 0)
            return False
        if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
            mover(pin_a, pin_b, 0)
            resp("ERR TIMEOUT_%s_%s" % (nombre, sentido))
            return False
        time.sleep_ms(5)
    mover(pin_a, pin_b, 0)
    return True

def estirar(sentido, fc):
    return hasta_final(EST_A, EST_B, sentido, fc, TO_ESTIRAMIENTO, "ESTIRAMIENTO")

def actuador(sentido, fc):
    return hasta_final(ACT_A, ACT_B, sentido, fc, TO_ACTUADOR, "ACTUADOR")

def paso_almacen(sentido):
    mover(ALM_A, ALM_B, sentido)
    if not espera(T_SALIDA_IR):
        mover(ALM_A, ALM_B, 0)
        return False
    t0 = time.ticks_ms()
    while not almacen_en_posicion():
        if revisar_stop():
            mover(ALM_A, ALM_B, 0)
            return False
        if time.ticks_diff(time.ticks_ms(), t0) > TO_ALMACEN:
            mover(ALM_A, ALM_B, 0)
            resp("ERR TIMEOUT_ALMACEN_%s" % sentido)
            return False
        time.sleep_ms(20)
    mover(ALM_A, ALM_B, 0)
    return True

def banda_atras(ms):
    mover(BANDA_A, BANDA_B, 'A')
    ok = espera(ms)
    mover(BANDA_A, BANDA_B, 0)
    return ok

# ---------------- PLATAFORMA Y GARRA ----------------
def arrancar(pwm_on, pwm_off):
    pwm_off.duty(0)
    pwm_on.duty(PWM_ARRANQUE)
    time.sleep_ms(T_ARRANQUE)
    pwm_on.duty(VELOCIDAD)

def mover_hasta_fc(pwm_on, pwm_off, fc, nombre):
    if presionado(fc):
        return True
    arrancar(pwm_on, pwm_off)
    t0 = time.ticks_ms()
    while not presionado(fc):
        if revisar_stop():
            parar_motor()
            return False
        if time.ticks_diff(time.ticks_ms(), t0) > TO_MOTOR:
            parar_motor()
            resp("ERR TIMEOUT_%s" % nombre)
            return False
        time.sleep_ms(5)
    parar_motor()
    return True

def bajar_tiempo(ms):
    arrancar(MOT_B, MOT_A)
    ok = espera(ms)
    parar_motor()
    return ok

def angulo(pwm, grados):
    grados = max(0, min(180, grados))
    us = US_MIN + (grados / 180) * (US_MAX - US_MIN)
    pwm.duty_ns(int(us * 1000))

def mover_servo(pwm, desde, hasta):
    """Movimiento gradual: los MG995 no aguantan saltos grandes."""
    paso = PASO_SERVO if hasta > desde else -PASO_SERVO
    g = desde
    while (paso > 0 and g < hasta) or (paso < 0 and g > hasta):
        g += paso
        angulo(pwm, g)
        time.sleep_ms(T_PASO_SERVO)
    angulo(pwm, hasta)

def rutina_garra():
    resp("EVT GARRA INICIO")
    angulo(GARRA, GARRA_ABIERTA)
    angulo(PLATAFORMA, PLAT_ARRIBA)
    if not espera(1000):                                   return False

    if not mover_hasta_fc(MOT_A, MOT_B, FC_ARRIBA, "FC_ARRIBA"): return False
    if not bajar_tiempo(T_BAJADA):                         return False

    angulo(GARRA, GARRA_CERRADA)
    if not espera(1000):                                   return False

    mover_servo(PLATAFORMA, PLAT_ARRIBA, PLAT_ABAJO)

    if not mover_hasta_fc(MOT_B, MOT_A, FC_ABAJO, "FC_ABAJO"):   return False

    angulo(GARRA, GARRA_ABIERTA)
    if not espera(1000):                                   return False

    if not mover_hasta_fc(MOT_A, MOT_B, FC_ARRIBA, "FC_ARRIBA"): return False

    mover_servo(PLATAFORMA, PLAT_ABAJO, PLAT_ARRIBA)
    resp("EVT GARRA FIN")
    return True

# ---------------- SECUENCIA: RECOLECCION ----------------
def recolectar(n):
    global productos
    for i in range(n):
        resp("EVT CICLO %d/%d" % (i + 1, n))
        if not estirar('A', FC_EST_A2):     return False
        if not espera(T_ESPERA_EXT):        return False
        if not estirar('B', FC_EST_B):      return False
        if not banda_atras(T_BANDA_ATRAS):  return False
        if not espera(T_ANTES_ALMACEN):     return False
        if not paso_almacen('B'):           return False
        productos += 1
        resp("EVT ALMACENADO %d" % productos)
        if not espera(T_ENTRE_CICLOS):      return False
    return True

# ---------------- SECUENCIA: ENTREGA ----------------
def entregar(n):
    global productos
    if n > 3:
        pasos, sentido = n - 3, 'A'
    elif n < 3:
        pasos, sentido = 3 - n, 'B'
    else:
        pasos, sentido = 0, 'A'

    resp("EVT REPOSICION %d %s" % (pasos, sentido))
    for i in range(pasos):
        if not paso_almacen(sentido):       return False

    for i in range(n):
        resp("EVT PRODUCTO %d/%d" % (i + 1, n))
        if not actuador('A', FC_ACT_A):     return False
        if not espera(T_ACT_EXTENDIDO):     return False
        if not actuador('B', FC_ACT_B):     return False
        if not rutina_garra():              return False
        if productos > 0:
            productos -= 1
        resp("EVT ENTREGADO %d" % (i + 1))
        if i < n - 1:
            if not paso_almacen('B'):       return False
        if not espera(T_ENTRE_PRODUCTOS):   return False
    return True

# ---------------- DESPACHADOR ----------------
def ejecutar(nombre, funcion, n):
    global ocupado, abortar
    if ocupado:
        resp("ERR OCUPADO")
        return
    if n < 1 or n > MAX_PRODUCTOS:
        resp("ERR CANTIDAD_INVALIDA")
        return
    ocupado = True
    abortar = False
    resp("ACK %s %d" % (nombre, n))
    try:
        ok = funcion(n)
    finally:
        parar_todo()
        ocupado = False
    if abortar:
        resp("ERR ABORTADO")
    elif ok:
        resp("OK %s %d" % (nombre, n))
    else:
        resp("ERR FALLO %s" % nombre)

def procesar(linea):
    cmd = linea.upper()
    if cmd == "STOP":
        parar_todo()
        resp("OK STOP")
    elif cmd == "EST?":
        resp("EST productos=%d ocupado=%d" % (productos, 1 if ocupado else 0))
    elif cmd == "RESET":
        parar_todo()
        resp("OK RESET")
        time.sleep_ms(200)
        import machine
        machine.reset()
    elif cmd.startswith("REC:"):
        try:    ejecutar("REC", recolectar, int(cmd[4:]))
        except: resp("ERR FORMATO")
    elif cmd.startswith("ENT:"):
        try:    ejecutar("ENT", entregar, int(cmd[4:]))
        except: resp("ERR FORMATO")
    else:
        resp("ERR COMANDO_DESCONOCIDO")

# ---------------- MAIN ----------------