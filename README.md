# Robot móvil autónomo con navegación LiDAR y gestión centralizada de pedidos

Proyecto de grado — Ingeniería en Control y Automatización
Universidad Distrital Francisco José de Caldas · Facultad Tecnológica

Robot móvil autónomo para la recolección, transporte y despacho de productos líquidos
en la planta pasteurizadora del laboratorio de control y automatización (edificio
Techne, sexto piso). El vehículo navega mediante LiDAR 2D con SLAM y fusión sensorial
inercial, y se integra a un sistema centralizado de gestión de pedidos con base de
datos en tiempo real.

**Autores:** Diego Alejandro Tunjo Moreno · Andrés Felipe Mosquera Ardila
**Director:** Jorge Eduardo Porras Bohada
**Modalidad:** Monografía

---

## Estado del proyecto

| Subsistema | Estado |
|---|---|
| Percepción (LiDAR, IMU, encoders, ultrasonidos) | Funcional |
| Firmware embebido ESP32 | Funcional |
| Puente serie ESP32 ↔ ROS 2 | Funcional |
| Fusión sensorial (EKF) | Funcional |
| SLAM y construcción de mapas | Funcional |
| Aplicación web y base de datos | Funcional (independiente) |
| Integración robot ↔ base de datos | En desarrollo |
| Mecanismo de almacenamiento y despacho | En desarrollo |
| Planificación y navegación autónoma punto a punto | Pendiente |
| Control de velocidad en lazo cerrado | Deshabilitado (ver limitaciones) |

---

## Arquitectura general

El sistema se organiza en tres bloques independientes que se comunican entre sí:

```
┌──────────────────────┐     HTTP/JSON      ┌──────────────────────┐
│  Aplicación web      │ ─────────────────► │  API REST (FastAPI)  │
│  React + Material-UI │ ◄───────────────── │  + SQLAlchemy        │
└──────────────────────┘                    └──────────┬───────────┘
                                                       │
                                            ┌──────────▼───────────┐
                                            │  MySQL               │
                                            │  BD «pasteurizadora» │
                                            └──────────┬───────────┘
                                                       │ pymysql
                                            ┌──────────▼───────────┐
                                            │  Raspberry Pi 4      │
                                            │  Ubuntu 24.04        │
                                            │  ROS 2 Jazzy         │
                                            └──────────┬───────────┘
                                                       │ USB serie
                                            ┌──────────▼───────────┐
                                            │  ESP32 (MicroPython) │
                                            │  Sensores + motores  │
                                            └──────────────────────┘
```

La frontera entre la Raspberry Pi y el ESP32 sigue un criterio simple: **todo lo que
deba ocurrir con periodicidad garantizada vive en el ESP32** (interrupciones de
encoder, disparo de ultrasonidos, PWM, lectura I2C de la IMU); **todo lo que requiera
memoria o cómputo vive en la Pi** (fusión sensorial, SLAM, planificación, red).

---

## 1. Hardware

### Componentes

| Subsistema | Componente | Función |
|---|---|---|
| Cómputo | Raspberry Pi 4 | ROS 2, fusión sensorial, mapeo, red |
| Cómputo | ESP32 | Firmware embebido, adquisición y control |
| Percepción | LiDAR 2D LDROBOT STL-19P | Barrido láser para mapeo |
| Percepción | IMU MPU6050 (I2C) | Aceleración lineal y velocidad angular |
| Percepción | Encoders ópticos monocanal ×2 | Estimación de velocidad por costado |
| Percepción | HC-SR04 ×3 | Obstáculos fuera del plano de barrido |
| Tracción | Motores DC con reductor ×4 | Configuración diferencial |
| Tracción | Puentes H L298N ×2 | Etapa de potencia |
| Energía | Packs LiPo 4S con BMS ×2 | Alimentación separada |
| Energía | Convertidores reductores | 5.2 V para cómputo, rail auxiliar para LiDAR |
| Comunicación | Adaptador USB-UART CP2102 | Enlace del LiDAR |

Chasis de **dos niveles y planta cuadrada**. Nivel inferior: motores, baterías y
puentes H (centro de gravedad bajo). Nivel superior: Pi, ESP32, convertidores, mástil
del LiDAR y espacio reservado para la plataforma rotatoria.

### Arquitectura de alimentación

```
Pack A (4S LiPo + BMS) ──► Buck 5.2 V ──► Raspberry Pi 4 + ESP32
                       └─► Buck auxiliar ──► LiDAR STL-19P

Pack B (4S LiPo + BMS) ──► L298N ×2 ──► Motores DC ×4

                    ─── P− común entre ambos ramales ───
```

**Tres reglas que no se pueden saltar:**

1. **Tierra común obligatoria.** Las señales PWM del ESP32 están referidas a su masa;
   sin referencia compartida los puentes H leen niveles lógicos incorrectos.
2. **LiDAR en rail separado.** Se corta el cable VBUS del adaptador CP2102 y se
   conservan solo D+/D−/GND. Alimentado desde el USB de la Pi provoca caída de
   tensión y limitación de frecuencia del procesador.
3. **Ajustar 5.2 V midiendo en el conector USB-C de la Pi bajo carga completa**, no en
   los bornes del convertidor. La caída en el cable es apreciable.

---

## 2. Firmware ESP32 (MicroPython)

Lazo principal de periodo fijo que en cada iteración lee la IMU, calcula velocidad por
costado, actualiza ultrasonidos, atiende comandos serie, aplica la consigna y transmite
una trama de estado completa.

### Adquisición

- **IMU MPU6050.** Lectura del bloque de registros en una única transacción I2C (leer
  registro por registro devuelve ejes de instantes distintos). Correcciones de bias y
  escala por eje aplicadas sobre el valor convertido.
- **Calibración del giróscopo en el arranque.** Promediado con el vehículo inmóvil,
  precedido de ~2 s de asentamiento. El robot **debe estar quieto** durante este
  intervalo o el movimiento se resta permanentemente de todas las lecturas.
- **Encoders.** Conteo en rutinas de interrupción (nunca por muestreo en el lazo).
  Antirrebote por software de **15 ms**. Constantes de pulsos medidas físicamente:
  `IZQ = 16.1`, `DER = 28.1`.
- **Ultrasonidos.** Disparo **secuencial**, nunca simultáneo (el eco de un sensor lo
  recibiría otro). Timeout de eco; las lecturas fallidas se marcan como no válidas.

### Control

- Conversión de `(v, ω)` a consigna por costado mediante el modelo diferencial.
- Umbral mínimo de PWM para evitar la zona muerta del reductor.
- **Watchdog de comunicación:** sin trama válida durante un intervalo, los motores se
  llevan a cero automáticamente.
- **Lazo cerrado deshabilitado** (`USAR_LAZO_CERRADO = False`). Ver limitaciones.

### Protocolo serie

Texto plano sobre USB, tramas delimitadas por salto de línea. Se eligió texto sobre
binario por depurabilidad: cualquier terminal serie sirve para inspeccionarlo.

| Sentido | Contenido |
|---|---|
| ESP32 → Pi | Aceleración (3 ejes), velocidad angular (3 ejes), desplazamiento y velocidad por costado, 3 distancias ultrasónicas, marca de tiempo `t_ms` |
| Pi → ESP32 | Velocidad lineal y velocidad angular |

La marca de tiempo del ESP32 permite descartar tramas retrasadas y calcular `dt`
correctamente. **No usar el reloj de la Pi:** cuando las tramas llegan en ráfaga
produce `dt = 0`.

---

## 3. Stack ROS 2

**Plataforma:** Raspberry Pi 4 · Ubuntu 24.04 (Noble) ARM64 · ROS 2 Jazzy
**Workspace:** `~/ros2_ws/`
**Paquete propio:** `carro_bringup` (ament_python)

### Tópicos

| Tópico | Tipo | Productor | Consumidor |
|---|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` | `esp32_bridge_node` | EKF |
| `/wheel/odometry` | `nav_msgs/Odometry` | `esp32_bridge_node` | EKF |
| `/ultrasonic/*` | `sensor_msgs/Range` | `esp32_bridge_node` | capa de seguridad |
| `/scan` | `sensor_msgs/LaserScan` | `ldlidar_stl_ros2` | `slam_toolbox` |
| `/odometry/filtered` | `nav_msgs/Odometry` | `robot_localization` | `slam_toolbox` |
| `/map` | `nav_msgs/OccupancyGrid` | `slam_toolbox` | planificador |
| `/cmd_vel` | `geometry_msgs/Twist` | teleop / planificador | `esp32_bridge_node` |

**Detalles que importan:**

- En `/imu/data` la orientación se marca **no disponible** poniendo el primer elemento
  de su covarianza en negativo. Si se deja un cuaternión identidad sin marcar, el EKF
  lo toma como medición real y tira de la estimación hacia orientación nula.
- De `/wheel/odometry` **solo se usa `twist.linear.x`**. La pose se descarta con
  covarianza alta: la pose integra el error de los encoders, la velocidad no.
- La velocidad angular **no** se deriva de la diferencia entre costados. Las cuatro
  ruedas fijas deslizan lateralmente al girar y la diferencia sobrestima el giro real.
  El yaw rate sale íntegro de la IMU.

### Fusión sensorial (`robot_localization`)

EKF a **20 Hz**, modo 2D.

| Fuente | Variable usada | Descartado y por qué |
|---|---|---|
| `/wheel/odometry` | `vx` | Pose (deriva), `vyaw` (deslizamiento) |
| `/imu/data` | `vyaw` | Orientación (no disponible), aceleraciones (integrarlas añade deriva) |

### Árbol de transformadas

```
map ──(slam_toolbox)──► odom ──(EKF)──► base_link ──┬─► laser_frame
                                                    ├─► imu_link
                                                    └─► ultrasonic_*_link
```

`odom → base_link` es continua pero deriva; `map → odom` da saltos pero no acumula
error. El planificador local trabaja en `odom`, el global en `map`. **Solo el EKF
publica `odom → base_link`** — la publicación duplicada desde el nodo puente produce
oscilaciones en la pose.

### Archivos de lanzamiento

```bash
# Capa baja: puente serie + EKF. Para pruebas de tracción y odometría.
ros2 launch carro_bringup bringup.launch.py

# Pila completa: lo anterior + driver LiDAR + TF estáticas + slam_toolbox
ros2 launch carro_bringup slam.launch.py
```

El orden de arranque importa: el puente debe publicar antes de que arranque el EKF, y
el EKF debe publicar `odom → base_link` antes de que `slam_toolbox` se active.

---

## 4. Aplicación web — Sistema de Gestión Pasteurizadora

Aplicación SPA para centralizar la operación comercial: clientes, catálogo de
productos, pedidos y usuarios, con control de acceso por roles y autenticación JWT.

### Stack

| Capa | Tecnologías |
|---|---|
| Presentación | React 19.2, Material-UI 9.2, Vite 8.1, React Router 7.18, Axios 1.18, React Hook Form, SweetAlert2 |
| Enrutamiento | FastAPI 0.139, Uvicorn 0.50 |
| Lógica de negocio | Services + Pydantic 2.13 |
| Acceso a datos | SQLAlchemy 2.0, PyMySQL 1.2 |
| Persistencia | MySQL 5.7+ |
| Seguridad | Python-Jose 3.5 (JWT), Bcrypt 5.0, Passlib 1.7 |

### Modelo de datos

Cinco tablas normalizadas:

- **`usuarios`** — control de acceso, independiente del resto
  (`id_usuario`, `usuario`, `password`, `rol`, `email`, `activo`, `fecha_creacion`,
  `ultimo_login`, `intentos_fallidos`)
- **`clientes`** — `id_cliente`, `nombre`, `apellido`, `email` (único), `telefono`,
  `direccion`, `fecha_registro`
- **`productos`** — `id_producto`, `nombre`, `descripcion`, `precio`, `fecha_creacion`
- **`pedidos`** — `id_pedido`, `id_cliente` (FK), `fecha_pedido`, `estado`, `total`
- **`detalle_pedidos`** — `id_detalle`, `id_pedido` (FK), `id_producto` (FK),
  `cantidad`, `precio_unitario`

**Relaciones:** `clientes 1─N pedidos`, `pedidos 1─N detalle_pedidos`,
`productos 1─N detalle_pedidos`. La relación N:M entre pedidos y productos se resuelve
mediante `detalle_pedidos`.

`detalle_pedidos.precio_unitario` no es redundante con `productos.precio`: congela el
precio en el momento de la operación para que cambiar el catálogo no altere
retroactivamente pedidos ya registrados.

### Seguridad

- JWT firmado con HS256, vigencia de 60 minutos, enviado en `Authorization: Bearer`
- Contraseñas con bcrypt (12 rondas)
- Rate limiting: 5 intentos fallidos → bloqueo de 15 minutos
- CORS restringido a los orígenes declarados
- RBAC con dos roles:

| Rol | Permisos |
|---|---|
| `ADMIN` | Acceso total, incluida gestión de usuarios |
| `CONSULTA` | Solo lectura sobre clientes, productos y pedidos |

La verificación de rol se hace **en el servidor**, no solo ocultando opciones del menú.

### API REST

| Método | Endpoint | Función |
|---|---|---|
| `POST` | `/auth/login` | Autenticación; devuelve el token |
| `GET/POST/PUT/DELETE` | `/clientes[/{id}]` | CRUD de clientes |
| `GET/POST/PUT/DELETE` | `/productos[/{id}]` | CRUD de productos |
| `GET/POST/PUT/DELETE` | `/pedidos[/{id}]` | CRUD de pedidos con detalle |
| `POST/PUT/DELETE` | `/usuarios[/{id}]` | Gestión de usuarios (solo `ADMIN`) |
| `GET` | `/` | Verificación de servicio activo |
| `GET` | `/db` | Verificación de conexión a la BD |

Todos requieren token válido excepto `/auth/login`.

### Punto de integración con el robot

El campo **`pedidos.estado`** es el enlace entre el mundo digital y el físico:

```
pendiente ──► el robot toma la misión ──► transporte ──► entregado
```

El acceso desde la Pi se hará con `pymysql` en **hilo dedicado con cola**, igual que el
puente serie. Una consulta a BD remota puede tardar un tiempo indeterminado si el WiFi
se degrada, y esa espera nunca debe bloquear el executor de ROS 2.

---

## 5. Estructura de directorios

```
proyecto/
├── ros2_ws/
│   └── src/
│       └── carro_bringup/              # paquete ament_python
│           ├── carro_bringup/
│           │   └── esp32_bridge_node.py
│           ├── launch/
│           │   ├── bringup.launch.py
│           │   └── slam.launch.py
│           └── config/
│               ├── ekf.yaml
│               └── slam_toolbox.yaml
│
├── firmware/                           # MicroPython para el ESP32
│
└── AplicacionBD/
    ├── backend/
    │   ├── app/
    │   │   ├── auth/          core/         database/
    │   │   ├── dependencies/  models/       repositories/
    │   │   ├── routers/       schemas/      services/
    │   │   └── main.py
    │   ├── requirements.txt
    │   └── create_admin.py
    └── frontend/
        └── src/
            ├── api/  auth/  components/  pages/  services/
            ├── App.jsx
            └── main.jsx
```

Correspondencia sistemática en el backend: cada entidad tiene modelo, repositorio,
esquema, servicio y router. En el frontend, página y servicio.

---

## 6. Puesta en marcha

### Robot

```bash
# 1. Verificar tensión de ambos packs antes de encender

# 2. Alimentar primero el ramal de cómputo y esperar el arranque

# 3. Comprobar que no hay limitación por bajo voltaje
vcgencmd get_throttled          # 0x0 = correcto
vcgencmd measure_volts core

# 4. Poner el robot QUIETO sobre superficie plana y alimentar el ramal de potencia
#    (los primeros segundos calibran el bias del giróscopo)

# 5. Verificar que los dispositivos fueron reconocidos
ls -l /dev/serial/by-id/

# 6. Capa baja y verificación de tópicos
ros2 launch carro_bringup bringup.launch.py
ros2 topic hz /imu/data
ros2 topic echo /wheel/odometry --once

# 7. Prueba de teleoperación
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 8. Pila completa
ros2 launch carro_bringup slam.launch.py
ros2 topic hz /scan
ros2 lifecycle get /slam_toolbox        # debe estar 'active'

# 9. Recorrer el perímetro antes que el interior (favorece el cierre de lazo)

# 10. Guardar el mapa antes de detener el sistema
ros2 service call /slam_toolbox/serialize_map ...
```

### Aplicación web

```bash
# Backend
cd AplicacionBD/backend
pip install -r requirements.txt
python create_admin.py                  # crea el usuario administrador inicial
uvicorn app.main:app --reload

# Frontend
cd AplicacionBD/frontend
npm install
npm run dev
```

Configurar en los archivos `.env` la cadena de conexión a MySQL, la `SECRET_KEY` de
firma de tokens, los orígenes permitidos por CORS y la URL base de la API. **Los `.env`
no deben versionarse.**

---

## 7. Limitaciones conocidas

| Limitación | Impacto | Mitigación |
|---|---|---|
| Encoders monocanal | No hay información de sentido de giro; sensibles al rebote | Antirrebote de 15 ms; lazo cerrado deshabilitado; odometría solo como entrada del EKF |
| Asimetría entre costados | `IZQ=16.1` vs `DER=28.1`; el robot deriva en recta | Yaw rate tomado íntegro de la IMU |
| Deslizamiento lateral | 4 ruedas fijas obligan a deslizar en los giros | No se usa la odometría diferencial para el giro |
| Sin referencia absoluta de orientación | El yaw deriva a largo plazo | Brújula magnética prevista, no integrada aún |
| LiDAR 2D | No ve obstáculos fuera del plano de barrido | Ultrasonidos como capa complementaria |
| Ethernet dañado en la Pi | El PHY vive pero reporta `Link detected: no` | WiFi como único camino de red |
| RDP de GNOME sin monitor | Requiere salida de vídeo activa | Dummy plug HDMI |
| VNC + XFCE headless | Falla con `cannot open display: wayland-0` por la sesión GNOME concurrente | Sin resolver |
| Planta cuadrada | La envolvente al girar depende de la orientación | Declarar la huella como polígono, no como radio único |

---

## 8. Gotchas — errores que ya costaron tiempo

### Firmware

- **Antirrebote inicializado en cero.** `ticks_diff(now, 0)` devuelve un valor negativo
  grande y desactiva el filtro permanentemente. Inicializar con `time.ticks_us()`.
- **`dt` con el reloj de la Pi.** Cuando las tramas llegan en ráfaga da `dt = 0`. Usar
  el campo `t_ms` que envía el ESP32.

### ROS 2

- **`readline()` bloqueante en un hilo** muere en silencio si se comprueba `rclpy.ok()`
  antes de que ROS termine de inicializar. Usar un timer con lecturas no bloqueantes
  basadas en `in_waiting`.
- **`slam_toolbox` en Jazzy es un `LifecycleNode`.** Sin `TRANSITION_CONFIGURE` y
  `TRANSITION_ACTIVATE` explícitos en el launch queda creado pero inerte: todos los
  procesos aparecen corriendo y no pasa nada.
- **YAML de `slam_toolbox`:** la clave raíz debe coincidir exactamente con el nombre del
  nodo (`slam_toolbox:`), y `ceres_loss_function` debe ser la cadena entrecomillada
  `"None"`, no `None` suelto.
- **Driver `ldlidar`:** el STL-19P necesita el perfil `LDLiDAR_LD19`, y hay que añadir
  a mano `#include <pthread.h>` en `log_module.cpp` para que compile con GCC en Noble.
- **`frame_id` del LiDAR** debe coincidir con el de la TF estática. Si no coincide,
  `slam_toolbox` descarta todos los barridos y el mapa queda vacío aunque `/scan`
  publique bien.
- **TF duplicada.** Si el nodo puente y el EKF publican ambos `odom → base_link`, la
  pose oscila entre las dos fuentes.

### Energía y baterías

- **El USB-C de la Pi 4 trabaja a 5.1 V.** Los periféricos suben la corriente, no la
  tensión, y hunden el rail. Aislar los de alto consumo (arranque del motor del LiDAR,
  picos de WiFi del ESP32) en un rail aparte.
- **Celdas con capacidad implausible** (p. ej. 6800 mAh en formato 18650) son
  falsificadas o recicladas.
- **Corriente de reposo del BMS** drena de la celda más baja del stack y desbalancea
  durante el almacenamiento. Desconectar a nivel de celda si se guarda más de dos
  semanas.

### Recuperación

- **Corrupción del sistema de archivos** tras apagones bruscos: recuperable con
  `fsck.ext4 -y /dev/mmcblk0p2` desde el shell de emergencia de BusyBox, al que se
  llega quitando `quiet splash` de `cmdline.txt`.

---

## 9. Trabajo pendiente

- [ ] Integración `pymysql` desde la Pi con hilo dedicado y cola
- [ ] Mecanismo de almacenamiento y despacho (plataforma rotatoria de 6
      compartimientos + actuador lineal)
- [ ] Planificación de trayectorias y navegación autónoma punto a punto
- [ ] Monitoreo de batería vía ADC del ESP32 publicando `/battery_state`, con aviso
      alrededor de 13 V antes del corte del BMS
- [ ] Sustituir los encoders por unidades en cuadratura y reactivar el lazo cerrado
- [ ] Integrar la brújula magnética en el EKF para acotar la deriva de yaw
- [ ] Caracterización cuantitativa: error de localización, tiempo de entrega,
      eficiencia de rutas, tasa de éxito en misiones completas
- [ ] Resolver el escritorio remoto headless
- [ ] `connection.autoconnect-priority` en el perfil WiFi para que la Pi prefiera la
      red conocida al arrancar

---

## 10. Herramientas de diagnóstico en uso

```bash
vcgencmd get_throttled           # limitación por voltaje o temperatura
vcgencmd measure_volts core      # tensión del núcleo
arp-scan --localnet              # descubrimiento de equipos en la red
ros2 topic hz <tópico>           # frecuencia real de publicación
ros2 lifecycle get <nodo>        # estado de un nodo gestionado
ros2 run tf2_tools view_frames   # inspección del árbol de transformadas
```

Metodología de depuración: siempre en capas, **hardware → firmware → ROS 2**. No se
sube de capa hasta descartar la inferior.

---

## Licencia y uso

Trabajo académico desarrollado en el marco del proyecto curricular de Ingeniería en
Control y Automatización de la Universidad Distrital Francisco José de Caldas. El
prototipo y su documentación quedan disponibles como plataforma didáctica para
prácticas y trabajos de grado posteriores.
