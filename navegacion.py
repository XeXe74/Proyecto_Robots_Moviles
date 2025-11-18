import math
import asyncio
import heapq
from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import Create3, event
from irobot_edu_sdk.music import Note

# ---------------- CONFIGURACIÓN Y CONSTANTES (AJUSTADAS) ----------------
k_att = 1.0  # Coeficiente para la fuerza de atracción (normalizada)
k_rep = 1500.0  # Coeficiente para la fuerza de repulsión (reducido)
d0 = 50.0  # Distancia de influencia de los obstáculos (reducida)
k_tangent = 0.5
v_crucero = 30.0
v_minima = 8.0
t_rampa = 5.0
tolerancia = 10.0
dt = 0.1
radio_giro_minimo = 20.0
factor_suavizado_giro = 2.0
tiempo_chequeo = 3.0
umbral_progreso = 5.0
max_intentos_minimo = 3

robot = Create3(Bluetooth("C3_UIEC_Grupo2"))

# ---------------- VARIABLES GLOBALES PARA LA NAVEGACIÓN ----------------
x_goal, y_goal = 0, 0

# --- PUNTOS DE INTERÉS ---
PUNTOS_INTERES = {
    "Despacho y Baños": (-496.70, -572.90, 118.70),
    "Punto Columnas": (-78.20, -333.70, 101.30),
    "Garaje": (-324.50, -55.40, 13.50),
    "Escaleras 1": (-298.80, -578.00, 178.50),
    "Pasillo Inicio": (324.00, -133.80, -19.20),
    "Entrada": (883.10, -503.20, 257.10),
    "Ascensores": (211.90, -477.70, 172.60),
    "Administración": (347.90, 269.60, 93.90),
    "Sala 06": (343.10, 1645.80, 1.90),
    "Punto Pousa": (314.10, 2235.70, 3.00),
    "Pousa": (198.90, 2268.70, 74.40),
    "Escaleras 2": (335.60, 2547.80, 12.60),
    "Estación Carga": (-350.00, -50.00, 0.00)
}

# --- GRAFO DE ADYACENCIA ---
GRAFO = {
    "Estación Carga": ["Pasillo Inicio"],
    "Entrada": ["Pasillo Inicio"],
    "Pasillo Inicio": ["Entrada", "Ascensores", "Punto Columnas", "Administración", "Estación Carga"],
    "Ascensores": ["Pasillo Inicio", "Punto Columnas"],
    "Punto Columnas": ["Pasillo Inicio", "Escaleras 1", "Despacho y Baños", "Garaje"],
    "Escaleras 1": ["Punto Columnas"],
    "Despacho y Baños": ["Punto Columnas"],
    "Garaje": ["Punto Columnas"],
    "Administración": ["Pasillo Inicio", "Sala 06"],
    "Sala 06": ["Administración", "Punto Pousa"],
    "Punto Pousa": ["Sala 06", "Pousa", "Escaleras 2"],
    "Pousa": ["Punto Pousa"],
    "Escaleras 2": ["Punto Pousa"]
}

# ---------------- FUNCIONES DE GRAFO Y NAVEGACIÓN ----------------

def calcular_distancia(p1_nombre, p2_nombre):
    """Calcula la distancia euclidiana entre dos puntos (en cm)."""
    x1, y1, _ = PUNTOS_INTERES[p1_nombre]
    x2, y2, _ = PUNTOS_INTERES[p2_nombre]
    return math.hypot(x2 - x1, y2 - y1)


def dijkstra(grafo, punto_inicio, punto_final):
    """
    Encuentra el camino más corto entre dos nodos usando el algoritmo de Dijkstra.
    Retorna la lista de nodos a visitar en orden (el camino).
    """
    cola_prioridad = [(0, punto_inicio, [punto_inicio])]
    distancias = {nodo: float('inf') for nodo in grafo}
    distancias[punto_inicio] = 0

    while cola_prioridad:
        dist_actual, nodo_actual, camino_actual = heapq.heappop(cola_prioridad)

        if dist_actual > distancias[nodo_actual]:
            continue

        if nodo_actual == punto_final:
            return camino_actual

        for vecino in grafo[nodo_actual]:
            peso = calcular_distancia(nodo_actual, vecino)
            distancia_nueva = dist_actual + peso

            if distancia_nueva < distancias[vecino]:
                distancias[vecino] = distancia_nueva
                camino_nuevo = camino_actual + [vecino]
                heapq.heappush(cola_prioridad, (distancia_nueva, vecino, camino_nuevo))

    return None


def distancia_sensor(valor):
    try:
        return 632.19 / valor + 5.97 if valor > 0 else 999
    except Exception:
        return 999


async def get_pos(robot):
    pos = await robot.get_position()
    if pos is None:
        return None, None, None
    return pos.x, pos.y, pos.heading


def fuerza_atractiva(x, y):
    """
    Fuerza de atracción con magnitud constante (normalizada).
    Esto evita que la fuerza sea demasiado grande cuando el objetivo está lejos.
    """
    dx = x_goal - x
    dy = y_goal - y
    dist = math.hypot(dx, dy)
    if dist > 0:
        return (k_att * dx / dist, k_att * dy / dist)
    return (0, 0)


def fuerza_repulsiva(x, y, dist_ir, heading):
    """
    Fuerza de repulsión ajustada.
    Se ha reducido k_rep y d0 para un comportamiento menos "asustadizo".
    """
    Fx, Fy = 0, 0
    angulos = [-60, -40, -20, 0, 20, 40, 60]
    pesos = [0.7, 0.9, 1.0, 1.3, 1.0, 0.9, 0.7]

    for i, val in enumerate(dist_ir):
        d = distancia_sensor(val)
        if d < d0:
            mag = k_rep * (1 / d - 1 / d0) / (d ** 2)
            mag = min(mag, 10.0)

            ang_global = math.radians((heading + angulos[i]) % 360)

            Fx_norm = -pesos[i] * mag * math.cos(ang_global)
            Fy_norm = -pesos[i] * mag * math.sin(ang_global)

            Fx_tang = -Fy_norm * k_tangent
            Fy_tang = Fx_norm * k_tangent

            Fx += Fx_norm + Fx_tang
            Fy += Fy_norm + Fy_tang

    mag_rep = math.hypot(Fx, Fy)
    if mag_rep > 15.0:
        Fx = (Fx / mag_rep) * 15.0
        Fy = (Fy / mag_rep) * 15.0

    return Fx, Fy


def rampa_aceleracion(t, v_crucero, t_rampa):
    """Función de aceleración gradual."""
    if t < t_rampa:
        return v_crucero * (t / t_rampa)
    return v_crucero


def freno_suave(distancia_obj, distancia_meta, v_actual, min_dist_obj=40, rango_meta=50):
    """
    Freno suave mejorado.
    Reduce la velocidad de forma más progresiva al acercarse a un obstáculo o al destino.
    """
    v_frenada = v_actual
    if distancia_obj < min_dist_obj:
        factor_obj = max(0.1, distancia_obj / min_dist_obj)
        v_frenada = v_actual * factor_obj

    if distancia_meta < rango_meta:
        factor_meta = max(0.2, distancia_meta / rango_meta)
        v_frenada = min(v_frenada, v_actual * factor_meta)

    return v_frenada


def calcular_velocidades_ackermann(v_base, error_angular):
    v_efectiva = max(v_base, v_minima)
    error_angular = max(min(error_angular, 45), -45)
    factor_giro = abs(error_angular) / 45.0
    radio_deseado = radio_giro_minimo + (100 - radio_giro_minimo) * (1 - factor_giro) * factor_suavizado_giro
    L = 23.3

    if abs(error_angular) > 2:
        omega = v_efectiva / radio_deseado
        if error_angular < 0:
            omega = -omega
        v_left = v_efectiva - omega * (L / 2)
        v_right = v_efectiva + omega * (L / 2)
    else:
        v_left = v_efectiva
        v_right = v_efectiva

    velocidad_minima_rueda = v_minima * 0.3
    if v_left < velocidad_minima_rueda:
        v_left = velocidad_minima_rueda
    if v_right < velocidad_minima_rueda:
        v_right = velocidad_minima_rueda

    v_left = min(v_left, v_crucero)
    v_right = min(v_right, v_crucero)

    return v_left, v_right


async def escape_minimo_local_ackermann(robot, direccion='izquierda'):
    print(f" ESCAPE DE MÍNIMO LOCAL (estilo coche) - Girando a la {direccion}")
    duracion = 2.0
    pasos = int(duracion / dt)

    if direccion == 'izquierda':
        v_left = 8
        v_right = 20
    else:
        v_left = 20
        v_right = 8

    for _ in range(pasos):
        await robot.set_wheel_speeds(v_left, v_right)
        await asyncio.sleep(dt)

    await robot.set_wheel_speeds(20, 20)
    await asyncio.sleep(1.0)

    await robot.set_wheel_speeds(0, 0)
    print(" Escape completado, retomando navegación")
    await asyncio.sleep(0.5)


async def navegar_a_waypoint(robot):
    """
    Navega hacia el waypoint actual con una lógica de fuerzas y evasión mejorada.
    """
    global x_goal, y_goal

    tiempo_inicio = asyncio.get_event_loop().time()
    ang_prev = None
    modo_evasion = False
    tiempo_evasion = 0

    dist_meta_anterior = None
    tiempo_ultimo_chequeo = tiempo_inicio
    intentos_minimo = 0

    while True:
        x, y, heading = await get_pos(robot)

        if x is None:
            print("Error: No se pudo obtener la posición del robot.")
            await robot.set_wheel_speeds(0, 0)
            return False

        prox = (await robot.get_ir_proximity()).sensors
        dist_frente = distancia_sensor(prox[3])
        dist_meta = math.hypot(x_goal - x, y_goal - y)

        if dist_meta < tolerancia:
            await robot.set_wheel_speeds(0, 0)
            await robot.set_lights_rgb(0, 255, 0)
            await robot.play_note(Note.C5, 0.6)
            print(f"✅ Waypoint alcanzado en ({x_goal:.1f}, {y_goal:.1f})")
            await asyncio.sleep(1.0)
            return True

        # ============ DETECCIÓN DE MÍNIMO LOCAL ============
        tiempo_actual = asyncio.get_event_loop().time()
        if tiempo_actual - tiempo_ultimo_chequeo >= tiempo_chequeo:
            if dist_meta_anterior is not None:
                progreso = dist_meta_anterior - dist_meta
                if progreso < umbral_progreso:
                    intentos_minimo += 1
                    await robot.set_wheel_speeds(0, 0)
                    await asyncio.sleep(0.3)
                    direccion = 'izquierda' if intentos_minimo % 2 == 1 else 'derecha'
                    await escape_minimo_local_ackermann(robot, direccion)
                    tiempo_inicio = asyncio.get_event_loop().time()
                    tiempo_ultimo_chequeo = tiempo_inicio
                    dist_meta_anterior = None
                    ang_prev = None
                    if intentos_minimo >= max_intentos_minimo:
                        print("❌ Demasiados intentos de escape, abortando waypoint.")
                        return False
                    continue
                else:
                    intentos_minimo = 0
            dist_meta_anterior = dist_meta
            tiempo_ultimo_chequeo = tiempo_actual

        # ============ LÓGICA DE EVASIÓN REACTIVA ============
        if dist_frente < 30 and not modo_evasion:
            modo_evasion = True
            tiempo_evasion = asyncio.get_event_loop().time()

        if modo_evasion:
            tiempo_actual = asyncio.get_event_loop().time()
            if tiempo_actual - tiempo_evasion < 2.0:
                izquierda = distancia_sensor(prox[1])
                derecha = distancia_sensor(prox[5])
                if izquierda > derecha:
                    await robot.set_wheel_speeds(5, 20)
                else:
                    await robot.set_wheel_speeds(20, 5)
            elif tiempo_actual - tiempo_evasion < 3.0:
                await robot.set_wheel_speeds(20, 20)
            else:
                modo_evasion = False
            await asyncio.sleep(dt)
            continue

        # ============ CÁLCULO DE FUERZAS Y MOVIMIENTO ============
        Fx_att, Fy_att = fuerza_atractiva(x, y)
        Fx_rep, Fy_rep = fuerza_repulsiva(x, y, prox, heading)

        Fx_total = Fx_att + Fx_rep
        Fy_total = Fy_att + Fy_rep

        ang_obj = math.degrees(math.atan2(Fy_total, Fx_total))

        if ang_prev is None:
            ang_prev = ang_obj
        else:
            ang_obj = 0.8 * ang_prev + 0.2 * ang_obj
            ang_prev = ang_obj

        t_actual = asyncio.get_event_loop().time() - tiempo_inicio
        v_target = rampa_aceleracion(t_actual, v_crucero, t_rampa)

        v_target = freno_suave(dist_frente, dist_meta, v_target)

        error_ang = (ang_obj - heading + 540) % 360 - 180
        v_left, v_right = calcular_velocidades_ackermann(v_target, error_ang)

        await robot.set_wheel_speeds(v_left, v_right)
        await asyncio.sleep(dt)


# ---------------- BUCLE PRINCIPAL Y MENÚ ----------------
def mostrar_menu():
    print("\n================ GUÍA DE LA UNIVERSIDAD ================")
    opciones = [k for k in PUNTOS_INTERES.keys() if k != "Estación Carga"]
    for i, nombre in enumerate(opciones):
        print(f" {i + 1}: {nombre}")
    print(" 0: Salir")

    while True:
        try:
            seleccion = input("\nElige tu destino (número): ")
            num_seleccion = int(seleccion)

            if num_seleccion == 0:
                return None, None
            elif 1 <= num_seleccion <= len(opciones):
                nombre_destino = opciones[num_seleccion - 1]
                return nombre_destino, PUNTOS_INTERES[nombre_destino]
            else:
                print("Opción no válida. Inténtalo de nuevo.")
        except ValueError:
            print("Entrada no válida. Por favor, introduce un número.")


async def iniciar_navegacion(robot):
    """
    Controla el flujo de navegación: movimiento inicial, pide destino, calcula camino y lo recorre.
    """
    global x_goal, y_goal

    punto_actual = "Estación Carga"
    destino_inicial = "Entrada"

    print(f"\n🤖 Iniciando en: {punto_actual}")

    # 1. Movimiento inicial: salir de la estación de carga
    print("1. Saliendo de la estación de carga (retroceso forzado 4.0s)...")
    await robot.set_lights_rgb(255, 0, 0)
    await robot.set_wheel_speeds(-10, -10)
    await asyncio.sleep(4.0)
    await robot.set_wheel_speeds(0, 0)
    await asyncio.sleep(0.5)

    # 2. Alineación de Rumbo
    x_target, y_target, _ = PUNTOS_INTERES["Pasillo Inicio"]
    x, y, heading = await get_pos(robot)
    if x is not None:
        target_rad = math.atan2(y_target - y, x_target - x)
        target_deg = math.degrees(target_rad)
        error_deg = (target_deg - heading + 540) % 360 - 180

        print(f"  > Pos actual ({x:.1f}, {y:.1f}), Heading: {heading:.1f}°")
        print(f"  > Realizando giro inicial de {error_deg:.1f}° hacia Pasillo Inicio...")

        if abs(error_deg) > 5:
            await robot.set_lights_rgb(255, 255, 0)
            if error_deg > 0:
                await robot.turn_left(error_deg)
            else:
                await robot.turn_right(abs(error_deg))
            await asyncio.sleep(1.0)

        await robot.set_lights_rgb(0, 0, 255)

    # 3. Navegar de Estación Carga a Entrada
    print(f"3. Calculando ruta: {punto_actual} → {destino_inicial}")
    camino_inicial = dijkstra(GRAFO, punto_actual, destino_inicial)

    if not camino_inicial:
        print("ERROR: No se encontró un camino inicial. Abortando.")
        return

    print(f"✅ Camino inicial: {' → '.join(camino_inicial)}")

    for waypoint_nombre in camino_inicial[1:]:
        x_target, y_target, _ = PUNTOS_INTERES[waypoint_nombre]
        x_goal, y_goal = x_target, y_target

        print(f"\n▶️ Yendo a **{waypoint_nombre}**...")
        exito = await navegar_a_waypoint(robot)

        if not exito:
            print(f"⚠️ Fallo al alcanzar el waypoint {waypoint_nombre}. Abortando navegación inicial.")
            return

        punto_actual = waypoint_nombre

    print(f"\n🎉 ¡Llegada a **{punto_actual}**! Esperando instrucciones.")
    await robot.play_note(Note.A4, 0.5)

    # 4. Bucle de menú y navegación
    while True:
        destino_nombre, destino_coords = mostrar_menu()

        if destino_nombre is None:
            print("Guía finalizado. ¡Hasta pronto!")
            break

        if destino_nombre == punto_actual:
            await robot.play_note(Note.C3, 0.3)
            continue

        print(f"\nCalculando camino de **{punto_actual}** a **{destino_nombre}**...")

        camino = dijkstra(GRAFO, punto_actual, destino_nombre)

        if not camino:
            print("ERROR: No se encontró un camino entre los puntos.")
            await robot.set_lights_rgb(255, 0, 0)
            await robot.play_note(Note.C2, 1.0)
            continue

        print(f"✅ Camino encontrado: {' → '.join(camino)}")
        await robot.set_lights_rgb(0, 0, 255)

        for i, waypoint_nombre in enumerate(camino[1:]):
            x_target, y_target, _ = PUNTOS_INTERES[waypoint_nombre]
            x_goal, y_goal = x_target, y_target

            print(f"\n▶️ Yendo de **{punto_actual}** a **{waypoint_nombre}** ({i + 1}/{len(camino) - 1})...")

            exito = await navegar_a_waypoint(robot)

            if not exito:
                print(f"⚠️ Fallo al alcanzar el waypoint {waypoint_nombre}. Abortando navegación.")
                await robot.set_lights_rgb(255, 0, 0)
                await robot.play_note(Note.C2, 0.5)
                break

            punto_actual = waypoint_nombre

        if punto_actual == destino_nombre:
            print(f"\n🎉 ¡Llegada exitosa a **{destino_nombre}**!")
            await robot.set_lights_rgb(0, 255, 0)
            await robot.play_note(Note.G5, 0.5)

        await asyncio.sleep(2.0)


@event(robot.when_play)
async def play(robot):
    await robot.reset_navigation()
    await robot.set_lights_rgb(255, 192, 203)
    await asyncio.sleep(1.0)

    await iniciar_navegacion(robot)


robot.play()
