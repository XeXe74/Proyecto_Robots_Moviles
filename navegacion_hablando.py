import math
import asyncio
import heapq
import speech_recognition as sr
import pyttsx3
import time

from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import Create3, event
from irobot_edu_sdk.music import Note

# ---------------- CONFIGURACIÓN Y CONSTANTES ----------------
k_att = 1.0
k_rep = 3500.0
d0 = 100.0
k_tangent = 0.3
v_crucero = 30.0
v_minima = 8.0
t_rampa = 5.0
tolerancia = 10.0
dt = 0.05
radio_giro_minimo = 20.0
factor_suavizado_giro = 0.8
# tiempo_chequeo = 1.5  # ❌ COMENTADO - Ya no se usa
# umbral_progreso = 8.0  # ❌ COMENTADO - Ya no se usa
# max_intentos_minimo = 3  # ❌ COMENTADO - Ya no se usa
RADIO_PUNTO_INTERMEDIO = 15.0
UMBRAL_FRENTE_MINIMO = 35.0

robot = Create3(Bluetooth("C3_UIEC_Grupo2"))

# --- MOTOR DE VOZ ---
try:
    tts_engine = pyttsx3.init()
except (ImportError, RuntimeError) as e:
    print(f"Error al inicializar el motor de voz: {e}")
    print("El programa continuará sin feedback de audio.")
    tts_engine = None

# ---------------- VARIABLES GLOBALES ----------------
x_goal, y_goal = 0, 0

# --- PUNTOS DE INTERÉS Y ALIAS ---
PUNTOS_INTERES = {
    "Despacho y Baños": (-496.70, -572.90, 118.70), "Punto Columnas": (-78.20, -333.70, 101.30),
    "Garaje": (-304.50, -85.40, 13.50), "Escaleras 1": (-298.80, -578.00, 178.50),
    "Pasillo Inicio": (324.00, -153.80, -19.20), "Entrada": (883.10, -503.20, 257.10),
    "Ascensores": (211.90, -477.70, 172.60), "Administración": (317.90, 269.60, 93.90),
    "Sala 06": (300.10, 1645.80, 1.90), "Punto Luis": (304.10, 2255.70, 3.00),
    "Luis": (218.90, 2268.70, 74.40), "Escaleras 2": (335.60, 2547.80, 12.60),
    "Estación Carga": (-350.00, -50.00, 0.00)
}
ALIAS_DESTINOS = {
    "despacho y baños": "Despacho y Baños", "baños": "Despacho y Baños", "baño": "Despacho y Baños",
    "despacho": "Despacho y Baños", "columnas": "Punto Columnas", "punto columnas": "Punto Columnas",
    "garaje": "Garaje", "escaleras uno": "Escaleras 1", "primeras escaleras": "Escaleras 1",
    "pasillo": "Pasillo Inicio", "inicio del pasillo": "Pasillo Inicio", "entrada": "Entrada",
    "ascensores": "Ascensores", "ascensor": "Ascensores", "administración": "Administración",
    "sala 6": "Sala 06", "sala seis": "Sala 06", "punto luis": "Punto Luis", "luis": "Luis",
    "escaleras dos": "Escaleras 2", "segundas escaleras": "Escaleras 2", "cargar": "Estación Carga",
    "estación de carga": "Estación Carga", "terminar": "SALIR", "salir": "SALIR", "para": "SALIR"
}

# --- GRAFO DE ADYACENCIA ---
GRAFO = {
    "Estación Carga": ["Pasillo Inicio"], "Entrada": ["Pasillo Inicio"],
    "Pasillo Inicio": ["Entrada", "Ascensores", "Punto Columnas", "Administración", "Estación Carga"],
    "Ascensores": ["Pasillo Inicio", "Punto Columnas"],
    "Punto Columnas": ["Pasillo Inicio", "Escaleras 1", "Despacho y Baños", "Garaje", "Ascensores"],
    "Escaleras 1": ["Punto Columnas"], "Despacho y Baños": ["Punto Columnas"], "Garaje": ["Punto Columnas"],
    "Administración": ["Pasillo Inicio", "Sala 06"], "Sala 06": ["Administración", "Punto Luis"],
    "Punto Luis": ["Sala 06", "Luis", "Escaleras 2"], "Luis": ["Punto Luis"], "Escaleras 2": ["Punto Luis"]
}


# ---------------- FUNCIONES ----------------

def _decir_texto(texto):
    """Función síncrona que hace que el ordenador diga un texto."""
    if tts_engine:
        try:
            tts_engine.say(texto)
            tts_engine.runAndWait()
        except Exception as e:
            print(f"Error al intentar hablar: {e}")


def _escuchar_y_procesar_comando():
    """Función síncrona para escuchar y procesar un comando de voz."""
    r = sr.Recognizer()
    with sr.Microphone() as source:
        print("\n🎤 Ajustando ruido ambiental... Espera.")
        r.adjust_for_ambient_noise(source, duration=1)

        # Primero el robot habla
        _decir_texto("¿A dónde quieres ir?")

        # Espera 2 segundos para que termine de sonar completamente
        time.sleep(2)

        # AHORA sí mostramos el mensaje y empezamos a escuchar
        print("✅ ¡Listo! Di a dónde quieres ir...")

        try:
            audio = r.listen(source, timeout=5, phrase_time_limit=8)
            print("🧠 Procesando tu voz...")
            texto = r.recognize_google(audio, language='es-ES')
            print(f"🗣️ Has dicho: '{texto}'")
            texto_lower = texto.lower()
            for alias, destino_real in ALIAS_DESTINOS.items():
                if alias in texto_lower:
                    print(f"🎯 Destino reconocido: '{destino_real}'")
                    return destino_real

            _decir_texto("No he reconocido un destino válido.")
            print("❓ No he reconocido un destino válido en tu comando.")
            return None
        except sr.WaitTimeoutError:
            print("🔇 No he detectado ninguna voz.")
            return None
        except sr.UnknownValueError:
            _decir_texto("No te he entendido. ¿Puedes repetirlo?")
            print("🤔 No he podido entender lo que has dicho.")
            return None
        except sr.RequestError as e:
            print(f"API Error: {e}")
            return None


def calcular_distancia(p1_nombre, p2_nombre):
    x1, y1, _ = PUNTOS_INTERES[p1_nombre]
    x2, y2, _ = PUNTOS_INTERES[p2_nombre]
    return math.hypot(x2 - x1, y2 - y1)


def dijkstra(grafo, punto_inicio, punto_final):
    cola_prioridad = [(0, punto_inicio, [punto_inicio])]
    distancias = {nodo: float('inf') for nodo in grafo}
    distancias[punto_inicio] = 0
    while cola_prioridad:
        dist_actual, nodo_actual, camino_actual = heapq.heappop(cola_prioridad)
        if dist_actual > distancias[nodo_actual]: continue
        if nodo_actual == punto_final: return camino_actual
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
    if pos is None: return None, None, None
    return pos.x, pos.y, pos.heading


def fuerza_atractiva(x, y):
    dx, dy = x_goal - x, y_goal - y
    dist = math.hypot(dx, dy)
    if dist > 0: return (k_att * dx / dist, k_att * dy / dist)
    return (0, 0)


def fuerza_repulsiva(x, y, dist_ir, heading):
    Fx, Fy = 0, 0
    angulos = [-60, -40, -20, 0, 20, 40, 60]
    pesos = [0.5, 0.7, 1.3, 2.2, 1.3, 0.7, 0.5]

    for i, val in enumerate(dist_ir):
        d = distancia_sensor(val)
        if d < d0:
            mag = min(k_rep * (1 / d - 1 / d0) / (d ** 2), 15.0)
            ang_global = math.radians((heading + angulos[i]) % 360)
            Fx_norm = -pesos[i] * mag * math.cos(ang_global)
            Fy_norm = -pesos[i] * mag * math.sin(ang_global)
            Fx += Fx_norm - (Fy_norm * k_tangent)
            Fy += Fy_norm + (Fx_norm * k_tangent)

    mag_rep = math.hypot(Fx, Fy)
    if mag_rep > 25.0: Fx, Fy = (Fx / mag_rep) * 25.0, (Fy / mag_rep) * 25.0
    return Fx, Fy


def rampa_aceleracion(t, v_crucero, t_rampa):
    if t < t_rampa: return v_crucero * (t / t_rampa)
    return v_crucero


def freno_suave(distancia_obj, distancia_meta, v_actual, min_dist_obj=70, rango_meta=50):
    v_frenada = v_actual
    if distancia_obj < min_dist_obj: v_frenada = v_actual * max(0.1, distancia_obj / min_dist_obj)
    if distancia_meta < rango_meta: v_frenada = min(v_frenada, v_actual * max(0.2, distancia_meta / rango_meta))
    return v_frenada


def calcular_velocidades_ackermann(v_base, error_angular):
    v_efectiva = max(v_base, v_minima)
    error_angular = max(min(error_angular, 45), -45)
    factor_giro = abs(error_angular) / 45.0
    radio_deseado = radio_giro_minimo + (100 - radio_giro_minimo) * (1 - factor_giro) * factor_suavizado_giro
    L = 23.3
    if abs(error_angular) > 2:
        omega = v_efectiva / radio_deseado * (-1 if error_angular < 0 else 1)
        v_left, v_right = v_efectiva - omega * (L / 2), v_efectiva + omega * (L / 2)
    else:
        v_left, v_right = v_efectiva, v_efectiva
    velocidad_minima_rueda = v_minima * 0.3
    if v_left < velocidad_minima_rueda: v_left = velocidad_minima_rueda
    if v_right < velocidad_minima_rueda: v_right = velocidad_minima_rueda
    return min(v_left, v_crucero), min(v_right, v_crucero)


# ❌ FUNCIÓN COMENTADA - Ya no se usa
# async def escape_minimo_local_ackermann(robot, direccion='izquierda'):
#     print(f" ESCAPE DE MÍNIMO LOCAL - Girando a la {direccion}")
#     v_left, v_right = (8, 20) if direccion == 'izquierda' else (20, 8)
#     for _ in range(int(2.0 / dt)):
#         await robot.set_wheel_speeds(v_left, v_right)
#         await asyncio.sleep(dt)
#     await robot.set_wheel_speeds(20, 20);
#     await asyncio.sleep(1.0)
#     await robot.set_wheel_speeds(0, 0);
#     print(" Escape completado");
#     await asyncio.sleep(0.5)


async def recorrer_camino_suave(robot, camino, punto_partida):
    global x_goal, y_goal
    if not camino or len(camino) < 2: return False, punto_partida
    indice_waypoint_objetivo = 1
    punto_actual_nombre = punto_partida
    tiempo_inicio = asyncio.get_event_loop().time()
    ang_prev = None
    modo_evasion = False
    # dist_meta_anterior = None  # ❌ COMENTADO
    # tiempo_ultimo_chequeo = tiempo_inicio  # ❌ COMENTADO
    # intentos_minimo = 0  # ❌ COMENTADO

    while indice_waypoint_objetivo < len(camino):
        waypoint_objetivo_nombre = camino[indice_waypoint_objetivo]
        x_goal, y_goal = PUNTOS_INTERES[waypoint_objetivo_nombre][:2]
        x, y, heading = await get_pos(robot)
        if x is None:
            await robot.set_wheel_speeds(0, 0);
            return False, punto_actual_nombre
        dist_meta = math.hypot(x_goal - x, y_goal - y)
        es_ultimo_waypoint = (indice_waypoint_objetivo == len(camino) - 1)
        radio_deteccion = tolerancia if es_ultimo_waypoint else RADIO_PUNTO_INTERMEDIO
        if dist_meta < radio_deteccion:
            punto_actual_nombre = waypoint_objetivo_nombre
            if es_ultimo_waypoint:
                await robot.set_wheel_speeds(0, 0)
                return True, punto_actual_nombre
            else:
                print(f"⚪️ Pasando por **{punto_actual_nombre}**, continuando...")
                indice_waypoint_objetivo += 1
                # dist_meta_anterior = None  # ❌ COMENTADO
                # tiempo_ultimo_chequeo = asyncio.get_event_loop().time()  # ❌ COMENTADO
                # intentos_minimo = 0  # ❌ COMENTADO
                continue
        prox = (await robot.get_ir_proximity()).sensors
        dist_frente = distancia_sensor(prox[3])
        # tiempo_actual = asyncio.get_event_loop().time()  # ❌ COMENTADO

        # ❌❌❌ TODO EL BLOQUE DE DETECCIÓN DE MÍNIMOS LOCALES COMENTADO ❌❌❌
        # if tiempo_actual - tiempo_ultimo_chequeo >= tiempo_chequeo:
        #     if dist_meta_anterior is not None:
        #         progreso = dist_meta_anterior - dist_meta
        #         if progreso < umbral_progreso and dist_frente < UMBRAL_FRENTE_MINIMO:
        #             intentos_minimo += 1
        #             await robot.set_wheel_speeds(0, 0);
        #             await asyncio.sleep(0.3)
        #             await escape_minimo_local_ackermann(robot, 'izquierda' if intentos_minimo % 2 == 1 else 'derecha')
        #             tiempo_inicio = tiempo_ultimo_chequeo = asyncio.get_event_loop().time()
        #             dist_meta_anterior = ang_prev = None
        #             if intentos_minimo >= max_intentos_minimo: return False, punto_actual_nombre
        #             continue
        #         else:
        #             intentos_minimo = 0
        #     dist_meta_anterior = dist_meta
        #     tiempo_ultimo_chequeo = tiempo_actual

        if dist_frente < 40 and not modo_evasion:
            modo_evasion = True
            tiempo_evasion = asyncio.get_event_loop().time()
        if modo_evasion:
            if asyncio.get_event_loop().time() - tiempo_evasion < 2.0:
                izquierda = distancia_sensor(prox[1]);
                derecha = distancia_sensor(prox[5])
                await robot.set_wheel_speeds(5, 20) if izquierda > derecha else await robot.set_wheel_speeds(20, 5)
            elif asyncio.get_event_loop().time() - tiempo_evasion < 3.0:
                await robot.set_wheel_speeds(20, 20)
            else:
                modo_evasion = False
            await asyncio.sleep(dt);
            continue
        Fx_att, Fy_att = fuerza_atractiva(x, y)
        Fx_rep, Fy_rep = fuerza_repulsiva(x, y, prox, heading)
        Fx_total, Fy_total = Fx_att + Fx_rep, Fy_att + Fy_rep
        ang_obj = math.degrees(math.atan2(Fy_total, Fx_total))
        if ang_prev is None:
            ang_prev = ang_obj
        else:
            ang_obj = 0.7 * ang_prev + 0.3 * ang_obj

            ang_prev = ang_obj
        t_actual = asyncio.get_event_loop().time() - tiempo_inicio
        v_target = freno_suave(dist_frente, dist_meta, rampa_aceleracion(t_actual, v_crucero, t_rampa))
        error_ang = (ang_obj - heading + 540) % 360 - 180
        v_left, v_right = calcular_velocidades_ackermann(v_target, error_ang)
        await robot.set_wheel_speeds(v_left, v_right)
        await asyncio.sleep(dt)
    return False, punto_actual_nombre


async def iniciar_navegacion(robot):
    punto_actual = "Estación Carga";
    print(f"\n🤖 Iniciando en: {punto_actual}")
    await robot.set_lights_rgb(255, 0, 0);
    await robot.set_wheel_speeds(-10, -10)
    await asyncio.sleep(4.0);
    await robot.set_wheel_speeds(0, 0);
    await asyncio.sleep(0.5)
    x_target, y_target, _ = PUNTOS_INTERES["Pasillo Inicio"]
    x, y, heading = await get_pos(robot)
    if x is not None:
        error_deg = (math.degrees(math.atan2(y_target - y, x_target - x)) - heading + 540) % 360 - 180
        print(f"  > Pos actual ({x:.1f}, {y:.1f}), H: {heading:.1f}°. Girando {error_deg:.1f}°...")
        if abs(error_deg) > 5:
            await robot.set_lights_rgb(255, 255, 0)
            await robot.turn_left(error_deg) if error_deg > 0 else await robot.turn_right(abs(error_deg))
            await asyncio.sleep(1.0)
    await robot.set_lights_rgb(0, 0, 255)
    camino_inicial = dijkstra(GRAFO, punto_actual, "Entrada")
    if not camino_inicial: print("ERROR: No se encontró camino inicial."); return
    print(f"✅ Camino inicial: {' → '.join(camino_inicial)}")
    await asyncio.to_thread(_decir_texto, "Iniciando secuencia. Yendo a la entrada.")
    exito, punto_actual = await recorrer_camino_suave(robot, camino_inicial, punto_actual)
    if not exito: print(f"⚠️ Fallo inicial. Último punto: {punto_actual}."); return
    print(f"\n🎉 ¡Llegada a **{punto_actual}**! Preparado para recibir órdenes de voz.")
    await robot.play_note(Note.A4, 0.5)

    while True:
        destino_nombre = await asyncio.to_thread(_escuchar_y_procesar_comando)
        if destino_nombre is None:
            await robot.play_note(Note.C3, 0.3);
            continue
        if destino_nombre == "SALIR":
            print("Guía finalizado.");
            await asyncio.to_thread(_decir_texto, "Hasta pronto.");
            break
        if destino_nombre == punto_actual:
            print("Ya estamos aquí.");
            await asyncio.to_thread(_decir_texto, "Ya estamos en ese destino.");
            continue

        print(f"\nCalculando camino de **{punto_actual}** a **{destino_nombre}**...")
        camino = dijkstra(GRAFO, punto_actual, destino_nombre)
        if not camino:
            print(f"ERROR: No hay camino a {destino_nombre}.");
            await asyncio.to_thread(_decir_texto, "No he podido encontrar un camino a ese destino.");
            continue

        print(f"✅ Camino: {' → '.join(camino)}");
        await robot.set_lights_rgb(0, 0, 255)
        await asyncio.to_thread(_decir_texto, f"Yendo a {destino_nombre.replace('_', ' ')}")
        exito, punto_actual = await recorrer_camino_suave(robot, camino, punto_actual)

        if exito:
            print(f"\n🎉 ¡Llegada a **{punto_actual}**!");
            await robot.set_lights_rgb(0, 255, 0);
            await robot.play_note(Note.G5, 0.5)
            await asyncio.to_thread(_decir_texto, "Ha llegado a su destino.")
        else:
            print(f"⚠️ Fallo en nav. Último punto: {punto_actual}.");
            await robot.set_lights_rgb(255, 0, 0);
            await robot.play_note(Note.C2, 0.5)
            await asyncio.to_thread(_decir_texto, "Se ha producido un error en la navegación.")
        await asyncio.sleep(1.0)


@event(robot.when_play)
async def play(robot):
    await robot.reset_navigation();
    await robot.set_lights_rgb(255, 192, 203);
    await asyncio.sleep(1.0)
    await iniciar_navegacion(robot)


robot.play()
