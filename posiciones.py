import asyncio
import math
import keyboard  # pip install keyboard

from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import Create3, event
from irobot_edu_sdk.music import Note

robot = Create3(Bluetooth(name="C3_UIEC_Grupo2"))

# Velocidad base
SPEED = 60

# Colores por dirección
COLORS = {
    "w": (0, 255, 0),
    "s": (255, 0, 0),
    "a": (0, 0, 255),
    "d": (255, 255, 0),
}

# Variables globales
last_direction = None
saved_positions = []
initial_pose = None
moving = False  # 🔥 Nueva variable para controlar el estado


async def get_position_mem(robot):
    """Copia la posición actual del robot"""
    pos = await robot.get_position()
    # Creamos una nueva clase Position para copiar los valores actuales
    return type('Position', (), {'x': pos.x, 'y': pos.y, 'heading': pos.heading})


async def control_loop():
    global last_direction, saved_positions, initial_pose, moving

    # Reset de navegación SOLO UNA VEZ al inicio
    await robot.reset_navigation()
    await asyncio.sleep(1.0)  # 🔥 Esperar más tiempo

    # Señal luminosa ROSA (inicio)
    await robot.set_lights_on_rgb(255, 192, 203)
    await robot.play_note(Note.C4, 0.5)

    print("Control con teclado activado.")
    print("Usa W/A/S/D para mover, G para guardar posición, Esc para salir.")

    # Guardamos posición inicial usando get_position_mem()
    initial_pose = await get_position_mem(robot)

    print(f"\n📌 ORIGEN FIJADO EN: x={initial_pose.x:.2f}, y={initial_pose.y:.2f}, θ={initial_pose.heading:.2f}\n")

    try:
        while True:
            direction = None
            key_pressed = False  # 🔥 Para detectar si hay tecla presionada

            # --- CONTROL DIRECCIONES ---
            if keyboard.is_pressed("w"):
                direction = "w"
                key_pressed = True
                await robot.set_wheel_speeds(SPEED, SPEED)
                moving = True

            elif keyboard.is_pressed("s"):
                direction = "s"
                key_pressed = True
                await robot.set_wheel_speeds(-SPEED, -SPEED)
                moving = True

            elif keyboard.is_pressed("a"):
                direction = "a"
                key_pressed = True
                await robot.set_wheel_speeds(-SPEED, SPEED)
                moving = True

            elif keyboard.is_pressed("d"):
                direction = "d"
                key_pressed = True
                await robot.set_wheel_speeds(SPEED, -SPEED)
                moving = True

            elif keyboard.is_pressed("g"):
                # 🔥 Solo establecer velocidad a 0, NO usar stop()
                await robot.set_wheel_speeds(0, 0)
                await asyncio.sleep(0.5)  # Esperar estabilización

                # Obtener posición actual usando get_position_mem()
                posicion_actual = await get_position_mem(robot)
                await asyncio.sleep(0.2)

                # Calcular posición relativa al origen
                x_rel = posicion_actual.x - initial_pose.x
                y_rel = posicion_actual.y - initial_pose.y
                theta_rel = posicion_actual.heading - initial_pose.heading

                # Calcular distancia desde el origen
                distancia_desde_origen = math.sqrt(x_rel ** 2 + y_rel ** 2)

                saved_positions.append((x_rel, y_rel, theta_rel))

                print(f"📍 Guardada posición relativa:")
                print(f"   x={x_rel:.2f} cm, y={y_rel:.2f} cm, θ={theta_rel:.2f}°")
                print(f"   Distancia desde origen: {distancia_desde_origen:.2f} cm")

                # Señal luminosa para confirmar guardado
                await robot.set_lights_on_rgb(0, 255, 255)  # Cyan
                await robot.play_note(Note.C5, 0.3)
                await asyncio.sleep(0.3)

                moving = False
                key_pressed = True  # Para no llamar a stop() abajo

            elif keyboard.is_pressed("esc"):
                await robot.set_wheel_speeds(0, 0)  # 🔥 Velocidad a 0 en lugar de stop()

                # Señal verde (finalización)
                await robot.set_lights_on_rgb(0, 255, 0)
                await robot.play_note(Note.C6, 0.5)

                print("\nSaliendo...\n")
                print("📌 POSICIONES GUARDADAS:")
                for i, p in enumerate(saved_positions):
                    distancia = math.sqrt(p[0] ** 2 + p[1] ** 2)
                    print(f"  {i + 1}: x={p[0]:.2f} cm, y={p[1]:.2f} cm, θ={p[2]:.2f}° | Distancia: {distancia:.2f} cm")
                break

            # 🔥 Solo detener si no se presiona ninguna tecla Y estaba en movimiento
            if not key_pressed and moving:
                await robot.set_wheel_speeds(0, 0)
                moving = False

            # Cambiar luces según dirección
            if direction and direction != last_direction:
                r, g, b = COLORS[direction]
                await robot.set_lights_on_rgb(r, g, b)
                last_direction = direction

            await asyncio.sleep(0.1)

    except KeyboardInterrupt:
        await robot.set_wheel_speeds(0, 0)
        print("Interrumpido.")


@event(robot.when_play)
async def ready(robot):
    await control_loop()


robot.play()

# Despacho y baños: x=-496.70 cm, y=-572.90 cm, θ=118.70°
# Punto columnas: x=-78.20 cm, y=-333.70 cm, θ=101.30°
# Garaje: x=-324.50 cm, y=-20.40 cm, θ=13.50°
# Escaleras: x=-298.80 cm, y=-578.00 cm, θ=178.50°
# Pasillo:x=324.00 cm, y=-133.80 cm, θ=-19.20°
# Entrada: x=883.10 cm, y=-523.20 cm, θ=257.10°
# Ascensores: x=211.90 cm, y=-477.70 cm, θ=172.60°
# Adiministración:x=347.90 cm, y=269.60 cm, θ=93.90°
# Sala 06:x=343.10 cm, y=1645.80 cm, θ=1.90°
# Punto Pousa:x=314.10 cm, y=2235.70 cm, θ=3.00°
# Pousa:x=198.90 cm, y=2268.70 cm, θ=74.40°
# Escaleras: x=335.60 cm, y=2547.80 cm, θ=12.60°