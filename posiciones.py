import asyncio
import keyboard  # pip install keyboard

from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import Create3, event
from irobot_edu_sdk.music import Note

robot = Create3(Bluetooth(name="C3_UIEC_Grupo2"))

# Velocidad base
SPEED = 15

# Colores por dirección
COLORS = {
    "w": (0, 255, 0),
    "s": (255, 0, 0),
    "a": (0, 0, 255),
    "d": (255, 255, 0),
}

# Variables globales
last_direction = None
saved_positions = []  # <- NUEVO


async def control_loop():
    global last_direction, saved_positions

    await robot.set_lights_on_rgb(255, 192, 203)
    print("Control con teclado activado.")
    print("Usa W/A/S/D para mover, G para guardar posición, Esc para salir.")

    try:
        while True:
            direction = None

            # --- CONTROL DIRECCIONES ---
            if keyboard.is_pressed("w"):
                direction = "w"
                await robot.set_wheel_speeds(SPEED, SPEED)

            elif keyboard.is_pressed("s"):
                direction = "s"
                await robot.set_wheel_speeds(-SPEED, -SPEED)
                await robot.play_note(Note.A7_SHARP, 0.2)

            elif keyboard.is_pressed("a"):
                direction = "a"
                await robot.set_wheel_speeds(-SPEED, SPEED)

            elif keyboard.is_pressed("d"):
                direction = "d"
                await robot.set_wheel_speeds(SPEED, -SPEED)

            elif keyboard.is_pressed("g"):           # ← NUEVA TECLA
                pose = await robot.get_position()   # obtener odometría
                saved_positions.append(
                    (pose.x, pose.y, pose.heading)
                )
                print(f"📍 Posición guardada: x={pose.x:.2f}, y={pose.y:.2f}, θ={pose.heading:.2f}")
                await asyncio.sleep(0.4)  # evitar doble pulsación

            elif keyboard.is_pressed("esc"):
                await robot.stop()
                print("\nSaliendo...")
                print("\n📌 POSICIONES GUARDADAS:")
                for i, p in enumerate(saved_positions):
                    print(f"  {i+1}: x={p[0]:.2f}, y={p[1]:.2f}, θ={p[2]:.2f}")
                break

            else:
                await robot.stop()

            # --- CAMBIO DE COLOR SEGÚN DIRECCIÓN ---
            if direction and direction != last_direction:
                r, g, b = COLORS[direction]
                await robot.set_lights_on_rgb(r, g, b)
                last_direction = direction

            await asyncio.sleep(0.2)

    except KeyboardInterrupt:
        await robot.stop()
        print("Interrumpido.")


@event(robot.when_play)
async def ready(robot):
    await control_loop()


robot.play()
