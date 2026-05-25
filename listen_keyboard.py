#!/usr/bin/env python3
import evdev
import time
from pymycobot.mycobot import MyCobot

def get_keyboard_device():
    devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
    for dev in devices:
        # Procurar por "Keyboard" no nome do dispositivo
        if 'Keyboard' in dev.name:
            return dev
    return None

def dance():
    try:
        mc = MyCobot('/dev/ttyTHS1', 1000000)
        # Power on para garantir que os motores estao ligados
        mc.power_on()
    except Exception as e:
        print(f"Erro MyCobot: {e}")
        return

    print("Dancando...")
    mc.send_angles([0, 0, 0, 0, 0, 0], 50)
    time.sleep(2)
    mc.send_angles([30, -10, -20, 0, 0, 0], 60)
    time.sleep(1)
    mc.send_angles([-30, -10, -20, 0, 0, 0], 60)
    time.sleep(1)
    mc.send_angles([0, -20, -30, -10, 0, 0], 60)
    time.sleep(1)
    mc.send_angles([0, -5, -15, -20, 0, 0], 80)
    time.sleep(0.5)
    mc.send_angles([0, -25, -35, 10, 0, 0], 80)
    time.sleep(0.5)
    mc.send_angles([0, -5, -15, -20, 0, 0], 80)
    time.sleep(0.5)
    mc.send_angles([0, 0, 0, 0, 0, 0], 50)
    time.sleep(2)
    print("Pronto!")

def main():
    dev = get_keyboard_device()
    if not dev:
        print("Nenhum teclado fisico foi detectado conectado na Jetson Nano!")
        return
    print(f"Lendo eventos diretamente do teclado fisico: {dev.name}...")
    print("Pressione a barra de ESPACO para o MyCobot dancar.")
    print("Para sair do script pressione Ctrl+C")
    
    # Grab exclusividade se necessario dev.grab()
    try:
        for event in dev.read_loop():
            if event.type == evdev.ecodes.EV_KEY:
                key_event = evdev.categorize(event)
                if key_event.keystate == key_event.key_down:
                    if key_event.keycode == 'KEY_SPACE':
                        dance()
    except PermissionError:
        print("Erro de Permissao! Este script precisa ser rodado como root (sudo).")
        print("Exemplo: sudo ./listen_keyboard.py")
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
