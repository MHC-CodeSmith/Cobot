#!/usr/bin/env python3
import evdev
import time
from select import select
from pymycobot.mycobot import MyCobot

def dance():
    try:
        mc = MyCobot('/dev/ttyTHS1', 1000000)
        mc.power_on()
    except Exception as e:
        return

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

def main():
    # Pega TODOS os teclados para não ter erro de escutar a interface errada
    devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
    kbds = [d for d in devices if 'Keyboard' in d.name]
    
    if not kbds:
        return
    
    # Cria um mapa de file descriptors para usar com select()
    devices_map = {dev.fd: dev for dev in kbds}
    
    try:
        while True:
            r, w, x = select(devices_map, [], [])
            for fd in r:
                for event in devices_map[fd].read():
                    if event.type == evdev.ecodes.EV_KEY:
                        key_event = evdev.categorize(event)
                        # Verifica se pressionou SPACE
                        if key_event.keystate == key_event.key_down:
                            if key_event.keycode == 'KEY_SPACE':
                                dance()
    except Exception:
        pass

if __name__ == '__main__':
    main()
