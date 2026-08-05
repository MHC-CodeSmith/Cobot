#!/usr/bin/env python3
"""
nano_micro_bridge.py — Servidor HTTP ultra-rápido (~30ms) na Jetson Nano
=========================================================================
Executa na porta 8088 e oferece REST endpoints para controlar o MyCobot 280
diretamente via pymycobot + Jetson.GPIO, sem depender do ROS 2 CLI.

Endpoints:
  GET  /get_angles        → retorna os 6 ângulos atuais das juntas
  GET  /move_joints?j=..&speed=..  → move juntas para posição
  POST /pump/on           → liga bomba de sucção
  POST /pump/off          → desliga bomba de sucção
  POST /servos/release    → libera servos (modo ensino)
  POST /servos/lock       → trava servos
  GET  /panic             → desliga tudo (bomba + servos lock)
  GET  /status            → status geral

Uso:  python3 nano_micro_bridge.py
      ou: nohup python3 nano_micro_bridge.py > /tmp/micro_bridge.log 2>&1 &
"""
import json
import time
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs

# ── Hardware ──────────────────────────────────────────────────────────
PORT = '/dev/ttyTHS1'
BAUD = 1000000
HTTP_PORT = 8088

# Suction Pump V2.0 — BCM pins (mesmo do mycobot_bridge.py)
PUMP_SOLENOID_PIN = 20
PUMP_VALVE_PIN = 21

mc = None
gpio_ready = False
pump_active = False
serial_lock = threading.Lock()
valve_timer = None


def init_hardware():
    """Inicializa pymycobot e GPIO."""
    global mc, gpio_ready
    try:
        from pymycobot.mycobot280 import MyCobot280
        mc = MyCobot280(PORT, BAUD)
        time.sleep(0.5)
        try:
            mc.set_fresh_mode(1)
            time.sleep(0.1)
        except Exception:
            pass
        print(f"[BRIDGE] MyCobot280 conectado em {PORT}")
    except Exception as e:
        print(f"[BRIDGE] WARN: pymycobot falhou: {e}")
        mc = None

    try:
        import Jetson.GPIO as GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(PUMP_SOLENOID_PIN, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(PUMP_VALVE_PIN, GPIO.OUT, initial=GPIO.HIGH)
        gpio_ready = True
        print("[BRIDGE] Jetson.GPIO inicializado (Pump V2.0 pronto)")
    except Exception as e:
        print(f"[BRIDGE] WARN: GPIO indisponível: {e}")
        gpio_ready = False


def do_pump_on():
    global pump_active, valve_timer
    if pump_active:
        return True, "Pump already active"
    if not gpio_ready:
        return False, "GPIO indisponível"
    try:
        import Jetson.GPIO as GPIO
        # Cancela timer de válvula se existir
        if valve_timer is not None:
            valve_timer.cancel()
            valve_timer = None
        GPIO.output(PUMP_VALVE_PIN, GPIO.HIGH)   # fecha deflação
        GPIO.output(PUMP_SOLENOID_PIN, GPIO.LOW)  # abre solenoide → sucção
        time.sleep(0.05)
        pump_active = True
        return True, "Pump activated"
    except Exception as e:
        return False, f"pump_on failed: {e}"


def do_pump_off():
    global pump_active, valve_timer
    if not gpio_ready:
        return False, "GPIO indisponível"
    try:
        import Jetson.GPIO as GPIO
        GPIO.output(PUMP_SOLENOID_PIN, GPIO.HIGH)  # fecha solenoide
        time.sleep(0.05)
        GPIO.output(PUMP_VALVE_PIN, GPIO.LOW)      # abre deflação → solta objeto

        # Fecha deflação após 1s
        def close_valve():
            global valve_timer
            try:
                GPIO.output(PUMP_VALVE_PIN, GPIO.HIGH)
            except Exception:
                pass
            valve_timer = None

        if valve_timer is not None:
            valve_timer.cancel()
        valve_timer = threading.Timer(1.0, close_valve)
        valve_timer.start()

        pump_active = False
        return True, "Pump deactivated"
    except Exception as e:
        return False, f"pump_off failed: {e}"


def do_release():
    if mc is None:
        return False, "MyCobot não conectado"
    try:
        with serial_lock:
            mc.release_all_servos()
        return True, "Servos soltos"
    except Exception as e:
        return False, f"release failed: {e}"


def do_lock():
    if mc is None:
        return False, "MyCobot não conectado"
    try:
        with serial_lock:
            mc.power_on()
        return True, "Servos travados"
    except Exception as e:
        return False, f"lock failed: {e}"


def do_get_angles():
    if mc is None:
        return False, None
    try:
        with serial_lock:
            angles = mc.get_angles()
        if angles and len(angles) >= 6:
            return True, angles[:6]
        return False, None
    except Exception:
        return False, None


def do_move_joints(joints, speed=60):
    if mc is None:
        return False, "MyCobot não conectado"
    try:
        with serial_lock:
            mc.send_angles(joints[:6], speed)
        return True, "Movimento enviado"
    except Exception as e:
        return False, f"move failed: {e}"


# ── HTTP Handler ──────────────────────────────────────────────────────

class BridgeHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        # Silencia logs de cada request para não poluir
        pass

    def _respond(self, data, status=200):
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(json.dumps(data).encode())

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path
        params = parse_qs(parsed.query)

        if path == '/get_angles':
            ok, angles = do_get_angles()
            if ok:
                self._respond({"success": True, "joints": angles})
            else:
                self._respond({"success": False, "joints": None}, 503)

        elif path == '/move_joints':
            raw_j = params.get('j', [None])[0]
            speed = int(params.get('speed', ['60'])[0])
            if not raw_j:
                self._respond({"success": False, "message": "Parâmetro j obrigatório"}, 400)
                return
            try:
                joints = [float(x) for x in raw_j.split(',')]
            except ValueError:
                self._respond({"success": False, "message": "Formato inválido"}, 400)
                return
            ok, msg = do_move_joints(joints, speed)
            self._respond({"success": ok, "message": msg}, 200 if ok else 500)

        elif path == '/status':
            self._respond({
                "success": True,
                "pump_active": pump_active,
                "gpio_ready": gpio_ready,
                "mycobot_connected": mc is not None
            })

        elif path == '/panic':
            do_pump_off()
            do_lock()
            self._respond({"success": True, "message": "Panic: pump off + servos locked"})

        elif path in ('/pump/on', '/pump/off', '/servos/release', '/servos/lock'):
            # Permitir GET também para simplicidade
            self.do_POST()

        else:
            self._respond({"error": "Not found"}, 404)

    def do_POST(self):
        parsed = urlparse(self.path)
        path = parsed.path

        if path == '/pump/on':
            ok, msg = do_pump_on()
            self._respond({"success": ok, "message": msg}, 200 if ok else 500)
        elif path == '/pump/off':
            ok, msg = do_pump_off()
            self._respond({"success": ok, "message": msg}, 200 if ok else 500)
        elif path == '/servos/release':
            ok, msg = do_release()
            self._respond({"success": ok, "message": msg}, 200 if ok else 500)
        elif path == '/servos/lock':
            ok, msg = do_lock()
            self._respond({"success": ok, "message": msg}, 200 if ok else 500)
        else:
            self._respond({"error": "Not found"}, 404)


def main():
    init_hardware()
    server = HTTPServer(('0.0.0.0', HTTP_PORT), BridgeHandler)
    print(f"[BRIDGE] HTTP Micro-Bridge ativo na porta {HTTP_PORT}")
    print(f"[BRIDGE] Endpoints: /pump/on, /pump/off, /servos/release, /servos/lock, /get_angles, /move_joints, /status, /panic")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        if gpio_ready:
            try:
                import Jetson.GPIO as GPIO
                GPIO.output(PUMP_SOLENOID_PIN, GPIO.HIGH)
                GPIO.output(PUMP_VALVE_PIN, GPIO.HIGH)
                GPIO.cleanup()
            except Exception:
                pass


if __name__ == '__main__':
    main()
