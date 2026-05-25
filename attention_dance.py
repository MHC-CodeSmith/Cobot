#!/usr/bin/env python3
import sys
import time
import termios
import tty
from pymycobot.mycobot import MyCobot

# Inicializa o MyCobot (porta padrão do Jetson Nano)
try:
    mc = MyCobot('/dev/ttyTHS1', 1000000)
    # Apenas para garantir que os motores estão ligados
    mc.power_on()
except Exception as e:
    print(f"Erro ao conectar com o MyCobot: {e}")
    sys.exit(1)

def get_char():
    """Lê um único caractere do terminal (sem precisar apertar Enter)"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def make_attention_dance():
    print("\n[!] Zerando todas as juntas...")
    mc.send_angles([0, 0, 0, 0, 0, 0], 50)
    time.sleep(2)
    
    print("[!] O robô está prestando atenção em você...")
    
    # 1. Olha para a esquerda
    mc.send_angles([30, -10, -20, 0, 0, 0], 60)
    time.sleep(1)
    
    # 2. Olha para a direita
    mc.send_angles([-30, -10, -20, 0, 0, 0], 60)
    time.sleep(1)
    
    # 3. Olha para o centro e levemente para cima (curioso)
    mc.send_angles([0, -20, -30, -10, 0, 0], 60)
    time.sleep(1)
    
    # 4. Faz um movimento rápido de aceno (sim) com a "cabeça"
    mc.send_angles([0, -5, -15, -20, 0, 0], 80)
    time.sleep(0.5)
    mc.send_angles([0, -25, -35, 10, 0, 0], 80)
    time.sleep(0.5)
    mc.send_angles([0, -5, -15, -20, 0, 0], 80)
    time.sleep(0.5)
    
    # 5. Volta para a posição zero
    print("[!] Dança finalizada. Voltando à pose de repouso...")
    mc.send_angles([0, 0, 0, 0, 0, 0], 50)
    time.sleep(2)
    print("\n------------------------------------------------")
    print(" Pressione 'Espaço' para repetir, ou 'q' para sair")

def main():
    print("\n================================================")
    print("          Animação 'Prestando Atenção'          ")
    print("================================================")
    print("-> O script vai usar a biblioteca pymycobot direta.")
    print("-> Pressione 'Espaço' para rodar a animação.")
    print("-> Pressione 'q' para sair.")
    print("------------------------------------------------")
    
    while True:
        ch = get_char()
        if ch == ' ':
            make_attention_dance()
        elif ch.lower() == 'q':
            print("\nSaindo...")
            break

if __name__ == '__main__':
    main()
