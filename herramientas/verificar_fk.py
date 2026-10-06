#!/usr/bin/env python3
"""Compara fk(q) contra lo que reporta el brazo — la verificación del ítem 1.

Se corre EN EL JETSON, con el puerto serie libre (sin sync_plan_nx corriendo).
Lleva el brazo a varias poses, lee get_coords() y mide el error.

    python3 verificar_fk.py                 # las poses por defecto
    python3 verificar_fk.py --solo-leer     # no mueve: solo compara donde está

Criterio del reto: error ≤ 10 mm.
"""
import argparse, math, os, sys, time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'arm_broker'))
from arm_broker import fk                                        # noqa: E402

POSES = {
    'cero':    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'ready':   [0.0, -0.5, 0.5, 0.0, 0.5, 0.0],
    'girada':  [0.6, -0.4, 0.4, 0.0, 0.3, 0.0],
    'baja':    [0.0, -1.2, 1.2, 0.0, 0.0, 0.0],
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--puerto', default='/dev/ttyUSB0')
    ap.add_argument('--baud', type=int, default=1000000)
    ap.add_argument('--velocidad', type=int, default=30)
    ap.add_argument('--espera', type=float, default=5.0)
    ap.add_argument('--solo-leer', action='store_true')
    args = ap.parse_args()

    try:
        from pymycobot.mycobot import MyCobot
    except ImportError:
        from pymycobot import MyCobot

    brazo = MyCobot(args.puerto, args.baud)
    print(f'{"pose":10} {"FK (x,y,z)":>26}  {"robot (x,y,z)":>26}  {"error":>8}')
    print('-' * 78)

    errores = []
    poses = {'donde este': None} if args.solo_leer else POSES

    for nombre, q_rad in poses.items():
        if q_rad is not None:
            brazo.send_angles([v * 180.0 / math.pi for v in q_rad], args.velocidad)
            time.sleep(args.espera)
        leidos = brazo.get_angles()
        coords = brazo.get_coords()
        if not leidos or not coords or len(coords) < 3:
            print(f'{nombre:10}  el brazo no respondió; repite')
            continue

        q_real = [v * math.pi / 180.0 for v in leidos]
        px, py, pz = fk.fk(q_real)
        rx, ry, rz = coords[0], coords[1], coords[2]
        err = math.sqrt((px - rx) ** 2 + (py - ry) ** 2 + (pz - rz) ** 2)
        errores.append(err)
        marca = 'OK' if err <= 10.0 else '>10mm'
        print(f'{nombre:10} ({px:7.1f},{py:7.1f},{pz:7.1f})  '
              f'({rx:7.1f},{ry:7.1f},{rz:7.1f})  {err:6.1f} {marca}')

    if errores:
        print('-' * 78)
        print(f'error medio {sum(errores)/len(errores):.1f} mm   '
              f'máximo {max(errores):.1f} mm   criterio del reto: ≤ 10 mm')
        if max(errores) > 10.0:
            print('\nSi el error es grande y constante, la tabla DH necesita ajuste.')
            print('Si crece con la distancia, revisen los parámetros a_i.')


if __name__ == '__main__':
    main()
