#!/usr/bin/env python3
"""Demuestra las tres causas de rechazo de la admision, con motivo explicito.

El enunciado puntua "admision validada con FK y rechazos razonados" y pide que
TODO rechazo indique causa: workspace, limite o paso excesivo. Este script manda
un goal que dispara cada causa y despues lee el CSV de rechazos del broker para
comprobar que los tres motivos quedaron escrito.

    python3 validar_rechazos.py                 # con el broker recien arrancado
    python3 validar_rechazos.py --csv evidencia/rechazos.csv

Como el broker arranca en [0,0,0,0,0,0], la primera prueba lo deja en esa pose
de referencia. Ahi la distancia articular de cada caso es fija, y por eso el
resultado no depende de donde estuviera el brazo antes.

Las tres poses se eligieron con fk.py para que cada una trippedie SOLO su causa:
los limites se comprueban antes que el workspace, asi que la pose de workspace
tiene que estar dentro de los limites articulares.
"""

import argparse
import csv
import math
import os
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'arm_broker'))
from arm_broker import fk                                        # noqa: E402
from arm_broker_interfaces.action import MoveArm                  # noqa: E402

# Pose de referencia: es la que el broker asume en su q_actual inicial
HOME = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Un caso por cada causa que broker._validar puede devolver
CASOS = [
    ('limite',
     [0.0, 0.0, 3.0, 0.0, 0.0, 0.0],
     '3_Joint fuera de rango: 3.0 rad, limite [-2.53, 2.53]'),
    ('workspace',
     [0.0, 2.3, 1.4, 0.0, 2.1, -1.7],
     'efector demasiado cerca de la base'),
    ('paso',
     [0.0, 0.0, 1.8, 0.0, 0.0, 0.0],
     'salto articular grande desde la pose de referencia'),
]


def comprobar_localmente(paso_max):
    """Verifica con fk.py que cada caso pisa solo la causa que dice, antes de mandar nada.

    Sirve para no confundir un fallo del script con un rechazo del broker: si aqui
    el caso no se sostiene, el error es del caso de prueba, no del broker.
    """
    print('Comprobacion local con fk.py (no es el broker, es una red de seguridad):')
    ok = True
    for causa, q, _ in CASOS:
        dentro = fk.dentro_de_limites(q)[0]
        ws, motivo_ws = fk.dentro_del_workspace(q)[0], fk.dentro_del_workspace(q)[1]
        paso = fk.paso_articular(HOME, q)

        if causa == 'limite':
            bien = not dentro
        elif causa == 'workspace':
            bien = dentro and not ws
        else:
            bien = dentro and ws and paso > paso_max

        marca = 'ok' if bien else 'MAL'
        print(f'  [{marca}] {causa:10} limites={dentro} workspace={ws} '
              f'paso_desde_home={paso:.2f}')
        if not ws:
            print(f'             motivo del workspace: {motivo_ws}')
        ok = ok and bien

    if not ok:
        print('\nALGUN CASO NO SE SOSTIENE: el error esta en este script, no en el broker.')
    return ok


class Validador(Node):
    def __init__(self, csv_rechazos):
        super().__init__('validador_rechazos')
        self.cli = ActionClient(self, MoveArm, 'move_arm')
        self.csv = csv_rechazos

    def enviar(self, q, client_id):
        """Manda un goal y devuelve True si el broker lo RECHAZO en la admision."""
        goal = MoveArm.Goal()
        goal.joint_positions = q
        goal.client_id = client_id
        goal.priority = 1

        fut = self.cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        handle = fut.result()
        if handle is None:
            raise RuntimeError('el broker no respondio el goal')
        if handle.accepted:
            # Aceptado: no era un caso de rechazo. Se cancela para no mover el brazo.
            handle.cancel_goal_async()
            return False
        return True

    def correr(self, paso_max):
        if not self.cli.wait_for_server(timeout_sec=20.0):
            raise RuntimeError('el broker no aparece. Esta corriendo?')

        # Primero se comanda la pose de referencia para que el paso se mida desde ella
        print(f'\n[0] Se comanda la pose de referencia {HOME} ...')
        ok = self.enviar(HOME, 'validador')
        if ok:
            print('    el broker rechazo la pose de referencia. No se puede seguir:')
            print('    la pose inicial [0,0,0,0,0,0] deberia ser admisible.')
            return False
        print('    aceptada (o en curso). El broker quedo en [0,0,0,0,0,0].')

        print(f'\n[1] Se mandan los tres casos (paso_max_rad={paso_max:.2f}):')
        enviados = []
        for causa, q, desc in CASOS:
            rech = self.enviar(q, f'validador_{causa}')
            enviados.append((causa, rech, desc))
            print(f'    {causa:10} {"RECHAZADO" if rech else "ACEPTADO  <-- FALLA"}  '
                  f'{desc}')

        print(f'\n[2] Se lee {self.csv}:')
        if not os.path.isfile(self.csv):
            print(f'    no existe {self.csv}. El broker no registro rechazos.')
            print(f'    Se arranco con -p archivo_rechazos:=... ?')
            return False

        with open(self.csv, newline='', encoding='utf-8') as f:
            filas = list(csv.DictReader(f))

        print(f'    {len(filas)} rechazos registrados')
        print(f'    {"cliente":18} {"causa":10} motivo')
        for fila in filas[-len(CASOS):]:
            print(f'    {fila["client_id"]:18} {fila["causa"]:10} {fila["motivo"]}')

        print(f'\n[3] Cada causa aparecio en el CSV:')
        causas = {f['causa'] for f in filas}
        bien = True
        for causa, rech, desc in enviados:
            ok_c = rech and causa in causas
            bien = bien and ok_c
            print(f'    {"ok" if ok_c else "MAL"}  {causa}')
        return bien


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--csv', default='rechazos.csv',
                    help='CSV de rechazos que escribe el broker')
    ap.add_argument('--paso-max', type=float, default=1.2,
                    help='el mismo paso_max_rad con el que se levanto el broker')
    args = ap.parse_args()

    if not comprobar_localmente(args.paso_max):
        return 1

    rclpy.init()
    nodo = Validador(args.csv)
    try:
        return 0 if nodo.correr(args.paso_max) else 1
    except RuntimeError as e:
        print(f'\nFALLO: {e}')
        return 1
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
