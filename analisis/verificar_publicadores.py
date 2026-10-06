#!/usr/bin/env python3
"""Comprueba el requisito eliminatorio: UN solo publicador en /joint_states.

El enunciado dice que la metrica "violaciones de exclusion mutua" sale del
"bag de /joint_states: publicadores concurrentes" y que el criterio es cero.
Lo que se puede violar no es solo que dos pedidos se solapen: es que el brazo
obedaiga mensajes intercalados de dos publicadores. Por eso se cuenta
publicadores, no solo solapes.

    ros2 run --prefix 'python3' ...   # no: esto se corre directo
    python3 verificar_publicadores.py

Cierra con codigo 0 si hay exactamente un publicador y se llama arm_broker.
Con cualquier otro caso cierra con codigo 1, que es lo que hace util el script
en un pipeline. Con --salida guarda la salida de ros2 topic info como evidencia.
"""

import argparse
import re
import subprocess
import sys

TOPICO = '/joint_states'
NODO_ESPERADO = 'arm_broker'


def leer_topico(topic):
    """Devuelve la salida de `ros2 topic info <topic> --verbose`."""
    cmd = ['ros2', 'topic', 'info', topic, '--verbose']
    p = subprocess.run(cmd, capture_output=True, text=True)
    if p.returncode != 0:
        sys.exit(f'FALLO: no se pudo consultar {topic}.\n'
                 f'       Si el broker esta corriendo, el problema es de descubrimiento:\n'
                 f'       ros2 daemon stop && ros2 daemon start\n'
                 f'{p.stderr.strip()}')
    return p.stdout


def analizar(salida):
    """Extrae (numero de publicadores, lista de nombres de nodo) de la salida de ros2."""
    m = re.search(r'Publisher count:\s*(\d+)', salida)
    if not m:
        sys.exit('FALLO: la salida de `ros2 topic info` no tiene "Publisher count".\n'
                 f'--- salida ---\n{salida}')
    n = int(m.group(1))

    # Cada endpoint publica un bloque con "Node name: <nombre>"
    nombres = re.findall(r'Node name:\s*(\S+)', salida)
    return n, nombres


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--topico', default=TOPICO)
    ap.add_argument('--nodo', default=NODO_ESPERADO)
    ap.add_argument('--salida', help='guarda la salida de ros2 topic info en este archivo')
    args = ap.parse_args()

    salida = leer_topico(args.topico)
    if args.salida:
        with open(args.salida, 'w', encoding='utf-8') as f:
            f.write(salida)
        print(f'Evidencia guardada en {args.salida}')

    n, nombres = analizar(salida)
    unicos = sorted(set(nombres))

    print(f'\ntopico            : {args.topico}')
    print(f'publicadores      : {n}')
    print(f'nombres de nodo   : {unicos}')

    fallos = []
    if n != 1:
        fallos.append(f'Hay {n} publicadores y debe haber exactamente 1. '
                      f'El enunciado anula el puntaje de exclusion mutua.')
    elif args.nodo not in unicos:
        fallos.append(f'El unico publicador es {unicos} y deberia ser {args.nodo}. '
                      f'Solo el broker habla con el driver.')

    if fallos:
        for f in fallos:
            print(f'\nFALLA: {f}')
        return 1

    print(f'\nOK: un solo publicador ({args.nodo}). '
          f'Requisito eliminatorio cumplido.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
