#!/usr/bin/env python3
"""Lee un bag de ROS 2 y escribe dos CSV: uno de /arm/queue_state y otro de /joint_states.

    python3 exportar_csv.py <carpeta_del_bag> [--salida .]

En Humble no existe `ros2 bag export`, así que el bag se lee con rosbag2_py.
Entregado completo: es instrumentación, no es lo que evalúa el reto.
"""

import argparse
import csv
import os
import sys

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError:
    sys.exit('Falta ROS 2 en esta terminal: source /opt/ros/humble/setup.bash')


def abrir(ruta):
    lector = rosbag2_py.SequentialReader()
    lector.open(
        rosbag2_py.StorageOptions(uri=ruta, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''))
    tipos = {t.name: t.type for t in lector.get_all_topics_and_types()}
    return lector, tipos


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--salida', default='.')
    args = ap.parse_args()

    if not os.path.isdir(args.bag):
        sys.exit(f'No encuentro la carpeta del bag: {args.bag}')
    os.makedirs(args.salida, exist_ok=True)

    lector, tipos = abrir(args.bag)
    print('Tópicos en el bag:')
    for n, t in tipos.items():
        print(f'  {n}  ({t})')

    f_cola = open(os.path.join(args.salida, 'queue_state.csv'), 'w', newline='')
    w_cola = csv.writer(f_cola)
    w_cola.writerow(['t_ns', 'executing_client', 'executing_goal_id', 'executing_elapsed_s',
                     'queue_length', 'queued_goal_ids', 'queued_clients',
                     'queued_priorities', 'queued_wait_s',
                     'total_accepted', 'total_rejected', 'total_completed'])

    f_joint = open(os.path.join(args.salida, 'joint_states.csv'), 'w', newline='')
    w_joint = csv.writer(f_joint)
    w_joint.writerow(['t_ns', 'j1', 'j2', 'j3', 'j4', 'j5', 'j6'])

    n_cola = n_joint = 0
    while lector.has_next():
        topico, datos, t_ns = lector.read_next()
        tipo = tipos.get(topico)
        if not tipo:
            continue
        msg = deserialize_message(datos, get_message(tipo))

        if topico.endswith('queue_state'):
            w_cola.writerow([
                t_ns, msg.executing_client, msg.executing_goal_id,
                f'{msg.executing_elapsed_s:.3f}',
                msg.queue_length,
                '|'.join(msg.queued_goal_ids),
                '|'.join(msg.queued_clients),
                '|'.join(str(p) for p in msg.queued_priorities),
                '|'.join(f'{v:.3f}' for v in msg.queued_wait_s),
                msg.total_accepted, msg.total_rejected, msg.total_completed])
            n_cola += 1
        elif topico.endswith('joint_states'):
            pos = list(msg.position) + [0.0] * 6
            w_joint.writerow([t_ns] + [f'{v:.5f}' for v in pos[:6]])
            n_joint += 1

    f_cola.close()
    f_joint.close()
    print(f'\nqueue_state.csv   {n_cola} filas')
    print(f'joint_states.csv  {n_joint} filas')
    if n_cola == 0:
        print('\nOJO: no se grabó /arm/queue_state. Sin él no hay métricas del ítem 3.')
        print('     ros2 bag record -o corrida /arm/queue_state /joint_states')


if __name__ == '__main__':
    main()
