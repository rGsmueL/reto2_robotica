#!/usr/bin/env python3
"""Genera un CSV de poses para que los cuatro clientes lo reproduzcan.

    python3 generar_carga.py --n 40 --semilla 7 --salida carga.csv

Misma semilla = mismo archivo. Úsenlo para que las corridas de FIFO y de la otra
política sean comparables: si cada corrida usa poses distintas, las métricas no
se pueden comparar.
"""
import argparse, csv, os, random, sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'arm_broker'))
from arm_broker import fk                                        # noqa: E402

RANGOS = [(-0.8, 0.8), (-1.0, -0.2), (0.2, 1.0), (-0.3, 0.3), (-0.5, 0.5), (-0.5, 0.5)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=40)
    ap.add_argument('--semilla', type=int, default=7)
    ap.add_argument('--salida', default='carga.csv')
    args = ap.parse_args()

    random.seed(args.semilla)
    filas, intentos = [], 0
    while len(filas) < args.n and intentos < args.n * 50:
        intentos += 1
        q = [round(random.uniform(lo, hi), 4) for lo, hi in RANGOS]
        if fk.dentro_de_limites(q)[0] and fk.dentro_del_workspace(q)[0]:
            filas.append(q)

    with open(args.salida, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['# q1', 'q2', 'q3', 'q4', 'q5', 'q6  (radianes)'])
        w.writerows(filas)
    print(f'{len(filas)} poses alcanzables en {args.salida}  (semilla {args.semilla})')


if __name__ == '__main__':
    main()
