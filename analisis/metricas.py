#!/usr/bin/env python3
"""Métricas del ítem 3 a partir de queue_state.csv, y la figura comparativa.

Convención de prioridad: MAYOR número = MÁS urgente (la declara MoveArm.action).
El índice de inanición es, por tanto, la espera máxima del número MÁS BAJO.

    python3 metricas.py fifo/queue_state.csv prioridad/queue_state.csv

Calcula espera media y p95 por prioridad, índice de inanición y equidad de Jain.
Entregado completo: es instrumentación, no es lo que evalúa el reto.
"""

import argparse
import csv
import os
import statistics
import sys


def leer(ruta):
    """Devuelve (esperas_por_cliente, esperas_por_prioridad, completados)."""
    vistos = {}
    por_cliente = {}
    por_prioridad = {}
    completados = 0

    with open(ruta, newline='') as f:
        for fila in csv.DictReader(f):
            ids = [g for g in fila['queued_goal_ids'].split('|') if g]
            clientes = [c for c in fila['queued_clients'].split('|') if c]
            prioridades = [p for p in fila['queued_priorities'].split('|') if p]
            esperas = [w for w in fila['queued_wait_s'].split('|') if w]
            for gid, c, p, w in zip(ids, clientes, prioridades, esperas):
                previo = vistos.get(gid)
                espera = float(w)
                if previo is None or espera > previo[2]:
                    vistos[gid] = (c, int(p), espera)
            completados = max(completados, int(fila['total_completed'] or 0))

    for cliente, prioridad, espera in vistos.values():
        por_cliente.setdefault(cliente, []).append(espera)
        por_prioridad.setdefault(prioridad, []).append(espera)
    return por_cliente, por_prioridad, completados


def p95(xs):
    if not xs:
        return 0.0
    ordenados = sorted(xs)
    k = max(0, min(len(ordenados) - 1, int(round(0.95 * (len(ordenados) - 1)))))
    return ordenados[k]


def jain(valores):
    """Equidad de Jain: 1.0 = reparto perfecto, 1/n = uno se lo lleva todo."""
    if not valores or sum(valores) == 0:
        return 0.0
    n = len(valores)
    return (sum(valores) ** 2) / (n * sum(v * v for v in valores))


def jain_esperas(por_cliente):
    """Jain sobre la espera media de cada cliente, no sobre el número de goals.

    El Jain de la línea de arriba usa cuántos goals atendió cada cliente. Si el
    broker acepta todo, ese número es idéntico para los cuatro y el índice da
    1.0 aunque uno espere el triple que otro. Éste sí baja cuando una política
    sacrifica a alguien.
    """
    medias = [statistics.mean(ws) for ws in por_cliente.values() if ws]
    return jain(medias)


def _filas(ruta):
    with open(ruta, newline='') as f:
        for fila in csv.DictReader(f):
            yield fila


def contadores(ruta):
    """Máximos observados de los contadores del broker."""
    acc = rej = comp = 0
    for fila in _filas(ruta):
        acc = max(acc, int(fila['total_accepted'] or 0))
        rej = max(rej, int(fila['total_rejected'] or 0))
        comp = max(comp, int(fila['total_completed'] or 0))
    return acc, rej, comp


def exclusiones(ruta):
    """¿Hubo dos pedidos ejecutándose a la vez? (requisito del ítem 2)

    Busca dos señales en el log de /arm/queue_state:

      - reentrada: el mismo goal_id aparece como ejecutándose en dos tramos
        separados de la traza, o sea que el worker volvió a él sin haberlo
        terminado;
      - doble reserva: un goal_id está en executing_goal_id y también en
        queued_goal_ids en la misma muestra.

    Límite honesto: queue_state se publica cada 200 ms, así que sólo se ven
    solapes de 200 ms o más. Es la resolución de la instrumentación, no una
    limitación del broker.
    """
    tramos = {}
    anterior = None
    for fila in _filas(ruta):
        gid = (fila['executing_goal_id'] or '').strip()
        if gid != anterior:
            if anterior:
                tramos[anterior] = tramos.get(anterior, 0) + 1
            anterior = gid
    if anterior:
        tramos[anterior] = tramos.get(anterior, 0) + 1
    reentrada = sorted(g for g, n in tramos.items() if n > 1)

    solapes = set()
    for fila in _filas(ruta):
        ej = (fila['executing_goal_id'] or '').strip()
        if ej and ej in [g for g in fila['queued_goal_ids'].split('|') if g]:
            solapes.add(ej)

    return reentrada, sorted(solapes)


def resumen(nombre, ruta):
    por_cliente, por_prioridad, completados = leer(ruta)
    todas = [w for ws in por_cliente.values() for w in ws]

    print(f'\n=== {nombre}   ({ruta})')
    if not todas:
        print('  sin datos: ¿se grabó /arm/queue_state?')
        return None

    print(f'  pedidos observados : {len(todas)}')
    print(f'  completados        : {completados}')
    print(f'  espera media       : {statistics.mean(todas):.2f} s')
    print(f'  espera p95         : {p95(todas):.2f} s')
    print(f'  espera máxima      : {max(todas):.2f} s')

    print('  por prioridad:')
    for pr in sorted(por_prioridad):
        ws = por_prioridad[pr]
        print(f'    prioridad {pr}: n={len(ws):3d}  media={statistics.mean(ws):6.2f}s  '
              f'p95={p95(ws):6.2f}s  máx={max(ws):6.2f}s')

    peor = min(por_prioridad) if por_prioridad else None
    inanicion = max(por_prioridad[peor]) if peor is not None else 0.0
    print(f'  índice de inanición: {inanicion:.2f} s  '
          f'(espera máxima de prioridad {peor}, la menos urgente)')

    atendidos = {c: len(ws) for c, ws in sorted(por_cliente.items())}
    print(f'  goals por cliente  : {atendidos}')
    print(f'  equidad de Jain    : {jain(list(atendidos.values())):.3f}  (reparto por número de goals)')

    esperas_medias = {c: statistics.mean(ws) for c, ws in sorted(por_cliente.items())}
    linea = '  '.join(f'{c}={w:.1f}s' for c, w in esperas_medias.items())
    print(f'  espera media       : {linea}')
    print(f'  Jain de esperas    : {jain_esperas(por_cliente):.3f}  '
          f'(reparto por espera; el de arriba no baja si nadie se rechaza)')

    acc, rej, comp = contadores(ruta)
    print(f'  contador broker    : aceptados={acc} rechazados={rej} completados={comp}')

    reentrada, solapes = exclusiones(ruta)
    if reentrada or solapes:
        print(f'  EXCLUSIÓN MUTUA    : FALLA — {len(reentrada)} reentradas, {len(solapes)} doble reserva')
        for gid in (reentrada + solapes)[:5]:
            print(f'      {gid}')
    else:
        print('  exclusión mutua    : OK (0 reentradas, 0 doble reserva; '
              'resolución 200 ms)')

    return {'nombre': nombre, 'todas': todas, 'por_prioridad': por_prioridad,
            'atendidos': atendidos, 'esperas_medias': esperas_medias,
            'inanicion': inanicion,
            'exclusiones': len(reentrada) + len(solapes),
            'rechazados': rej}


def figura(datos, salida='comparacion_politicas.png'):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('\nSin matplotlib; no genero la figura.  pip install matplotlib')
        return

    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 4.2))

    nombres = [d['nombre'] for d in datos]
    medias = [statistics.mean(d['todas']) for d in datos]
    p95s = [p95(d['todas']) for d in datos]
    x = range(len(nombres))
    ax1.bar([i - 0.2 for i in x], medias, 0.4, label='media')
    ax1.bar([i + 0.2 for i in x], p95s, 0.4, label='p95')
    ax1.set_xticks(list(x)); ax1.set_xticklabels(nombres)
    ax1.set_ylabel('espera (s)'); ax1.set_title('Espera por política'); ax1.legend()

    ax2.bar(nombres, [jain(list(d['atendidos'].values())) for d in datos])
    ax2.set_ylim(0, 1.05); ax2.set_ylabel('equidad de Jain')
    ax2.set_title('Reparto entre clientes'); ax2.axhline(1.0, ls='--', lw=0.8)

    ax3.bar([i - 0.2 for i in x], [d['rechazados'] for d in datos], 0.4,
            label='goals rechazados')
    ax3.bar([i + 0.2 for i in x], [d['exclusiones'] for d in datos], 0.4,
            label='violaciones excl. mutua')
    ax3.set_xticks(list(x)); ax3.set_xticklabels(nombres)
    ax3.set_title('Descartes y validación')
    ax3.set_ylabel('pedidos')
    ax3.legend()

    fig.tight_layout()
    fig.savefig(salida, dpi=150)
    print(f'\nFigura: {salida}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csvs', nargs='+', help='un queue_state.csv por política')
    ap.add_argument('--salida', default='comparacion_politicas.png')
    args = ap.parse_args()

    datos = []
    for ruta in args.csvs:
        if not os.path.isfile(ruta):
            sys.exit(f'No encuentro {ruta}')
        r = resumen(os.path.basename(os.path.dirname(os.path.abspath(ruta))) or ruta, ruta)
        if r:
            datos.append(r)

    if len(datos) >= 2:
        figura(datos, args.salida)
    elif datos:
        print('\nCon un solo CSV no hay comparación. Graben una corrida por política.')


if __name__ == '__main__':
    main()
