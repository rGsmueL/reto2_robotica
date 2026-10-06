#!/usr/bin/env python3
"""Simula una corrida y devuelve las metricas que despues se mediran.

El enunciado pide entregar, FIRMADO y ANTES de medir, el diseno previo con la
prediccion del p95 por politica. Esa prediccion sale de aqui: la simulacion usa
el MISMO politicas.py que corre el broker, de modo que no puede apartarse del
broker por un detalle de politica.

    python3 simular_corrida.py --politica fifo
    python3 simular_corrida.py --ambos --salida prediccion.md

Que simule y no calcule la verdad: el broker real tiene latencias de red,
feedback a 5 Hz y el disco de por medio. La simulacion predice el ORDEN y la
magnitud de las diferencias entre politicas, no el digito exacto. El contraste
contra lo medido va en DISENO_PREVIO.md, seccion 6.
"""

import argparse
import csv
import math
import os
import statistics
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'arm_broker'))
from arm_broker import fk                                        # noqa: E402
from arm_broker.politicas import POLITICAS                      # noqa: E402

INFINITO = float('inf')


# 1. La traza oficial
def leer_traza(ruta):
    """Devuelve la lista de poses del CSV, en radianes, en el orden del archivo."""
    poses = []
    with open(ruta, newline='') as f:
        for fila in csv.reader(f):
            if not fila or fila[0].lstrip().startswith('#'):
                continue
            poses.append([float(v) for v in fila])
    return poses


# 2. Un cliente
class ClienteSim:
    """Programa los instantes en que un cliente manda sus goals.

    `secuencial`: manda uno, espera el resultado, duerme `pausa_s` y manda el
    siguiente. Nunca tiene mas de un goal pendiente, asi que la cola se queda
    corta y las esperas miden contencion, no saturacion.

    `asincrono`: manda toda la traza sin esperar resultados, con `pausa_s` entre
    envios. Deja decenas de pedidos pendientes de golpe y la cola se llena: es lo
    que hace que una politica de prioridad tenga algo que decidir.
    """

    def __init__(self, cliente_id, prioridad, poses, repeticiones, pausa,
                 modo, t_inicio):
        self.cliente_id = cliente_id
        self.prioridad = prioridad
        self.pausa = pausa
        self.modo = modo

        objetivos = list(poses) * repeticiones
        self.objetivos = objetivos
        self.indice = 0            # proximo goal por mandar
        self.t_envio = t_inicio    # instante del proximo envio
        self.en_vuelo = False      # secuencial: hay un goal sin resultado

    def instante_envio(self):
        """Instante del proximo envio, o INFINITO si todavia no puede mandar."""
        if self.indice >= len(self.objetivos):
            return INFINITO
        if self.modo == 'secuencial' and self.en_vuelo:
            return INFINITO
        return self.t_envio

    def tomar(self):
        """Se consume el proximo goal y se devuelve su pose."""
        q = self.objetivos[self.indice]
        self.indice += 1
        if self.modo == 'secuencial':
            self.en_vuelo = True
        else:
            self.t_envio += self.pausa
        return q

    def completado(self, t_fin):
        """El goal de este cliente termino: en secuencial programa el siguiente."""
        if self.modo == 'secuencial':
            self.en_vuelo = False
            self.t_envio = t_fin + self.pausa


# 3. Lo que la política ve
class PedidoSim:
    """El mismo contrato que Pedido del broker: la política solo mira estos campos.

    Ojo al orden, que es el mismo que en broker.py: `siguiente()` se consulta ANTES de
    fijar `t_inicio_ejec`, así que al puntuar, `espera_s` cae al respaldo "ahora". Por eso
    el reloj de la simulación es una referencia a la corrida y no un 0.0 cualquiera.
    """

    def __init__(self, t_llegada, cliente, joint_positions, corrida):
        self.t_llegada = t_llegada
        self.client_id = cliente.cliente_id
        self.priority = cliente.prioridad
        self.joint_positions = joint_positions
        self.t_inicio_ejec = None
        self._corrida = corrida

    @property
    def espera_s(self):
        fin = self.t_inicio_ejec if self.t_inicio_ejec is not None else self._corrida.t
        return fin - self.t_llegada



# 4. La corrida
class Corrida:
    def __init__(self, politica, cola_max, paso_max, duracion):
        self.politica = politica
        self.cola_max = cola_max
        self.paso_max = paso_max
        self.duracion = duracion

        self.q_actual = [0.0] * 6    # el broker arranca en el origen
        self.pendientes = []         # [PedidoSim]
        self.t = 0.0                 # reloj de la simulación, en segundos
        self.ocupado_hasta = 0.0     # cuándo el brazo vuelve a estar libre
        self.aceptados = 0
        self.rechazados = []         # (t, cliente, causa)
        self.completados = []        # (cliente, prioridad, espera_s, ejec_s)
        self.pico_cola = 0

    def validar(self, q):
        """La misma validación del broker, en el mismo orden."""
        if any(not math.isfinite(v) for v in q):
            return 'limite'
        if not fk.dentro_de_limites(q)[0]:
            return 'limite'
        if not fk.dentro_del_workspace(q)[0]:
            return 'workspace'
        if fk.paso_articular(self.q_actual, q) > self.paso_max:
            return 'paso'
        return None

    def admitir(self, t, cliente, q):
        causa = self.validar(q)
        if causa is None and len(self.pendientes) >= self.cola_max:
            causa = 'cola_llena'
        if causa is not None:
            self.rechazados.append((t, cliente.cliente_id, causa))
            return False
        self.pendientes.append(PedidoSim(t, cliente, q, self))
        self.pico_cola = max(self.pico_cola, len(self.pendientes))
        self.aceptados += 1
        return True

    def desencolar(self, t):
        """El worker saca un pedido de la cola y lo ejecuta entero.

        La política devuelve el ÍNDICE dentro de pendientes, igual que en el broker:
        se saca el objeto antes de borrarlo de la lista.
        Devuelve el cliente_id del pedido ejecutado, o None si la cola estaba vacía.
        """
        i = self.politica.siguiente(self.pendientes)
        if i is None:
            return None

        elegido = self.pendientes.pop(i)
        elegido.t_inicio_ejec = t
        self.ocupado_hasta = t + self.duracion

        # La revalidación del paso que hace el broker al ejecutar: la pose de
        # referencia pudo haber cambiado por los pedidos de delante
        if fk.paso_articular(self.q_actual, elegido.joint_positions) > self.paso_max:
            self.rechazados.append((t, elegido.client_id, 'paso_al_ejecutar'))
            return elegido.client_id

        espera = t - elegido.t_llegada
        self.completados.append((elegido.client_id, elegido.priority, espera,
                                 self.duracion))
        self.q_actual = list(elegido.joint_positions)
        self.politica.atendido(elegido)
        return elegido.client_id


def correr(politica_nombre, tau, clientes, cola_max, paso_max, duracion, t_limite):
    """Bucle de eventos: el tiempo salta al próximo envío o a que el brazo quede libre.

    `ocupado_hasta` solo cuenta como evento si hay cola: con la cola vacía el brazo ya
    está libre y el único reloj que importa es el próximo envío. Si no, el mínimo entre
    ambos tiempos no avanza nunca y el bucle se cuelga.
    """
    if politica_nombre == 'fifo':
        politica = POLITICAS['fifo']()
    else:
        politica = POLITICAS[politica_nombre](tau)

    c = Corrida(politica, cola_max, paso_max, duracion)
    t = 0.0

    while t < t_limite:
        prox = min(cl.instante_envio() for cl in clientes)

        if not c.pendientes:
            if prox == INFINITO:
                break                               # nadie manda ni queda cola
            t = max(t, prox)                        # brazo libre: se espera el envío
        else:
            # con cola, ocurre lo que sea primero: un envío o que el brazo se libere
            opciones = [x for x in (prox, c.ocupado_hasta) if x != INFINITO]
            t = max(t, min(opciones))

        c.t = t
        for cl in clientes:
            while cl.instante_envio() <= t:
                c.admitir(t, cl, cl.tomar())

        if c.pendientes and t >= c.ocupado_hasta:
            cliente_id = c.desencolar(t)
            if cliente_id is not None:
                for cl in clientes:
                    if cl.cliente_id == cliente_id:
                        cl.completado(t + duracion)

    return c



# 5. Métricas
def p95(xs):
    if not xs:
        return 0.0
    o = sorted(xs)
    return o[max(0, min(len(o) - 1, round(0.95 * (len(o) - 1))))]


def jain(vals):
    """Equidad de Jain: 1.0 reparto perfecto, 1/n uno se lo lleva todo."""
    if not vals or sum(vals) == 0:
        return 0.0
    return (sum(vals) ** 2) / (len(vals) * sum(v * v for v in vals))


def metricas(c, politica_nombre):
    esperas = [x[2] for x in c.completados]
    por_cli, por_pri = {}, {}
    for cli, pri, esp, _ in c.completados:
        por_cli.setdefault(cli, []).append(esp)
        por_pri.setdefault(pri, []).append(esp)

    peor = min(por_pri) if por_pri else None
    causas = {}
    for _, _, causa in c.rechazados:
        causas[causa] = causas.get(causa, 0) + 1

    return {
        'politica': politica_nombre,
        'aceptados': c.aceptados,
        'rechazados': len(c.rechazados),
        'causas': causas,
        'completados': len(c.completados),
        'pico_cola': c.pico_cola,
        'espera_media': statistics.mean(esperas) if esperas else 0.0,
        'espera_p95': p95(esperas),
        'espera_max': max(esperas) if esperas else 0.0,
        'por_prioridad': {k: (len(v), statistics.mean(v), p95(v), max(v))
                          for k, v in sorted(por_pri.items())},
        'inanicion': max(por_pri[peor]) if peor is not None else 0.0,
        'peor_prioridad': peor,
        'jain_goals': jain([len(v) for v in por_cli.values()]),
        'jain_esperas': jain([statistics.mean(v) for v in por_cli.values()]),
        'medias_por_cliente': {k: statistics.mean(v) for k, v in sorted(por_cli.items())},
    }


# 6. Armado y salida
def armar_clientes(args, traza):
    return [ClienteSim(f'c{i + 1}', p, traza, args.repeticiones, args.pausa_s,
                       args.modo, 0.0)
            for i, p in enumerate(args.prioridades[:args.n_clientes])]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--traza', default='carga.csv')
    ap.add_argument('--n-clientes', type=int, default=4)
    ap.add_argument('--prioridades', default='1,2,3,4')
    ap.add_argument('--politica', default='fifo', choices=list(POLITICAS))
    ap.add_argument('--ambos', action='store_true', help='corre las dos políticas')
    ap.add_argument('--tau', type=float, default=12.0)
    ap.add_argument('--duracion', type=float, default=1.5,
                    help='duracion_movimiento_s del broker')
    ap.add_argument('--pausa-s', type=float, default=2.0, dest='pausa_s')
    ap.add_argument('--repeticiones', type=int, default=1)
    ap.add_argument('--cola-max', type=int, default=20)
    ap.add_argument('--paso-max', type=float, default=1.6)
    ap.add_argument('--modo', default='secuencial',
                    choices=['secuencial', 'asincrono'])
    ap.add_argument('--salida')
    args = ap.parse_args()

    args.prioridades = [int(x) for x in args.prioridades.split(',')]
    traza = leer_traza(args.traza)
    politicas = list(POLITICAS) if args.ambos else [args.politica]

    # Tope generouso: una pasada no puede pasar de unos goals por cliente
    n_goals = len(traza) * args.repeticiones * args.n_clientes
    t_limite = 20.0 * n_goals * (args.duracion + args.pausa_s) + 1000.0

    filas = []
    for pol in politicas:
        clientes = armar_clientes(args, traza)
        c = correr(pol, args.tau, clientes, args.cola_max, args.paso_max,
                   args.duracion, t_limite)
        filas.append(metricas(c, pol))

    if args.salida:
        escribir(args.salida, filas, args)
    imprimir(filas, args, traza)
    return 0


def imprimir(filas, args, traza):
    print(f'\ntraza={args.traza} ({len(traza)} poses)  clientes={args.n_clientes}  '
          f'prioridades={args.prioridades}')
    print(f'modo={args.modo}  duracion={args.duracion}s  pausa={args.pausa_s}s  '
          f'cola_max={args.cola_max}  paso_max={args.paso_max}  tau={args.tau}')
    for m in filas:
        print(f'\n=== {m["politica"]}')
        print(f'  aceptados / rechazados / completados : '
              f'{m["aceptados"]} / {m["rechazados"]} / {m["completados"]}')
        if m['causas']:
            print(f'  rechazos por causa                   : {m["causas"]}')
        print(f'  pico de cola                         : {m["pico_cola"]}')
        print(f'  espera media / p95 / máxima          : '
              f'{m["espera_media"]:.2f} / {m["espera_p95"]:.2f} / '
              f'{m["espera_max"]:.2f} s')
        print(f'  indice de inanicion (prioridad {m["peor_prioridad"]})    : '
              f'{m["inanicion"]:.2f} s')
        print(f'  Jain goals / Jain esperas            : '
              f'{m["jain_goals"]:.3f} / {m["jain_esperas"]:.3f}')
        print(f'  espera media por cliente             : '
              f'{ {k: round(v, 2) for k, v in m["medias_por_cliente"].items()} }')
        print('  por prioridad:')
        for pr, (n, med, p, mx) in m['por_prioridad'].items():
            print(f'    {pr}: n={n:3d} media={med:6.2f}s p95={p:6.2f}s max={mx:6.2f}s')


def escribir(ruta, filas, args):
    """Deja la prediccion en markdown, para pegarla en DISENO_PREVIO.md."""
    with open(ruta, 'w', encoding='utf-8') as f:
        f.write('# Prediccion (simulacion) — escrita ANTES de medir\n\n')
        f.write(f'- traza: `{args.traza}`, {args.n_clientes} clientes, '
                f'prioridades {args.prioridades}\n')
        f.write(f'- modo `{args.modo}`, `duracion_movimiento_s={args.duracion}`, '
                f'`pausa_s={args.pausa_s}`\n')
        f.write(f'- `cola_max={args.cola_max}`, `paso_max_rad={args.paso_max}`, '
                f'`tau={args.tau}`\n\n')
        f.write('| politica | aceptados | rechazados | pico cola | media | p95 | max '
                '| inanicion | Jain goals | Jain esperas |\n')
        f.write('|---|---|---|---|---|---|---|---|---|---|\n')
        for m in filas:
            f.write(f'| {m["politica"]} | {m["aceptados"]} | {m["rechazados"]} | '
                    f'{m["pico_cola"]} | {m["espera_media"]:.2f} | '
                    f'{m["espera_p95"]:.2f} | {m["espera_max"]:.2f} | '
                    f'{m["inanicion"]:.2f} | {m["jain_goals"]:.3f} | '
                    f'{m["jain_esperas"]:.3f} |\n')
        f.write('\n## Espera media por cliente\n\n')
        for m in filas:
            celdas = ' | '.join(f'{v:.2f}' for v in m['medias_por_cliente'].values())
            f.write(f'- **{m["politica"]}**: {celdas}\n')
    print(f'\nPrediccion escrita en {ruta}')


if __name__ == '__main__':
    sys.exit(main())
