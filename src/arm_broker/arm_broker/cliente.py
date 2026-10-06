""" Cliente del broker: cada integrante levanta el suyo — Reto 2 """

import csv
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from arm_broker_interfaces.action import MoveArm


# 1. Nodo cliente
class Cliente(Node):
    """Se envían al broker las poses de una traza, una tras otra, y se espera cada resultado"""
    """El cliente NUNCA publica en /joint_states: solo envía goals a la acción move_arm"""

    def __init__(self):
        # Nombre del nodo por defecto; para distinguir a varios clientes se cambia al lanzar:
        # ros2 run arm_broker cliente --ros-args -r __node:=cliente_1 -p client_id:=integrante1
        super().__init__('arm_client')

        # Parámetros de ROS 2: se pasan con --ros-args -p nombre:=valor
        self.declare_parameter('client_id', 'alumno')    # Nombre que aparece en /arm/queue_state
        self.declare_parameter('priority', 1)            # 0..255, mayor número = más urgente
        self.declare_parameter('traza', '')              # CSV de poses; vacío = dos poses de prueba
        self.declare_parameter('repeticiones', 1)        # Vueltas completas a la traza
        self.declare_parameter('pausa_s', 0.5)           # Descanso entre un goal y el siguiente
        self.declare_parameter('modo', 'secuencial')     # secuencial | asincrono
        self.declare_parameter('inicio_unix', 0.0)       # Instante (epoch, s) en que empieza a enviar; 0 = ya

        self.client_id = self.get_parameter('client_id').value
        self.priority = int(self.get_parameter('priority').value)
        self.repeticiones = int(self.get_parameter('repeticiones').value)
        self.pausa = float(self.get_parameter('pausa_s').value)
        self.modo = str(self.get_parameter('modo').value)
        self.inicio_unix = float(self.get_parameter('inicio_unix').value)
        if self.modo not in ('secuencial', 'asincrono'):
            raise ValueError(f"modo desconocido: {self.modo}; use 'secuencial' o 'asincrono'")

        self.cli = ActionClient(self, MoveArm, 'move_arm')
        self.poses = self.cargar(self.get_parameter('traza').value)

    # 2. Carga de la traza
    def cargar(self, ruta):
        """Se leen las poses de un CSV: exactamente seis ángulos q1..q6 en rad por fila"""
        """Las filas vacías y las que empiezan con # se ignoran"""
        """Una fila con otra cantidad de valores, o con algo que no sea un número, es un error"""
        """Retorna una lista de poses, cada una una lista de 6 floats"""
        if not ruta:
            return [[0.3, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [-0.3, 0.0, 0.0, 0.0, 0.0, 0.0]]

        poses = []
        with open(ruta, newline='') as f:
            for n, fila in enumerate(csv.reader(f), start=1):
                if not fila or fila[0].lstrip().startswith('#'):
                    continue

                # Cada pose debe traer exactamente seis valores
                if len(fila) != 6:
                    raise ValueError(
                        f'{ruta}, línea {n}: cada pose debe tener 6 ángulos; '
                        f'llegaron {len(fila)}'
                    )

                # Cada valor debe ser un número finito
                try:
                    q = [float(v) for v in fila]
                except ValueError:
                    raise ValueError(f'{ruta}, línea {n}: hay un valor que no es un número: {fila}')
                if not all(math.isfinite(v) for v in q):
                    raise ValueError(f'{ruta}, línea {n}: hay un valor NaN o infinito: {fila}')

                poses.append(q)
        return poses

    # 3. Envío de goals
    def esperar_inicio(self):
        """Se espera hasta el instante `inicio_unix` antes de enviar el primer goal"""
        """Sirve para que todos los clientes de una corrida arranquen igual, aunque cada
        `ros2 run` tarde distinto en levantar"""
        falta = self.inicio_unix - time.time()
        if falta > 0:
            self.get_logger().info(f'[{self.client_id}] enviando en {falta:.1f}s...')
            time.sleep(falta)

    def armar_goal(self, q):
        """Se arma el goal con la pose q, el nombre y la prioridad de este cliente"""
        goal = MoveArm.Goal()
        goal.joint_positions = q
        goal.client_id = self.client_id
        goal.priority = self.priority
        return goal

    def correr(self):
        """Se envía la traza al broker en el modo elegido: secuencial o asíncrono"""
        """Retorna 0 si terminó, o 1 si el broker no apareció"""
        self.get_logger().info(f'[{self.client_id}] esperando al broker...')
        if not self.cli.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('El broker no aparece. ¿Está corriendo?')
            return 1

        self.esperar_inicio()

        if self.modo == 'asincrono':
            self.correr_asincrono()
        else:
            self.correr_secuencial()
        return 0

    def correr_secuencial(self):
        """Se envía cada pose y se espera su resultado antes de mandar la siguiente"""
        """Sirve para un movimiento básico: este cliente nunca tiene más de un goal en el broker"""
        # Se recorre la traza completa tantas veces como indique `repeticiones`
        for vuelta in range(self.repeticiones):
            for i, q in enumerate(self.poses):
                goal = self.armar_goal(q)

                # Se envía el goal; el broker responde enseguida si lo acepta o lo rechaza
                t0 = time.time()
                envio = self.cli.send_goal_async(goal, feedback_callback=self.feedback)
                rclpy.spin_until_future_complete(self, envio)
                handle = envio.result()

                # Un goal rechazado lleva su motivo en el log del broker y en rechazos.csv
                if not handle.accepted:
                    self.get_logger().warn(f'[{self.client_id}] pose {i}: RECHAZADA')
                    continue

                # Se espera el resultado: incluye cuánto esperó en la cola y cuánto tardó en ejecutarse
                res_fut = handle.get_result_async()
                rclpy.spin_until_future_complete(self, res_fut)
                r = res_fut.result().result
                self.get_logger().info(
                    f'[{self.client_id}] pose {i}: success={r.success} '
                    f'espera={r.wait_time_s:.2f}s ejec={r.exec_time_s:.2f}s '
                    f'total={time.time() - t0:.2f}s — {r.message}')
                time.sleep(self.pausa)

    def correr_asincrono(self):
        """Se envían todos los goals seguidos, sin esperar a que se ejecuten, y después se
        esperan los resultados"""
        """Deja varios pedidos de este cliente pendientes a la vez en el broker: es la carga que
        hace falta para que FIFO y Round Robin atiendan en órdenes distintos"""
        # Cuántos resultados faltan por llegar; los callbacks lo van descontando
        self.faltan = 0

        # Fase 1: se envía toda la traza; cada goal se acepta o se rechaza al instante
        for vuelta in range(self.repeticiones):
            for i, q in enumerate(self.poses):
                envio = self.cli.send_goal_async(self.armar_goal(q),
                                                 feedback_callback=self.feedback)
                rclpy.spin_until_future_complete(self, envio)
                handle = envio.result()

                if not handle.accepted:
                    self.get_logger().warn(f'[{self.client_id}] pose {i}: RECHAZADA')
                    continue

                # El resultado se informa cuando llegue, sin bloquear el envío de los demás
                self.faltan += 1
                res_fut = handle.get_result_async()
                res_fut.add_done_callback(lambda fut, i=i: self.resultado(i, fut))
                time.sleep(self.pausa)

        self.get_logger().info(
            f'[{self.client_id}] enviados; esperando {self.faltan} resultados...')

        # Fase 2: se gira el nodo hasta que hayan llegado todos los resultados
        while self.faltan > 0 and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def resultado(self, i, fut):
        """Se informa el resultado de la pose i cuando llega (modo asíncrono)"""
        r = fut.result().result
        self.faltan -= 1
        self.get_logger().info(
            f'[{self.client_id}] pose {i}: success={r.success} '
            f'espera={r.wait_time_s:.2f}s ejec={r.exec_time_s:.2f}s — {r.message}')

    # 4. Feedback del broker
    def feedback(self, msg):
        """Se muestra el estado del goal: QUEUED con su posición en la cola, o EXECUTING"""
        """Se limita a un mensaje por segundo para no llenar la terminal"""
        f = msg.feedback
        self.get_logger().info(
            f'[{self.client_id}] {f.state} pos={f.queue_position} t={f.elapsed_s:.1f}s',
            throttle_duration_sec=1.0)


# 5. Punto de entrada
def main(args=None):
    """Se inicia ROS 2, se corre el cliente hasta terminar y se sale con su código de retorno"""
    rclpy.init(args=args)

    # Una traza mal formada o un modo desconocido se informan y terminan el programa
    try:
        nodo = Cliente()
    except (ValueError, OSError) as e:
        print(f'ERROR: {e}', file=sys.stderr)
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(2)

    codigo = 0
    try:
        codigo = nodo.correr()
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(codigo)


if __name__ == '__main__':
    main()
