""" Cinemática directa del JetCobot — Ítem 1 del Reto 2 """

import math

# 1. Nombres de las articulaciones
JOINT_NAMES = [
    '1_Joint',
    '2_Joint',
    '3_Joint',
    '4_Joint',
    '5_Joint',
    '6_Joint',
]

# 2. Tabla Denavit Hartengber

# Tabla original:

# i     theta         d [mm]    a [mm]    alpha
# 1     q1            134.75       0       +90°
# 2     q2 - 90°        0       -110        0°
# 3     q3              0        -96        0°
# 4     q4 - 90°       63.4        0       +90°
# 5     q5 + 90°      75.55        0       -90°
# 6     q6             50          0        0°

# Las columnas son guardadas en este orden: alpha, a, d, offset_theta
DH = [
    ( math.pi / 2,    0.0, 134.75,           0.0),   # J1
    (         0.0, -110.0,   0.00, -math.pi / 2),    # J2
    (         0.0,  -96.0,   0.00,           0.0),   # J3
    ( math.pi / 2,    0.0,  63.40, -math.pi / 2),    # J4
    (-math.pi / 2,    0.0,  75.55,  math.pi / 2),    # J5
    (         0.0,    0.0,  50.00,           0.0),   # J6
]

# 3. Límites articulares 
# Todas las columnas estan en radianes
JOINT_LIMITS = [
    (-2.93, 2.93),
    (-2.36, 2.36),
    (-2.53, 2.53),
    (-2.58, 2.58),
    (-2.93, 2.93),
    (-3.14, 3.14),
]

# 4. Límites simples dek Workspace
# Filtro geométrico simple utilizado por el broker.
ALCANCE_MIN_MM = 80.0
ALCANCE_MAX_MM = 480.0

# 5. Matriz Homogenea DH Estandar
def _t(alpha, a, d, theta):
    """Se hace la construccion de la matriz A_i de una articulacion usando DH estandar, usando: A_i = Rot_z(theta) * Trans_z(d) * Trans_x(a) * Rot_x(alpha)"""
    """Parametros: alpha -> giro entre z(i-1) y zi alrededor de xi en rad
                   a -> distancia entre ejes z sobre xi en mm
                   d -> desplazamiento sobre z(i-1) en mm
                   theta -> angulo articular total en rad"""
    """Retorna la matriz homogenea 4x4 como lista de listas"""
    ca = math.cos(alpha)
    sa = math.sin(alpha)
    ct = math.cos(theta)
    st = math.sin(theta)

    return [
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,       sa,       ca,      d],
        [0.0,      0.0,      0.0,    1.0],
    ]

# 6. Multiplicacion de Matrices 4x4
def _mul(A, B):
    """Se multiplican dos matrices 4x4"""
    """Esto se hace para no depender de NumPy dentro del paquete ROS 2 """
    return [
        [
            sum(A[i][k] * B[k][j] for k in range(4))
            for j in range(4)
        ]
        for i in range(4)
    ]

# 7. Cinematica Directa : Matriz Completa T_0_6
def fk_matriz(q):
    """Se calcula la matriz homogenea T_0_6 desde la base hasta el efector"""
    """q representa la lista o tupla de 6 angulos artculares en radianes"""
    """Se retorna la matriz homogenea 4x4"""
    """La operacion que se realiza es T_0_6 = A1 * A2 * A3 * A4 * A5 * A6"""
    if len(q) != 6:
        raise ValueError(
            f'fk_matriz(q) requiere 6 articulaciones; solo se recibieron {len(q)}'
        )

    # Matriz identidad 4x4: El punto de partida
    T = [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]

    # Se hace el recorrido de cada fila DH junto al angulo articular correspondiente
    for (alpha, a, d, offset), theta in zip(DH, q):
        theta_dh = theta + offset #El angulo usado por DH incluye el offset geometrico
        A_i = _t(alpha, a, d, theta_dh) #Matriz Individual A_1
        T = _mul(T, A_i) #Acumulacion: T = A1, T = A1*A2,  T= A1*A2*A3, ...

    return T

# 8. Cinematica Directa: Solo la posicion XYZ
def fk(q):
    """Calcula la posición cartesiana (x,y,z) del efector final
    Entrada: q = q[q1,q2,q3,q4,q5,q6] en radianes rad
    Salida: (x,y,z) en milimetros mm 
    La posicion se obtiene de la ultima columna de T_0_6
        |R00 R01 R02 x|
        |R10 R11 R12 y|
    T = |R20 R21 R22 z|
        | 0   0   0  1|
    """
    T = fk_matriz(q)

    x = T[0][3]
    y = T[1][3]
    z = T[2][3]

    return x, y, z

# 9.Validacion de limites articulares
def dentro_de_limites(q):
    """Se comprueba que las seis articulaciones estén dentro de sus límites"""
    if len(q) != 6:
        return False, f'se esperaban 6 ángulos, llegaron {len(q)}'

    for i, (valor, (lo, hi)) in enumerate(zip(q, JOINT_LIMITS)):
        if not lo <= valor <= hi:
            return (
                False,
                f'{JOINT_NAMES[i]} fuera de rango: '
                f'{valor:.3f} rad, límite [{lo}, {hi}]'
            )

    return True, ''

# 10. Validacion Simple del Workspace
def dentro_del_workspace(q):
    """Uso de filtro básico del workspace usando la posición calculada por FK."""
    """Se calcula r = sqrt(x²+y²+z²) y se rechaza si r > alcance_maximo o r < alcance_minimo o z < 0 """
    x, y, z = fk(q)
    r = math.sqrt(x * x + y * y + z * z)

    if r > ALCANCE_MAX_MM:
        return (
            False,
            f'efector a {r:.0f} mm de la base, '
            f'máximo {ALCANCE_MAX_MM:.0f}'
        )

    if r < ALCANCE_MIN_MM:
        return False, f'efector a {r:.0f} mm de la base, demasiado cerca'

    if z < 0.0:
        return False, f'z = {z:.0f} mm: el efector quedaría bajo la base'

    return True, ''

# 11.Paso Articular Maximo
def paso_articular(q_desde, q_hasta):
    """Calculo de la máxima variación articular entre dos configuraciones, en radianes."""
    if len(q_desde) != 6 or len(q_hasta) != 6:
        raise ValueError('paso_articular requiere dos vectores de 6 articulaciones')

    return max(
        abs(b - a)
        for a, b in zip(q_desde, q_hasta)
    )
