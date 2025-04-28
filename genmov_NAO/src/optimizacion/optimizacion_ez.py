import numpy as np
import casadi as ca
import matplotlib.pyplot as plt


# Parámetros
N = 30  # Número de puntos
l1, l2 = 1.0, 0.9
A = np.array([1.7, -0.2])
E = np.array([1.2, 2.0])

# Variables de decisión: q = [q1_0, q2_0, ..., q1_N, q2_N]
q = ca.MX.sym("q", 2*(N+1))  # 2 ángulos por instante de tiempo

# Función de cinemática directa
def fkin(q1, q2):
    x = l1 * ca.cos(q1) + l2 * ca.cos(q1 + q2)
    y = l1 * ca.sin(q1) + l2 * ca.sin(q1 + q2)
    return ca.vertcat(x, y)

# Función objetivo (error total con la trayectoria deseada)
cost = 0
for k in range(N+1):
    q1k = q[2*k]
    q2k = q[2*k+1]
    xk = fkin(q1k, q2k)
    xd = A + (E - A) * k / N  # punto deseado (interpolación lineal)
    cost += ca.norm_2(xk - xd)  # suma de distancias

# Definir restricciones de caja
lbx = [0] * (2*(N+1))         # límite inferior
ubx = [np.pi] * (2*(N+1))     # límite superior

# Configurar el solver
nlp = {"x": q, "f": cost}
opts = {"ipopt.print_level": 0, "print_time": 0}
solver = ca.nlpsol("solver", "ipopt", nlp, opts)

# Resolver
sol = solver(x0=np.zeros(2*(N+1)), lbx=lbx, ubx=ubx)
q_opt = sol["x"].full().reshape(2, N+1)  # 2 filas: q1, q2

# Trayectoria cartesiana resultante
X = []
for k in range(N+1):
    q1, q2 = q_opt[0, k], q_opt[1, k]
    xk = fkin(q1, q2)
    X.append(ca.evalf(xk).full().flatten())
X = np.array(X).T  # 2 x (N+1)

# Graficar ángulos
plt.figure()
plt.plot(q_opt[0, :], label='q1')
plt.plot(q_opt[1, :], label='q2')
plt.legend()
plt.title("Ángulos articulares")
plt.grid()


# Graficar trayectoria cartesiana
plt.figure()
plt.plot(X[0], X[1], label="Trayectoria generada")
plt.plot([A[0], E[0]], [A[1], E[1]], 'or-', label="Trayectoria deseada")
plt.axis([0, 2, 0, 2])
plt.legend()
plt.title("Trayectoria en el espacio cartesiano")
plt.grid()
plt.show()
