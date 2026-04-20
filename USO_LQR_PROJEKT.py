import numpy as np
import scipy.linalg
from scipy import signal, linalg
from scipy.integrate import odeint
import matplotlib.pyplot as plt

U_test, I_ss, W_ss = 3.25, 0.6, 3.85
R_val, L_val, Tau_meas = 2.9, 0.006, 0.130

ke = (U_test - (R_val * I_ss)) / W_ss
kt = ke
b_fric = (kt * I_ss) / W_ss
J_val = Tau_meas * (b_fric + ((ke * kt) / R_val))

A = np.array([
    [0,       1,             0],
    [0,      -b_fric/J_val,  kt/J_val],
    [0,      -ke/L_val,     -R_val/L_val]
])

B_mat = np.array([[0], [0], [1/L_val]])

modes = {
    "SPORT": {
        'Q': [5000, 1, 1],
        'R': 0.1
    },
    "COMFORT": {
        'Q': [100, 10, 1],
        'R': 1.0
    },
    "ECO": {
        'Q': [100, 10, 50],
        'R': 1000.0
    }
}

for mode_name, params in modes.items():


    if mode_name == "COMFORT":
        Q_curr = np.diag(params['Q'])
        R_curr = params['R']
        print(f"TRYB: {mode_name}")
        Q = Q_curr
        R_reg = R_curr

        print(f"  Kary Q -> Poz: {params['Q'][0]}, Vel: {params['Q'][1]}, Curr: {params['Q'][2]}")
        print(f"  Kara R -> Sterowanie: {R_curr}")

print (f"Macierz Q \n Pozycja: {Q[0][0]}, \n Prędkość: {Q[1][1]},\n Prąd: {Q[2][2]}")
print(f"Wartość kary na Sterowanie U: {R_reg}")
P = linalg.solve_continuous_are(A, B_mat, Q, R_reg)
K = (1/R_reg) * B_mat.T @ P

def get_N_bar_continuous(A, B, K, target_state_index):
    A = np.array(A)
    B = np.array(B)
    K = np.array(K)

    C_ref = np.zeros((1, A.shape[0]))
    C_ref[0, target_state_index] = 1
    A_cl = A - B @ K
    inv_A_cl = np.linalg.pinv(A_cl)
    denominator = C_ref @ inv_A_cl @ B
    val = denominator.item()

    if abs(val) < 1e-9:
        return 0.0

    return -1.0 / val

def discretize_model_zoh(A, B, C, D=0, Ts=0.001):
    sys_continuous = (A, B, C, D)
    res = signal.cont2discrete(sys_continuous, dt=Ts, method='zoh')
    return res[0], res[1], res[2], res[3]

def motor_model_closed_loop(x, t, A, B_mat, K, U_max, N_bar, target_value):
    u = -K.dot(x) + N_bar * target_value
    u = np.clip(u, -U_max, U_max)
    dxdt = A.dot(x) + B_mat.flatten() * u
    return dxdt

t = np.linspace(0, 2, 2001)
x0 = [0.0, 0.0, 0.0]
target_index = 0

if target_index == 0:
    print ("Sterowanie Pozycja")
elif target_index == 1:
    print("Sterowanie Prędkością")
elif target_index == 2 :
    print("Sterowanie Prądu")
target_value = 3.28

N_bar = get_N_bar_continuous(A, B_mat, K, target_index)

args = (A, B_mat, K, U_test, N_bar, target_value)
y = odeint(motor_model_closed_loop, x0, t, args=args)

u_plot = []
for stan in y:
    val = -K.dot(stan) + N_bar * target_value
    u_plot.append(np.clip(val, -U_test, U_test))

Ts = 0.001
C = np.array([[1,0,1]])
Ad, Bd, Cd, Dd = discretize_model_zoh(A, B_mat, C, Ts=Ts)

print(f"--- Macierze Dyskretne (Ts = {Ts*1000} ms) ---")
print("Ad:\n", Ad)
print("\nBd:\n", Bd)

def print_c_matrix(name, matrix):
    rows, cols = matrix.shape
    print(f"\n// Macierz {name} [{rows}x{cols}]")
    print(f"float {name}[{rows}][{cols}] = {{")
    for row in matrix:
        line = "    {" + ", ".join(f"{val:.8f}f" for val in row) + "},"
        print(line)
    print("};")

Pd = linalg.solve_discrete_are(Ad, Bd, Q, R_reg)
F = linalg.inv(R_reg + Bd.T @ Pd @ Bd) @ (Bd.T @ Pd @ Ad)
print(f"Gain F:\n{F}")

print("\n--- KOD DO WKLEJENIA DO MIKROKONTROLERA ---")
print_c_matrix("A_discrete", Ad)
print_c_matrix("B_discrete", Bd)
print_c_matrix("F",F)



def get_N_bar_discrete(Ad, Bd, K, target_state_index):
    I = np.eye(Ad.shape[0])
    A_cl = Ad - Bd @ K
    C_ref = np.zeros((1, Ad.shape[0]))
    C_ref[0, target_state_index] = 1
    inv_part = np.linalg.pinv(I - A_cl)
    denominator = C_ref @ inv_part @ Bd
    val = denominator.item()
    if abs(val) < 1e-9:
        return 0.0
    return 1.0 / val

N_bar_d = get_N_bar_discrete(Ad, Bd, F, target_index)

print(f"\n// Stała N_bar (Feedforward Gain)")
print(f"float N_bar = {N_bar_d:.8f}f;")


steps = len(t)
yd = np.zeros((steps, 3))
u_plot_d = np.zeros(steps)
x_curr = np.array(x0)
yd[0] = x_curr

for k in range(steps - 1):
    u_val = -F.dot(x_curr) + N_bar_d * target_value
    u_val = np.clip(u_val, -U_test, U_test)
    u_plot_d[k] = u_val.item()
    x_next = Ad.dot(x_curr) + Bd.flatten() * u_val
    x_curr = x_next
    yd[k+1] = x_curr

u_plot_d[-1] = u_plot_d[-2]

fig, (ax_mech, ax_elec) = plt.subplots(2, 1, figsize=(10, 10), sharex=True)

ax_mech.plot(t, y[:, 0], color="red", label='Położenie (Ciągły)', linewidth=3)
ax_mech.step(t, yd[:, 0], color="black", where='post', linestyle='--', label='Położenie (Dyskretny)', alpha=0.4)
ax_mech.plot(t, y[:, 1], color="green", label='Prędkość (Ciągły)', linewidth=3)
ax_mech.step(t, yd[:, 1], where='post', linestyle='--', label='Prędkość (Dyskretny)', alpha=0.4, color="black")
ax_mech.plot(t, kt * y[:, 2], color="skyblue", label='Moment (Ciągły)', linewidth=3)
ax_mech.step(t, kt * yd[:, 2], where='post', linestyle='--', label='Moment (Dyskretny)', alpha=0.4, color="black")
ax_mech.axhline(target_value, color='red', linestyle=':', label='Cel')
ax_mech.set_title("Część Mechaniczna")
ax_mech.set_ylabel("Wartość")
ax_mech.grid(True)
ax_mech.legend()

ax_elec.plot(t, u_plot, color="red", label='Napięcie (Ciągły)', linewidth=3)
ax_elec.step(t, u_plot_d, where='post', linestyle='--', label='Napięcie (Dyskretny)', alpha=0.4, color="black")
ax_elec.plot(t, y[:, 2], color="green", label='Prąd (Ciągły)', linewidth=3)
ax_elec.step(t, yd[:, 2], where='post', linestyle='--', label='Prąd (Dyskretny)', alpha=0.4, color="black")
ax_elec.plot(t, kt * y[:, 2], color="skyblue", label='Moment (Ciągły)', linewidth=3)
ax_elec.step(t, kt * yd[:, 2], where='post', linestyle='--', label='Moment (Dyskretny)', alpha=0.4, color="black")
ax_elec.set_title("Część Elektryczna")
ax_elec.set_xlabel("Czas [s]")
ax_elec.set_ylabel("Wartość")
ax_elec.grid(True)
ax_elec.legend()

plt.tight_layout()
plt.show()