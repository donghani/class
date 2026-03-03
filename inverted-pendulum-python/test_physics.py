import numpy as np

M = 3.0
m = 0.5
l = 1.0
g = 9.81
dt = 0.02

Pos_Kp = 0.2
Pos_Kd = 0.2

Ang_Kp = 150.0
Ang_Kd = 40.0

y = np.array([0.0, 0.0, 0.1, 0.0])

for i in range(300):
    x, v, theta, omega = y
    
    target_theta = Pos_Kp * (0 - x) + Pos_Kd * (0 - v)
    target_theta = np.clip(target_theta, -0.2, 0.2)
    
    F = -Ang_Kp * (target_theta - theta) - Ang_Kd * (0 - omega)
    F = np.clip(F, -100, 100)
    
    sin_th = np.sin(theta)
    cos_th = np.cos(theta)
    denom = M + m - m * cos_th**2
    
    x_ddot = (F + m * l * omega**2 * sin_th - m * g * sin_th * cos_th) / denom
    theta_ddot = (g * sin_th - x_ddot * cos_th) / l
    
    y[0] += y[1] * dt
    y[1] += x_ddot * dt
    y[2] += y[3] * dt
    y[3] += theta_ddot * dt

print("Final state:", y)
