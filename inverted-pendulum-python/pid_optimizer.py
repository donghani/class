import numpy as np
import scipy.optimize as opt
import csv

# Physics Constants
M = 3.0   # Cart mass (kg)
m = 0.5   # Bob mass (kg)
l = 1.0   # Rod length (m)
g = 9.81  # Gravity
dt = 0.02
sim_duration = 10.0
steps = int(sim_duration / dt)

class PIDController_Headless:
    def __init__(self, kp, ki, kd, output_limits=(None, None)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0
        self.min_out, self.max_out = output_limits
        
    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        
    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        
        if self.min_out is not None:
            output = max(self.min_out, output)
        if self.max_out is not None:
            output = min(self.max_out, output)
        return output

def simulate(params):
    # params: [pos_kp, pos_ki, pos_kd, ang_kp, ang_ki, ang_kd]
    pos_pid = PIDController_Headless(params[0], params[1], params[2], output_limits=(-0.4, 0.4))
    ang_pid = PIDController_Headless(params[3], params[4], params[5], output_limits=(-200, 200))
    
    # Initial state: x=0, v=0, theta=0.1, omega=0
    state = np.array([0.0, 0.0, 0.1, 0.0])
    
    total_cost = 0.0
    
    for step in range(steps):
        t = step * dt
        x, v, th, om = state
        
        pos_error = 0.0 - x
        target_th = pos_pid.compute(pos_error, dt)
        
        th_error = target_th - th
        F = -ang_pid.compute(th_error, dt)
        
        # Penalties: we want x->0, th->0.
        if abs(th) > np.pi/2 or abs(x) > 10.0:
            total_cost += 100000.0 * (steps - step) # Huge penalty for falling / running away
            break
            
        # Cost is ITAE (Integral of Time-weighted Absolute Error) + LQR style penalty
        # Adding penalty on v, om, and F to suppress high frequency vibrations and instability
        cost_step = (t + 1) * (abs(x) + 20.0 * abs(th)) + 0.5 * v**2 + 2.0 * om**2 + 0.001 * F**2
        total_cost += cost_step
        
        # Physics Euler Step
        sin_th = np.sin(th)
        cos_th = np.cos(th)
        
        denom = M + m - m * cos_th**2
        x_acc = (F + m * l * om**2 * sin_th - m * g * sin_th * cos_th) / denom
        th_acc = (g * sin_th - x_acc * cos_th) / l
        
        state[0] += v * dt
        state[1] += x_acc * dt
        state[2] += om * dt
        state[3] += th_acc * dt
        
    return total_cost

def get_trajectory(params):
    pos_pid = PIDController_Headless(params[0], params[1], params[2], output_limits=(-0.4, 0.4))
    ang_pid = PIDController_Headless(params[3], params[4], params[5], output_limits=(-200, 200))
    state = np.array([0.0, 0.0, 0.1, 0.0])
    trajectory = []
    
    for step in range(steps):
        t = step * dt
        x, v, th, om = state
        trajectory.append((t, x, th))
        
        if abs(th) > np.pi/2 or abs(x) > 10.0:
            break
            
        pos_error = 0.0 - x
        target_th = pos_pid.compute(pos_error, dt)
        th_error = target_th - th
        F = -ang_pid.compute(th_error, dt)
        
        sin_th = np.sin(th)
        cos_th = np.cos(th)
        denom = M + m - m * cos_th**2
        x_acc = (F + m * l * om**2 * sin_th - m * g * sin_th * cos_th) / denom
        th_acc = (g * sin_th - x_acc * cos_th) / l
        
        state[0] += v * dt
        state[1] += x_acc * dt
        state[2] += om * dt
        state[3] += th_acc * dt
        
    return trajectory

def run_optimization():
    print("Starting PID optimization with Differential Evolution (this may take a minute)...")
    
    # Bounds to keep gains reasonable
    bounds = [
        (0.0, 5.0),    # pos_kp
        (0.0, 2.0),    # pos_ki
        (0.0, 5.0),    # pos_kd
        (50.0, 300.0), # ang_kp
        (0.0, 50.0),   # ang_ki
        (10.0, 100.0)  # ang_kd
    ]
    
    # Global search
    result = opt.differential_evolution(simulate, bounds, maxiter=20, popsize=5, disp=True, strategy='best1bin')
    
    best_params = result.x
    print("\nOptimization Finished!")
    print(f"Optimal PID Parameters:")
    print(f"  Position Loop: Kp={best_params[0]:.4f}, Ki={best_params[1]:.4f}, Kd={best_params[2]:.4f}")
    print(f"  Angle Loop:    Kp={best_params[3]:.4f}, Ki={best_params[4]:.4f}, Kd={best_params[5]:.4f}")
    
    # Run one last simulation to get the trajectory
    traj = get_trajectory(best_params)
    
    # Save the trajectory
    filename = 'error_log.csv'
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Time(s)', 'Position_Error(m)', 'Angle_Error(rad)'])
        for t, x, th in traj:
            writer.writerow([f"{t:.2f}", f"{x:.6f}", f"{th:.6f}"])
            
    print(f"\nSaved 10-second error trajectory to {filename}")

if __name__ == '__main__':
    run_optimization()
