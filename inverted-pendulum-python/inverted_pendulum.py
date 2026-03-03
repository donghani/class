import numpy as np
import tkinter as tk
from tkinter import ttk
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import time
import os

class PIDController:
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

class InvertedPendulumGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Inverted Pendulum PID Control")
        
        # Physics Constants
        self.M = 3.0   # Cart mass (kg)
        self.m = 0.5   # Bob mass (kg)
        self.l = 1.0   # Rod length (m)
        self.g = 9.81  # Gravity
        
        # State: [x, v, theta, omega]
        self.state = np.zeros(4)
        self.sim_time = 0.0
        self.dt = 0.02
        
        # History
        self.history = {'t': [], 'ke': [], 'pe': [], 'te': []}
        
        self.is_running = False
        self.snapshots_taken = 0
        self.snapshot_times = [1.0, 3.0] # Times to take snapshots

        # PIDs
        # Position PID (outer loop)
        self.pos_pid = PIDController(kp=0.4428, ki=0.1252, kd=2.7749, output_limits=(-0.4, 0.4))
        # Angle PID (inner loop)
        self.ang_pid = PIDController(kp=51.9314, ki=6.8996, kd=68.3069, output_limits=(-200, 200))
        
        self._build_gui()
        
    def _build_gui(self):
        # Control Panel
        control_frame = ttk.Frame(self.root, padding="10")
        control_frame.grid(row=0, column=0, sticky=(tk.N, tk.S, tk.E, tk.W))
        
        # Initial Angle
        ttk.Label(control_frame, text="Initial Angle (rad):").grid(row=0, column=0, sticky=tk.W)
        self.init_angle_var = tk.StringVar(value="0.1")
        ttk.Entry(control_frame, textvariable=self.init_angle_var, width=10).grid(row=0, column=1)

        # Position PID
        ttk.Label(control_frame, text="Position PID (Kp, Ki, Kd):").grid(row=1, column=0, sticky=tk.W, pady=(10,0))
        pos_frame = ttk.Frame(control_frame)
        pos_frame.grid(row=2, column=0, columnspan=2, sticky=tk.W)
        self.pos_p_var = tk.StringVar(value=str(self.pos_pid.kp))
        self.pos_i_var = tk.StringVar(value=str(self.pos_pid.ki))
        self.pos_d_var = tk.StringVar(value=str(self.pos_pid.kd))
        ttk.Entry(pos_frame, textvariable=self.pos_p_var, width=6).pack(side=tk.LEFT, padx=2)
        ttk.Entry(pos_frame, textvariable=self.pos_i_var, width=6).pack(side=tk.LEFT, padx=2)
        ttk.Entry(pos_frame, textvariable=self.pos_d_var, width=6).pack(side=tk.LEFT, padx=2)

        # Angle PID
        ttk.Label(control_frame, text="Angle PID (Kp, Ki, Kd):").grid(row=3, column=0, sticky=tk.W, pady=(10,0))
        ang_frame = ttk.Frame(control_frame)
        ang_frame.grid(row=4, column=0, columnspan=2, sticky=tk.W)
        self.ang_p_var = tk.StringVar(value=str(self.ang_pid.kp))
        self.ang_i_var = tk.StringVar(value=str(self.ang_pid.ki))
        self.ang_d_var = tk.StringVar(value=str(self.ang_pid.kd))
        ttk.Entry(ang_frame, textvariable=self.ang_p_var, width=6).pack(side=tk.LEFT, padx=2)
        ttk.Entry(ang_frame, textvariable=self.ang_i_var, width=6).pack(side=tk.LEFT, padx=2)
        ttk.Entry(ang_frame, textvariable=self.ang_d_var, width=6).pack(side=tk.LEFT, padx=2)

        # Buttons
        ttk.Button(control_frame, text="Start / Restart", command=self.start_sim).grid(row=5, column=0, columnspan=2, pady=20)
        ttk.Button(control_frame, text="Stop", command=self.stop_sim).grid(row=6, column=0, columnspan=2)
        
        # State display
        self.st_var = tk.StringVar(value="Time: 0.0s | x: 0.0 | th: 0.0")
        ttk.Label(control_frame, textvariable=self.st_var).grid(row=7, column=0, columnspan=2, pady=10)

        # Matplotlib Figures
        plot_frame = ttk.Frame(self.root)
        plot_frame.grid(row=0, column=1, sticky=(tk.N, tk.S, tk.E, tk.W))
        
        self.fig = Figure(figsize=(10, 8), dpi=100)
        
        # Animation Axes
        self.ax_anim = self.fig.add_subplot(211)
        self.ax_anim.set_xlim(-2, 2)
        self.ax_anim.set_ylim(-1.5, 1.5)
        self.ax_anim.set_aspect('equal')
        self.ax_anim.set_title('Inverted Pendulum Animation')
        self.ax_anim.grid(True)
        
        # Draw ground
        self.ax_anim.plot([-2, 2], [-0.1, -0.1], 'k-', lw=2)
        
        # Initialize graphics objects
        cart_w, cart_h = 0.4, 0.2
        self.cart_rect = matplotlib.patches.Rectangle((-cart_w/2, -cart_h/2), cart_w, cart_h, fc='blue')
        self.ax_anim.add_patch(self.cart_rect)
        self.rod_line, = self.ax_anim.plot([], [], 'r-', lw=3)
        self.bob_circle = matplotlib.patches.Circle((0,0), 0.1, fc='red')
        self.ax_anim.add_patch(self.bob_circle)
        
        # Energy Axes
        self.ax_energy = self.fig.add_subplot(212)
        self.ax_energy.set_title('Energy over Time')
        self.ax_energy.set_xlabel('Time (s)')
        self.ax_energy.set_ylabel('Energy (J)')
        self.line_ke, = self.ax_energy.plot([], [], label='Kinetic')
        self.line_pe, = self.ax_energy.plot([], [], label='Potential')
        self.line_te, = self.ax_energy.plot([], [], label='Total', linestyle='--')
        self.ax_energy.legend(loc='upper right')
        self.ax_energy.grid(True)
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        
    def start_sim(self):
        try:
            init_th = float(self.init_angle_var.get())
            # Update PIDs
            self.pos_pid.kp = float(self.pos_p_var.get())
            self.pos_pid.ki = float(self.pos_i_var.get())
            self.pos_pid.kd = float(self.pos_d_var.get())
            self.ang_pid.kp = float(self.ang_p_var.get())
            self.ang_pid.ki = float(self.ang_i_var.get())
            self.ang_pid.kd = float(self.ang_d_var.get())
        except ValueError:
            print("Invalid input parameters.")
            return
            
        self.state = np.array([0.0, 0.0, init_th, 0.0])
        self.sim_time = 0.0
        self.history = {'t': [], 'ke': [], 'pe': [], 'te': []}
        self.snapshots_taken = 0
        
        self.pos_pid.reset()
        self.ang_pid.reset()
        
        self.is_running = True
        self.ax_energy.set_xlim(0, 5)
        self.ax_energy.set_ylim(-10, 50)
        self.update_sim()

    def stop_sim(self):
        self.is_running = False
        
    def update_sim(self):
        if not self.is_running:
            return
            
        if self.sim_time > 10.0:
            self.is_running = False
            return
            
        # Physics step
        F = self.compute_control()
        self.step_physics(F)
        self.sim_time += self.dt
        
        # Energy
        ke, pe, te = self.compute_energy()
        self.history['t'].append(self.sim_time)
        self.history['ke'].append(ke)
        self.history['pe'].append(pe)
        self.history['te'].append(te)
        
        # Update graphics
        x, _, th, _ = self.state
        cart_w, cart_h = 0.4, 0.2
        self.cart_rect.set_xy((x - cart_w/2, -cart_h/2))
        
        bob_x = x + self.l * np.sin(th)
        bob_y = self.l * np.cos(th)
        
        self.rod_line.set_data([x, bob_x], [0, bob_y])
        self.bob_circle.set_center((bob_x, bob_y))
        
        # Auto-adjust camera
        if abs(x) > 1.5:
            self.ax_anim.set_xlim(x - 2, x + 2)
        else:
            self.ax_anim.set_xlim(-2, 2)

        # Update energy plots
        self.line_ke.set_data(self.history['t'], self.history['ke'])
        self.line_pe.set_data(self.history['t'], self.history['pe'])
        self.line_te.set_data(self.history['t'], self.history['te'])
        
        max_te = max(self.history['te']) if self.history['te'] else 50
        if max_te > 50:
            self.ax_energy.set_ylim(-10, max_te * 1.2)
        if self.sim_time > 5.0:
            self.ax_energy.set_xlim(0, self.sim_time + 1)
            
        self.st_var.set(f"Time: {self.sim_time:.2f}s | x: {x:.2f} | th: {th:.2f}")
        self.canvas.draw()
        
        # Snapshots
        if self.snapshots_taken < len(self.snapshot_times):
            if self.sim_time >= self.snapshot_times[self.snapshots_taken]:
                filename = f"snapshot_{self.snapshots_taken+1}.png"
                self.fig.savefig(filename)
                print(f"Saved snapshot to {filename}")
                self.snapshots_taken += 1
        
        self.root.after(int(self.dt * 1000), self.update_sim)
        
    def compute_control(self):
        x, v, th, om = self.state
        
        # Position control: target x is 0
        # If x > 0, error is negative.
        pos_error = 0.0 - x
        # Instead of generic derivative error (which relies on target_v - v), we just use error since target_v = 0.
        # But wait, the standard form uses `compute(error, dt)`.
        # Output of pos_pid is target_theta.
        # To move left (pos_error < 0), we want positive Kp to yield negative target_theta (lean left).
        target_th = self.pos_pid.compute(pos_error, self.dt)
        
        # Angle control: target th is target_th
        th_error = target_th - th
        # Output of ang_pid is force.
        # If th_error is positive (th < target_th), we need to lean right.
        # To lean right, cart must push left (negative force). 
        # So we should negate the output if Kp is positive.
        F = -self.ang_pid.compute(th_error, self.dt)
        return F
        
    def step_physics(self, F):
        x, v, th, om = self.state
        sin_th = np.sin(th)
        cos_th = np.cos(th)
        
        denom = self.M + self.m - self.m * cos_th**2
        x_acc = (F + self.m * self.l * om**2 * sin_th - self.m * self.g * sin_th * cos_th) / denom
        th_acc = (self.g * sin_th - x_acc * cos_th) / self.l
        
        # Euler integration
        self.state[0] += v * self.dt
        self.state[1] += x_acc * self.dt
        self.state[2] += om * self.dt
        self.state[3] += th_acc * self.dt
        
    def compute_energy(self):
        x, v, th, om = self.state
        
        v_bob_x = v + self.l * om * np.cos(th)
        v_bob_y = -self.l * om * np.sin(th)
        
        v_bob_sq = v_bob_x**2 + v_bob_y**2
        
        ke = 0.5 * self.M * v**2 + 0.5 * self.m * v_bob_sq
        pe = self.m * self.g * self.l * np.cos(th)
        
        return ke, pe, ke + pe

if __name__ == "__main__":
    root = tk.Tk()
    app = InvertedPendulumGUI(root)
    root.mainloop()
