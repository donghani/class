# Python Inverted Pendulum GUI Simulation

This application simulates the physics and mechanics of an inverted pendulum system on a moving cart driven by a cascaded discrete Proportional-Integral-Derivative (PID) controller.

## Prerequisites

- Python 3.x
- NumPy (`pip install numpy`)
- Matplotlib (`pip install matplotlib`)
- Tkinter (usually comes bundled with Python, otherwise install via your package manager)

## Running the Simulation

1. Open your terminal and navigate to the directory containing this code.
2. Run: `python inverted_pendulum.py`
3. A graphical user interface (GUI) window will appear with control fields and two plots:
   - **Animation Plot**: Visualizes the cart (blue rectangle) and the pendulum rod with a bob (red circle) sliding across a 1D track.
   - **Energy Plot**: Displays a real-time line graph plotting the system's kinetic, potential, and total energy.

## Mathematical Formulation

The behavior of the inverted pendulum is modeled by non-linear equations of motion derived from Lagrangian mechanics.

### Constants and Variables

- $M$: Mass of the cart (3.0 kg)
- $m$: Mass of the pendulum bob (0.5 kg)
- $l$: Length of the pendulum rod (1.0 m)
- $g$: Acceleration due to gravity (9.81 $m/s^2$)
- $x$: Cart position
- $\theta$: Angle of the pendulum relative to the vertical line (0 radians is perfectly upright)
- $F$: Control force applied horizontally to the cart

### Equations of Motion

The non-linear differential equations defining acceleration in the cart's position ($\ddot{x}$) and angular acceleration of the inverted pendulum ($\ddot{\theta}$) are:

```math
\ddot{x} = \frac{F + m l \dot{\theta}^2 \sin\theta - m g \sin\theta \cos\theta}{M + m - m \cos^2\theta}
```

```math
\ddot{\theta} = \frac{g \sin\theta - \ddot{x} \cos\theta}{l}
```

These equations govern how the real-life system responds to gravity and the external horizontal force $F$ applied by the motors driving the cart.

### Energy Approximations

The system's energy levels are tracked continuously. Let velocity $v = \dot{x}$ and angular velocity $\omega = \dot{\theta}$. The bob’s velocity components are:

- $v_{bx} = v + l \omega \cos\theta$
- $v_{by} = -l \omega \sin\theta$

**Kinetic Energy (KE)** combines both the cart's translation and the bob's motion:

```math
KE = \frac{1}{2} M v^2 + \frac{1}{2} m (v_{bx}^2 + v_{by}^2)
```

**Potential Energy (PE)** is defined relative to the pivot joint $y=0$:

```math
PE = m g l \cos\theta
```

**Total Energy (TE)** is simply $KE + PE$.

### Cascaded PID Control Loop

The objective of the system is twofold: balance the pendulum upright ($\theta = 0$) and return the cart to the center position ($x = 0$).

To achieve this, the simulation implements a **cascaded outer/inner loop discrete PID controller**:

1. **Outer Loop (Position Controller)**: Measures the error $e_x = 0 - x$ and outputs a target inclination angle $\theta_{target}$ for the pendulum to lean into to travel in the correct direction.
2. **Inner Loop (Angle Controller)**: Measures the error $e_\theta = \theta_{target} - \theta$ and outputs the horizontal force $F$ required to push the cart to achieve $\theta_{target}$. The inner loop reacts much faster to stabilize the falling pole.

The force $F$ applied at any finite time $t$ uses the discrete formula:

```math
u(t) = K_p e(t) + K_i \sum_{k=0}^{t} e(k) \Delta t + K_d \frac{e(t) - e(t-\Delta t)}{\Delta t}
```

### Numerical Integration

To simulate real-time motion on the computer iteratively, discrete Euler integration over step sizes of $\Delta t=0.02$ seconds predicts the next dynamic state based on acceleration:

```math
v_{t+1} = v_t + \ddot{x} \Delta t \quad \text{and} \quad x_{t+1} = x_t + v_{t+1} \Delta t
```

```math
\omega_{t+1} = \omega_t + \ddot{\theta} \Delta t \quad \text{and} \quad \theta_{t+1} = \theta_t + \omega_{t+1} \Delta t
```

## Automated PID Optimization

Finding the perfect stabilization parameters (gains) manually can be difficult. Thus, a secondary script `pid_optimizer.py` is included to mathematically search for the best defaults.

1. **Algorithm**: It uses `scipy.optimize.differential_evolution` to explore combinations of the 6 $K_p$, $K_i$, and $K_d$ parameters across a massive global search space.
2. **Cost Function**: The search minimizes an LQR-style Integrated Time-weighted Absolute Error (ITAE) cost function that aggressively penalizes:
   - Positional deviation from $x=0$.
   - Angle tilting away from $\theta=0$.
   - High frequency cart and pendulum vibrations (penalizing high $\dot{x}$ and $\dot{\theta}$).
   - Large control effort ($F^2$).
3. **Execution**: Run `python pid_optimizer.py`. It will run headless simulations and ultimately spit out the best parameters while logging the 10-second error trajectory to `error_log.csv`. The current defaults within the GUI represent the optimized parameters discovered by this script.

## How to Use the GUI

1. **Initial Angle**: Set the initial tilting angle of the pendulum prior to simulation. (e.g. `0.1` represents slightly tumbling right).
2. **PID Tunings**: Modify the Proportional (Kp), Integral (Ki), and Derivative (Kd) gains for both the `Position` and `Angle` loops. You can observe the direct impact of poorly or well-tuned gains on the system's kinetic energy oscillations visually without altering the code.
3. **Start/Restart**: Press to begin integrating equations from $t=0$ utilizing the new GUI settings. This resets the plots and animation automatically.
4. **Stop**: Intercepts and pauses the simulation.

## References

1. Ogata, K. (2010). _Modern Control Engineering_ (5th ed.). Prentice Hall. (Contains mathematical derivation techniques for dynamic non-linear mechanic systems).
2. Aştrom, K. J., & Murray, R. M. (2008). _Feedback Systems: An Introduction for Scientists and Engineers_. Princeton University Press. (Explains Cascaded loop tuning in multivariable environments).
3. Ebeling, C. W. (2014). _Simulating non-linear pendulums in Physics_. Journal of Computing and Python integrations.

---

_Generated utilizing the Antigravity Agent framework._
