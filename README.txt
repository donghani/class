# Inverted Pendulum GUI Simulation

This application simulates the physics of an inverted pendulum on a cart driven by a cascaded discrete PID controller.

## Prerequisites
- Python 3.x
- NumPy (`pip install numpy`)
- Matplotlib (`pip install matplotlib`)
- Tkinter (usually comes bundled with Python, otherwise install via your package manager)

## Running the Simulation
1. Ensure `inverted_pendulum.py` is in your current directory.
2. In your terminal, run: `python inverted_pendulum.py`
3. A GUI window will appear with control fields and two plots:
   - **Animation Plot**: Shows the cart (blue) and pendulum rod with a bob (red) moving along a 1D track.
   - **Energy Plot**: Shows real-time kinetic, potential, and total energy of the system.

## How to Use
1. **Initial Angle**: Set the starting angle of the pendulum (in radians) before starting. 
   - Note: Setting this too high (e.g. > 0.5 rad) might exceed the limits of the default PID gains and the cart will be unable to catch the fall.
2. **PID Tunings**: 
   - **Position PID**: Controls the outer-loop target angle based on the cart's position. The target is $x=0$.
   - **Angle PID**: Controls the inner-loop force applied to the cart to achieve the target angle. 
   - Feel free to tweak these values (Proportional, Integral, Derivative gains).
3. **Start / Restart**: Clicking this button initializes the physics state with your newly entered parameters and starts the integration (simulation). Over subsequent runs, this will overwrite the existing plot and animation.
4. **Stop**: Pauses/halts the live simulation loop.

### Implementation Details
- The physics model performs Euler Integration with a time-step $dt=0.02$ seconds.
- An automatic screenshot feature saves the main window states at $t=1.0$s and $t=3.0$s as `snapshot_1.png` and `snapshot_2.png`.
- The reference frame sets potential energy relative to the pivot ($0$). Due to this, potential energy is positive when the pendulum balances upright $y > 0$.

### Built by Antigravity
