import tkinter as tk
from inverted_pendulum import InvertedPendulumGUI

root = tk.Tk()
app = InvertedPendulumGUI(root)

# Auto start
app.init_angle_var.set("0.1")
app.start_sim()

def check_snapshots():
    if app.snapshots_taken >= 2:
        print("Snapshots taken, exiting.")
        root.quit()
    else:
        root.after(500, check_snapshots)

root.after(500, check_snapshots)
root.mainloop()
