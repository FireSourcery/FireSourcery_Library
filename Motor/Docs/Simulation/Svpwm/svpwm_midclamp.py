import os
import numpy as np
import matplotlib.pyplot as plt

def phase_voltages(radius, theta):
    """
    A balanced three phase set of peak radius per VBus, as svpwm_midclamp takes it. radius = 1/sqrt3 is the linear limit.
    """
    vA = radius * np.cos(theta)
    vB = radius * np.cos(theta - 2 * np.pi / 3)
    vC = radius * np.cos(theta + 2 * np.pi / 3)
    return vA, vB, vC

def svpwm_midclamp(vA, vB, vC):
    """
    Simulate the SVPWM midclamp algorithm and return duty cycles for phases A, B, and C.
    """
    # Find the maximum and minimum of the three phase voltages
    vMax = max(vA, vB, vC)
    vMin = min(vA, vB, vC)

    # Calculate the zero-sequence voltage (midclamp adjustment)
    vZero = (vMax + vMin) / 2.0 - .5

    # Adjust the phase voltages to ensure midclamp
    dutyA = vA - vZero
    dutyB = vB - vZero
    dutyC = vC - vZero

    return vZero, dutyA, dutyB, dutyC

# Generate test data
time = np.linspace(0, 1, 1000)  # Simulate one electrical cycle
vA, vB, vC = phase_voltages(1 / np.sqrt(3), 2 * np.pi * time)  # at the linear limit

# Calculate duty cycles for each time step
dutyA = []
dutyB = []
dutyC = []
vZero = []

for a, b, c in zip(vA, vB, vC):
    _vZero, dA, dB, dC = svpwm_midclamp(a, b, c)
    dutyA.append(dA)
    dutyB.append(dB)
    dutyC.append(dC)
    vZero.append(_vZero)

# Convert to numpy arrays for plotting
dutyA = np.array(dutyA)
dutyB = np.array(dutyB)
dutyC = np.array(dutyC)
vZero = np.array(vZero)

# Plot the results
plt.figure(figsize=(10, 6))
plt.plot(time, dutyA, label="Duty A", color="red")
plt.plot(time, dutyB, label="Duty B", color="green")
plt.plot(time, dutyC, label="Duty C", color="blue")
plt.plot(time, vA, label="Ratio A", color="red", alpha=.1)
plt.plot(time, vB, label="Ratio B", color="green", alpha=.1)
plt.plot(time, vC, label="Ratio C", color="blue", alpha=.1)
plt.plot(time, vZero, label="vZero", color="black")
plt.title("SVPWM Midclamp Duty Cycles")
plt.xlabel("Time (normalized to one electrical cycle)")
plt.ylabel("Duty Cycle")
plt.legend(loc="center left", bbox_to_anchor=(1.0, 0.5))
plt.grid()
plt.tight_layout()
plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), "SVPWM_Midclamp.png"), dpi=100)
plt.show()
