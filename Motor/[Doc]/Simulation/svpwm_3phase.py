import numpy as np
import matplotlib.pyplot as plt

def svpwm_midclamp(alpha, beta):
    """
    Simulate the SVPWM midclamp algorithm and return duty cycles for phases A, B, and C.
    """
    # Calculate the three phase voltages
    vA = alpha / np.sqrt(3)
    vB = (-0.5 * alpha + (np.sqrt(3) / 2) * beta) / np.sqrt(3)
    vC = (-0.5 * alpha - (np.sqrt(3) / 2) * beta) / np.sqrt(3)

    # Find the maximum and minimum of the three phase voltages
    vMax = max(vA, vB, vC)
    vMin = min(vA, vB, vC)

    # Calculate the zero-sequence voltage (midclamp adjustment)
    vZero = (vMax + vMin) / 2.0 - .5

    # Adjust the phase voltages to ensure midclamp
    dutyA = vA - vZero  
    dutyB = vB - vZero  
    dutyC = vC - vZero  

    return vZero, vA, vB, vC, dutyA, dutyB, dutyC

# Generate test data
time = np.linspace(0, 1, 1000)  # Simulate one electrical cycle
alpha = np.sin(2 * np.pi * time)  # Alpha component (sinusoidal)
beta = np.cos(2 * np.pi * time)   # Beta component (cosine)

# Calculate duty cycles for each time step
dutyA = []
dutyB = []
dutyC = []
vZero = []
vA = []
vB = []
vC = []

for a, b in zip(alpha, beta):
    _vZero, _vA, _vB, _vC, dA, dB, dC = svpwm_midclamp(a, b)
    dutyA.append(dA)
    dutyB.append(dB)
    dutyC.append(dC) 
    vZero.append(_vZero)
    vA.append(_vA)
    vB.append(_vB)
    vC.append(_vC) 

# Convert to numpy arrays for plotting
dutyA = np.array(dutyA)
dutyB = np.array(dutyB)
dutyC = np.array(dutyC)
vZero = np.array(vZero)
vA = np.array(vA)
vB = np.array(vB)
vC = np.array(vC)

# Plot the results
plt.figure(figsize=(10, 6))
plt.plot(time, dutyA, label="Duty A", color="red")
plt.plot(time, dutyB, label="Duty B", color="green")
plt.plot(time, dutyC, label="Duty C", color="blue") 
plt.plot(time, vA, label="Scalar A", color="red", alpha=.1)
plt.plot(time, vB, label="Scalar B", color="green", alpha=.1)
plt.plot(time, vC, label="Scalar C", color="blue", alpha=.1) 
plt.title("SVPWM Midclamp Duty Cycles")
plt.xlabel("Time (normalized to one electrical cycle)")
plt.ylabel("Duty Cycle")
plt.legend()
plt.grid()
plt.show()
