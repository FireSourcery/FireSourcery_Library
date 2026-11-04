"""
The sensorless math in float, mirroring the C functions the geometry figures draw.

    stator_i_next        Motor/Motor/Math/stator_math.h
    smo_sat, smo_sigma, smo_z, lpf_step, foc_rotor_of_emf
                         Motor/Motor/Math/foc_sensorless_math.h
    angle_pll_error, angle_pll_normalize, AnglePll_Correct, AnglePll_Predict
                         Math/Angle/AnglePll.h
    EMF_OBSERVER_CONFIG_DEFAULT gains
                         Motor/Motor/Model/EmfObserver.h

Units: per unit for v, i, e; rad for angles; one control cycle per step.
"""
import math

# ---------------------------------------------------------------------------
# Stator, per axis: the period through the winding, z or e in place of the EMF
# ---------------------------------------------------------------------------
def stator_i_next(r, g, i, v, e):
    """i + G·(v − R·i − e). G = 1/L at the rad base, one step per control cycle."""
    return i + g * (v - r * i - e)

def J(x):
    """The quarter turn, J·(d, q) = (−q, d)."""
    return (-x[1], x[0])

# ---------------------------------------------------------------------------
# SMO, per axis
# ---------------------------------------------------------------------------
def smo_sat(thr, x):
    return max(-1.0, min(1.0, x / thr))

def smo_sigma(i_est, i_meas):
    return i_est - i_meas

def smo_z(k_smo, thr, sigma):
    return k_smo * smo_sat(thr, sigma)

def lpf_step(k_lp, y, x):
    return y + k_lp * (x - y)

def smo_error_map(sigma, g, r, k_smo, thr, e):
    """
    σ across one step, model and plant through the same winding, the plant's EMF e, the model's z of the last sample:
        σ' = (1 − G·R)·σ − G·z(σ) + G·e
    Inside the boundary layer its slope is the pole 1 − G·(R + K/thr).
    """
    return (1.0 - g * r) * sigma - g * smo_z(k_smo, thr, sigma) + g * e

def smo_pole(g, r, k_smo, thr):
    return 1.0 - g * (r + k_smo / thr)

def smo_fixed_point(r, k_smo, thr, e):
    """σ* inside the layer: R·σ + (K/thr)·σ = e."""
    return e / (r + k_smo / thr)

def lpf_coef(freq, cutoff_hz=100.0):
    """dt / (τ + dt) = 1 / (1 + Freq·τ), as EMF_OBSERVER_CONFIG_DEFAULT"""
    return 1.0 / (1.0 + freq / (2.0 * math.pi * cutoff_hz))

# ---------------------------------------------------------------------------
# Rotor vector and the phase detector
# ---------------------------------------------------------------------------
def foc_rotor_of_emf(sign, e_x, e_y):
    """−j·sign·e: the EMF a quarter turn back, along the rotor flux."""
    return (sign * e_y, -sign * e_x)

def angle_pll_error(x, y, sin, cos):
    """|x|·sin(θ̂ − θ) = x·sin θ̂ − y·cos θ̂"""
    return x * sin - y * cos

def angle_pll_normalize(mag_floor, mag, error):
    return 0.0 if mag <= mag_floor else error / mag

# ---------------------------------------------------------------------------
# AnglePll: PI on the error, predict every control cycle
# ---------------------------------------------------------------------------
KP_Q15 = 328      # EMF_OBSERVER_CONFIG_DEFAULT .Pll.Kp, Q17.15
KI_Q15 = 51472    # .Pll.Ki, Q17.15, per second

def pll_gains(freq):
    """
    The loop in rad per control cycle, for a normalized error e = sin(θ − θ̂) (PID_ProcPI: setpoint 0 − feedback).
        kp: rad per cycle of ω̂ per unit e        Kp · 32768 [angle16] · 2π / 65536
        ki: rad per cycle of ω̂, added per cycle  (Ki / Freq) · 32768 · 2π / 65536
    """
    kp = (KP_Q15 / 32768.0) * math.pi
    ki = (KI_Q15 / 32768.0) / freq * math.pi
    return kp, ki

def pll_continuous(freq):
    """Kp' [1/s], Ki' [1/s²], ω_n [rad/s], ζ of Δ'' + Kp'·cosΔ·Δ' + Ki'·sinΔ = α, small Δ"""
    kp, ki = pll_gains(freq)
    kp_s = kp * freq
    ki_s = ki * freq * freq
    wn = math.sqrt(ki_s)
    return kp_s, ki_s, wn, kp_s / (2.0 * wn)

class AnglePll:
    """θ̂, ω̂ per control cycle. predict() is CAPTURE_ANGLE's, correct() the correction after the write."""
    def __init__(self, freq, theta=0.0, omega=0.0):
        self.kp, self.ki = pll_gains(freq)
        self.theta = theta
        self.omega = omega
        self.integral = omega

    def predict(self):
        self.theta += self.omega
        return self.theta

    def correct(self, error):
        self.integral += self.ki * error
        self.omega = self.kp * error + self.integral
        return self.omega
