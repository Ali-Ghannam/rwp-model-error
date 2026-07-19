import numpy as np 
from rwp.params import M_BAR_L, G, I_P, J_WHEEL, B_P, B_W, A_DIST, THETA_0, SIGMA_DIST

def f_nominal(x, tau):
    """Continuous-time nominal dynamics (frictionless).
    
    Input: x = [theta, theta_dot, phi_dot]
    Output: dx/dt = [theta_dot, theta_ddot, phi_ddot]
    """
    theta, theta_dot, phi_dot = x

    theta_ddot = (M_BAR_L*G*np.sin(theta)-tau)/I_P
    phi_ddot = tau / J_WHEEL - theta_ddot

    return np.array ([theta_dot, theta_ddot, phi_ddot])

def f_true (x, tau):
    
    """Continuous-time TRUE PLANT dynamics (simulation only).

    Nominal dynamics plus the two committed engineered model errors:
      - parametric: viscous pivot friction (-B_P*theta_dot)
      - structural: localized disturbance torque tau_disturbance(theta)
    Wheel-bearing friction B_W is retained in the equations but committed
    to 0 for all core experiments (preserves phi_dot-decoupling of theta_ddot).

    FIREWALL: this function and the true-plant parameters must never be
    used by the controller, identifier, or residual model.

    Input:  x = [theta, theta_dot, phi_dot], tau = motor torque [N m]
    Output: dx/dt = [theta_dot, theta_ddot, phi_ddot]
    """

    theta, theta_dot, phi_dot =x
    tau_dist = tau_disturbance(theta)

    theta_ddot = (M_BAR_L*G*np.sin(theta)-tau-B_P*theta_dot 
                  + tau_dist + B_W*phi_dot)/I_P
    phi_ddot = (tau-B_W*phi_dot)/J_WHEEL-theta_ddot

    return np.array ([theta_dot, theta_ddot, phi_ddot])

def tau_disturbance(theta):
    """Localized structural disturbance torque (true plant only)."""
    return A_DIST * np.exp(-(theta - THETA_0)**2 / (2 * SIGMA_DIST**2))