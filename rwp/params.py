# params.py - single source of truth for all physical constants.

# --- Raw physical constants ---
L = 0.30          # pendulum length [m]
M_ARM = 0.15      # arm mass [kg]
M_WHEEL = 0.10    # wheel mass [kg]
J_WHEEL = 2e-4    # wheel spin inertia [kg m^2]
TAU_MAX = 0.08    # motor torque limit [N m]
G = 9.81          # gravity [m/s^2]
DT = 0.01         # timestep [s]

# --- Derived constants (nominal model) ---
M_BAR_L = M_ARM * (L / 2) + M_WHEEL * L        # effective first mass moment [kg m]
I_P = (1/3) * M_ARM * L**2 + M_WHEEL * L**2    # pendulum inertia about pivot [kg m^2]

# --- True plant only (unknown to the controller) ---
# These define the engineered model error. Nothing controller-side
# (mpc.py, identify.py, residual.py) may import from this block.
B_P = 1e-3          # viscous pivot friction coefficient [N m s/rad]
B_W = 0.0           # wheel-bearing friction [N m s/rad] - committed to 0
A_DIST = 0.02       # disturbance peak torque [N m] (~25% of TAU_MAX)
THETA_0 = np.pi / 4 # disturbance center angle [rad]
SIGMA_DIST = 0.2    # disturbance angular width [rad]