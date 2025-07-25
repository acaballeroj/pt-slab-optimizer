import numpy as np
import cvxpy as cp
import math
import matplotlib.pyplot as plt

# ------------------------------------------------------
# GEOMETRY AND LOAD PARAMETERS
# ------------------------------------------------------

# Slab dimensions (in meters)
Lx, Ly = 8.0, 12.0               
t_slab = 0.40                     # Slab thickness (m)

# Load parameters
# If using structural software: replace q_dead and q_live with imported load values
gamma_concrete = 25              # Concrete unit weight (kN/m³)
q_dead = gamma_concrete * t_slab # Dead load (kN/m²)
q_live = 5.0                      # Live load (kN/m²)
q_total = q_dead + q_live        # Total design load (kN/m²)

# Simple moment estimation (midspan)
L = Ly
w = q_total
M = w * L**2 / 8

# Moment to stress conversion
b = 1.0
I = (b * t_slab**3) / 12
y = t_slab / 2

M_Nmm = M * 1e6
I_mm4 = I * 1e12
y_mm = y * 1e3
sigma_max = (M_Nmm * y_mm) / I_mm4
print(f"Estimated σ_max at midspan = {sigma_max:.2f} MPa")


# ------------------------------------------------------
# TENDON PARAMETERS
# ------------------------------------------------------

# Tendon layout definition (uniformly spaced)
tendon_spacing = 1.0
tendon_x_positions = np.arange(0.5, Lx - 0.5 + 1e-6, tendon_spacing)

# Tendon properties (can be retrieved from structural software or design config)
cord_area = 150              # mm²
n_cord_max = 10
sigma_u = 1860               # MPa
prestress_ratio = 0.7
Ptendon = cord_area * sigma_u * prestress_ratio / 1e3  # kN

# Influence area assumption (1m strip)
area_influence = t_slab * 1.0                           # m²
stress_per_tendon = (Ptendon / area_influence) / 1000  # MPa


# ------------------------------------------------------
# ECCENTRICITY PROFILE (PARABOLIC)
# ------------------------------------------------------

# Tendon vertical position assumed parabolic; replace with actual profile from software if available
cover = 0.05
ecc_factor = 1.0

e_max = t_slab / 2 - cover
e_design = ecc_factor * e_max

print(f"Maximum usable eccentricity: {e_max:.3f} m")
print(f"Design eccentricity used: {e_design:.3f} m")

def tendon_profile(y, Ly, e0):
    return 4 * e0 * (y / Ly) * (1 - y / Ly)


# ------------------------------------------------------
# CONTROL POINT GENERATION
# ------------------------------------------------------

# Number of control points (can be imported from FEA mesh or slab layout)
Npc = 10000
aspect_ratio = Lx / Ly

Ny = int(math.sqrt(Npc / aspect_ratio))
if Ny % 2 == 0:
    Ny += 1
Nx = int(Npc / Ny)
if Nx % 2 == 0:
    Nx += 1

print(f"Using {Nx * Ny} control points in a {Nx} x {Ny} grid")

x_control = np.linspace(0.5, Lx - 0.5, Nx)
y_control = np.linspace(0.5, Ly - 0.5, Ny)

control_points = [(x, y) for x in x_control for y in y_control]
X = np.array([pt[0] for pt in control_points])
Y = np.array([pt[1] for pt in control_points])


# ------------------------------------------------------
# INFLUENCE MATRIX CALCULATION
# ------------------------------------------------------

# If using structural software: this matrix A should be obtained directly by running influence load cases

def tendon_influence_with_eccentricity(xc, x_cp, y_cp, sigma=0.75):
    dx = abs(x_cp - xc)
    infl_base = np.exp(- (dx ** 2) / (2 * sigma ** 2))
    ecc = tendon_profile(y_cp, Ly, e_design)
    return infl_base * (1 + ecc / (t_slab / 2))

A = []
for x_cp, y_cp in control_points:
    row = [tendon_influence_with_eccentricity(x_t, x_cp, y_cp) for x_t in tendon_x_positions]
    A.append(row)
A = np.array(A)


# ------------------------------------------------------
# TARGET STRESSES (FROM DL + LL)
# ------------------------------------------------------

# Replace this parabola with stress results imported from structural software
yc, b = Ly / 2, Ly / 2
sigma_targets = [sigma_max * (1 - ((y_cp - yc) ** 2 / b ** 2)) for _, y_cp in control_points]
load_vector = np.array(sigma_targets)


# ------------------------------------------------------
# PRECHECK: ALL TENDONS ACTIVE
# ------------------------------------------------------

x_all_active = np.full(len(tendon_x_positions), n_cord_max)
induced_stress_full = A @ (x_all_active * stress_per_tendon)
combined_stress_full = load_vector - induced_stress_full

violations = induced_stress_full < load_vector
n_violations = np.sum(violations)

print("\n=== PRECHECK: All Tendons Active ===")
if n_violations == 0:
    print("✅ All stress constraints are satisfied with all tendons active.")
else:
    print(f"❌ {n_violations} control points do NOT meet the stress requirements.")
    print("⚠️  The maximum allowed post-tensioning is not sufficient to meet stress criteria across the entire slab.")
    print("   → Consider increasing the number of strands per tendon, reducing span, or increasing slab thickness.")
    raise RuntimeError("🚫 Optimization aborted: constraints cannot be met even with all tendons fully activated.")


# ------------------------------------------------------
# OPTIMIZATION: MINIMIZE NUMBER OF STRANDS
# ------------------------------------------------------

# Variables: number of strands per tendon (continuous)
n = cp.Variable(len(tendon_x_positions))
objective = cp.Minimize(cp.sum(n))
constraints = [
    A @ (n * stress_per_tendon) >= load_vector,
    n >= 0,
    n <= n_cord_max
]

problem = cp.Problem(objective, constraints)
print("Solving...")
result = problem.solve(solver=cp.ECOS)
print("Done.")

if problem.status != cp.OPTIMAL:
    raise ValueError("❌ Optimization failed. Status: " + str(problem.status))

# Round strands to nearest upper integer (to ensure constraints are satisfied)
x_opt = np.ceil(n.value).astype(int)
induced_stress = A @ (x_opt * stress_per_tendon)
final_stress = load_vector - induced_stress

# Final verification
violations = induced_stress < load_vector
n_violations = np.sum(violations)
if n_violations > 0:
    print(f"⚠️ Warning: {n_violations} stress constraints violated after rounding.")
else:
    print("✅ All stress constraints satisfied after rounding.")


# ------------------------------------------------------
# POST-TENSIONING MASS CALCULATION
# ------------------------------------------------------

# Replace with quantity takeoff from structural software if available
tendon_length = Ly
steel_density = 7850
strand_area_m2 = cord_area * 1e-6
mass_per_strand_kg = strand_area_m2 * tendon_length * steel_density

total_strands_full = len(tendon_x_positions) * n_cord_max
mass_full = total_strands_full * mass_per_strand_kg

total_strands_opt = np.sum(x_opt)
mass_opt = total_strands_opt * mass_per_strand_kg

print("\n=== POST-TENSIONING MASS COMPARISON ===")
print(f"Full PT: {total_strands_full:.0f} strands → {mass_full:.1f} kg")
print(f"Optimized PT: {total_strands_opt:.0f} strands → {mass_opt:.1f} kg")
print(f"Steel reduction: {(1 - mass_opt / mass_full) * 100:.1f}%")


# ------------------------------------------------------
# PLOT: OPTIMIZED TENDON LAYOUT
# ------------------------------------------------------

plt.figure(figsize=(10, 4))
plt.bar(tendon_x_positions, x_opt, width=0.8, align='center', color='steelblue', edgecolor='black')
plt.title("Optimized Tendon Layout")
plt.xlabel("Tendon Position (m)")
plt.ylabel("Number of Strands")
plt.xticks(tendon_x_positions)
plt.grid(True, axis='y', linestyle='--', alpha=0.7)
plt.tight_layout()
plt.show()


# ------------------------------------------------------
# PLOTS: STRESS DISTRIBUTIONS
# ------------------------------------------------------

vmin = min(load_vector.min(), induced_stress.min(), final_stress.min())
vmax = max(load_vector.max(), induced_stress.max(), final_stress.max())

fig, axs = plt.subplots(1, 3, figsize=(18, 5))

sc1 = axs[0].scatter(X, Y, c=load_vector, cmap='jet', vmin=vmin, vmax=vmax)
axs[0].set_title("Initial Stress (σ_DL+LL)")
plt.colorbar(sc1, ax=axs[0], label="MPa")

sc2 = axs[1].scatter(X, Y, c=induced_stress, cmap='jet', vmin=vmin, vmax=vmax)
axs[1].set_title("Induced Stress (σ_PT)")
plt.colorbar(sc2, ax=axs[1], label="MPa")

sc3 = axs[2].scatter(X, Y, c=final_stress, cmap='jet', vmin=vmin, vmax=vmax)
axs[2].set_title("Final Stress (σ_DL+LL - σ_PT)")
plt.colorbar(sc3, ax=axs[2], label="MPa")

for ax in axs:
    ax.set_aspect('equal')
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")

plt.tight_layout(rect=[0, 0.05, 1, 1])
fig.text(0.5, 0.01, "Note: Negative values indicate compression, positive values indicate tension.", 
         ha='center', fontsize=10, style='italic')
plt.show()
