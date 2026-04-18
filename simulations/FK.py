import math

# -----------------------------
# Link lengths (meters)
# -----------------------------
L1 = 0.08   # thigh
L2 = 0.12   # lower leg

# -----------------------------
# Example joint angles (radians)
# Replace these with your actual values if needed
# -----------------------------
theta1 = -0.60   # thigh angle
theta2 = 0.48    # knee angle

# -----------------------------
# Forward kinematics
# -----------------------------
x = L1 * math.cos(theta1) + L2 * math.cos(theta1 + theta2)
z = L1 * math.sin(theta1) + L2 * math.sin(theta1 + theta2)

print("Forward Kinematics Result")
print(f"Thigh angle (theta1): {theta1:.4f} rad")
print(f"Knee angle  (theta2): {theta2:.4f} rad")
print(f"Foot position:")
print(f"x = {x:.4f} m")
print(f"z = {z:.4f} m")