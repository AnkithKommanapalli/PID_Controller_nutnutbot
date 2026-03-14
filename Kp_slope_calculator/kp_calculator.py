import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import linregress

# --- MANUAL DATA ENTRY ---
# Combined PWM values (X-axis)
#pwm_values = [150, 150, 150, 170, 170, 200, 200, 200, 220, 220, 255, 255, 255]

pwm_values = [70, 100, 100, 150, 150, 150, 170, 170, 200, 200, 200, 220, 220, 255, 255, 255]

# Combined Left motor steady-state speeds in m/s (Y-axis 1)
left_speed_values = [0.211, 0.324, 0.320, 0.487, 0.473, 0.477, 0.542, 0.529, 0.600, 0.613, 0.593, 0.647, 0.635, 0.721, 0.712, 0.705]
#left_speed_values = [0.487, 0.473, 0.477, 0.542, 0.529, 0.600, 0.613, 0.593, 0.647, 0.635, 0.721, 0.712, 0.705]

# Combined Right motor steady-state speeds in m/s (Y-axis 2)
right_speed_values = [0.271, 0.467, 0.478, 0.597, 0.611, 0.604, 0.626, 0.614, 0.655, 0.650, 0.658, 0.667, 0.674, 0.716, 0.724, 0.723]
#right_speed_values = [0.597, 0.611, 0.604, 0.626, 0.614, 0.655, 0.650, 0.658, 0.667, 0.674, 0.716, 0.724, 0.723]

# Ensure data is in NumPy arrays
x = np.array(pwm_values)
y_left = np.array(left_speed_values)
y_right = np.array(right_speed_values)

# --- LINEAR REGRESSION ---
# Left Motor Fit
slope_left, intercept_left, r_value_left, p_value_left, std_err_left = linregress(x, y_left)
line_left = slope_left * x + intercept_left

# Right Motor Fit
slope_right, intercept_right, r_value_right, p_value_right, std_err_right = linregress(x, y_right)
line_right = slope_right * x + intercept_right

# --- PLOT CREATION ---
plt.figure(figsize=(10, 6))

# Plot Left Data & Fit
plt.scatter(x, y_left, color='red', alpha=0.6, label='Left Motor Data')
plt.plot(x, line_left, color='red', linestyle='--', label=f'Left Fit ($K_v$: {slope_left:.4f})')

# Plot Right Data & Fit
plt.scatter(x, y_right, color='blue', alpha=0.6, label='Right Motor Data')
plt.plot(x, line_right, color='blue', linestyle='--', label=f'Right Fit ($K_v$: {slope_right:.4f})')

# Formatting
plt.title('Speed vs PWM Characteristic (Left vs Right Motor)', fontsize=14, fontweight='bold')
plt.xlabel('PWM Value', fontsize=12)
plt.ylabel('Steady-state Speed (m/s)', fontsize=12)
plt.grid(True, linestyle=':', alpha=0.7)

# Display the Gains (Kv) on the plot
plt.text(np.min(x), np.max(y_right)*0.9, 
         rf'Left $Slope_L \approx {slope_left:.4f} \frac{{m/s}}{{PWM}}$' + '\n' +
         rf'Right $Slope_R \approx {slope_right:.4f} \frac{{m/s}}{{PWM}}$', 
         fontsize=12, bbox=dict(facecolor='white', alpha=0.8))

plt.legend(loc='lower right')
plt.tight_layout()

# Save and Show
plt.savefig("speed_vs_pwm_both_motors.png")
plt.show()

# --- TERMINAL OUTPUT ---
print("--- LEFT MOTOR ---")
print(f"Calculated Slope (Gain K_v): {slope_left:.6f} m/s per PWM unit")
print(f"Intercept (Y-intercept): {intercept_left:.4f}")
print(f"Estimated PWM Dead-zone (X-intercept): {-intercept_left/slope_left:.1f}")
print(f"R-squared (Fit quality): {r_value_left**2:.4f}\n")

print("--- RIGHT MOTOR ---")
print(f"Calculated Slope (Gain K_v): {slope_right:.6f} m/s per PWM unit")
print(f"Intercept (Y-intercept): {intercept_right:.4f}")
print(f"Estimated PWM Dead-zone (X-intercept): {-intercept_right/slope_right:.1f}")
print(f"R-squared (Fit quality): {r_value_right**2:.4f}")