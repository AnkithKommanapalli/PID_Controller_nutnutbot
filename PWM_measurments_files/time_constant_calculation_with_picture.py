import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit
import os
import re

# --- CONFIGURATION ---
file_path = "PWM_70_dur7.0s.csv"
window_size = 0        # Change this to adjust the moving average smoothing (1 = no smoothing)

# --- STEADY STATE EXTENSION ---
add_steady_state = True        # Set to True to append artificial steady-state points
added_duration = 10.0           # Number of seconds to add at the end
points_per_second = 50         # Resolution of the added points 

# --- AUTO-DETECT PWM ---
pwm_match = re.search(r'pwm_?(\d+)', file_path.lower())
pwm_value = pwm_match.group(1) if pwm_match else "Unknown"

# Theoretical DC motor response (1st order step response)
def motor_response(t, v_max, tau):
    if tau <= 0:
        return np.inf
    return v_max * (1 - np.exp(-t / tau))

# Moving average filter
def apply_moving_average(data, w_size):
    if w_size <= 1:
        return data
    padded_data = np.pad(data, (w_size - 1, 0), mode='edge')
    return np.convolve(padded_data, np.ones(w_size) / w_size, mode='valid')

times = []
speed_left = []
speed_right = []

try:
    # --- 1. DATA READING ---
    with open(file_path, 'r') as file:
        lines = file.readlines()

    for line in lines[1:]:
        line = line.strip()
        if not line or line.startswith("---"):
            continue
            
        parts = line.split(',')
        if len(parts) >= 3: 
            try:
                t_val = float(parts[0])
                v1_val = float(parts[1]) 
                v2_val = float(parts[2]) 

                times.append(t_val)
                speed_left.append(v1_val)
                speed_right.append(v2_val)
                
            except ValueError:
                continue

    t_data = np.array(times)
    vl_data_raw = np.array(speed_left)
    vr_data_raw = np.array(speed_right)

    # Apply windowing average
    vl_data = apply_moving_average(vl_data_raw, window_size)
    vr_data = apply_moving_average(vr_data_raw, window_size)

    # --- 2. STEADY STATE EXTENSION ---
    if add_steady_state and len(t_data) > 0:
        # Average the last 10 points (or fewer if dataset is small) to get a stable final speed
        n_avg = min(10, len(vl_data))
        final_vl = np.mean(vl_data[-n_avg:])
        final_vr = np.mean(vr_data[-n_avg:])

        num_new_points = int(added_duration * points_per_second)
        if num_new_points > 0:
            # Estimate time step from the last two points
            dt = t_data[-1] - t_data[-2] if len(t_data) > 1 else 0.05 
            t_ext = np.linspace(t_data[-1] + dt, t_data[-1] + added_duration, num_new_points)

            vl_ext = np.full(num_new_points, final_vl)
            vr_ext = np.full(num_new_points, final_vr)

            # Create arrays specifically for fitting
            t_data_fit = np.concatenate((t_data, t_ext))
            vl_data_fit = np.concatenate((vl_data, vl_ext))
            vr_data_fit = np.concatenate((vr_data, vr_ext))
        else:
            t_data_fit, vl_data_fit, vr_data_fit = t_data, vl_data, vr_data
    else:
        # If toggled off, just use original data for fitting
        t_data_fit, vl_data_fit, vr_data_fit = t_data, vl_data, vr_data
        t_ext, vl_ext, vr_ext = [], [], [] # Empty arrays for plotting logic later

    # --- 3. CURVE FIT ---
    guess_vmax_l = np.max(vl_data_fit) if len(vl_data_fit) > 0 else 0.5
    guess_vmax_r = np.max(vr_data_fit) if len(vr_data_fit) > 0 else 0.5
    
    p0_l, p0_r = [guess_vmax_l, 0.05], [guess_vmax_r, 0.05]
    bounds = (0, np.inf)

    # Fit using the extended data 
    popt_l, _ = curve_fit(motor_response, t_data_fit, vl_data_fit, p0=p0_l, bounds=bounds)
    v_max_l, tau_l = popt_l

    popt_r, _ = curve_fit(motor_response, t_data_fit, vr_data_fit, p0=p0_r, bounds=bounds)
    v_max_r, tau_r = popt_r

    # --- 4. PLOT CREATION ---
    plt.figure(figsize=(10, 6))
    
    # Plot original smoothed data
    label_suffix = f"(Smoothed, w={window_size})" if window_size > 1 else "(Raw)"
    plt.plot(t_data, vl_data, color='red', linestyle='-', alpha=0.4, label=f'Left {label_suffix}')
    plt.plot(t_data, vr_data, color='blue', linestyle='-', alpha=0.4, label=f'Right {label_suffix}')

    # Plot the artificial steady-state points (dotted to distinguish them)
    if add_steady_state and len(t_ext) > 0:
        plt.plot(t_ext, vl_ext, color='red', linestyle=':', alpha=0.6, linewidth=2, label='Left (Added SS)')
        plt.plot(t_ext, vr_ext, color='blue', linestyle=':', alpha=0.6, linewidth=2, label='Right (Added SS)')

    if len(t_data_fit) > 0:
        t_fit = np.linspace(0, np.max(t_data_fit), 300)
        vl_fit = motor_response(t_fit, v_max_l, tau_l)
        vr_fit = motor_response(t_fit, v_max_r, tau_r)

        plt.plot(t_fit, vl_fit, color='red', linestyle='--', linewidth=2.5, 
                 label=rf'Left Fit ($\tau={tau_l:.3f}$s, $V_{{max}}={v_max_l:.3f}$ m/s)')
        plt.plot(t_fit, vr_fit, color='blue', linestyle='--', linewidth=2.5, 
                 label=rf'Right Fit ($\tau={tau_r:.3f}$s, $V_{{max}}={v_max_r:.3f}$ m/s)')

    plt.title(f'Motor Response (PWM = {pwm_value})', fontsize=14, fontweight='bold')
    plt.xlabel('Time (s)', fontsize=12) 
    plt.ylabel('Speed (m/s)', fontsize=12)
    plt.legend(loc='lower right', fontsize=10)
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.tight_layout()

    # --- 5. SAVE AS PNG ---
    save_filename = f"PWM_{pwm_value}.png"
    
    counter = 1
    base_name = save_filename.replace(".png", "")
    while os.path.exists(save_filename):
        save_filename = f"{base_name}_{counter}.png"
        counter += 1

    plt.savefig(save_filename, format='png', dpi=300)
    print(f"Graph successfully saved as: {save_filename}")
    plt.show()

except FileNotFoundError:
    print(f"Error: Could not find '{file_path}'.")
except Exception as e:
    print(f"An unexpected error occurred: {e}")