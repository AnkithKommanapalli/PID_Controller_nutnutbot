import os
import re
import matplotlib.pyplot as plt
import numpy as np

# --- CONFIGURATION ---
file_path = "SYNCH_RAMP_PID_0.5mps_8.0s_Ramp_run1.csv"  
SPIKE_THRESHOLD = 0.2      # Max allowed speed jump (m/s) between consecutive ticks
SPEED_WINDOW_SIZE = 70      # Moving average window for speed (0 or 1 = disabled)

# Extract reference speed from the filename using regex (fallback if CSV doesn't provide it)
filename = os.path.basename(file_path)
speed_match = re.search(r'pid_speed_([\d\.]+)mps', filename)
if not speed_match:
    # Fallback: grab the first "Xmps" pattern if present
    speed_match = re.search(r'([\d\.]+)mps', filename)
ref_speed_val = float(speed_match.group(1)) if speed_match else 0.0

# Helper function to remove physically impossible spikes
def remove_spikes(data, threshold):
    if not data: return data
    clean_data = [data[0]]
    for i in range(1, len(data)):
        # If the speed jumps by more than the threshold, repeat the last good value
        if abs(data[i] - clean_data[-1]) > threshold:
            clean_data.append(clean_data[-1])
        else:
            clean_data.append(data[i])
    return clean_data

# Helper function for moving average
def apply_moving_average(data, window):
    # If window is 0 or 1, or data is too short, return the original data
    if window <= 1 or len(data) < window:
        return data
    return list(np.convolve(data, np.ones(window)/window, mode='valid'))

times = []
speed_left = []
speed_right = []
base_target = []  # From new-format CSVs (Base_Target_m_s)
pwm_left = []
pwm_right = []

try:
    with open(file_path, 'r') as file:
        header_line = file.readline().strip()
        header_parts = header_line.split(',') if header_line else []

        # Detect new-format CSV via header names
        is_new_format = "Time_s" in header_parts or "Base_Target_m_s" in header_parts

        if is_new_format:
            # Build a name->index map for robustness
            col_index = {name: idx for idx, name in enumerate(header_parts)}

            time_idx = col_index.get("Time_s")
            base_target_idx = col_index.get("Base_Target_m_s")
            left_speed_idx = col_index.get("Left_Speed_m_s")
            right_speed_idx = col_index.get("Right_Speed_m_s")
            pwm_left_idx = col_index.get("PWM_Left")
            pwm_right_idx = col_index.get("PWM_Right")

            for line in file:
                parts = line.strip().split(',')
                if not parts or len(parts) < 2:
                    continue
                try:
                    if time_idx is not None and time_idx < len(parts):
                        times.append(float(parts[time_idx]))
                    if left_speed_idx is not None and left_speed_idx < len(parts):
                        speed_left.append(float(parts[left_speed_idx]))
                    if right_speed_idx is not None and right_speed_idx < len(parts):
                        speed_right.append(float(parts[right_speed_idx]))
                    if base_target_idx is not None and base_target_idx < len(parts):
                        base_target.append(float(parts[base_target_idx]))
                    if pwm_left_idx is not None and pwm_left_idx < len(parts):
                        pwm_left.append(float(parts[pwm_left_idx]))
                    if pwm_right_idx is not None and pwm_right_idx < len(parts):
                        pwm_right.append(float(parts[pwm_right_idx]))
                except ValueError:
                    # Skip lines with malformed numeric data
                    continue
        else:
            # Old format: assume fixed column positions
            # time, left_speed, right_speed, ..., pwm_left, pwm_right
            for line in file:
                parts = line.strip().split(',')
                if len(parts) >= 7:
                    try:
                        times.append(float(parts[0]))
                        speed_left.append(float(parts[1]))
                        speed_right.append(float(parts[2]))
                        pwm_left.append(float(parts[5]))
                        pwm_right.append(float(parts[6]))
                    except ValueError:
                        continue

    # 1. Remove spikes from the speed data
    speed_left_clean = remove_spikes(speed_left, SPIKE_THRESHOLD)
    speed_right_clean = remove_spikes(speed_right, SPIKE_THRESHOLD)

    # 2. Apply moving average ONLY to cleaned speeds
    speed_left_final = apply_moving_average(speed_left_clean, SPEED_WINDOW_SIZE)
    speed_right_final = apply_moving_average(speed_right_clean, SPEED_WINDOW_SIZE)

    # Adjust the times array for the speed plots if averaging is active
    if SPEED_WINDOW_SIZE > 1 and len(times) >= SPEED_WINDOW_SIZE:
        times_speed = times[SPEED_WINDOW_SIZE - 1:]
    else:
        times_speed = times

    # --- PLOTTING ---
    fig, ax1 = plt.subplots(figsize=(10, 7)) # Slightly taller to make room for the legend below
    
    # 1. Plot the finalized wheel speeds (uses times_speed array)
    line1, = ax1.plot(times_speed, speed_left_final, label='Left Wheel Speed', color='blue', linewidth=2)
    line2, = ax1.plot(times_speed, speed_right_final, label='Right Wheel Speed', color='red', linewidth=2)
    
    # 2. Plot the reference/target speed
    reference_lines = []
    # Prefer the Base_Target_m_s column if available
    if base_target and len(base_target) == len(times):
        # If it's effectively constant, draw a single horizontal line
        if max(base_target) - min(base_target) < 1e-3:
            ref_val = base_target[0]
            line_ref = ax1.axhline(y=ref_val, color='green', linestyle='--', linewidth=2,
                                   label=f'Reference Speed ({ref_val} m/s)')
            reference_lines.append(line_ref)
        else:
            # Varying target: plot vs time
            line_ref, = ax1.plot(times, base_target, color='green', linestyle='--', linewidth=2,
                                 label='Reference Speed')
            reference_lines.append(line_ref)
    elif ref_speed_val > 0:
        line_ref = ax1.axhline(y=ref_speed_val, color='green', linestyle='--', linewidth=2,
                               label=f'Reference Speed ({ref_speed_val} m/s)')
        reference_lines.append(line_ref)

    # Formatting for primary Y-axis
    ax1.set_xlabel('Time (s)', fontsize=12)
    ax1.set_ylabel('Speed (m/s)', fontsize=12, color='black')
    ax1.tick_params(axis='y', labelcolor='black')
    
    max_speed = 0.0
    if speed_left_final:
        max_speed = max(max_speed, max(speed_left_final))
    if speed_right_final:
        max_speed = max(max_speed, max(speed_right_final))
    if base_target:
        max_speed = max(max_speed, max(base_target))
    elif ref_speed_val > 0:
        max_speed = max(max_speed, ref_speed_val)
    ax1.set_ylim(0, max_speed * 1.2 if max_speed > 0 else 1)
    ax1.grid(True, linestyle='--', alpha=0.7)

    # 3. Plot the RAW PWM values on secondary Y-axis (Matched colors, lower opacity)
    ax2 = ax1.twinx()  
    line4, = ax2.plot(times, pwm_left, label='Left PWM', color='blue', linestyle=':', linewidth=1.5, alpha=0.4)
    line5, = ax2.plot(times, pwm_right, label='Right PWM', color='red', linestyle=':', linewidth=1.5, alpha=0.4)
    
    # Force PWM to the bottom of the graph
    actual_max_pwm = max(max(pwm_left) if len(pwm_left) > 0 else 255, 
                         max(pwm_right) if len(pwm_right) > 0 else 255)
    ax2.set_ylim(0, actual_max_pwm * 3.5) 

    ax2.set_ylabel('PWM Value', fontsize=12, color='black')
    ax2.tick_params(axis='y', labelcolor='black')
    
    # Legend - Kept completely outside the plot area
    lines_all = [line1, line2] + reference_lines + [line4, line5]
    labels_all = [l.get_label() for l in lines_all]
    ax1.legend(lines_all, labels_all, loc='upper center', bbox_to_anchor=(0.5, -0.12), ncol=3)

    # Title
    plt.title(f'PID Validation (Reference Speed: {ref_speed_val} m/s)', fontsize=14, fontweight='bold')
    
    # Show the graph
    plt.subplots_adjust(bottom=0.25) 
    plt.show()

except FileNotFoundError:
    print(f"Error: Could not find the file '{file_path}'. Please check the name and path.")
except Exception as e:
    print(f"An error occurred: {e}")