import os
import re
import matplotlib.pyplot as plt
import numpy as np

# --- CONFIGURATION ---
file_path = "Ramp/PID_0.5mps_dur6.0s_ramp.csv"  
SPIKE_THRESHOLD = 0.1      # Max allowed speed jump (m/s) between consecutive ticks
SPEED_WINDOW_SIZE = 60      # Moving average window for speed (0 or 1 = disabled)

# Extract reference speed from the filename using regex (used for title)
filename = os.path.basename(file_path)
speed_match = re.search(r'pid_speed_([\d\.]+)mps', filename)
ref_speed_val = float(speed_match.group(1)) if speed_match else 0.5

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
pwm_left = []
pwm_right = []
target_speed = [] # Added to hold dynamic target speeds

try:
    with open(file_path, 'r') as file:
        # Read and skip the header line
        header = file.readline()
        
        # Extract the data
        for line in file:
            parts = line.strip().split(',')
            if len(parts) >= 8: # Expecting 8 columns based on the new format
                try:
                    times.append(float(parts[0]))
                    speed_left.append(float(parts[1]))
                    speed_right.append(float(parts[2]))
                    # Indices 3 and 4 are Dist_m, skipping them for this plot
                    pwm_left.append(float(parts[5]))
                    pwm_right.append(float(parts[6]))
                    target_speed.append(float(parts[7]))
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
    
    # 2. Plot the dynamic target speed (uses full times array)
    line3, = ax1.plot(times, target_speed, label='Target Speed', color='green', linestyle='--', linewidth=2)

    # Formatting for primary Y-axis
    ax1.set_xlabel('Time (s)', fontsize=12)
    ax1.set_ylabel('Speed (m/s)', fontsize=12, color='black')
    ax1.tick_params(axis='y', labelcolor='black')
    
    max_speed = max(max(speed_left_final) if len(speed_left_final) > 0 else 0, 
                    max(speed_right_final) if len(speed_right_final) > 0 else 0, 
                    max(target_speed) if len(target_speed) > 0 else ref_speed_val)
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
    lines_all = [line1, line2, line3, line4, line5]
    labels_all = [l.get_label() for l in lines_all]
    ax1.legend(lines_all, labels_all, loc='upper center', bbox_to_anchor=(0.5, -0.12), ncol=3)

    # Title
    plt.title(f'PID Validation (Max Reference Speed: {ref_speed_val} m/s)', fontsize=14, fontweight='bold')
    
    plt.subplots_adjust(bottom=0.25) 
    
    # Save the graph to the same directory as the CSV file
    save_path = os.path.splitext(file_path)[0] + ".png"
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"Plot saved successfully to: {save_path}")
    
    # Show the graph
    plt.show()

except FileNotFoundError:
    print(f"Error: Could not find the file '{file_path}'. Please check the name and path.")
except Exception as e:
    print(f"An error occurred: {e}")