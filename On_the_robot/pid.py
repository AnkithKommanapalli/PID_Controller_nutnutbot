import spidev
import time
import math
import csv
import os

# --- TEST CONFIGURATION ---
TARGET_SPEED_MPS = 0.4  # Target speed in m/s
DURATION = 4.0          # Maximum duration of the test in seconds
LOOP_DELAY = 0.001       # Delay between readings (0.05s = 20 Hz)


Kp_R = 749
Ki_R= 2637
Kp_L= 690
Ki_L = 1921



PWM_MIN = 0
PWM_MAX = 220

# --- KINEMATICS CONSTANTS ---
PPR = 4096
RADIUS = 0.0225       # meters
CIRCUMFERENCE = 2 * math.pi * RADIUS
METERS_PER_TICK = CIRCUMFERENCE / PPR

# --- SPI SETUP: TEENSY (PWM) ---
spi_pwm = spidev.SpiDev()
spi_pwm.open(0, 0)          # Bus 0, Device 0
spi_pwm.max_speed_hz = 1000000
spi_pwm.mode = 0

# --- SPI SETUP: FPGA (ODOMETRY) ---
spi_odo = spidev.SpiDev()
spi_odo.open(1, 0)          # Bus 1, Device 0
spi_odo.max_speed_hz = 1000000
spi_odo.mode = 0

def get_tick_delta(curr, prev):
    # Safely calculate the difference while handling 16-bit integer rollovers
    return ((curr - prev + 32768) % 65536) - 32768

def send_pwm(pwm_left, pwm_right):
    # Send independent PWM to Teensy on Bus 0
    # Assuming Index 4 is Left, Index 6 is Right. Swap if needed!
    tx_frame = [0, 1, 0, 1, int(pwm_right), 0, int(pwm_left), 0]
    spi_pwm.xfer2(tx_frame)

def get_odometry_counts():
    # Read Odometry from FPGA on Bus 1 by sending empty bytes
    resp = spi_odo.xfer2([0x00] * 8)
    
    # Extract Odo 1 (Bytes 2 & 3) and Odo 2 (Bytes 4 & 5)
    odo1_raw = (resp[2] << 8) | resp[3]
    odo2_raw = (resp[4] << 8) | resp[5]
    
    # Convert to signed 16-bit (Two's Complement)
    odo1 = odo1_raw if odo1_raw < 32768 else odo1_raw - 65536
    odo2 = odo2_raw if odo2_raw < 32768 else odo2_raw - 65536
    
    return odo1, odo2

if __name__ == "__main__":
    # --- PREPARE DATA STORAGE ---
    script_dir = os.path.dirname(os.path.abspath(__file__))
    base_filename = f"PID_{TARGET_SPEED_MPS}mps_{DURATION}s"
    
    # Start with run number 1
    run_number = 1
    filename = f"{base_filename}_run{run_number}.csv"
    filepath = os.path.join(script_dir, filename)

    # If the file exists, increment the run number until we find an available filename
    while os.path.exists(filepath):
        run_number += 1
        filename = f"{base_filename}_run{run_number}.csv"
        filepath = os.path.join(script_dir, filename)

    data_log = []

    try:
        print(f"Starting PI Control: Target = {TARGET_SPEED_MPS} m/s, Duration = {DURATION} seconds")
        print(f"Data will be saved to: {filename}")
        
        # 1. Initial dummy reads and safety stop
        send_pwm(0, 0)
        prev_odo1, prev_odo2 = get_odometry_counts()
        time.sleep(0.01)
        
        # 2. Setup variables
        total_dist_right = 0.0
        total_dist_left  = 0.0
        
        # PI State variables
        integral_L = 0.0
        integral_R = 0.0
        
        start_time = time.time()
        prev_time = start_time
        
        # --- CONTROL LOOP ---
        while True:
            curr_time = time.time()
            elapsed_total = curr_time - start_time
            
            # Check if duration is met
            if elapsed_total >= DURATION:
                break
                
            curr_odo1, curr_odo2 = get_odometry_counts()
            
            # Calculate dt
            dt = curr_time - prev_time
            
            if dt > 0:
                # Calculate tick deltas (Right wheel is inverted based on previous logic)
                delta_ticks_right = -get_tick_delta(curr_odo1, prev_odo1) 
                delta_ticks_left  = get_tick_delta(curr_odo2, prev_odo2)  
                
                # Accumulate distance
                total_dist_right += delta_ticks_right * METERS_PER_TICK
                total_dist_left  += delta_ticks_left  * METERS_PER_TICK
                
                # Calculate current speed
                speed_right_mps = (delta_ticks_right * METERS_PER_TICK) / dt
                speed_left_mps  = (delta_ticks_left  * METERS_PER_TICK) / dt
                
                # --- PI CONTROLLER CALCULATION ---
                # 1. Calculate Error
                error_L = TARGET_SPEED_MPS - speed_left_mps
                error_R = TARGET_SPEED_MPS - speed_right_mps
                
                # 2. Accumulate Integral (No Anti-Windup)
                integral_L += error_L * dt
                integral_R += error_R * dt
                
                # 3. Calculate Control Effort
                u_L = (Kp_L * error_L) + (Ki_L * integral_L)
                u_R = (Kp_R * error_R) + (Ki_R * integral_R)
                
                # 4. Clamp Output to valid PWM range (0 to 255/PWM_MAX)
                pwm_L = max(PWM_MIN, min(PWM_MAX, u_L))
                pwm_R = max(PWM_MIN, min(PWM_MAX, u_R))
                
                # Send the calculated PWMs to the motors
                send_pwm(pwm_L, pwm_R)
                
                # Print live stats to console
                print(f"Time: {elapsed_total:4.2f}s | "
                      f"LEFT Spd: {speed_left_mps:5.2f} (PWM: {int(pwm_L):3}) | "
                      f"RIGHT Spd: {speed_right_mps:5.2f} (PWM: {int(pwm_R):3})")
                
                # Save to RAM buffer for CSV later
                data_log.append([elapsed_total, speed_left_mps, speed_right_mps, 
                                 total_dist_left, total_dist_right, pwm_L, pwm_R])
            
            # Update previous values for next loop
            prev_odo1, prev_odo2 = curr_odo1, curr_odo2
            prev_time = curr_time
            
            # Delay to keep sampling rate stable
            time.sleep(LOOP_DELAY)

    except KeyboardInterrupt:
        print("\nTest interrupted early by user.")

    finally:
        # 1. ALWAYS stop the motors first for safety
        send_pwm(0, 0)
        
        # 2. Close both SPI buses
        spi_pwm.close()
        spi_odo.close()
        
        # 3. Write the collected data to the CSV file
        print(f"\nMotors stopped. Test complete. Captured {len(data_log)} data points.")
        print("Writing data to CSV...")
        
        with open(filepath, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Time_s', 'Left_Speed_m_s', 'Right_Speed_m_s', 
                             'Left_Dist_m', 'Right_Dist_m', 'PWM_Left', 'PWM_Right'])
            
            for row in data_log:
                writer.writerow([f"{row[0]:.4f}", f"{row[1]:.4f}", f"{row[2]:.4f}", 
                                 f"{row[3]:.4f}", f"{row[4]:.4f}", f"{row[5]:.1f}", f"{row[6]:.1f}"])
                
        print(f"Data safely saved to:\n{filepath}")