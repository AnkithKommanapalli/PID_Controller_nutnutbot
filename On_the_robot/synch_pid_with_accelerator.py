import spidev
import time
import math
import csv
import os

# --- TEST CONFIGURATION ---
TARGET_SPEED_MPS = 0.5  # Maximum target speed in m/s
DURATION = 8.0          # Maximum duration of the test in seconds
LOOP_DELAY = 0.001      # Delay between readings
ACCEL_MPS2 = 0.4        # Acceleration rate in m/s^2 (0.4 means it takes 1 sec to reach 0.4 m/s)

# --- PI CONTROLLER CONSTANTS ---
Kp_R = 749
Ki_R= 2637
Kp_L= 690
Ki_L = 1921

# --- SYNCHRONIZATION CONSTANT ---
K_SYNC = 2.0            

PWM_MIN = 0
PWM_MAX = 220

# --- KINEMATICS CONSTANTS ---
PPR = 4096
RADIUS = 0.0225       # meters
CIRCUMFERENCE = 2 * math.pi * RADIUS
METERS_PER_TICK = CIRCUMFERENCE / PPR

# --- SPI SETUP: TEENSY (PWM) ---
spi_pwm = spidev.SpiDev()
spi_pwm.open(0, 0)          
spi_pwm.max_speed_hz = 1000000
spi_pwm.mode = 0

# --- SPI SETUP: FPGA (ODOMETRY) ---
spi_odo = spidev.SpiDev()
spi_odo.open(1, 0)          
spi_odo.max_speed_hz = 1000000
spi_odo.mode = 0

def get_tick_delta(curr, prev):
    return ((curr - prev + 32768) % 65536) - 32768

def send_pwm(pwm_left, pwm_right):
    tx_frame = [0, 1, 0, 1, int(pwm_right), 0, int(pwm_left), 0]
    spi_pwm.xfer2(tx_frame)

def get_odometry_counts():
    resp = spi_odo.xfer2([0x00] * 8)
    odo1_raw = (resp[2] << 8) | resp[3]
    odo2_raw = (resp[4] << 8) | resp[5]
    odo1 = odo1_raw if odo1_raw < 32768 else odo1_raw - 65536
    odo2 = odo2_raw if odo2_raw < 32768 else odo2_raw - 65536
    return odo1, odo2

if __name__ == "__main__":
    # --- PREPARE DATA STORAGE ---
    script_dir = os.path.dirname(os.path.abspath(__file__))
    base_filename = f"SYNCH_RAMP_PID_{TARGET_SPEED_MPS}mps_{DURATION}s_Ramp"
    
    run_number = 1
    filename = f"{base_filename}_run{run_number}.csv"
    filepath = os.path.join(script_dir, filename)

    while os.path.exists(filepath):
        run_number += 1
        filename = f"{base_filename}_run{run_number}.csv"
        filepath = os.path.join(script_dir, filename)

    data_log = []

    try:
        print(f"Starting Control: Target = {TARGET_SPEED_MPS}m/s, Accel = {ACCEL_MPS2}m/s^2, Sync = {K_SYNC}")
        print(f"Data will be saved to: {filename}")
        
        send_pwm(0, 0)
        prev_odo1, prev_odo2 = get_odometry_counts()
        time.sleep(0.01)
        
        total_dist_right = 0.0
        total_dist_left  = 0.0
        integral_L = 0.0
        integral_R = 0.0
        
        # This will hold our ramping target speed
        current_base_speed = 0.0 
        
        start_time = time.time()
        prev_time = start_time
        
        # --- CONTROL LOOP ---
        while True:
            curr_time = time.time()
            elapsed_total = curr_time - start_time
            
            if elapsed_total >= DURATION:
                break
                
            curr_odo1, curr_odo2 = get_odometry_counts()
            dt = curr_time - prev_time
            
            if dt > 0:
                delta_ticks_right = -get_tick_delta(curr_odo1, prev_odo1) 
                delta_ticks_left  = get_tick_delta(curr_odo2, prev_odo2)  
                
                total_dist_right += delta_ticks_right * METERS_PER_TICK
                total_dist_left  += delta_ticks_left  * METERS_PER_TICK
                
                speed_right_mps = (delta_ticks_right * METERS_PER_TICK) / dt
                speed_left_mps  = (delta_ticks_left  * METERS_PER_TICK) / dt
                
                # --- SPEED RAMP ---
                # Gradually increase the base speed until it hits the target
                if current_base_speed < TARGET_SPEED_MPS:
                    current_base_speed += ACCEL_MPS2 * dt
                    # Clamp to max target speed so we don't overshoot
                    if current_base_speed > TARGET_SPEED_MPS:
                        current_base_speed = TARGET_SPEED_MPS
                
                # --- SYNCHRONIZATION ---
                distance_error = total_dist_left - total_dist_right
                sync_adjustment = K_SYNC * distance_error
                
                current_target_L = current_base_speed - sync_adjustment
                current_target_R = current_base_speed + sync_adjustment
                
                # --- PI CONTROLLER ---
                error_L = current_target_L - speed_left_mps
                error_R = current_target_R - speed_right_mps
                
                integral_L += error_L * dt
                integral_R += error_R * dt
                
                u_L = (Kp_L * error_L) + (Ki_L * integral_L)
                u_R = (Kp_R * error_R) + (Ki_R * integral_R)
                
                pwm_L = max(PWM_MIN, min(PWM_MAX, u_L))
                pwm_R = max(PWM_MIN, min(PWM_MAX, u_R))
                
                send_pwm(pwm_L, pwm_R)
                
                # Terminal output is simplified to focus on the ramp
                print(f"Time: {elapsed_total:4.2f}s | Target: {current_base_speed:4.2f} | L: {speed_left_mps:5.2f} | R: {speed_right_mps:5.2f}")
                
                # Log expanded data
                data_log.append([
                    elapsed_total, current_base_speed, sync_adjustment,
                    speed_left_mps, speed_right_mps, 
                    total_dist_left, total_dist_right, 
                    pwm_L, pwm_R
                ])
            
            prev_odo1, prev_odo2 = curr_odo1, curr_odo2
            prev_time = curr_time
            time.sleep(LOOP_DELAY)

    except KeyboardInterrupt:
        print("\nTest interrupted early by user.")

    finally:
        send_pwm(0, 0)
        spi_pwm.close()
        spi_odo.close()
        
        print(f"\nMotors stopped. Captured {len(data_log)} data points.")
        print("Writing data to CSV...")
        
        with open(filepath, mode='w', newline='') as file:
            writer = csv.writer(file)
            # Updated Headers
            writer.writerow(['Time_s', 'Base_Target_m_s', 'Sync_Adj', 
                             'Left_Speed_m_s', 'Right_Speed_m_s', 
                             'Left_Dist_m', 'Right_Dist_m', 
                             'PWM_Left', 'PWM_Right'])
            
            for row in data_log:
                writer.writerow([f"{row[0]:.4f}", f"{row[1]:.4f}", f"{row[2]:.4f}", 
                                 f"{row[3]:.4f}", f"{row[4]:.4f}", 
                                 f"{row[5]:.4f}", f"{row[6]:.4f}", 
                                 f"{row[7]:.1f}", f"{row[8]:.1f}"])
                
        print(f"Data safely saved to:\n{filepath}")