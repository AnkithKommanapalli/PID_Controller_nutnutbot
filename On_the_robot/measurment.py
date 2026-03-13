import spidev
import time
import math
import csv
import os

# --- TEST CONFIGURATION ---
TARGET_PWM = 220      # PWM applied to both wheels (0-255)
DURATION = 5.0        # Duration of the test in seconds
LOOP_DELAY = 0.01     # Delay between readings (0.05s = 20 Hz)

# --- KINEMATICS CONSTANTS (From Code 1) ---
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

def send_pwm(pwm_val):
    # Send PWM to Teensy on Bus 0
    tx_frame = [0, 1, 0, 1, pwm_val, 0, pwm_val, 0]
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
    base_filename = f"PWM_{TARGET_PWM}_dur{DURATION}s"
    filename = f"{base_filename}.csv"
    filepath = os.path.join(script_dir, filename)

    # FIXED: Check if file exists, append a counter, and actually increment it!
    counter = 1
    while os.path.exists(filepath):
        filename = f"{base_filename}_{counter}.csv"
        filepath = os.path.join(script_dir, filename)
        counter += 1

    data_log = []

    try:
        print(f"Starting test: PWM = {TARGET_PWM}, Duration = {DURATION} seconds")
        
        # 1. Initial dummy reads to clear buffers and get baseline odometry
        send_pwm(0)
        prev_odo1, prev_odo2 = get_odometry_counts()
        time.sleep(0.01)
        
        # 2. Setup variables
        total_dist_right = 0.0
        total_dist_left  = 0.0
        
        start_time = time.time()
        prev_time = start_time
        
        # --- HIGH SPEED LOOP ---
        while True:
            curr_time = time.time()
            elapsed_total = curr_time - start_time
            
            # Check if duration is met
            if elapsed_total >= DURATION:
                break
                
            # Send PWM to Teensy, then instantly read odometry from FPGA
            send_pwm(TARGET_PWM)
            curr_odo1, curr_odo2 = get_odometry_counts()
            
            # Calculate dt
            dt = curr_time - prev_time
            
            if dt > 0:
                # Calculate tick deltas (Right wheel is inverted)
                delta_ticks_right = -get_tick_delta(curr_odo1, prev_odo1) 
                delta_ticks_left  = get_tick_delta(curr_odo2, prev_odo2)  
                
                # Accumulate distance
                total_dist_right += delta_ticks_right * METERS_PER_TICK
                total_dist_left  += delta_ticks_left  * METERS_PER_TICK
                
                # Calculate speed
                speed_right_mps = (delta_ticks_right * METERS_PER_TICK) / dt
                speed_left_mps  = (delta_ticks_left  * METERS_PER_TICK) / dt
                
                # Print live stats to console
                print(f"Time: {elapsed_total:5.2f}s | LEFT -> Spd: {speed_left_mps:6.3f} m/s, Dist: {total_dist_left:6.3f} m | RIGHT -> Spd: {speed_right_mps:6.3f} m/s, Dist: {total_dist_right:6.3f} m")
                
                # Save to RAM buffer for CSV later
                data_log.append([elapsed_total, speed_left_mps, speed_right_mps, total_dist_left, total_dist_right])
            
            # Update previous values for next loop
            prev_odo1, prev_odo2 = curr_odo1, curr_odo2
            prev_time = curr_time
            
            # Delay to keep sampling rate stable
            time.sleep(LOOP_DELAY)

    except KeyboardInterrupt:
        print("\nTest interrupted early by user.")

    finally:
        # 1. ALWAYS stop the motors first for safety
        send_pwm(0)
        
        # 2. Close both SPI buses
        spi_pwm.close()
        spi_odo.close()
        
        # 3. Write the collected data to the CSV file
        print(f"\nMotors stopped. Test complete. Captured {len(data_log)} data points.")
        print("Writing data to CSV...")
        
        with open(filepath, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Time_s', 'Left_Speed_m_s', 'Right_Speed_m_s', 'Left_Dist_m', 'Right_Dist_m'])
            
            for row in data_log:
                writer.writerow([f"{row[0]:.4f}", f"{row[1]:.4f}", f"{row[2]:.4f}", f"{row[3]:.4f}", f"{row[4]:.4f}"])
                
        print(f"Data safely saved to:\n{filepath}")