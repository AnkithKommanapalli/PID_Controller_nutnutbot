import spidev
import time
import math

# --- Kinematics Constants ---
PPR = 4096
RADIUS = 0.0225  # meters
CIRCUMFERENCE = 2 * math.pi * RADIUS
METERS_PER_TICK = CIRCUMFERENCE / PPR

# --- SPI Setup ---
spi = spidev.SpiDev()
spi.open(1, 0)
spi.max_speed_hz = 1000000  # 1 MHz
spi.mode = 0                # Mode 0

def get_odometry_counts():
    # Send 8 empty bytes to clock out the 64-bit payload
    resp = spi.xfer2([0x00] * 8)
    
    # Extract Odo 1 (Bytes 2 & 3) and Odo 2 (Bytes 4 & 5)
    odo1_raw = (resp[2] << 8) | resp[3]
    odo2_raw = (resp[4] << 8) | resp[5]
    
    # Convert to signed 16-bit (Two's Complement)
    odo1 = odo1_raw if odo1_raw < 32768 else odo1_raw - 65536
    odo2 = odo2_raw if odo2_raw < 32768 else odo2_raw - 65536
    
    return odo1, odo2

def get_tick_delta(curr, prev):
    # Safely calculate the difference while handling 16-bit integer rollovers
    return ((curr - prev + 32768) % 65536) - 32768

if __name__ == "__main__":
    try:
        print("Calculating speed and distance... Press Ctrl+C to stop.")
        
        # Reset distance to 0 at the start of the run
        total_dist1 = 0.0
        total_dist2 = 0.0
        
        # Initial readings
        prev_odo1, prev_odo2 = get_odometry_counts()
        prev_time = time.time()
        
        while True:
            time.sleep(0.1) # 10Hz sampling rate
            
            curr_odo1, curr_odo2 = get_odometry_counts()
            curr_time = time.time()
            
            # Calculate tick deltas
            delta_ticks1 = get_tick_delta(curr_odo1, prev_odo1)
            delta_ticks2 = get_tick_delta(curr_odo2, prev_odo2)
            
            # Accumulate total distance traveled (in meters)
            total_dist1 += delta_ticks1 * METERS_PER_TICK
            total_dist2 += delta_ticks2 * METERS_PER_TICK
            
            # Calculate elapsed time and speed (m/s)
            dt = curr_time - prev_time
            speed1_mps = (delta_ticks1 * METERS_PER_TICK) / dt
            speed2_mps = (delta_ticks2 * METERS_PER_TICK) / dt
            
            print(f"W1:  {speed1_mps:6.3f} m/s  |  Dist: {total_dist1:6.3f} m   ---   W2:  {speed2_mps:6.3f} m/s  |  Dist: {total_dist2:6.3f} m")
            
            # Update for next loop
            prev_odo1, prev_odo2 = curr_odo1, curr_odo2
            prev_time = curr_time
            
    except KeyboardInterrupt:
        print("\nStopping SPI...")
    
    finally:
        spi.close()