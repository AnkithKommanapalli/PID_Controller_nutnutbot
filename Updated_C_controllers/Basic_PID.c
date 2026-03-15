#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <time.h>
#include <math.h>
#include <signal.h>

// --- TEST CONFIGURATION ---
#define TARGET_SPEED_MPS 0.4  // Target speed in m/s
#define DURATION 4.0          // Maximum duration of the test in seconds
#define LOOP_DELAY_US 1000    // Delay between readings (1000 us = 0.001s = 1000 Hz)

#define KP_R 749.0
#define KI_R 2637.0
#define KP_L 690.0
#define KI_L 1921.0

#define PWM_MIN 0.0
#define PWM_MAX 255.0         // Updated to 255

// --- KINEMATICS CONSTANTS ---
#define PPR 4096.0
#define RADIUS 0.0225         // meters
#define PI 3.14159265358979323846
#define CIRCUMFERENCE (2.0 * PI * RADIUS)
#define METERS_PER_TICK (CIRCUMFERENCE / PPR)

// --- GLOBALS FOR SIGNAL HANDLING ---
int spi_pwm_fd = -1;
int spi_odo_fd = -1;
volatile sig_atomic_t keep_running = 1;

// --- DATA LOGGING STRUCTURE ---
typedef struct {
    double time;
    double left_speed;
    double right_speed;
    double left_dist;
    double right_dist;
    double pwm_left;
    double pwm_right;
} LogEntry;

#define MAX_LOG_ENTRIES 10000
LogEntry data_log[MAX_LOG_ENTRIES];
int log_count = 0;

// --- HELPER FUNCTIONS ---

// Signal handler for Ctrl+C
void int_handler(int dummy) {
    keep_running = 0;
}

// Get high-resolution monotonic time in seconds
double get_time() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + (ts.tv_nsec / 1e9);
}

// Safely calculate difference while handling 16-bit wrapping
int16_t get_tick_delta(uint16_t curr, uint16_t prev) {
    // In C, unsigned 16-bit subtraction naturally handles wrapping. 
    // Casting to signed 16-bit gives the correct negative/positive delta.
    return (int16_t)(curr - prev);
}

// Send PWM over SPI (Bus 0)
void send_pwm(uint8_t pwm_left, uint8_t pwm_right) {
    if (spi_pwm_fd < 0) return;
    uint8_t tx[] = {0, 1, 0, 1, pwm_right, 0, pwm_left, 0};
    uint8_t rx[8] = {0};
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = 8,
        .speed_hz = 1000000,
        .bits_per_word = 8,
    };
    ioctl(spi_pwm_fd, SPI_IOC_MESSAGE(1), &tr);
}

// Read Odometry over SPI (Bus 1)
void get_odometry_counts(uint16_t *odo1, uint16_t *odo2) {
    if (spi_odo_fd < 0) return;
    uint8_t tx[8] = {0};
    uint8_t rx[8] = {0};
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = 8,
        .speed_hz = 1000000,
        .bits_per_word = 8,
    };
    ioctl(spi_odo_fd, SPI_IOC_MESSAGE(1), &tr);
    
    *odo1 = (rx[2] << 8) | rx[3];
    *odo2 = (rx[4] << 8) | rx[5];
}

// Simple clamp function for doubles
double clamp(double value, double min, double max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// --- MAIN CONTROL LOOP ---
int main() {
    // 1. Setup Signal Handler
    signal(SIGINT, int_handler);

    // 2. Open SPI Devices
    spi_pwm_fd = open("/dev/spidev0.0", O_RDWR);
    spi_odo_fd = open("/dev/spidev1.0", O_RDWR);
    
    if (spi_pwm_fd < 0 || spi_odo_fd < 0) {
        printf("Error: Could not open SPI devices. Are you running as root/with privileges?\n");
        return -1;
    }

    // Set SPI Modes (Mode 0)
    uint8_t mode = SPI_MODE_0;
    ioctl(spi_pwm_fd, SPI_IOC_WR_MODE, &mode);
    ioctl(spi_odo_fd, SPI_IOC_WR_MODE, &mode);

    // 3. Prepare CSV File
    char filename[256];
    int run_number = 1;
    while (1) {
        snprintf(filename, sizeof(filename), "PID_%.1fmps_%.1fs_run%d.csv", TARGET_SPEED_MPS, DURATION, run_number);
        if (access(filename, F_OK) != 0) { // File doesn't exist
            break;
        }
        run_number++;
    }

    printf("Starting PI Control: Target = %.2f m/s, Duration = %.2f seconds\n", TARGET_SPEED_MPS, DURATION);
    printf("Data will be saved to: %s\n", filename);

    // 4. Initial Safety Setup
    send_pwm(0, 0);
    uint16_t prev_odo1 = 0, prev_odo2 = 0;
    get_odometry_counts(&prev_odo1, &prev_odo2);
    usleep(10000); // 10ms

    double total_dist_right = 0.0;
    double total_dist_left = 0.0;
    
    double integral_L = 0.0;
    double integral_R = 0.0;

    // Define Integral clamps based on Max PWM contribution
    // This prevents windup from exceeding what the system can output
    double max_integral_L = PWM_MAX / KI_L;
    double max_integral_R = PWM_MAX / KI_R;

    double start_time = get_time();
    double prev_time = start_time;

    // 5. Control Loop
    while (keep_running) {
        double curr_time = get_time();
        double elapsed_total = curr_time - start_time;

        if (elapsed_total >= DURATION || log_count >= MAX_LOG_ENTRIES) {
            break;
        }

        uint16_t curr_odo1, curr_odo2;
        get_odometry_counts(&curr_odo1, &curr_odo2);

        double dt = curr_time - prev_time;

        if (dt > 0) {
            // Calculate tick deltas (Right wheel inverted per original script)
            int16_t delta_ticks_right = -get_tick_delta(curr_odo1, prev_odo1);
            int16_t delta_ticks_left  = get_tick_delta(curr_odo2, prev_odo2);

            total_dist_right += delta_ticks_right * METERS_PER_TICK;
            total_dist_left  += delta_ticks_left * METERS_PER_TICK;

            double speed_right_mps = (delta_ticks_right * METERS_PER_TICK) / dt;
            double speed_left_mps  = (delta_ticks_left * METERS_PER_TICK) / dt;

            // --- PI CONTROLLER ---
            double error_L = TARGET_SPEED_MPS - speed_left_mps;
            double error_R = TARGET_SPEED_MPS - speed_right_mps;

            // Accumulate Integral
            integral_L += error_L * dt;
            integral_R += error_R * dt;

            // ANTI-WINDUP: Clamp the integral term
            integral_L = clamp(integral_L, -max_integral_L, max_integral_L);
            integral_R = clamp(integral_R, -max_integral_R, max_integral_R);

            // Calculate Control Effort
            double u_L = (KP_L * error_L) + (KI_L * integral_L);
            double u_R = (KP_R * error_R) + (KI_R * integral_R);

            // Clamp Output to valid PWM range
            double pwm_L = clamp(u_L, PWM_MIN, PWM_MAX);
            double pwm_R = clamp(u_R, PWM_MIN, PWM_MAX);

            send_pwm((uint8_t)pwm_L, (uint8_t)pwm_R);

            printf("Time: %4.2fs | LEFT Spd: %5.2f (PWM: %3d) | RIGHT Spd: %5.2f (PWM: %3d)\n", 
                   elapsed_total, speed_left_mps, (int)pwm_L, speed_right_mps, (int)pwm_R);

            // Log Data
            data_log[log_count++] = (LogEntry){
                elapsed_total, speed_left_mps, speed_right_mps, 
                total_dist_left, total_dist_right, pwm_L, pwm_R
            };
        }

        prev_odo1 = curr_odo1;
        prev_odo2 = curr_odo2;
        prev_time = curr_time;

        usleep(LOOP_DELAY_US);
    }

    // 6. Cleanup & Save
    send_pwm(0, 0); // Safety stop
    close(spi_pwm_fd);
    close(spi_odo_fd);

    if (!keep_running) {
        printf("\nTest interrupted early by user.\n");
    }

    printf("\nMotors stopped. Test complete. Captured %d data points.\n", log_count);
    printf("Writing data to CSV...\n");

    FILE *fp = fopen(filename, "w");
    if (fp != NULL) {
        fprintf(fp, "Time_s,Left_Speed_m_s,Right_Speed_m_s,Left_Dist_m,Right_Dist_m,PWM_Left,PWM_Right\n");
        for (int i = 0; i < log_count; i++) {
            fprintf(fp, "%.4f,%.4f,%.4f,%.4f,%.4f,%.1f,%.1f\n",
                    data_log[i].time, data_log[i].left_speed, data_log[i].right_speed,
                    data_log[i].left_dist, data_log[i].right_dist,
                    data_log[i].pwm_left, data_log[i].pwm_right);
        }
        fclose(fp);
        printf("Data safely saved to: %s\n", filename);
    } else {
        printf("Error: Could not open %s for writing.\n", filename);
    }

    return 0;
}

//gcc .\Basic_PID.c -o .\Basic_PID -lm