#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>
#include <stdio.h>
#include <iostream>
#include <cmath>
#include <csignal>
#include <cstdlib> // Required for checking environment variables
#include <sstream>
#include <iomanip>
#include <algorithm>

// --- SIGNAL HANDLING FOR CTRL+C EXIT ---
volatile sig_atomic_t stop_program = 0;

void handle_sigint(int sig) {
    std::cout << "\nCaught signal " << sig << " (Ctrl+C). Shutting down safely..." << std::endl;
    stop_program = 1;
}
// -----------------------------------------

int main()
{
    // Register the Ctrl+C signal handler
    std::signal(SIGINT, handle_sigint);

    // --- AUTO-DETECT DISPLAY ---
    // Check if X11 (DISPLAY) or Wayland (WAYLAND_DISPLAY) environment variables exist
    bool hasDisplay = (std::getenv("DISPLAY") != nullptr || std::getenv("WAYLAND_DISPLAY") != nullptr);

    if (hasDisplay) {
        std::cout << "--- GUI MODE ---" << std::endl;
        std::cout << "Display detected. Opening video window. Press ESC in the window or Ctrl+C in terminal to quit." << std::endl;
    } else {
        std::cout << "--- HEADLESS MODE ---" << std::endl;
        std::cout << "No display detected. Running in background. Press Ctrl+C in terminal to quit." << std::endl;
    }
    std::cout << "---------------------" << std::endl;

    cv::VideoCapture cap(0, cv::CAP_V4L2);

    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera." << std::endl;
        return -1;
    }

    // --- STABILITY SETTINGS ---
    // Set camera resolution to 1280x720 (720p)
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    cap.set(cv::CAP_PROP_FPS, 60);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);

    // --- 3D POSE CALIBRATION ---
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 
        600, 0, 320, 
        0, 600, 240, 
        0, 0, 1);
    cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64F); 
    
    float markerLength = 0.05f; 

    cv::Ptr<cv::aruco::Dictionary> dict =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

    // Visual Box Settings (centered for 1280x720)
    cv::Point2f screenCenter(640.0f, 360.0f);
    // Scale box size from 640x480 (400x300) to 1280x720
    int boxWidth = 800;   // 400 * (1280/640)
    int boxHeight = 250;  // 300 * (720/480)
    cv::Rect overlayBox(screenCenter.x - boxWidth / 2, screenCenter.y - boxHeight / 2, boxWidth, boxHeight);

    // --- UPDATED TOLERANCES ---
    // Offset is now used mainly for display; alignment is based on
    // marker being inside the overlay box + orientation constraints.
    double max_angle_horizontal_deg = 10.0; // red line ~horizontal within ±25° (0 or 180)
    double max_angle_vertical_deg   = 10.0; // green line ~vertical within ±25° (±90)

    while (!stop_program) {
        cv::Mat frame;
        bool success = cap.read(frame);

        if (!success || frame.empty()) {
            std::cout << "\rFrame dropped/empty...           " << std::flush;
            continue; 
        }

        cv::flip(frame, frame, -1); 

        // Draw standard overlays
        cv::rectangle(frame, overlayBox, cv::Scalar(255, 0, 0), 2); 
        cv::drawMarker(frame, screenCenter, cv::Scalar(255, 255, 0), cv::MARKER_CROSS, 20, 1);

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(frame, dict, corners, ids, parameters);

        if (!ids.empty()) {
            cv::aruco::drawDetectedMarkers(frame, corners, ids);

            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);

            bool isAligned = false;
            double distance_m = -1.0; // closest marker distance in meters

            for (size_t i = 0; i < ids.size(); i++) {
                int markerId = ids[i];
                cv::drawFrameAxes(frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 0.5f);

                double offset_x = tvecs[i][0];
                double offset_y = tvecs[i][1];
                double physical_offset = std::sqrt(offset_x * offset_x + offset_y * offset_y);

                // Use Z-component of translation as forward distance (meters)
                double current_distance_m = tvecs[i][2];
                if (distance_m < 0.0 || current_distance_m < distance_m) {
                    distance_m = current_distance_m;
                }

                cv::Point2f pt0 = corners[i][0]; 
                cv::Point2f pt1 = corners[i][1]; 
                cv::Point2f pt2 = corners[i][2]; 
                cv::Point2f pt3 = corners[i][3]; 
                
                // --- ORIENTATION MEASURES ---
                // Top edge (pt0 -> pt1) should be "red" and ~horizontal
                double dx_h = pt1.x - pt0.x;
                double dy_h = pt1.y - pt0.y;
                double angle_h_deg = std::atan2(dy_h, dx_h) * 180.0 / CV_PI; // 0 or 180 is horizontal

                // Reduce angle to [0,180] then measure distance to 0/180
                double angle_h_abs = std::fabs(angle_h_deg);
                double angle_h_mod = std::fmod(angle_h_abs, 180.0);
                double diff_h = std::min(angle_h_mod, 180.0 - angle_h_mod);
                bool isHorizontal = (diff_h <= max_angle_horizontal_deg);

                // Left edge (pt0 -> pt3) should be "green" and ~vertical (±90°)
                double dx_v = pt3.x - pt0.x;
                double dy_v = pt3.y - pt0.y;
                double angle_v_deg = std::atan2(dy_v, dx_v) * 180.0 / CV_PI;
                double diff_v = std::fabs(std::fabs(angle_v_deg) - 90.0);
                bool isVertical = (diff_v <= max_angle_vertical_deg);

                cv::Point2f ref_pt = pt0 + cv::Point2f(60, 0); 
                cv::line(frame, pt0, ref_pt, cv::Scalar(0, 255, 255), 2, cv::LINE_AA); 
                cv::line(frame, pt0, pt1, cv::Scalar(255, 0, 255), 2, cv::LINE_AA); 
                
                // Show horizontal angle on the marker for quick visual debug
                std::string onBlockAngleText = std::to_string((int)angle_h_deg) + " deg";
                cv::putText(frame, onBlockAngleText, pt0 + cv::Point2f(-10, -10), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);

                // Marker center in image coordinates
                cv::Point2f markerCenter = (pt0 + pt1 + pt2 + pt3) * 0.25f;
                bool inBox = overlayBox.contains(markerCenter);

                // Alignment condition: valid ID (36 or 47) + in box around origin + red ~horizontal + green ~vertical
                bool validId = (markerId == 36 || markerId == 47);
                if (validId && inBox && isHorizontal && isVertical) {
                    isAligned = true;
                }

                std::string offsetText = "Offset: " + std::to_string(physical_offset * 100.0).substr(0, 4) + " cm";
                std::string diffText = "H: " + std::to_string(diff_h).substr(0, 4) + " V: " + std::to_string(diff_v).substr(0, 4);

                cv::putText(frame, offsetText, cv::Point(20, 90), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
                cv::putText(frame, diffText, cv::Point(400, 40), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
            }

            // Draw distance text in bottom-right of the overlay box
            if (distance_m > 0.0) {
                std::ostringstream oss;
                oss << std::fixed << std::setprecision(2) << distance_m;
                std::string distText = "Dist: " + oss.str() + " m";

                int baseline = 0;
                cv::Size textSize = cv::getTextSize(distText, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, &baseline);
                cv::Point textOrg(
                    overlayBox.x + overlayBox.width - textSize.width - 10,
                    overlayBox.y + overlayBox.height - 10
                );

                cv::putText(frame, distText, textOrg, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
            }

            // --- ID COLOR / VALIDITY LABEL (bottom-left) ---
            if (!ids.empty()) {
                int mainId = ids[0];
                std::string idText;
                cv::Scalar idColor;

                if (mainId == 36) {
                    idText = "ID 36: BLUE";
                    idColor = cv::Scalar(255, 0, 0); // Blue in BGR
                } else if (mainId == 47) {
                    idText = "ID 47: YELLOW";
                    idColor = cv::Scalar(0, 255, 255); // Yellow in BGR
                } else {
                    idText = "ID INVALID";
                    idColor = cv::Scalar(0, 0, 255); // Red
                }

                cv::Point idOrg(20, frame.rows - 20);
                cv::putText(frame, idText, idOrg, cv::FONT_HERSHEY_SIMPLEX, 0.8, idColor, 2);
            }

            if (isAligned) {
                cv::putText(frame, "ALIGNED", cv::Point(20, 50), cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(0, 255, 0), 3);
                std::cout << "\rID: " << ids[0] << " -> ALIGNED      " << std::flush;
                
                // Save an image for verification (useful in headless mode)
                cv::imwrite("aligned_scan.jpg", frame); 

            } else {
                cv::putText(frame, "MISALIGNED", cv::Point(20, 50), cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(0, 0, 255), 3);
                std::cout << "\rID: " << ids[0] << " -> MISALIGNED   " << std::flush;
            }
        } else {
            std::cout << "\rWaiting for ArUco marker...      " << std::flush;
        }

        // --- GUI HANDLING ---
        if (hasDisplay) {
            cv::imshow("ArUco Scanner", frame);
            char key = (char)cv::waitKey(1);
            if (key == 27) break; // Press ESC to exit
        }
    }

    cap.release();
    if (hasDisplay) {
        cv::destroyAllWindows();
    }
    std::cout << "\nCamera released. Goodbye!" << std::endl;
    return 0;
}

//g++ -Wall -Wextra -pedantic aruco_two.cpp -o aruco_two `pkg-config --cflags --libs opencv4`
// libcamerify ./aruco_two on the real_VNC
//pushed_correctly