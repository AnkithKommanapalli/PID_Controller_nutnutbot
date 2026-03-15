#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>
#include <stdio.h>
#include <iostream>
#include <cmath>
#include <csignal>
#include <cstdlib> // Required for checking environment variables

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
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, 20);
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

    // Visual Box Settings
    cv::Point2f screenCenter(320.0f, 240.0f);
    int boxWidth = 300;  
    int boxHeight = 100; 
    cv::Rect overlayBox(screenCenter.x - boxWidth / 2, screenCenter.y - boxHeight / 2, boxWidth, boxHeight);

    // Tolerances
    double max_offset_meters = 0.01; 
    double max_angle_degrees = 30.0; 

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

            for (size_t i = 0; i < ids.size(); i++) {
                cv::drawFrameAxes(frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 0.5f);

                double offset_x = tvecs[i][0];
                double offset_y = tvecs[i][1];
                double physical_offset = std::sqrt(offset_x * offset_x + offset_y * offset_y);

                cv::Point2f pt0 = corners[i][0]; 
                cv::Point2f pt1 = corners[i][1]; 
                
                double dx = pt1.x - pt0.x;
                double dy = pt1.y - pt0.y;
                double angle_deg = std::atan2(dy, dx) * 180.0 / CV_PI;
                double angle_diff = angle_deg - 0.0; 

                cv::Point2f ref_pt = pt0 + cv::Point2f(60, 0); 
                cv::line(frame, pt0, ref_pt, cv::Scalar(0, 255, 255), 2, cv::LINE_AA); 
                cv::line(frame, pt0, pt1, cv::Scalar(255, 0, 255), 2, cv::LINE_AA); 
                
                std::string onBlockAngleText = std::to_string((int)angle_deg) + " deg";
                cv::putText(frame, onBlockAngleText, pt0 + cv::Point2f(-10, -10), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);

                bool isCentered = (physical_offset <= max_offset_meters);
                bool isStraight = (std::abs(angle_deg) <= max_angle_degrees);

                if (isCentered && isStraight) {
                    isAligned = true;
                }

                std::string offsetText = "Offset: " + std::to_string(physical_offset * 100.0).substr(0, 4) + " cm";
                std::string diffText = "Angle Diff: " + std::to_string(angle_diff).substr(0, 5) + " deg";

                cv::putText(frame, offsetText, cv::Point(20, 90), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
                cv::putText(frame, diffText, cv::Point(400, 40), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
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