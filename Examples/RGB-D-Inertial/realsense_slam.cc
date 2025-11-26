/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 *
 * ADAPTED FOR OFFLINE REAL SENSE IMU-RGBD PROCESSING (IMU_RGBD Mode).
 */

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <signal.h>
#include <sstream>
#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgcodecs.hpp>

#include <System.h>

#include <json.h>
#include <CLI11.hpp>

using namespace std;
using nlohmann::json;
const double MS_TO_S = 1e-3; ///< Milliseconds to second conversion

void signal_callback_handler(int signum)
{
    cout << "realsense_offline_rgbd_slam.cc Caught signal " << signum << endl;
    exit(signum);
}

// --- 1. IMU Telemetry Loading ---
bool LoadTelemetry(const string &path_to_telemetry_file,
                   vector<double> &vAccTimestamps,
                   vector<cv::Point3f> &vAcc,
                   vector<cv::Point3f> &vGyr)
{

    std::ifstream file;
    file.open(path_to_telemetry_file.c_str());
    if (!file.is_open())
    {
        cerr << "Failed to open IMU JSON file at: " << path_to_telemetry_file << endl;
        return false;
    }
    json j;
    file >> j;

    const auto accl = j["ACCL"];
    const auto gyro = j["GYRO"];

    std::map<double, cv::Point3f> sorted_acc;
    std::map<double, cv::Point3f> sorted_gyr;

    // Load and sort ACCL data using continuous time stamp (cts)
    for (const auto &e : accl)
    {
        cv::Point3f v((float)e["value"][0], (float)e["value"][1], (float)e["value"][2]);
        // sorted_acc.insert(std::make_pair((double)e["cts"] * MS_TO_S, v));
        sorted_acc.insert(std::make_pair((double)e["timestamp"], v));
    }
    // Load and sort GYRO data using continuous time stamp (cts)
    for (const auto &e : gyro)
    {
        cv::Point3f v((float)e["value"][0], (float)e["value"][1], (float)e["value"][2]);
        // sorted_gyr.insert(std::make_pair((double)e["cts"] * MS_TO_S, v));
        sorted_gyr.insert(std::make_pair((double)e["timestamp"], v));
    }

    // Determine IMU start time for absolute timestamps (no subtraction)
    if (sorted_acc.empty() || sorted_gyr.empty())
    {
        cerr << "IMU JSON is empty." << endl;
        return false;
    }

    // Use absolute time for correct synchronization with TUM timestamps
    for (auto acc : sorted_acc)
    {
        vAccTimestamps.push_back(acc.first);
        vAcc.push_back(acc.second);
    }
    for (auto gyr : sorted_gyr)
    {
        vGyr.push_back(gyr.second);
    }

    file.close();
    cout << "Extracting IMU Telemetry success! ACC: " << vAcc.size() << ", GYR: " << vGyr.size() << endl;
    return true;
}

// --- 2. TUM Timestamp and File Path Loading (2-column TUM format) ---
bool LoadImages(const string &strPathTimeFile,
                const string &strPathFramesDir,
                vector<string> &vstrImageFilenames,
                vector<string> &vstrDepthFilenames,
                vector<double> &vTimeStamps)
{

    std::ifstream f;
    f.open(strPathTimeFile.c_str());
    if (!f.is_open())
    {
        cerr << "Failed to open TUM timestamp file at: " << strPathTimeFile << endl;
        return false;
    }

    while (!f.eof())
    {
        string s;
        getline(f, s);
        if (!s.empty() && s[0] != '#')
        { // Skip comments and empty lines
            stringstream ss;
            ss << s;
            double t;
            string relative_path; // This will read the entire path (e.g., "color/123.png")

            // Read the two fields in the TUM format: timestamp and the rest of the relative path
            if (!(ss >> t))
                continue;
            if (!(ss >> relative_path))
                continue;

            // Construct full path: strPathFramesDir/relative_path
            string full_path = strPathFramesDir + "/" + relative_path;

            // Populate the correct filename vector based on the file being read.
            if (strPathTimeFile.find("rgb.txt") != string::npos)
            {
                vstrImageFilenames.push_back(full_path);
            }
            else if (strPathTimeFile.find("depth.txt") != string::npos)
            {
                vstrDepthFilenames.push_back(full_path);
            }

            vTimeStamps.push_back(t);
        }
    }
    f.close();
    return true;
}

// --- 3. RAW Depth File Reading ---
cv::Mat ReadRawDepth(const std::string &strPathFile, int width, int height)
{
    // Depth is expected to be 16-bit single channel (CV_16UC1)
    cv::Mat depth_map(height, width, CV_16UC1);
    std::ifstream file(strPathFile, std::ios::binary);

    if (!file.is_open())
    {
        cerr << "ERROR: Failed to open RAW depth file: " << strPathFile << endl;
        return cv::Mat();
    }

    // Read the binary data directly into the OpenCV Mat data pointer
    file.read((char *)depth_map.data, depth_map.total() * depth_map.elemSize());

    if (file.gcount() != (long)(depth_map.total() * depth_map.elemSize()))
    {
        cerr << "WARNING: Incomplete read of RAW depth file: " << strPathFile << endl;
    }

    file.close();
    return depth_map;
}

int main(int argc, char **argv)
{
    signal(SIGINT, signal_callback_handler);

    // --- CLI Parsing ---
    CLI::App app{"RealSense Off-Line IMU-RGBD SLAM"};

    std::string vocabulary = "../../Vocabulary/ORBvoc.txt";
    app.add_option("-v,--vocabulary", vocabulary)->capture_default_str();

    std::string setting = "RealSense_D435i.yaml";
    app.add_option("-s,--setting", setting)->capture_default_str();

    std::string input_data_dir; // Directory containing frames/ and imu_data.json
    app.add_option("-i,--input_data_dir", input_data_dir)->required();

    std::string output_trajectory_csv;
    app.add_option("-o,--output_trajectory_csv", output_trajectory_csv);

    std::string load_map;
    app.add_option("-l,--load_map", load_map);

    std::string save_map;
    app.add_option("--save_map", save_map);

    bool enable_gui = false;
    app.add_flag("-g,--enable_gui", enable_gui);

    int num_threads = 4;
    app.add_flag("-n,--num_threads", num_threads);

    std::string mask_img_path;
    app.add_option("--mask_img", mask_img_path);

    // Aruco tag for initialization
    int aruco_dict_id = cv::aruco::DICT_4X4_50;
    app.add_option("--aruco_dict_id", aruco_dict_id);

    int init_tag_id = 13;
    app.add_option("--init_tag_id", init_tag_id);

    float init_tag_size = 0.16; // in meters
    app.add_option("--init_tag_size", init_tag_size);

    // if lost more than max_lost_frames, terminate
    // disable the check if <= 0
    int max_lost_frames = -1;
    app.add_option("--max_lost_frames", max_lost_frames);

    int depth_width = 848;
    app.add_option("--depth_width", depth_width);

    int depth_height = 480;
    app.add_option("--depth_height", depth_height);

    try
    {
        app.parse(argc, argv);
    }
    catch (const CLI::ParseError &e)
    {
        return app.exit(e);
    }
    cv::setNumThreads(num_threads);

    // --- Path Setup and Data Loading ---
    string strFramesPath = input_data_dir + "/frames";
    string strImuJsonPath = input_data_dir + "/imu_data.json";

    // 1. Load IMU Telemetry
    vector<double> imuTimestamps;
    vector<cv::Point3f> vAcc, vGyr;
    if (!LoadTelemetry(strImuJsonPath, imuTimestamps, vAcc, vGyr))
    {
        return 1;
    }

    // 2. Load RGB and Depth Timestamps
    vector<string> vstrImageFilenames;
    vector<string> vstrDepthFilenames;
    vector<double> vImageTimestamps;
    vector<double> vDepthTimestamps;

    // Load RGB (vstrImageFilenames is populated)
    if (!LoadImages(strFramesPath + "/rgb.txt", strFramesPath, vstrImageFilenames, vstrDepthFilenames, vImageTimestamps))
        return 1;

    // Load Depth (vstrDepthFilenames is populated)
    if (!LoadImages(strFramesPath + "/depth.txt", strFramesPath, vstrImageFilenames, vstrDepthFilenames, vDepthTimestamps))
        return 1;

    // --- 3. Load Settings and Resolution ---
    cv::FileStorage fsSettings(setting, cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        cerr << "Failed to open setting file at: " << setting << endl;
        return 1;
    }

    // RGB resolution (Primary SLAM resolution from YAML)
    cv::Size img_rgb_size(fsSettings["Camera.width"], fsSettings["Camera.height"]);

    // Depth map actual resolution (from CLI arguments)
    cv::Size img_depth_size(depth_width, depth_height);

    // --- 4. Frame Count Mismatch Handling (Trimming for Synchronization) ---
    size_t nRGB = vstrImageFilenames.size();
    size_t nDepth = vstrDepthFilenames.size();
    const double fps = fsSettings["Camera.fps"];
    const double interval_half = (1.0 / fps) / 2.0;

    const double epilson = 1e-3;
    double diff;

    // while (
    //     vDepthTimestamps[0] - vImageTimestamps[0] > epilson
    // ){
    //     vImageTimestamps.erase(vImageTimestamps.begin());
    // }

    cout << "[INFO] Image-Depth diff: " << vImageTimestamps[0] - vDepthTimestamps[0] << std::endl;

    const int nImages = vstrImageFilenames.size();
    cout << "Found " << nImages << " synchronised RGB-D frames." << endl;

    // load mask image
    cv::Mat mask_img;
    if (!mask_img_path.empty())
    {
        mask_img = cv::imread(mask_img_path, cv::IMREAD_GRAYSCALE);
        if (mask_img.size() != img_rgb_size)
        {
            std::cout << "Mask img size mismatch! Converting " << mask_img.size() << " to " << img_rgb_size << endl;
            cv::resize(mask_img, mask_img, img_rgb_size);
        }
    }

    // Create SLAM system (using IMU_RGBD mode for Depth utilization)
    cv::Ptr<cv::aruco::Dictionary> aruco_dict = cv::aruco::getPredefinedDictionary(aruco_dict_id);
    ORB_SLAM3::System SLAM(
        vocabulary,
        setting,
        ORB_SLAM3::System::IMU_RGBD, // Using IMU_RGBD mode
        enable_gui,
        load_map,
        save_map,
        // nullptr,  // aruco_dict
        // 0,        // init_tag_id
        // 0.0f      // init_tag_size
        aruco_dict,
        init_tag_id,
        init_tag_size

    );

    // --- Main Tracking Loop ---
    std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
    size_t last_imu_idx = 0;
    size_t last_depth_idx = 0;

    int n_lost_frames = 0;

    for (int frame_idx = 0; frame_idx < nImages; frame_idx++)
    {
        double tframe = (double)frame_idx / fps;

        // 1. Read RGB and Depth frames
        cv::Mat im = cv::imread(vstrImageFilenames[frame_idx], cv::IMREAD_COLOR);

        // sync RGB & Depth frame
        double TsFrame, TsDepth, TsDepthNext;
        while (last_depth_idx < vDepthTimestamps.size())
        {
            TsFrame = vImageTimestamps[frame_idx];
            TsDepth = vDepthTimestamps[last_depth_idx];

            if (TsDepth + epilson < TsFrame)
            {
                if (last_depth_idx + 1 < vDepthTimestamps.size())
                {
                    TsDepthNext = vDepthTimestamps[last_depth_idx + 1];
                    if (TsDepthNext > TsFrame + epilson)
                    {
                        cerr << "Warning: Next Depth frame is too far ahead. Not incrementing last_depth_idx for Image frame " << frame_idx << endl;
                        break;
                    }

                    last_depth_idx++;
                }
                else
                {
                    cerr << "Warning: Reached end of Depth timestamps, but still behind Image frame " << frame_idx << endl;
                    break;
                }
            }
            else if (std::abs(TsDepth - TsFrame) <= epilson)
            {
                break;
            }
            else
            {
                cerr << "Warning: Current Depth is too far ahead (> " << epilson * 1000 << "ms). Skipping Depth matching for Image frame " << frame_idx << endl;
                break;
            }
        }

        // Read RAW depth using its original resolution
        cv::Mat depth_raw = ReadRawDepth(vstrDepthFilenames[frame_idx], img_depth_size.width, img_depth_size.height);

        if (im.empty() || depth_raw.empty())
        {
            cerr << "Failed to load frame " << frame_idx << endl;
            continue;
        }

        // 2. Resize Depth Map to match RGB resolution (MANDATORY for TrackRGBD)
        cv::Mat depth_track;
        // Use nearest neighbor interpolation to preserve depth values without smoothing
        cv::resize(depth_raw, depth_track, img_rgb_size, 0, 0, cv::INTER_NEAREST);

        cv::Mat im_track = im.clone();
        if (im_track.size() != img_rgb_size)
        {
            cv::resize(im_track, im_track, img_rgb_size);
        }

        // apply mask image if loaded
        if (!mask_img.empty())
        {
            im_track.setTo(cv::Scalar(0, 0, 0), mask_img);
        }

        // NOTE: for test, check masked
        if (frame_idx == 0)
        {
            std::string output_path = "/home/sungjoon/Desktop/asdf.png";
            bool success = cv::imwrite(output_path, im_track);
            if (success)
            {
                std::cout << "Saved im_track (frame " << frame_idx << ") to " << output_path << std::endl;
            }
            else
            {
                std::cerr << "Error: Failed to save im_track to " << output_path << std::endl;
            }
        }

        // --- IMU Measurement Gathering ---
        vImuMeas.clear();

        // Gather IMU measurements up to the current frame time (tframe)

        std::cerr << "TsFrame: " << vImageTimestamps[frame_idx] - 1.76361e9 << " TsDepth: " << vDepthTimestamps[last_depth_idx] - 1.76361e9 << " ImuTimestamps " << imuTimestamps[last_imu_idx] - 1.76361e9 << " tframe: " << tframe << std::endl;

        while (imuTimestamps[last_imu_idx] <= vImageTimestamps[frame_idx])
        {
            vImuMeas.push_back(
                ORB_SLAM3::IMU::Point(
                    vAcc[last_imu_idx].x,
                    vAcc[last_imu_idx].y,
                    vAcc[last_imu_idx].z,
                    vGyr[last_imu_idx].x,
                    vGyr[last_imu_idx].y,
                    vGyr[last_imu_idx].z,
                    imuTimestamps[last_imu_idx]));
            last_imu_idx++;
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // Pass the image to the SLAM system
        SLAM.TrackRGBD(im_track, depth_track, vImageTimestamps[frame_idx], vImuMeas);

        // Check lost frames
        int mTS = SLAM.GetTrackingState();
        bool has_tracking = (mTS == 2 || mTS == 3);

        if (!has_tracking)
        {
            n_lost_frames += 1;
            std::cout << "n_lost_frames=" << n_lost_frames << std::endl;
        }

        if ((max_lost_frames > 0) && (n_lost_frames >= max_lost_frames))
        {
            std::cout << "Lost tracking on " << n_lost_frames << " >= " << max_lost_frames << " frames. Terminating!" << endl;
            SLAM.Shutdown();
            return 1;
        }

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();

        if (frame_idx % 100 == 0 && ttrack > 0)
        {
            std::cout << "Video FPS: " << fps << "\n";
            std::cout << "ORB-SLAM 3 running at: " << 1. / ttrack << " FPS\n";
        }
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    if (!output_trajectory_csv.empty())
    {
        SLAM.SaveTrajectoryCSV(output_trajectory_csv);
    }

    return 0;
}