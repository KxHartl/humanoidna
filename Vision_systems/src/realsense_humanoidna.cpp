#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <X11/Xlib.h>

namespace fs = std::filesystem;

namespace {

struct StreamSettings {
    int depth_width = 848;
    int depth_height = 480;
    int fps = 30;
};

// Change this variable to define where PNG and PCD captures are stored.
// If it is relative, it is resolved against the discovered data directory.
const fs::path kCaptureOutputDir = "kutija";

void initialize_x11_thread_support()
{
    if (XInitThreads() == 0) {
        throw std::runtime_error("Failed to initialize X11 thread support (XInitThreads).\n"
                                 "This can cause random crashes when OpenCV and PCL both use GUI windows.");
    }
}

bool has_graphical_display()
{
    const char* display_name = std::getenv("DISPLAY");
    if (display_name == nullptr || display_name[0] == '\0') {
        return false;
    }

    Display* display = XOpenDisplay(nullptr);
    if (display == nullptr) {
        return false;
    }

    XCloseDisplay(display);
    return true;
}

// Look for ../data first because that is the natural runtime layout when the
// executable is started from the build directory. A local data/ fallback keeps
// the program usable from the repository root too.
fs::path locate_data_dir()
{
    const fs::path cwd = fs::current_path();
    const fs::path first_choice = cwd / ".." / "data";
    if (fs::exists(first_choice)) {
        return fs::weakly_canonical(first_choice);
    }

    const fs::path fallback = cwd / "data";
    if (fs::exists(fallback)) {
        return fs::weakly_canonical(fallback);
    }

    throw std::runtime_error("Could not find a data directory near the current working directory.");
}

fs::path resolve_capture_output_dir(const fs::path& data_dir)
{
    fs::path output_dir = kCaptureOutputDir;
    if (output_dir.is_relative()) {
        output_dir = data_dir / output_dir;
    }

    fs::create_directories(output_dir);
    return fs::weakly_canonical(output_dir);
}

// Read the JSON twice for two different jobs:
// 1. OpenCV FileStorage extracts the basic stream settings we need for startup.
// 2. librealsense advanced mode consumes the raw JSON text to apply the camera setup.
std::string read_text_file(const fs::path& file_path)
{
    std::ifstream input(file_path);
    if (!input) {
        throw std::runtime_error("Failed to open " + file_path.string());
    }

    std::ostringstream buffer;
    buffer << input.rdbuf();
    return buffer.str();
}

StreamSettings load_stream_settings(const fs::path& json_path)
{
    cv::FileStorage file(json_path.string(), cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON);
    if (!file.isOpened()) {
        throw std::runtime_error("Failed to parse " + json_path.string());
    }

    const cv::FileNode viewer = file["viewer"];
    if (viewer.empty()) {
        throw std::runtime_error("Missing 'viewer' section in " + json_path.string());
    }

    StreamSettings settings;
    std::string width_text;
    std::string height_text;
    std::string fps_text;

    viewer["stream-width"] >> width_text;
    viewer["stream-height"] >> height_text;
    viewer["stream-fps"] >> fps_text;

    if (!width_text.empty()) {
        settings.depth_width = std::stoi(width_text);
    }
    if (!height_text.empty()) {
        settings.depth_height = std::stoi(height_text);
    }
    if (!fps_text.empty()) {
        settings.fps = std::stoi(fps_text);
    }

    return settings;
}

rs2::device find_device_by_serial(rs2::context& context, const std::string& serial)
{
    rs2::device_list devices = context.query_devices();
    for (rs2::device device : devices) {
        if (device.supports(RS2_CAMERA_INFO_SERIAL_NUMBER) &&
            serial == device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)) {
            return device;
        }
    }

    throw std::runtime_error("Could not find the requested RealSense device after reconnect.");
}

// The JSON exported by RealSense Viewer is an advanced-mode configuration for
// D400-series cameras. Loading it here keeps the program behavior close to the
// setup that was tuned in the Viewer.
std::string prepare_camera(const std::string& json_text)
{
    rs2::context context;
    rs2::device_list devices = context.query_devices();
    if (devices.size() == 0) {
        throw std::runtime_error("No RealSense device detected.");
    }

    rs2::device device = devices.front();
    std::string serial = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

    std::cout << "Using device: "
              << device.get_info(RS2_CAMERA_INFO_NAME)
              << " (serial " << serial << ")\n";

    try {
        if (device.supports(RS2_CAMERA_INFO_PRODUCT_LINE) &&
            std::string(device.get_info(RS2_CAMERA_INFO_PRODUCT_LINE)) == "D400") {
            rs400::advanced_mode advanced(device);

            if (!advanced.is_enabled()) {
                std::cout << "Enabling advanced mode so the JSON camera setup can be applied...\n";
                advanced.toggle_advanced_mode(true);
                std::this_thread::sleep_for(std::chrono::seconds(5));
                device = find_device_by_serial(context, serial);
            }

            rs400::advanced_mode advanced_after(device);
            advanced_after.load_json(json_text);
            std::cout << "Applied camera setup from JSON.\n";
        } else {
            std::cout << "Device is not in the D400 family, so advanced-mode JSON was skipped.\n";
        }
    } catch (const rs2::error& error) {
        std::cerr << "Could not apply advanced-mode JSON: " << error.what() << '\n';
    }

    return serial;
}

std::string make_date_stamp()
{
    const auto now = std::chrono::system_clock::now();
    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);

    std::tm local_time{};
    localtime_r(&now_time, &local_time);

    std::ostringstream date_stamp;
    date_stamp << std::put_time(&local_time, "%Y%m%d");
    return date_stamp.str();
}

int find_next_capture_index(const fs::path& output_dir, const std::string& date_stamp)
{
    const std::string prefix = "image_" + date_stamp + "_";
    const std::string suffix = ".png";
    int max_index = 0;

    for (const fs::directory_entry& entry : fs::directory_iterator(output_dir)) {
        if (!entry.is_regular_file()) {
            continue;
        }

        const std::string name = entry.path().filename().string();
        if (name.size() != prefix.size() + 4 + suffix.size()) {
            continue;
        }
        if (name.rfind(prefix, 0) != 0) {
            continue;
        }
        if (name.compare(name.size() - suffix.size(), suffix.size(), suffix) != 0) {
            continue;
        }

        const std::string index_text = name.substr(prefix.size(), 4);
        const bool all_digits = std::all_of(
            index_text.begin(),
            index_text.end(),
            [](char c) { return c >= '0' && c <= '9'; });
        if (!all_digits) {
            continue;
        }

        const int index = std::stoi(index_text);
        if (index >= 1 && index <= 9999) {
            max_index = std::max(max_index, index);
        }
    }

    if (max_index >= 9999) {
        throw std::runtime_error("Daily capture index limit reached (9999). Use a new output folder or a new date.");
    }

    return max_index + 1;
}

// We keep the point cloud organized so every cloud sample still corresponds to
// one image pixel. Invalid depth samples become NaNs, which PCL understands.
pcl::PointCloud<pcl::PointXYZRGB>::Ptr make_structured_cloud(
    const rs2::points& points,
    const rs2::video_frame& color_frame)
{
    using PointT = pcl::PointXYZRGB;

    auto cloud = pcl::make_shared<pcl::PointCloud<PointT>>();
    cloud->width = static_cast<std::uint32_t>(points.get_profile().as<rs2::video_stream_profile>().width());
    cloud->height = static_cast<std::uint32_t>(points.get_profile().as<rs2::video_stream_profile>().height());
    cloud->is_dense = false;
    cloud->points.resize(points.size());

    const auto color_profile = color_frame.get_profile().as<rs2::video_stream_profile>();
    const int color_width = color_profile.width();
    const int color_height = color_profile.height();
    const auto* color_data =
        reinterpret_cast<const std::uint8_t*>(color_frame.get_data());
    const auto* vertices = points.get_vertices();
    const auto* tex_coords = points.get_texture_coordinates();
    const float nan_value = std::numeric_limits<float>::quiet_NaN();

    for (std::size_t i = 0; i < points.size(); ++i) {
        PointT& point = cloud->points[i];
        point.x = nan_value;
        point.y = nan_value;
        point.z = nan_value;
        point.r = 0;
        point.g = 0;
        point.b = 0;

        const rs2::vertex& vertex = vertices[i];
        if (!std::isfinite(vertex.z) || vertex.z <= 0.f) {
            continue;
        }

        point.x = vertex.x;
        point.y = vertex.y;
        point.z = vertex.z;

        const int u = std::clamp(
            static_cast<int>(tex_coords[i].u * color_width + 0.5f), 0, color_width - 1);
        const int v = std::clamp(
            static_cast<int>(tex_coords[i].v * color_height + 0.5f), 0, color_height - 1);
        const int color_index = (v * color_width + u) * 3;

        point.b = color_data[color_index + 0];
        point.g = color_data[color_index + 1];
        point.r = color_data[color_index + 2];
    }

    return cloud;
}

void save_capture(
    const fs::path& output_dir,
    const cv::Mat& color_image,
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud)
{
    const std::string date_stamp = make_date_stamp();
    const int capture_index = find_next_capture_index(output_dir, date_stamp);

    std::ostringstream sequence;
    sequence << std::setw(4) << std::setfill('0') << capture_index;

    const std::string capture_id = date_stamp + "_" + sequence.str();
    const fs::path image_path = output_dir / ("image_" + capture_id + ".png");
    const fs::path cloud_path = output_dir / ("point_cloud_" + capture_id + ".pcd");

    if (!cv::imwrite(image_path.string(), color_image)) {
        std::cerr << "Failed to save image to " << image_path << '\n';
    } else {
        std::cout << "Saved " << image_path << '\n';
    }

    if (pcl::io::savePCDFileBinary(cloud_path.string(), *cloud) != 0) {
        std::cerr << "Failed to save point cloud to " << cloud_path << '\n';
    } else {
        std::cout << "Saved " << cloud_path << '\n';
    }
}

}  // namespace

int main()
{
    try {
        initialize_x11_thread_support();
        if (!has_graphical_display()) {
            throw std::runtime_error(
                "No X11 display detected. Run from a desktop session with DISPLAY set.");
        }

        // Phase 1: find the configuration and output directory that lives next
        // to the project, then read the stream settings and the raw JSON text.
        const fs::path data_dir = locate_data_dir();
        const fs::path capture_output_dir = resolve_capture_output_dir(data_dir);
        const fs::path json_path = data_dir / "realsense_humanoidna.json";
        const StreamSettings settings = load_stream_settings(json_path);
        const std::string json_text = read_text_file(json_path);

        // Phase 2: discover the camera, apply the JSON setup, and lock the
        // pipeline to that specific device serial number.
        const std::string serial = prepare_camera(json_text);

        rs2::pipeline pipeline;
        rs2::config config;
        config.enable_device(serial);
        config.enable_stream(
            RS2_STREAM_DEPTH,
            settings.depth_width,
            settings.depth_height,
            RS2_FORMAT_Z16,
            settings.fps);
        config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, settings.fps);

        rs2::pipeline_profile profile = pipeline.start(config);
        rs2::align align_to_color(RS2_STREAM_COLOR);
        rs2::pointcloud pointcloud_builder;

        // Phase 3: create one 2D image window and one 3D point-cloud window.
        cv::namedWindow("Color Image", cv::WINDOW_AUTOSIZE);

        auto viewer = pcl::make_shared<pcl::visualization::PCLVisualizer>("Colored Point Cloud");
        viewer->setBackgroundColor(0.08, 0.08, 0.08);
        viewer->addCoordinateSystem(0.1);
        viewer->initCameraParameters();
        viewer->setCameraPosition(
            0.0, 0.0, -0.45,
            0.0, 0.0, 1.0,
            0.0, -1.0, 0.0);
        viewer->setCameraClipDistances(0.01, 10.0);

        cv::Mat last_color_image;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr last_cloud =
            pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

        std::cout << "Streaming started.\n";
                std::cout << "Point-cloud filtering: off (all valid depth points are shown).\n";
                std::cout << "Capture output directory: " << capture_output_dir << "\n";
        std::cout << "Press Space to save the current image and structured point cloud.\n";
        std::cout << "Press ESC or close the point-cloud window to exit.\n";

        while (!viewer->wasStopped()) {
            // Phase 4: grab synchronized frames, align depth to color, and
            // build a structured RGB point cloud where each point still maps
            // back to one image pixel.
            rs2::frameset frames = pipeline.wait_for_frames();
            frames = align_to_color.process(frames);

            const rs2::video_frame color_frame = frames.get_color_frame();
            const rs2::depth_frame depth_frame = frames.get_depth_frame();
            if (!color_frame || !depth_frame) {
                continue;
            }

            pointcloud_builder.map_to(color_frame);
            const rs2::points points = pointcloud_builder.calculate(depth_frame);
            last_cloud = make_structured_cloud(points, color_frame);

            last_color_image = cv::Mat(
                                   cv::Size(color_frame.get_width(), color_frame.get_height()),
                                   CV_8UC3,
                                   const_cast<void*>(color_frame.get_data()),
                                   cv::Mat::AUTO_STEP)
                                   .clone();

            // Phase 5: refresh the 2D and 3D views so learning stays visual.
            cv::imshow("Color Image", last_color_image);

            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(last_cloud);
            if (!viewer->updatePointCloud(last_cloud, rgb, "cloud")) {
                viewer->addPointCloud(last_cloud, rgb, "cloud");
                viewer->setPointCloudRenderingProperties(
                    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
            }
            viewer->getRenderWindow()->Render();

            // Phase 6: Space saves the latest image and organized color cloud
            // using timestamped filenames in the configured output directory.
            const int key = cv::waitKey(1);
            if (key == 27) {
                break;
            }
            if (key == ' ') {
                save_capture(capture_output_dir, last_color_image, last_cloud);
            }
        }

        pipeline.stop();
        cv::destroyAllWindows();
        return 0;
    } catch (const rs2::error& error) {
        std::cerr << "RealSense error: " << error.what() << '\n';
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
    }

    return 1;
}
