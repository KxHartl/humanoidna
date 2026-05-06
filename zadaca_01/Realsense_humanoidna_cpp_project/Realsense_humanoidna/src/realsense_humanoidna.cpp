#include <algorithm>
#include <chrono>
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

namespace fs = std::filesystem;

namespace {

struct StreamSettings {
    int depth_width = 848;
    int depth_height = 480;
    int fps = 30;
};

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

std::string make_timestamp()
{
    const auto now = std::chrono::system_clock::now();
    const auto milliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);

    std::tm local_time{};
    localtime_r(&now_time, &local_time);

    std::ostringstream stamp;
    stamp << std::put_time(&local_time, "%Y%m%d_%H%M%S")
          << '_' << std::setw(3) << std::setfill('0') << milliseconds.count();
    return stamp.str();
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
    const fs::path& data_dir,
    const cv::Mat& color_image,
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud)
{
    const std::string timestamp = make_timestamp();
    const fs::path image_path = data_dir / ("image_" + timestamp + ".png");
    const fs::path cloud_path = data_dir / ("point_cloud_" + timestamp + ".pcd");

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
        // Phase 1: find the configuration and output directory that lives next
        // to the project, then read the stream settings and the raw JSON text.
        const fs::path data_dir = locate_data_dir();
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

        cv::Mat last_color_image;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr last_cloud =
            pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

        std::cout << "Streaming started.\n";
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
                    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
            }
            viewer->spinOnce(1, false);

            // Phase 6: Space saves the latest image and organized color cloud
            // using timestamped filenames in the data directory.
            const int key = cv::waitKey(1);
            if (key == 27) {
                break;
            }
            if (key == ' ') {
                save_capture(data_dir, last_color_image, last_cloud);
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
