#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#if defined(__linux__) && __has_include(<X11/Xlib.h>)
#include <X11/Xlib.h>
#ifdef Success
#undef Success
#endif
#ifdef Status
#undef Status
#endif
#define RGB_PCD_HAS_X11 1
#endif

#include <Eigen/Geometry>

#include <pcl/common/centroid.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/common/point_tests.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace fs = std::filesystem;

namespace {

constexpr char kWindow2d[] = "2D Segmentation";

enum class FitMode {
    kNone = 0,
    kSphere = 1,
    kClusterObb = 2,
    kCylinder = 3,
};

struct AppOptions {
    fs::path dataRoot{"."};
    std::optional<fs::path> imagesDir;
    std::optional<fs::path> cloudsDir;
};

struct DatasetPair {
    std::string key;
    fs::path imagePath;
    fs::path cloudPath;
};

struct UiState {
    int datasetIndex = 0;
    int lowH = 0;
    int lowS = 0;
    int lowV = 0;
    int highH = 179;
    int highS = 255;
    int highV = 255;
    int mode = static_cast<int>(FitMode::kNone);
    bool dirty = true;
};

struct ProcessingConfig {
    float sphereDistanceThreshold = 0.01F;
    float sphereMinInlierRatio = 0.15F;

    float cylinderDistanceThreshold = 0.015F;
    float cylinderMinInlierRatio = 0.12F;
    float cylinderNormalDistanceWeight = 0.1F;
    float cylinderMinRadius = 0.005F;
    float cylinderMaxRadius = 0.5F;

    float clusterTolerance = 0.02F;
    int clusterMinSize = 40;
    int clusterMaxSize = 200000;
};

struct LoadedPair {
    cv::Mat imageBgr;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
};

struct FilterResult {
    cv::Mat mask;
    cv::Mat filteredImage;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud;
    int activeMaskPixels = 0;
};

struct SphereDetection {
    bool found = false;
    pcl::ModelCoefficients coefficients;
    pcl::PointIndices inliers;
    double inlierRatio = 0.0;
};

struct CylinderDetection {
    bool found = false;
    pcl::ModelCoefficients coefficients;
    pcl::PointIndices inliers;
    double inlierRatio = 0.0;
};

struct ClusterObb {
    Eigen::Vector3f position = Eigen::Vector3f::Zero();
    Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity();
    Eigen::Vector3f extents = Eigen::Vector3f::Zero();
    Eigen::Vector4f centroid = Eigen::Vector4f::Zero();
    std::size_t pointCount = 0;
};

std::string toLower(std::string text) {
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return text;
}

const std::set<std::string>& imageExtensions() {
    static const std::set<std::string> extensions = {
        ".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"};
    return extensions;
}

bool isImageFile(const fs::path& filePath) {
    return imageExtensions().count(toLower(filePath.extension().string())) > 0;
}

bool startsWith(const std::string& text, const std::string& prefix) {
    return text.size() >= prefix.size() &&
           std::equal(prefix.begin(), prefix.end(), text.begin());
}

void stripFirstMatchingPrefix(std::string& text, const std::vector<std::string>& prefixes) {
    for (const auto& prefix : prefixes) {
        if (startsWith(text, prefix)) {
            text.erase(0, prefix.size());
            return;
        }
    }
}

std::string normalizedPairKey(const fs::path& filePath, bool isCloudFile) {
    std::string key = toLower(filePath.stem().string());

    static const std::vector<std::string> imagePrefixes = {
        "image_", "img_", "rgb_"};
    static const std::vector<std::string> cloudPrefixes = {
        "point_cloud_", "pointcloud_", "cloud_", "pcd_"};

    if (isCloudFile) {
        stripFirstMatchingPrefix(key, cloudPrefixes);
    } else {
        stripFirstMatchingPrefix(key, imagePrefixes);
    }

    return key;
}

std::string modeName(FitMode mode) {
    switch (mode) {
        case FitMode::kNone:
            return "No fitting";
        case FitMode::kSphere:
            return "Sphere fitting";
        case FitMode::kClusterObb:
            return "Cluster + OBB";
        case FitMode::kCylinder:
            return "Cylinder fitting";
        default:
            return "Unknown";
    }
}

void printUsage(const char* programName) {
    std::cout << "Usage: " << programName << " [--data-root <path>] [--images-dir <path> --clouds-dir <path>]\n"
              << "\n"
              << "Examples:\n"
              << "  " << programName << " --data-root ./dataset\n"
              << "  " << programName << " --images-dir ./dataset/images --clouds-dir ./dataset/clouds\n";
}

bool parseArguments(int argc, char** argv, AppOptions& options) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];

        if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return false;
        }

        auto readPathArg = [&](const std::string& flag, std::optional<fs::path>* outOptional, fs::path* outRequired) -> bool {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for " << flag << '\n';
                return false;
            }
            const fs::path value = argv[++i];
            if (outOptional != nullptr) {
                *outOptional = value;
            }
            if (outRequired != nullptr) {
                *outRequired = value;
            }
            return true;
        };

        if (arg == "--data-root") {
            if (!readPathArg(arg, nullptr, &options.dataRoot)) {
                return false;
            }
        } else if (arg == "--images-dir") {
            if (!readPathArg(arg, &options.imagesDir, nullptr)) {
                return false;
            }
        } else if (arg == "--clouds-dir") {
            if (!readPathArg(arg, &options.cloudsDir, nullptr)) {
                return false;
            }
        } else if (arg.rfind("--", 0) == 0) {
            std::cerr << "Unknown argument: " << arg << '\n';
            return false;
        } else {
            options.dataRoot = arg;
        }
    }

    if ((options.imagesDir.has_value() && !options.cloudsDir.has_value()) ||
        (!options.imagesDir.has_value() && options.cloudsDir.has_value())) {
        std::cerr << "Both --images-dir and --clouds-dir must be provided together.\n";
        return false;
    }

    return true;
}

void scanDirectory(const fs::path& directory,
                   std::map<std::string, fs::path>& images,
                   std::map<std::string, fs::path>& clouds,
                   bool collectImages,
                   bool collectClouds) {
    if (!fs::exists(directory)) {
        std::cerr << "Directory not found: " << directory << '\n';
        return;
    }

    for (const auto& entry : fs::recursive_directory_iterator(directory, fs::directory_options::skip_permission_denied)) {
        if (!entry.is_regular_file()) {
            continue;
        }

        const fs::path filePath = entry.path();
        const std::string extension = toLower(filePath.extension().string());

        if (collectImages && imageExtensions().count(extension) > 0) {
            const std::string key = normalizedPairKey(filePath, false);
            const auto [it, inserted] = images.emplace(key, filePath);
            if (!inserted) {
                std::cerr << "Duplicate image key '" << key
                          << "'. Keeping " << it->second
                          << " and ignoring " << filePath << '\n';
            }
        }

        if (collectClouds && extension == ".pcd") {
            const std::string key = normalizedPairKey(filePath, true);
            const auto [it, inserted] = clouds.emplace(key, filePath);
            if (!inserted) {
                std::cerr << "Duplicate cloud key '" << key
                          << "'. Keeping " << it->second
                          << " and ignoring " << filePath << '\n';
            }
        }
    }
}

std::vector<DatasetPair> discoverPairs(const AppOptions& options) {
    std::map<std::string, fs::path> images;
    std::map<std::string, fs::path> clouds;

    if (options.imagesDir.has_value() && options.cloudsDir.has_value()) {
        scanDirectory(*options.imagesDir, images, clouds, true, false);
        scanDirectory(*options.cloudsDir, images, clouds, false, true);
    } else {
        scanDirectory(options.dataRoot, images, clouds, true, true);
    }

    std::vector<DatasetPair> pairs;
    for (const auto& [key, imagePath] : images) {
        const auto cloudIt = clouds.find(key);
        if (cloudIt != clouds.end()) {
            pairs.push_back({key, imagePath, cloudIt->second});
        }
    }

    std::sort(pairs.begin(), pairs.end(), [](const DatasetPair& a, const DatasetPair& b) {
        return a.key < b.key;
    });

    return pairs;
}

std::optional<LoadedPair> loadPair(const DatasetPair& pair) {
    LoadedPair loaded;
    loaded.imageBgr = cv::imread(pair.imagePath.string(), cv::IMREAD_COLOR);
    if (loaded.imageBgr.empty()) {
        std::cerr << "Failed to load image: " << pair.imagePath << '\n';
        return std::nullopt;
    }

    loaded.cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(pair.cloudPath.string(), *loaded.cloud) == -1) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pair.cloudPath.string(), *xyzCloud) == -1) {
            std::cerr << "Failed to load PCD cloud: " << pair.cloudPath << '\n';
            return std::nullopt;
        }

        loaded.cloud->points.resize(xyzCloud->points.size());
        loaded.cloud->width = xyzCloud->width;
        loaded.cloud->height = xyzCloud->height;
        loaded.cloud->is_dense = xyzCloud->is_dense;

        for (std::size_t i = 0; i < xyzCloud->points.size(); ++i) {
            loaded.cloud->points[i].x = xyzCloud->points[i].x;
            loaded.cloud->points[i].y = xyzCloud->points[i].y;
            loaded.cloud->points[i].z = xyzCloud->points[i].z;
            loaded.cloud->points[i].r = 0;
            loaded.cloud->points[i].g = 0;
            loaded.cloud->points[i].b = 0;
        }
    }

    // If alignment is correct, overwrite cloud RGB with image colors to guarantee 2D/3D color coherence.
    if (loaded.cloud->isOrganized() &&
        loaded.cloud->width == static_cast<std::uint32_t>(loaded.imageBgr.cols) &&
        loaded.cloud->height == static_cast<std::uint32_t>(loaded.imageBgr.rows) &&
        loaded.cloud->points.size() == loaded.imageBgr.total()) {
        for (int r = 0; r < loaded.imageBgr.rows; ++r) {
            const auto* row = loaded.imageBgr.ptr<cv::Vec3b>(r);
            for (int c = 0; c < loaded.imageBgr.cols; ++c) {
                const std::size_t idx = static_cast<std::size_t>(r) * loaded.imageBgr.cols + c;
                auto& point = loaded.cloud->points[idx];
                point.r = row[c][2];
                point.g = row[c][1];
                point.b = row[c][0];
            }
        }
    }

    return loaded;
}

bool verifyOrganizedAlignment(const LoadedPair& loaded, std::string& message) {
    const auto& image = loaded.imageBgr;
    const auto& cloud = loaded.cloud;

    if (image.empty()) {
        message = "Image is empty";
        return false;
    }
    if (!cloud || cloud->empty()) {
        message = "Point cloud is empty";
        return false;
    }
    if (!cloud->isOrganized()) {
        message = "Point cloud is not organized (height <= 1)";
        return false;
    }
    if (cloud->width != static_cast<std::uint32_t>(image.cols) ||
        cloud->height != static_cast<std::uint32_t>(image.rows)) {
        std::ostringstream oss;
        oss << "Size mismatch: image=" << image.cols << "x" << image.rows
            << " cloud=" << cloud->width << "x" << cloud->height;
        message = oss.str();
        return false;
    }
    if (cloud->points.size() != image.total()) {
        std::ostringstream oss;
        oss << "Cardinality mismatch: points=" << cloud->points.size() << " pixels=" << image.total();
        message = oss.str();
        return false;
    }

    message = "Alignment verified";
    return true;
}

void onTrackbarChange(int, void* userData) {
    auto* uiState = static_cast<UiState*>(userData);
    if (uiState != nullptr) {
        uiState->dirty = true;
    }
}

void normalizeHsvBounds(UiState& uiState) {
    uiState.lowH = std::clamp(uiState.lowH, 0, 179);
    uiState.highH = std::clamp(uiState.highH, 0, 179);
    uiState.lowS = std::clamp(uiState.lowS, 0, 255);
    uiState.highS = std::clamp(uiState.highS, 0, 255);
    uiState.lowV = std::clamp(uiState.lowV, 0, 255);
    uiState.highV = std::clamp(uiState.highV, 0, 255);

    if (uiState.lowH > uiState.highH) {
        uiState.lowH = uiState.highH;
    }
    if (uiState.lowS > uiState.highS) {
        uiState.lowS = uiState.highS;
    }
    if (uiState.lowV > uiState.highV) {
        uiState.lowV = uiState.highV;
    }

    cv::setTrackbarPos("Low H", kWindow2d, uiState.lowH);
    cv::setTrackbarPos("High H", kWindow2d, uiState.highH);
    cv::setTrackbarPos("Low S", kWindow2d, uiState.lowS);
    cv::setTrackbarPos("High S", kWindow2d, uiState.highS);
    cv::setTrackbarPos("Low V", kWindow2d, uiState.lowV);
    cv::setTrackbarPos("High V", kWindow2d, uiState.highV);
}

void syncUiFromTrackbars(UiState& uiState) {
    uiState.datasetIndex = cv::getTrackbarPos("Dataset", kWindow2d);
    uiState.mode = cv::getTrackbarPos("Mode 0None1Sphere2OBB3Cyl", kWindow2d);
    uiState.lowH = cv::getTrackbarPos("Low H", kWindow2d);
    uiState.highH = cv::getTrackbarPos("High H", kWindow2d);
    uiState.lowS = cv::getTrackbarPos("Low S", kWindow2d);
    uiState.highS = cv::getTrackbarPos("High S", kWindow2d);
    uiState.lowV = cv::getTrackbarPos("Low V", kWindow2d);
    uiState.highV = cv::getTrackbarPos("High V", kWindow2d);
}

FitMode selectedMode(const UiState& uiState) {
    const int mode = std::clamp(uiState.mode, 0, 3);
    return static_cast<FitMode>(mode);
}

FilterResult buildMaskAndFilter(const LoadedPair& loaded, const UiState& uiState) {
    FilterResult result;

    cv::Mat hsv;
    cv::cvtColor(loaded.imageBgr, hsv, cv::COLOR_BGR2HSV);

    cv::inRange(
        hsv,
        cv::Scalar(uiState.lowH, uiState.lowS, uiState.lowV),
        cv::Scalar(uiState.highH, uiState.highS, uiState.highV),
        result.mask);

    result.activeMaskPixels = cv::countNonZero(result.mask);
    result.filteredImage = cv::Mat::zeros(loaded.imageBgr.size(), loaded.imageBgr.type());
    result.filteredCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    result.filteredCloud->points.reserve(static_cast<std::size_t>(result.activeMaskPixels));

    for (int r = 0; r < loaded.imageBgr.rows; ++r) {
        const auto* maskRow = result.mask.ptr<std::uint8_t>(r);
        const auto* imageRow = loaded.imageBgr.ptr<cv::Vec3b>(r);

        for (int c = 0; c < loaded.imageBgr.cols; ++c) {
            if (maskRow[c] == 0) {
                continue;
            }

            const std::size_t idx = static_cast<std::size_t>(r) * loaded.imageBgr.cols + c;
            const auto& sourcePoint = loaded.cloud->points[idx];
            if (!pcl::isFinite(sourcePoint)) {
                continue;
            }

            pcl::PointXYZRGB filteredPoint = sourcePoint;
            filteredPoint.r = imageRow[c][2];
            filteredPoint.g = imageRow[c][1];
            filteredPoint.b = imageRow[c][0];

            result.filteredCloud->points.push_back(filteredPoint);
            result.filteredImage.at<cv::Vec3b>(r, c) = imageRow[c];
        }
    }

    result.filteredCloud->width = static_cast<std::uint32_t>(result.filteredCloud->points.size());
    result.filteredCloud->height = 1;
    result.filteredCloud->is_dense = false;

    return result;
}

SphereDetection detectSphere(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const ProcessingConfig& config) {
    SphereDetection detection;

    if (!cloud || cloud->size() < 30) {
        return detection;
    }

    pcl::SACSegmentation<pcl::PointXYZRGB> segmentation;
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_SPHERE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setMaxIterations(3000);
    segmentation.setDistanceThreshold(config.sphereDistanceThreshold);
    segmentation.setInputCloud(cloud);
    segmentation.segment(detection.inliers, detection.coefficients);

    if (detection.inliers.indices.empty() || detection.coefficients.values.size() < 4) {
        return detection;
    }

    detection.inlierRatio = static_cast<double>(detection.inliers.indices.size()) /
                            static_cast<double>(std::max<std::size_t>(cloud->size(), 1U));
    detection.found = detection.inlierRatio >= config.sphereMinInlierRatio;
    return detection;
}

CylinderDetection detectCylinder(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const ProcessingConfig& config) {
    CylinderDetection detection;

    if (!cloud || cloud->size() < 50) {
        return detection;
    }

    const int kNeighbors = std::min(30, static_cast<int>(cloud->size()) - 1);
    if (kNeighbors < 5) {
        return detection;
    }

    auto normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
    auto kdTree = pcl::search::KdTree<pcl::PointXYZRGB>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGB>);

    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimator;
    normalEstimator.setInputCloud(cloud);
    normalEstimator.setSearchMethod(kdTree);
    normalEstimator.setKSearch(kNeighbors);
    normalEstimator.compute(*normals);

    pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> segmentation;
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_CYLINDER);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setNormalDistanceWeight(config.cylinderNormalDistanceWeight);
    segmentation.setMaxIterations(5000);
    segmentation.setDistanceThreshold(config.cylinderDistanceThreshold);
    segmentation.setRadiusLimits(config.cylinderMinRadius, config.cylinderMaxRadius);
    segmentation.setInputCloud(cloud);
    segmentation.setInputNormals(normals);
    segmentation.segment(detection.inliers, detection.coefficients);

    if (detection.inliers.indices.empty() || detection.coefficients.values.size() < 7) {
        return detection;
    }

    detection.inlierRatio = static_cast<double>(detection.inliers.indices.size()) /
                            static_cast<double>(std::max<std::size_t>(cloud->size(), 1U));
    detection.found = detection.inlierRatio >= config.cylinderMinInlierRatio;
    return detection;
}

std::vector<ClusterObb> extractClustersWithObb(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                                               const ProcessingConfig& config) {
    std::vector<ClusterObb> output;

    if (!cloud || cloud->empty()) {
        return output;
    }

    auto kdTree = pcl::search::KdTree<pcl::PointXYZRGB>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGB>);
    kdTree->setInputCloud(cloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> extraction;
    extraction.setClusterTolerance(config.clusterTolerance);
    extraction.setMinClusterSize(config.clusterMinSize);
    extraction.setMaxClusterSize(config.clusterMaxSize);
    extraction.setSearchMethod(kdTree);
    extraction.setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    extraction.extract(clusterIndices);

    output.reserve(clusterIndices.size());
    for (const auto& indices : clusterIndices) {
        auto clusterCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        clusterCloud->points.reserve(indices.indices.size());

        for (int idx : indices.indices) {
            clusterCloud->points.push_back(cloud->points[static_cast<std::size_t>(idx)]);
        }
        clusterCloud->width = static_cast<std::uint32_t>(clusterCloud->points.size());
        clusterCloud->height = 1;
        clusterCloud->is_dense = false;

        pcl::MomentOfInertiaEstimation<pcl::PointXYZRGB> featureEstimator;
        featureEstimator.setInputCloud(clusterCloud);
        featureEstimator.compute();

        pcl::PointXYZRGB minPointObb;
        pcl::PointXYZRGB maxPointObb;
        pcl::PointXYZRGB positionObb;
        Eigen::Matrix3f rotationalMatrixObb;
        featureEstimator.getOBB(minPointObb, maxPointObb, positionObb, rotationalMatrixObb);

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*clusterCloud, centroid);

        ClusterObb obb;
        obb.position = Eigen::Vector3f(positionObb.x, positionObb.y, positionObb.z);
        obb.orientation = Eigen::Quaternionf(rotationalMatrixObb);
        obb.extents = Eigen::Vector3f(
            std::abs(maxPointObb.x - minPointObb.x),
            std::abs(maxPointObb.y - minPointObb.y),
            std::abs(maxPointObb.z - minPointObb.z));
        obb.centroid = centroid;
        obb.pointCount = indices.indices.size();
        output.push_back(obb);
    }

    return output;
}

void drawOverlayText(cv::Mat& canvas, const std::vector<std::string>& lines) {
    int y = 26;
    for (const auto& line : lines) {
        cv::putText(canvas, line, cv::Point(12, y), cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
        cv::putText(canvas, line, cv::Point(12, y), cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        y += 28;
    }
}

cv::Mat composeDisplay(const LoadedPair& loaded,
                       const FilterResult& filtered,
                       const DatasetPair& pair,
                       const UiState& uiState,
                       int pairCount) {
    cv::Mat original = loaded.imageBgr.clone();
    cv::Mat masked = filtered.filteredImage.clone();

    cv::Mat combined;
    cv::hconcat(original, masked, combined);

    const FitMode mode = selectedMode(uiState);
    const int totalPixels = loaded.imageBgr.rows * loaded.imageBgr.cols;

    std::vector<std::string> overlayLines;
    overlayLines.emplace_back("Dataset: " + pair.key + " (" + std::to_string(uiState.datasetIndex + 1) + "/" + std::to_string(pairCount) + ")");
    overlayLines.emplace_back("Mode: " + modeName(mode) + " | Processing: HSV mask -> synchronized 2D/3D filter");
    overlayLines.emplace_back(
        "HSV Low(" + std::to_string(uiState.lowH) + "," + std::to_string(uiState.lowS) + "," + std::to_string(uiState.lowV) +
        ") High(" + std::to_string(uiState.highH) + "," + std::to_string(uiState.highS) + "," + std::to_string(uiState.highV) + ")");
    overlayLines.emplace_back(
        "Mask active pixels: " + std::to_string(filtered.activeMaskPixels) + "/" + std::to_string(totalPixels) +
        " | Filtered 3D points: " + std::to_string(filtered.filteredCloud->size()));

    drawOverlayText(combined, overlayLines);
    return combined;
}

void updateViewerBase(pcl::visualization::PCLVisualizer::Ptr viewer,
                      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& filteredCloud,
                      const std::string& statusText) {
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    viewer->setBackgroundColor(0.07, 0.07, 0.09);
    viewer->addCoordinateSystem(0.1);

    if (filteredCloud && !filteredCloud->empty()) {
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(filteredCloud);
        viewer->addPointCloud<pcl::PointXYZRGB>(filteredCloud, rgb, "filtered_cloud");
        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "filtered_cloud");
    }

    viewer->addText(statusText, 10, 10, 14, 0.95, 0.95, 0.95, "status_text");
    viewer->getRenderWindow()->Render();
}

void logHeader(const DatasetPair& pair, const FitMode mode, const FilterResult& filtered, const LoadedPair& loaded) {
    const int totalPixels = loaded.imageBgr.rows * loaded.imageBgr.cols;

    std::cout << "\n=== Dataset: " << pair.key << " ===\n";
    std::cout << "Mode: " << modeName(mode) << '\n';
    std::cout << "Mask active pixels: " << filtered.activeMaskPixels << "/" << totalPixels
              << " | Filtered 3D points: " << filtered.filteredCloud->size() << '\n';
}

void runModelAndRender(pcl::visualization::PCLVisualizer::Ptr viewer,
                       const DatasetPair& pair,
                       const LoadedPair& loaded,
                       const FilterResult& filtered,
                       const UiState& uiState,
                       const ProcessingConfig& config) {
    const FitMode mode = selectedMode(uiState);

    std::ostringstream status;
    status << "Dataset=" << pair.key
           << " | Mode=" << modeName(mode)
           << " | ActiveMask=" << filtered.activeMaskPixels
           << " | Points=" << filtered.filteredCloud->size();

    updateViewerBase(viewer, filtered.filteredCloud, status.str());
    logHeader(pair, mode, filtered, loaded);

    std::cout << std::fixed << std::setprecision(3);

    if (mode == FitMode::kNone) {
        std::cout << "No geometric fitting selected.\n";
        return;
    }

    if (filtered.filteredCloud->empty()) {
        std::cout << "No filtered points available for geometric analysis.\n";
        return;
    }

    if (mode == FitMode::kSphere) {
        const SphereDetection sphere = detectSphere(filtered.filteredCloud, config);

        if (!sphere.found) {
            std::cout << "Sphere model not accepted (RANSAC failed or inlier ratio too low).\n";
            return;
        }

        viewer->addSphere(sphere.coefficients, "sphere_model");
        viewer->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_COLOR, 0.2, 0.9, 0.2, "sphere_model");

        std::cout << "Sphere center: ("
                  << sphere.coefficients.values[0] << ", "
                  << sphere.coefficients.values[1] << ", "
                  << sphere.coefficients.values[2] << ")"
                  << " radius=" << sphere.coefficients.values[3]
                  << " inliers=" << sphere.inliers.indices.size()
                  << " ratio=" << sphere.inlierRatio << '\n';
        return;
    }

    if (mode == FitMode::kClusterObb) {
        const std::vector<ClusterObb> clusters = extractClustersWithObb(filtered.filteredCloud, config);

        if (clusters.empty()) {
            std::cout << "No Euclidean clusters extracted with current settings.\n";
            return;
        }

        int clusterId = 0;
        for (const auto& cluster : clusters) {
            const std::string shapeId = "obb_" + std::to_string(clusterId);
            viewer->addCube(cluster.position, cluster.orientation,
                            cluster.extents.x(), cluster.extents.y(), cluster.extents.z(), shapeId);
            viewer->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                shapeId);
            viewer->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_COLOR, 0.95, 0.7, 0.2, shapeId);

            std::cout << "Cluster " << clusterId
                      << " centroid: (" << cluster.centroid[0] << ", "
                      << cluster.centroid[1] << ", "
                      << cluster.centroid[2] << ")"
                      << " points=" << cluster.pointCount
                      << " extents=(" << cluster.extents.x() << ", "
                      << cluster.extents.y() << ", "
                      << cluster.extents.z() << ")\n";
            ++clusterId;
        }
        return;
    }

    if (mode == FitMode::kCylinder) {
        const CylinderDetection cylinder = detectCylinder(filtered.filteredCloud, config);

        if (!cylinder.found) {
            std::cout << "Cylinder model not accepted (RANSAC failed or inlier ratio too low).\n";
            return;
        }

        viewer->addCylinder(cylinder.coefficients, "cylinder_model");
        viewer->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_COLOR, 0.2, 0.7, 1.0, "cylinder_model");

        std::cout << "Cylinder axis point: ("
                  << cylinder.coefficients.values[0] << ", "
                  << cylinder.coefficients.values[1] << ", "
                  << cylinder.coefficients.values[2] << ")"
                  << " axis direction=("
                  << cylinder.coefficients.values[3] << ", "
                  << cylinder.coefficients.values[4] << ", "
                  << cylinder.coefficients.values[5] << ")"
                  << " radius=" << cylinder.coefficients.values[6]
                  << " inliers=" << cylinder.inliers.indices.size()
                  << " ratio=" << cylinder.inlierRatio << '\n';
        return;
    }
}

void buildTrackbars(UiState& uiState, int datasetCount) {
    cv::namedWindow(kWindow2d, cv::WINDOW_NORMAL);

    cv::createTrackbar("Dataset", kWindow2d, nullptr, std::max(datasetCount - 1, 0), onTrackbarChange, &uiState);
    cv::createTrackbar("Mode 0None1Sphere2OBB3Cyl", kWindow2d, nullptr, 3, onTrackbarChange, &uiState);

    cv::createTrackbar("Low H", kWindow2d, nullptr, 179, onTrackbarChange, &uiState);
    cv::createTrackbar("High H", kWindow2d, nullptr, 179, onTrackbarChange, &uiState);
    cv::createTrackbar("Low S", kWindow2d, nullptr, 255, onTrackbarChange, &uiState);
    cv::createTrackbar("High S", kWindow2d, nullptr, 255, onTrackbarChange, &uiState);
    cv::createTrackbar("Low V", kWindow2d, nullptr, 255, onTrackbarChange, &uiState);
    cv::createTrackbar("High V", kWindow2d, nullptr, 255, onTrackbarChange, &uiState);

    cv::setTrackbarPos("Dataset", kWindow2d, uiState.datasetIndex);
    cv::setTrackbarPos("Mode 0None1Sphere2OBB3Cyl", kWindow2d, uiState.mode);
    cv::setTrackbarPos("Low H", kWindow2d, uiState.lowH);
    cv::setTrackbarPos("High H", kWindow2d, uiState.highH);
    cv::setTrackbarPos("Low S", kWindow2d, uiState.lowS);
    cv::setTrackbarPos("High S", kWindow2d, uiState.highS);
    cv::setTrackbarPos("Low V", kWindow2d, uiState.lowV);
    cv::setTrackbarPos("High V", kWindow2d, uiState.highV);
}

}  // namespace

int main(int argc, char** argv) {
#if defined(RGB_PCD_HAS_X11)
    // OpenCV HighGUI and VTK/PCL may use different threads; XInitThreads helps avoid X11 race crashes.
    XInitThreads();
#endif

    AppOptions options;
    if (!parseArguments(argc, argv, options)) {
        return 1;
    }

    const std::vector<DatasetPair> pairs = discoverPairs(options);
    if (pairs.empty()) {
        std::cerr << "No aligned candidates found. Ensure image and .pcd files share the same stem key.\n";
        printUsage(argv[0]);
        return 1;
    }

    std::cout << "Discovered " << pairs.size() << " image-cloud filename pairs.\n";

    UiState uiState;
    ProcessingConfig config;
    buildTrackbars(uiState, static_cast<int>(pairs.size()));

    auto viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("Filtered 3D View"));
    viewer->initCameraParameters();

    int loadedPairIndex = -1;
    std::optional<LoadedPair> loadedPair;
    cv::Mat displayImage;

    bool running = true;
    while (running && !viewer->wasStopped()) {
        syncUiFromTrackbars(uiState);

        const int clampedIndex = std::clamp(uiState.datasetIndex, 0, static_cast<int>(pairs.size() - 1));
        if (clampedIndex != uiState.datasetIndex) {
            uiState.datasetIndex = clampedIndex;
            cv::setTrackbarPos("Dataset", kWindow2d, uiState.datasetIndex);
            uiState.dirty = true;
        }

        if (uiState.dirty || loadedPairIndex != uiState.datasetIndex) {
            normalizeHsvBounds(uiState);
            const DatasetPair& selectedPair = pairs[static_cast<std::size_t>(uiState.datasetIndex)];

            loadedPair = loadPair(selectedPair);
            loadedPairIndex = uiState.datasetIndex;

            if (!loadedPair.has_value()) {
                std::cerr << "Skipping invalid pair due to loading error: " << selectedPair.key << '\n';
                displayImage = cv::Mat::zeros(cv::Size(1280, 360), CV_8UC3);
                drawOverlayText(displayImage, {
                    "Failed to load selected pair: " + selectedPair.key,
                    "Check file format and data integrity."
                });
                uiState.dirty = false;
            } else {
                std::string verificationMessage;
                if (!verifyOrganizedAlignment(*loadedPair, verificationMessage)) {
                    std::cerr << "Alignment verification failed for key " << selectedPair.key
                              << ": " << verificationMessage << '\n';

                    cv::Mat warningView = loadedPair->imageBgr.clone();
                    drawOverlayText(warningView, {
                        "Alignment check failed for pair: " + selectedPair.key,
                        verificationMessage,
                        "Cloud must be organized and match image dimensions."
                    });
                    displayImage = warningView;

                    updateViewerBase(viewer, pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>()),
                                     "Alignment failed for current pair");
                } else {
                    const FilterResult filtered = buildMaskAndFilter(*loadedPair, uiState);
                    displayImage = composeDisplay(*loadedPair, filtered, selectedPair, uiState, static_cast<int>(pairs.size()));
                    runModelAndRender(viewer, selectedPair, *loadedPair, filtered, uiState, config);
                }
                uiState.dirty = false;
            }
        }

        if (!displayImage.empty()) {
            cv::imshow(kWindow2d, displayImage);
        }

        const int key = cv::waitKey(10);
        if (key == 27 || key == 'q' || key == 'Q') {
            running = false;
        }
    }

    cv::destroyAllWindows();
    return 0;
}
