User
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <fstream>

// Constants for block sizes
const int BLOCK_SIZE = 10;
const int SEARCH_BLOCK_SIZE = 1;
void savePointCloudAsPLY(const std::vector<std::tuple<float, float, float, uchar, uchar, uchar>>& pointCloud, const std::string& filePath);
void displayImage(const cv::Mat& img, const std::string& windowName) {
    cv::imshow(windowName, img);
    cv::waitKey(0);
    cv::destroyWindow(windowName);
}

double sumOfAbsDiff(const cv::Mat& block1, const cv::Mat& block2) {
    if (block1.size() != block2.size()) return -1.0;

    return cv::sum(cv::abs(block1 - block2))[0];
}

std::pair<int, int> compareBlocks(int y, int x, const cv::Mat& blockLeft, const cv::Mat& rightImage, int blockSize = BLOCK_SIZE) {
    int xMin = std::max(0, x - SEARCH_BLOCK_SIZE);
    int xMax = std::min(rightImage.cols - blockSize, x + SEARCH_BLOCK_SIZE);  

    bool first = true;
    double minSad = 0.0;
    std::pair<int, int> minIndex;

    for (int i = xMin; i <= xMax; ++i) { 
        if (i + blockSize > rightImage.cols || y + blockSize > rightImage.rows) continue; // Skip out-of-bounds

        cv::Mat blockRight = rightImage(cv::Rect(i, y, blockSize, blockSize)); 
        double sad = sumOfAbsDiff(blockLeft, blockRight);

        if (first || sad < minSad) {
            minSad = sad;
            minIndex = {y, i};
            first = false;
        }
    }

    return minIndex;
}
void getPointClouds(const cv::Mat& disparityMap, const cv::Mat& rgbImage, double focalLengthU, double focalLengthV, double cu, double cv, double baseline) {
    std::vector<std::tuple<float, float, float, uchar, uchar, uchar>> pointCloud;
    std::cout << "Creating PointClouds with RGB colors" << std::endl;

    for (int y = 0; y < disparityMap.rows; ++y) {
        for (int x = 0; x < disparityMap.cols; ++x) {
            double disparity = disparityMap.at<double>(y, x);

            if (disparity > 0) {  // Valid disparity
                double Z = focalLengthU  / (baseline*disparity);  // Calculate depth
                double X = Z * (x - cu) / focalLengthU;  // 3D X-coordinate
                double Y = Z * (y - cv) / focalLengthV;  // 3D Y-coordinate

                if (x < rgbImage.cols && y < rgbImage.rows) {
                    cv::Vec3b color = rgbImage.at<cv::Vec3b>(y, x); // RGB color
                    pointCloud.emplace_back(X, Y, Z, color[2], color[1], color[0]);
                }
            }
        }
    }

    savePointCloudAsPLY(pointCloud, "point_cloud_rgb.ply");
}

void savePointCloudAsPLY(const std::vector<std::tuple<float, float, float, uchar, uchar, uchar>>& pointCloud, const std::string& filePath) {
    std::ofstream plyFile(filePath);
    plyFile << "ply\nformat ascii 1.0\n";
    plyFile << "element vertex " << pointCloud.size() << "\n";
    plyFile << "property float x\nproperty float y\nproperty float z\n";
    plyFile << "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";

    for (const auto& [X, Y, Z, R, G, B] : pointCloud) {
        plyFile << X << " " << Y << " " << Z << " " << (int)R << " " << (int)G << " " << (int)B << "\n";
    }
    plyFile.close();
    std::cout << "3D Point Cloud with RGB saved as '" << filePath << "'.\n";
}

void getDisparityMapAndPointCloud() {
    cv::Mat leftImage = cv::imread("cam0/img1.png", cv::IMREAD_GRAYSCALE);
    cv::Mat rightImage = cv::imread("cam1/img1.png", cv::IMREAD_GRAYSCALE);
    cv::Mat rgbImage = cv::imread("cam0/img1.png"); // For color rendering

    if (leftImage.empty() || rightImage.empty() || leftImage.size() != rightImage.size()) {
        std::cerr << "Error loading images or size mismatch." << std::endl;
        return;
    }

    int h = leftImage.rows, w = leftImage.cols;
    cv::Mat disparityMap(h, w, CV_64F, cv::Scalar(0));

    for (int y = BLOCK_SIZE; y < h - BLOCK_SIZE; ++y) {
        for (int x = BLOCK_SIZE; x < w - BLOCK_SIZE; ++x) {
            cv::Mat blockLeft = leftImage(cv::Rect(x, y, BLOCK_SIZE, BLOCK_SIZE));
            auto minIndex = compareBlocks(y, x, blockLeft, rightImage, BLOCK_SIZE);
            disparityMap.at<double>(y, x) = std::abs(minIndex.second - x);
        }
    }

    cv::Mat disparityMapDisplay;
    disparityMap.convertTo(disparityMapDisplay, CV_8U, 255.0 / (2 * SEARCH_BLOCK_SIZE));
    cv::applyColorMap(disparityMapDisplay, disparityMapDisplay, cv::COLORMAP_HOT);

    cv::imshow("Disparity Map", disparityMapDisplay);
    cv::imwrite("depth_image.png", disparityMapDisplay);

    double focalLengthU = 458.654;
    double focalLengthV = 457.296;
    double cu = 367.215;
    double cv = 248.375;
    double baseline = 0.11;

    getPointClouds(disparityMap, rgbImage, focalLengthU, focalLengthV, cu, cv, baseline);
}

int main() {
    auto t1 = std::chrono::high_resolution_clock::now();
    getDisparityMapAndPointCloud();
    auto t2 = std::chrono::high_resolution_clock::now();

    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << "ms\n";
    return 0;
}
