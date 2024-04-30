#include <opencv2/opencv.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include <tuple>
#include <string>
#include <iostream>

struct Point3D {
    float x, y, z;
    uchar red, green, blue;
};

std::vector<Point3D> loadPlyFile(const std::string& filePath) {
    std::ifstream plyFile(filePath);
    if (!plyFile.is_open()) {
        std::cerr << "Error opening PLY file: " << filePath << std::endl;
        return {};
    }

    std::string line;
    std::vector<Point3D> points;

    // Skip header lines
    while (std::getline(plyFile, line) && line != "end_header") {}

    // Parse the remaining lines
    while (std::getline(plyFile, line)) {
        std::istringstream iss(line);
        Point3D point;
        int r, g, b;
        iss >> point.x >> point.y >> point.z >> r >> g >> b;
        point.red = static_cast<uchar>(r);
        point.green = static_cast<uchar>(g);
        point.blue = static_cast<uchar>(b);
        points.push_back(point);
    }

    return points;
}

cv::Mat projectPointsToImage(const std::vector<Point3D>& points, const cv::Mat& intrinsicMatrix, const cv::Size& imgSize) {
    cv::Mat img(imgSize, CV_8UC3, cv::Scalar(0, 0, 0));

    for (const auto& point : points) {
        // 3D point in homogeneous form
        cv::Mat point3D = (cv::Mat_<double>(3, 1) << point.x, point.y, point.z);

        // Projected 2D point
        cv::Mat uv = intrinsicMatrix * point3D;
        int u = static_cast<int>(uv.at<double>(0, 0) / uv.at<double>(2, 0));
        int v = static_cast<int>(uv.at<double>(1, 0) / uv.at<double>(2, 0));

        if (u >= 0 && u < imgSize.width && v >= 0 && v < imgSize.height) {
            img.at<cv::Vec3b>(v, u) = cv::Vec3b(point.blue, point.green, point.red);
        }
    }

    return img;
}

int main() {
    std::string plyFilePath = "point_cloud_rgb.ply";
    std::vector<Point3D> points = loadPlyFile(plyFilePath);

    double focalLengthU = 458.654; // Replace with actual intrinsic data
    double focalLengthV = 457.296;
    double cu = 367.215;
    double cv = 248.375;

    cv::Mat intrinsicMatrix = (cv::Mat_<double>(3, 3) << focalLengthU, 0, cu, 0, focalLengthV, cv, 0, 0, 1);

    cv::Size imgSize(752, 480); // Image size to render

    cv::Mat img = projectPointsToImage(points, intrinsicMatrix, imgSize);

    cv::imshow("Projected Point Cloud Image", img);
    cv::waitKey(0);
    cv::imwrite("projected_image.png", img);

    return 0;
}

