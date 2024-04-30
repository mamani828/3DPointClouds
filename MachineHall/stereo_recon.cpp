#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;

int main(int argc, char* argv[])
{
    if (argc != 3)
    {
        std::cerr << "Usage: " << argv[0] << " <left_img> <right_img>" << std::endl;
        return 1;
    }

    // Load the images
    Mat left_img = imread(argv[1], IMREAD_COLOR);
    Mat right_img = imread(argv[2], IMREAD_COLOR);

    if (left_img.empty() || right_img.empty())
    {
        std::cerr << "Error loading images." << std::endl;
        return 1;
    }

    // Display the images in separate windows
    imshow("Left Image", left_img);
    imshow("Right Image", right_img);

    // Wait for a key press before closing the windows
    waitKey(0);

    return 0;
}

