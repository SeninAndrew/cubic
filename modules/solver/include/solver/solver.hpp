#include "opencv2/opencv.hpp"

class CubicSolver
{
public:
    enum RotationVector
    {
        YELLOW_VECTOR, 
        BLUE_VECTOR, 
        RED_VECTOR
    };

    enum Color
    {
        YELLOW, 
        BLUE, 
        RED, 
        WHITE, 
        GREEN, 
        ORANGE, 
        NO_COLOR
    };

    CubicSolver();

    //Output:
    //
    cv::Mat Solve(cv::Mat stateCubes);

    cv::Mat getStateByPlanes(cv::Mat yellowPlane, cv::Mat bluePlane, cv::Mat redPlane, cv::Mat whitePlane, cv::Mat greenPlane, cv::Mat orangePlane);

    float getWeight(cv::Mat stateCubes);

private:
    class CubicSolverImpl;

    CubicSolverImpl *impl;
};
