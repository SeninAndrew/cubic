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
        ORANGE
    };

    CubicSolver();

    //Output:
    //
    cv::Mat Solve(cv::Mat stateCubes);

    static cv::Mat GetStateByPlanes(cv::Mat yellowPlane, cv::Mat bluePlane, cv::Mat redPlane, cv::Mat whitePlane, cv::Mat greenPlane, cv::Mat orangePlane);

    float getWeight(cv::Mat stateCubes);

private:
    class CubicSolverImpl;

    CubicSolverImpl *impl;
};
