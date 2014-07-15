#include "solver.hpp"
#include <iostream>
#include <vector>
#include <map>
#include <set>

using namespace std;
using namespace cv;

class CubicSolver::CubicSolverImpl
{
public:
    CubicSolverImpl()
    {
        dimension = 3;
        stateDim = dimension * dimension * dimension;
        initRotMats();
        createCubeToPlanesMap();
    }

    void initRotMats()
    {
        rotMats.resize(3);
        for (int v = 0; v < 3; v++)
        {
            rotMats[v].resize(dimension);
            for (int d = 0; d <dimension; d++)
            {
                Mat plane = getPlane(v, d);

                std::cout << plane << "\n";

                rotMats[v][d] = getRotation(plane);
            }
        }
    }

    float getWeight(Mat stateCubes)
    {
        float sum = 0;

        for (int i = 0; i < stateCubes.rows; i++)
        {
            if (stateCubes.at<int>(i) != i)
            {
                sum += 1;
            }
        }

        return sum;
    }

    Mat Solve(Mat stateCubes)
    {
        int maxDepth = 5;
        int depth = 0;

        //For each depth: start rotation on the current depth level
        vector<Mat > rotations(maxDepth);

        //Index of the next move to try on given depth level
        vector<int > nextMove(maxDepth, 0);

        //Best rotation and weight at the moment
        pair<Mat, float> best(Mat::eye(stateDim, stateDim, CV_32SC1), getWeight(stateCubes));
        
        //On the first depth we start from the eye rotation matrix (no rotation)
        rotations[depth] = Mat::eye(stateDim, stateDim, CV_32SC1);

        while (true)
        {  
            Mat currentRotation;

            //if next move to try on the current level is possible (below max)
            if (nextMove[depth] < rotMatsSeq.size())
            {
                //apply rotation
                currentRotation = rotMatsSeq[nextMove[depth]] * rotations[depth];

                //when we ar back on this level we will try next move
                nextMove[depth]++;
            }
            else //all moves on this depth level are checked, we need to decrease the depth
            {
                //If nothing to decrease - all combinations are done
                if (depth == 0) break;

                //Otherwise decrease the level;
                depth--;
                continue;
            }
             
            //check weight and update if better than the current best
            Mat newState = currentRotation * stateCubes;
            float weight = getWeight(newState);

            if (weight < best.second)
            {
                best.first = currentRotation;
                best.second = weight;
            }

            //if depth is not the max increase depth
            if (depth < maxDepth - 1)
            {            
                //set next depth move to 0 and increase the depth 
                nextMove[depth + 1] = 0;

                //Next level starts from the current rotation
                rotations[depth + 1] = currentRotation;
                
                depth++;
                continue;
            }
            else //(depth is max)
            {
                // try next move on the current level
                continue;
            }
        }

        return best.first;
    }

    Mat getStateByPlanes(Mat yellowPlane, Mat bluePlane, Mat redPlane, Mat whitePlane, Mat greenPlane, Mat orangePlane)
    {
        Mat state(stateDim, 1, CV_32SC1);
        Mat planes[] = {yellowPlane, bluePlane, redPlane, whitePlane, greenPlane, orangePlane};

        for (int i = 0; i < dimension; i++)
        {
            for (int j = 0; j < dimension; j++)
            {
                for (int k = 0; k < dimension; k++)
                {
                    int posIndex = getCubeIndex(i, j, k);

                    int indexes[3] = {i, j, k};
                    vector<int> cube(indexes, indexes + 3);

                    set<Color> planeColors = cubeToPlanes[cube];
                    set<Color> actualColors;

                    for_each(planeColors.begin(), planeColors.end(), 
                    [&, this](Color c)
                    {
                        int axis = getAxisByColor(c);
                        cv::Point point = this->getPlanePoint(axis, i, j, k);
                        actualColors.insert((Color)planes[c].at<int>(point));
                    });

                    int cubeIndex = getCubeIndex(planesToCube[actualColors]);
                    state.at<int>(posIndex) = cubeIndex;
                }
            }
        }

        //Check resulting state has all cubes

        //Check there are no dublicates of the cubes in the state matrix

        return state;
    }

    /*!
     * /brief Get indexes of rubic plane
     */
    Mat getPlane(int fixedDim, int planeIndex)
    {
        Mat plane(dimension, dimension, CV_32SC1);

        for (int i = 0; i < dimension; i++)
        {
            for (int j = 0; j < dimension; j++)
            {
                switch (fixedDim)
                {
                case 0:
                    plane.at<int>(i,j) = getCubeIndex(planeIndex, i, j);
                    break;
                case 1:
                    plane.at<int>(i,j) = getCubeIndex(i, planeIndex, j);
                    break;
                case 2:
                    plane.at<int>(i,j) = getCubeIndex(i, j, planeIndex);
                    break;
                }
            }
        }

        return plane;
    }

    Mat getRotation(Mat plane)
    {
        assert(plane.rows == dimension && plane.cols == dimension);

        Mat rotation = Mat::eye(stateDim, stateDim, CV_32SC1);
        Mat planeBorder(dimension * 2 + (dimension - 2) * 2, 1, CV_32SC1);

        int i;
        for (i = 0; i < dimension; i++)
        {
            planeBorder.at<int>(i) = plane.at<int>(0,i);
            planeBorder.at<int>(i + dimension + dimension - 2) = plane.at<int>(dimension - 1, dimension - i - 1);

            if (i !=0 && i != dimension -1)
            {
                planeBorder.at<int>(i + dimension - 1) = plane.at<int>(i, dimension -1);
                planeBorder.at<int>(i + dimension + dimension - 2 + dimension - 1) = plane.at<int>(dimension - i - 1, 0);
            }
        }

        for (i = 0; i < planeBorder.rows; i++)
        {
            int to = planeBorder.at<int>(i);
            int from = planeBorder.at<int>((i - (dimension - 1) + planeBorder.rows) % planeBorder.rows);
            rotation.at<int>(to, from) = 1;
        }
                
        return rotation;
    }

    inline int getCubeIndex(int d1, int d2, int d3)
    {
        return d3 + d2 * dimension + d1 * dimension * dimension;
    }
    inline int getCubeIndex(vector<int> indexes)
    {
        return getCubeIndex(indexes[0], indexes[1], indexes[2]);
    }

    //For each vector, plane index: rotation matrix
    vector<vector<Mat > > rotMats;
    vector<Mat > rotMatsSeq;

    //3
    int dimension;

    //dimension ^3
    int stateDim;

    inline CubicSolver::Color getPlaneByAxis(int axis, int index)
    {
        vector<Color> colors(dimension, NO_COLOR);

        switch (axis)
        {
        case 0:
            colors[0] = YELLOW;
            colors[2] = WHITE;
            break;
        case 1:
            colors[0] = BLUE;
            colors[2] = GREEN;
            break;
        case 2:
            colors[0] = RED;
            colors[2] = ORANGE;
            break;
        }

        return colors[index];
    }

    inline int getAxisByColor(CubicSolver::Color color)
    {
        switch (color)
        {
        case YELLOW:
            return 0;
            break;
        case BLUE:
            return 1;
            break;
        case RED:
            return 2;
            break;
        case WHITE:
            return 0;
            break;
        case GREEN:
            return 1;
            break;
        case ORANGE:
            return 2;
            break;
        }
    }

    cv::Point getPlanePoint(int axis, int i, int j, int k)
    {
        switch (axis)
        {
        case 0:
            return Point(j,k);
            break;
        case 1 :
            return Point(i,k);
            break;
        case 2:
            return Point(i,j);
            break;
        }
    }

    //From index of cube to set of colors
    map<vector<int>, set<CubicSolver::Color> > cubeToPlanes;
    map<set<CubicSolver::Color>, vector<int> > planesToCube;

    void createCubeToPlanesMap();
};

void CubicSolver::CubicSolverImpl::createCubeToPlanesMap()
{
    for (int i = 0; i < dimension; i++)
    {
        for (int j = 0; j < dimension; j++)
        {
            for (int k = 0; k < dimension; k++)
            {
                //Find corresponding color planes
                int indexes[3] = {i,j,k};
                vector<int> cube(indexes, indexes+3);

                set<CubicSolver::Color> colors;
                for (int ci = 0; ci < 3; ci++)
                {
                    CubicSolver::Color c = getPlaneByAxis(ci, cube[ci]);
                    if (c != NO_COLOR)
                    {
                        colors.insert(c);
                    }
                }

                cubeToPlanes[cube] = colors;
                planesToCube[colors] = cube;
            }
        }
    }
}

CubicSolver::CubicSolver()
{
    impl = new CubicSolverImpl;
}

Mat CubicSolver::Solve(Mat state)
{
    return this->impl->Solve(state);
}

float CubicSolver::getWeight(cv::Mat stateCubes)
{
    return this->impl->getWeight(stateCubes);
}

Mat CubicSolver::getStateByPlanes(Mat yellowPlane, Mat bluePlane, Mat redPlane, Mat whitePlane, Mat greenPlane, Mat orangePlane)
{
    return this->impl->getStateByPlanes(yellowPlane, bluePlane, redPlane, whitePlane, greenPlane, orangePlane);
}

Mat CubicSolver::askUserCubicState()
{

}