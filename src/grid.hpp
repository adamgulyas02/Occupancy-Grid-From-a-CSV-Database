#ifndef GRID_H
#define GRID_H

//Includes
#include <vector>
#include <iostream>
#include <cmath>
#include <string>
#include <array>
#include "robot.h"
#include "robotpos.h"
#include "sensor.h"

//Class for the occupancy grid
class OccupancyGrid {
    private:
        std::vector<std::vector<double>> occupancyGrid; //2D vector to store the data for the occupancy grid        
        std::vector<std::vector<bool>> cantBeAWall;
        std::vector<std::vector<bool>> isItPath;

    public:
        static constexpr double gridWidth=7.0; //Width of the grid
        static constexpr double gridHeight=7.0; //Height of the grid
        static constexpr double gridResolution=0.02; //Resolution of the grid
        OccupancyGrid(); //Constructor
        Position cellCenterCalculation(int row, int column); //Function to calculate the center of a cell
        std::array<std::array<double, 4>, 2>calculateDistancesAndAngles(double robotX, double robotY, double robotHeading, Position cellCenter); //Function to calculate the distances and angles to all sensors
        void updateGrid(double robotX, double robotY, double robotTheta, const double* sensorData); //Function to update the grid
        void saveToCSV(const std::string& filename) const; //Function to save the grid to a CSV file

};

#endif