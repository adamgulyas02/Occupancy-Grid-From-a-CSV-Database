//Includes
#include "grid.hpp"
#include "sensor.h"
#include <iostream>
#include <fstream>
#include <cmath>

//Determind the number of rows and columns in the grid
int rows=OccupancyGrid::gridHeight/OccupancyGrid::gridResolution;
int columns=OccupancyGrid::gridWidth/OccupancyGrid::gridResolution;

OccupancyGrid::OccupancyGrid() 
    : occupancyGrid(rows, std::vector<double>(columns, 2.0)) , //Initialize the grid with zeros
    cantBeAWall(rows, std::vector<bool>(columns, false)),
    isItPath(rows, std::vector<bool>(columns,false)) {}

//Function to calculate the center of a cell
Position OccupancyGrid::cellCenterCalculation(int row, int column) 
{
    double cellX=(column*gridResolution+gridResolution/2.0-gridWidth/8.0);
    double cellY=(row*gridResolution+gridResolution/2.0-gridHeight/8.0);
    return Position{cellX, cellY};
}

//Function to calculate the distances and angles to all sensors
std::array<std::array<double, 4>, 2> OccupancyGrid::calculateDistancesAndAngles(double robotX, double robotY, double robotHeading, Position cellCenter)
{
    std::array<std::array<double, 4>, 2> DistandAngles{}; //first row is distance, second row is angle
    double cellX=cellCenter.x; //Get the x coordinate of the cell
    double cellY=cellCenter.y; //Get the y coordinate of the cell

    double Sensor0_PosX=robotX+Robot::robotLength/2.0*cos(robotHeading)+Robot::robotWidth/2.0*(-sin(robotHeading)); //Calculate the x position of sensor 0
    double Sensor0_PosY=robotY+Robot::robotLength/2.0*sin(robotHeading)+Robot::robotWidth/2.0*(cos(robotHeading)); //Calculate the y position of sensor 0
    double Sensor1_PosX=robotX-Robot::robotLength/2.0*cos(robotHeading)+Robot::robotWidth/2.0*(-sin(robotHeading)); //Calculate the x position of sensor 1
    double Sensor1_PosY=robotY-Robot::robotLength/2.0*sin(robotHeading)+Robot::robotWidth/2.0*(cos(robotHeading)); //Calculate the y position of sensor 1
    double Sensor2_PosX=robotX-Robot::robotLength/2.0*cos(robotHeading)-Robot::robotWidth/2.0*(-sin(robotHeading)); //Calculate the x position of sensor 2
    double Sensor2_PosY=robotY-Robot::robotLength/2.0*sin(robotHeading)-Robot::robotWidth/2.0*(cos(robotHeading)); //Calculate the y position of sensor 2
    double Sensor3_PosX=robotX+Robot::robotLength/2.0*cos(robotHeading)-Robot::robotWidth/2.0*(-sin(robotHeading)); //Calculate the x position of sensor 3
    double Sensor3_PosY=robotY+Robot::robotLength/2.0*sin(robotHeading)-Robot::robotWidth/2.0*(cos(robotHeading)); //Calculate the y position of sensor 3

    DistandAngles[0][0]=sqrt(pow(Sensor0_PosX-cellX, 2)+pow(Sensor0_PosY-cellY, 2)); //Calculate the distance from sensor 0's position to the cell
    DistandAngles[0][1]=sqrt(pow(Sensor1_PosX-cellX, 2)+pow(Sensor1_PosY-cellY, 2)); //Calculate the distance from sensor 1's position to the cell
    DistandAngles[0][2]=sqrt(pow(Sensor2_PosX-cellX, 2)+pow(Sensor2_PosY-cellY, 2)); //Calculate the distance from sensor 2's position to the cell
    DistandAngles[0][3]=sqrt(pow(Sensor3_PosX-cellX, 2)+pow(Sensor3_PosY-cellY, 2)); //Calculate the distance from sensor 3's position to the cell

    DistandAngles[1][0] = atan2(cellY - Sensor0_PosY, cellX - Sensor0_PosX); //- robotHeading; //Calculate the angle from sensor 0's position to the cell
    DistandAngles[1][1] = atan2(cellY - Sensor1_PosY, cellX - Sensor1_PosX); //- robotHeading; //Calculate the angle from sensor 1's position to the cell
    DistandAngles[1][2] = atan2(cellY - Sensor2_PosY, cellX - Sensor2_PosX); //- robotHeading; //Calculate the angle from sensor 2's position to the cell
    DistandAngles[1][3] = atan2(cellY - Sensor3_PosY, cellX - Sensor3_PosX); // - robotHeading; //Calculate the angle from sensor 3's position to the cell

    return DistandAngles; //Return the distances and angles
}

void OccupancyGrid::updateGrid(double robotX, double robotY, double robotTheta, const double* sensorData) {
    // Convert robot coordinates to match the centered occupancy grid
    double transformedRobotX = robotX;
    double transformedRobotY = robotY;

    // Convert sensor data from time of flight to distance in meters
    double sensorDistances[4];
    for (int i = 0; i < 4; i++) {
        sensorDistances[i] = sensorData[i] * 343.0 / 2; // Round-trip time conversion
    }

    // Define the probability threshold for confirming occupancy
    const double probabilityThreshold = -3.0; // Adjust as needed

    // Iterate through all cells in the grid 
    for (int row = 0; row < gridWidth / gridResolution; row++) {
        for (int column = 0; column < gridHeight / gridResolution; column++) {
            // Calculate the center position of the current grid cell
            Position cellCenter = cellCenterCalculation(row, column);

            // Compute distances and angles from all sensors to the cell
            std::array<std::array<double, 4>, 2> distancesAndAngles = calculateDistancesAndAngles(transformedRobotX, transformedRobotY, robotTheta, cellCenter);

            // Check each sensor
            for (int k = 0; k < 4; k++) {
                double distanceToCell = distancesAndAngles[0][k]; // Distance from sensor k
                double angleToCell = distancesAndAngles[1][k];    // Angle from sensor k
                // Compute sensor's global orientation
                double sensorAngle = robotTheta + Sensor::sensorAngleOffset + k * M_PI / 2; 

                // Compute relative angle between sensor direction and cell
                double relativeAngle = angleToCell - sensorAngle;

                // Normalize angle to range [-π, π]
                while (relativeAngle >= M_PI) relativeAngle -= 2 * M_PI; 
                while (relativeAngle < -M_PI) relativeAngle += 2 * M_PI;

                // Check if the cell is within the sensor's FOV and in front of the sensor
                if (std::abs(relativeAngle) <= (Sensor::sensorConeAngle / 2) /*&&
                    isPointInFOV(distancesAndAngles[2][k], distancesAndAngles[3][k], sensorAngle, cellCenter.x, cellCenter.y) */ && 
                    std::abs(distanceToCell - sensorDistances[k]) <= gridResolution) {
                    
                    // Probability function based on relative angle
                    double angleFactor = 1.0 - (std::abs(relativeAngle) / (Sensor::sensorConeAngle / 2));

                    // Accumulate probabilities in the grid cell
                    occupancyGrid[row][column] -= angleFactor;
                }
                if(std::abs(relativeAngle) <= (Sensor::sensorConeAngle / 2) /*&&
                isPointInFOV(distancesAndAngles[2][k], distancesAndAngles[3][k], sensorAngle, cellCenter.x, cellCenter.y) */ &&
                (distanceToCell<(sensorDistances[k]-gridResolution*3)) || distanceToCell<Robot::robotWidth)
                {
                    cantBeAWall[row][column]=true;
                }
                if(std::abs(transformedRobotX-cellCenter.x)<gridResolution && std::abs(transformedRobotY-cellCenter.y)<gridResolution)
                {
                    isItPath[row][column]=true;
                }
            }
        }
    }
    //Normalize values for each cell. 
    for (int row = 0; row < gridWidth / gridResolution; row++) {
        for (int column = 0; column < gridHeight / gridResolution; column++) {
            if(cantBeAWall[row][column]==false){
                if (occupancyGrid[row][column] <= probabilityThreshold) {
                    occupancyGrid[row][column] = 0.0;
                } 
                else{
                    occupancyGrid[row][column] = 2.0;
                }
            }
            else occupancyGrid[row][column] = 0.0;
            if(isItPath[row][column]==true) occupancyGrid[row][column] = 1.0;
        }
    }
}

//Function to save the grid to a CSV file
void OccupancyGrid::saveToCSV(const std::string& filename) const {
    std::ofstream outFile(filename);
    if (!outFile.is_open()) {
        throw std::runtime_error("Failed to open file for writing: " + filename);
    }
    for (const auto &row : occupancyGrid) {
        for (const auto &cell : row) {
            outFile << cell << ',';
        }
        outFile << '\n';
    }
    outFile.close();
}