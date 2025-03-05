//Includes
#include <iostream>
#include <fstream>
#include <sstream>
#include "processCSV.hpp"

void ProcessCSV::readCSV(const std::string& filename, OccupancyGrid& grid){ //Function to read the CSV file and update the grid
    std::cout << "Opening file: " << filename << std::endl; //Print the name of the file
    std::ifstream dataStream(filename); //Open the file
    if (!dataStream.is_open()) { //Check if the file is open
        throw std::runtime_error("Could not open file " + filename);
    }
    std::string line; //Variable to store each line of the file
    getline (dataStream, line); //Skip the header
    while (getline(dataStream, line)) //Read each line of the file
    {
        std::stringstream lineStream(line); //Create a stringstream from the line
        std::string cell; //Variable to store each cell of the line
        std::vector<double> dataSet; //Vector to store the data from the line
        std::cout << "Parsing line: " << line << std::endl; //Print the current line
        while (getline(lineStream, cell, ',')) //Parse each cell of the line
        {
            dataSet.push_back(std::stod(cell)); //Convert the cell to a double and add it to the vector
        }
        
        if (dataSet.size() < 8) { //Check if the data is valid
            throw std::runtime_error("Invalid data in file " + filename);
        }
        double timestamp = dataSet[0]; //Get the timestamp
        double robotX = dataSet[1]; //Get the x position of the robot
        double robotY = dataSet[2]; //Get the y position of the robot
        double robotTheta = dataSet[3]; //Get the heading of the robot
        double sensorData[4] = {dataSet[4], dataSet[5], dataSet[6], dataSet[7]}; //Get the data from the ultrasound sensors

        grid.updateGrid(robotX, robotY, robotTheta, sensorData); //Update the grid with the new data
    }

 

    
}