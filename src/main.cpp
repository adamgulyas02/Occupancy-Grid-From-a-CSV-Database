//Includes
#include "grid.hpp"
#include "sensor.h"
#include <iostream>
#include "processCSV.hpp"

//Main function
int main() {
    ProcessCSV processor; //Create an instance of the ProcessCSV class
    OccupancyGrid grid; //Create an instance of the OccupancyGrid class
    std::string filename = "./inputAndOutputTable/robot.csv"; //Name of the input file
    processor.readCSV(filename, grid); //Read the CSV file and update the grid
    grid.saveToCSV("./inputAndOutputTable/output.csv"); //Save the grid to a CSV file
}