#ifndef PROCESSCSV_H
#define PROCESSCSV_H

#include <string>
#include "grid.hpp"

class ProcessCSV{
    public:
        void readCSV(const std::string& filename, OccupancyGrid& grid);
};

#endif