#ifndef LOGGER_TFG_ADRIA
#define LOGGER_TFG_ADRIA

#include <map>
#include <vector>
#include "gnuplot_i.h"
#include <mutex>
#include <chrono>
#include <future>

class ODrive;

#include "src/ODrive.h"

class Graph_Logger
{
private:
        
    Gnuplot *g1;
    
    std::map<std::string, std::vector<double>> m;
    std::vector<Gnuplot*> plots;
    std::mutex mutex;

public:
    double dt;
    Graph_Logger();
    Graph_Logger(double dt);
    ~Graph_Logger();


    void appendToBuffer(std::string bufferName,double value);
    std::vector<double> getBuffer(std::string bufferName);

    void allocMemory( std::vector<std::string> datasets,long aditional_nodes);
    void drawMaxMinLocations(std::vector<double>maximums,std::vector<double>maximums_i,std::vector<double>minimums,std::vector<double>minimums_i);
    void appendToFile(std::string fileName, double valueToAppend);
    
    void plot(std::string plotTitle, std::vector<std::string> datasets, std::string xlabel, std::string ylabel,bool scaleToSmallest, bool saveToFile);
    void plot_normalized(std::string plotTitle, std::vector<std::string> datasets);
    
    void plotSubtraction(std::string plotTitle, std::vector<std::string> datasets, std::string xlabel, std::string ylabel, bool saveToFile);
    void adquisitionThread(ODrive *odrive, std::future<void> futureObj);
    void appendVectorToBuffer(std::string bufferName,std::vector<double> vals);

};

#endif