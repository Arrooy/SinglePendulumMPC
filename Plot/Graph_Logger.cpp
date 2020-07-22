#include "Graph_Logger.h"

Graph_Logger::Graph_Logger(double dt)
{
    this->dt = dt;
    this->g1 = new Gnuplot(std::string("lines"));
}

Graph_Logger::~Graph_Logger(){
    if(plots.size() != 0)
    for(auto const& value : plots){
        delete value;
    }
}

void Graph_Logger::adquisitionThread(ODrive *odrive, std::future<void> futureObj)
{

    std::cout << "Doing odrive readings"<< std::endl;
    while (futureObj.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout)
    {
        appendToBuffer("ODrive real position", odrive->m0->castCPRToRad(odrive->m0->getPosEstimate()));
        appendToBuffer("ODrive real velocity", odrive->m0->castCPRToRads(odrive->m0->getVelEstimate()));
        appendToBuffer("ODrive real current", odrive->m0->getCurrent());
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}


void Graph_Logger::appendVectorToBuffer(std::string bufferName,std::vector<double> vals)
{
    auto it = this->m.find(bufferName);
    if(it != this->m.end()){
        //Vector exists.
        it->second.insert(it->second.end(), vals.begin(), vals.end());
    }else{
        //Must create the vector.
        this->m.insert(std::pair<std::string, std::vector<double>>(bufferName,vals));
    }
}
void Graph_Logger::appendToBuffer(std::string bufferName,double value){
    auto it = this->m.find(bufferName);
    if(it != this->m.end()){
        //Vector exists.
        it->second.push_back(value);
    }else{
        //Must create the vector.
        std::vector<double> newVector(1,value);
        this->m.insert(std::pair<std::string, std::vector<double>>(bufferName,newVector));
    }
}

void Graph_Logger::plot(std::string plotTitle, std::vector<std::string> datasets, std::string xlabel, std::string ylabel, bool saveToFile)
{

    Gnuplot *g1 = new Gnuplot(std::string("lines"));

    g1->reset_all();
    if(saveToFile)
        g1->savetops("Plots/" + plotTitle);
    g1->set_title(plotTitle);
    g1->set_grid();
    g1->set_xlabel(xlabel);
    g1->set_ylabel(ylabel);

    long min = 0;
    bool isFirstIteration = true;

    for(auto &dataset : datasets){
        long size = m[dataset].size();
        if(isFirstIteration){
            min = size;
            isFirstIteration = false;
        }else if (size <= min){
            min = size;
        }
    }

    for(auto &dataset : datasets){
        std::vector<double> values = m[dataset];
        std::vector<double> x;
        long size = m[dataset].size();

        for(auto i = 0; i < size; ++i){
            x.push_back(min * (i + i * dt) / size );
        }
        g1->set_style("linespoints").plot_xy(x,values,dataset);
    }
    g1->showonscreen();
    plots.push_back(g1);
}   

void Graph_Logger::plotSubtraction(std::string plotTitle, std::vector<std::string> datasets, std::string xlabel, std::string ylabel, bool saveToFile)
{

}



/*
void Graph_Logger::appendToFile(std::string fileName, float valueToAppend){
    using namespace std;
    ofstream myfile (fileName,std::ios::out | std::ios::app);
    if (myfile.is_open())
    {
        myfile << "This is a line.\n";
        myfile << "This is another line.\n";
        myfile.close();
    }
    else cout << "Unable to open file";
}*/
