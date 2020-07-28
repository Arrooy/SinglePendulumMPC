#include "Controller.h"
#include <csignal>


bool Controller::signalFlag  = false;

void wait_for_key ();
void recordFreeFall();

int main(int argc, char ** argv) {
    recordFreeFall();
    /*
    Controller c(
        std::string("/home/adria/TFG/SinglePendulumMPC/SinglePendulum_cpp/single_pendulum_description/urdf/single_pendulum.urdf"),//Model path
        std::string("/home/adria/TFG/SinglePendulumMPC/config.yaml")); // Configuration path

    std::signal(SIGINT, c.signalHandler);
    
    c.connectODrive();
    

    // Create a trajectory with 300 nodes.
    c.createDOCP(true);
    
    c.createTrajectory();
    c.initGraphs();
   
    // Refactor the class to have only 100 nodes for the MPC.
    c.createDOCP(false);

    c.controlLoop();
    
    c.stopMotors();
    c.stopGraphs();
    c.showGraphs();

    wait_for_key();*/
    return 0;
}

void recordFreeFall() {
    Controller c(
        std::string("/home/adria/TFG/SinglePendulumMPC/SinglePendulum_cpp/single_pendulum_description/urdf/single_pendulum.urdf"),//Model path
        std::string("/home/adria/TFG/SinglePendulumMPC/config.yaml")); // Configuration path

    //std::signal(SIGINT, c.signalHandler);
    
    c.connectODrive();
    c.odrive->m0->disable();
    
    int rec_time = 3;               //In seconds
    int adquisition_period = 100;   //In us
    long loopIterations = (long)((rec_time * 1.0e6) / (double)adquisition_period);

    Graph_Logger * graph_logger = new Graph_Logger(1e-4);
    
    long i = 0;
    while(i++ < loopIterations){
        graph_logger->appendToBuffer("testPos",c.odrive->m0->getPosEstimateInRad());
       // graph_logger->appendToBuffer("testVel",c.odrive->m0->getVelEstimateInRads());
        
        if(i % (loopIterations / 10) == 0)
            std::cout << "Iteration: " << i << std::endl;
        usleep(100);
    }
    std::cout << "Data adquisition done! Calculating dt..." << std::endl;

    auto buffer = graph_logger->getBuffer("testPos");
    long buffer_size = buffer.size();
    double period = -1;
    
    std::vector<double> maximums;
    std::vector<double> minimums;

    std::vector<double> maximums_i;
    std::vector<double> minimums_i;
    //int distanceToRefresh = 150;
    
    double startingPoint = buffer[0];
    double max = startingPoint;
    double min = startingPoint;
    double val = 0;
    bool lookingForMin = true;
    
    //Method 1
    for(long i  = 0; i < buffer_size; i++){
        val = buffer[i];
        if(lookingForMin){
            if(val <= min){
                min = val;
            }else if(val - 0.01 > min){
                if(std::find(minimums.begin(), minimums.end(), val) == minimums.end()){
                    minimums.push_back(min);
                    minimums_i.push_back(i);
                }
                max = min;
                min = startingPoint;
                lookingForMin = false;
            }
        }else{
            if(val >= max){
                max = val;
            }else if(val + 0.01 <= max){
                if(std::find(maximums.begin(), maximums.end(), val) == maximums.end()){
                    maximums.push_back(max);
                    maximums_i.push_back(i);
                }
                min = max;
                max = minimums[0];
                lookingForMin = true;
            }
        }   
    }
    /*
    //Method 2
    int a = 0;
    for(long i  = 1; i < buffer_size; i++){
        
        double dy = buffer[i] / buffer[i-1];
        std::cout << dy << std::endl;
        if(std::abs(dy-1) <= 0.00001)
        {
            if(a % 2 == 0)
            {
                //Minim
                minimums.push_back(buffer[i]);
                minimums_i.push_back(i);
            }
            else
            {
                //Maxim
                maximums.push_back(buffer[i]);
                maximums_i.push_back(i);
            }
            a++;
        }
    }
    */
    
    // std::cout << "Buffer: "<< std::flush;

    // for(int i = 0; i < 10; i++)
    //     std::cout << buffer[i] << " " << std::flush;
    // std::cout << std::endl;


    // std::cout << "Buffer times: "<< std::flush;

    // for(int i = 0; i < 10; i++)
    //     std::cout << i << " " << std::flush;
    // std::cout << std::endl;


    // std::cout << "Min points: " << std::flush;
    // for(int i = 0; i < 10; i++){
    //     std::cout << minimums[i] << " ";
    // }
    // std::cout << std::endl;
    
    // std::cout << "Min times: " << std::flush;
    // for(int i = 0; i < 10; i++){
    //     std::cout << minimums_i[i] << " ";
    // }
    // std::cout << std::endl;
    

    // std::cout << "Max points: " << std::flush;
    // for(int i = 0; i < maximums.size(); i++){
    //     std::cout << maximums[i] << " ";
    // }
    // std::cout << std::endl;

    // std::cout << "Max times: " << std::flush;
    // for(int i = 0; i < maximums_i.size(); i++){
    //     std::cout << maximums_i[i] << " ";
    // }
    // std::cout << std::endl;

    std::cout << "Found " << maximums_i.size() << " maxes and " << minimums_i.size() << " minimums." << std::endl;

        
    std::vector<std::string> datasets = {"testPos"};
    graph_logger->plot("Positions", datasets,"ms","rad", true, false);
    
    graph_logger->drawMaxMinLocations(maximums,maximums_i,minimums,minimums_i);
    

    std::vector<double> dts;

    //Elimina l'ultima dada si es imparell.
    if(maximums_i.size() % 2 != 0){
        maximums_i.pop_back();
        maximums.pop_back();
    }
    
    for(int i = 0; i < maximums_i.size(); i++){
        long x = maximums_i[i];  //0
        long x1 = maximums_i[i + 1]; // 1
        dts.push_back(x1 - x);
    }

    if(minimums_i.size() % 2 != 0){
        minimums_i.pop_back();
        minimums.pop_back();
    }

    for(int i = 0; i < minimums_i.size(); i++){
        long x = minimums_i[i];  //0
        long x1 = minimums_i[i + 1]; // 1
        dts.push_back(x1 - x);
    }

    std::cout << "Periods: "<< std::flush;
    for(int i = 0; i < dts.size(); i++){
        std::cout << dts[i] << " ";
        period += dts[i];
    }
    std::cout << std::endl;
    

    period /= dts.size();
    period = (period * 100.0) / 1000.0 ;// ms

    std::cout << "Calculated freq: "<< 1000.0/(period) << " dt = " << period << "ms or "<< period / 1000.0 << "s" << std::endl;
    
    wait_for_key();
}

void wait_for_key ()
{
    std::cout << std::endl << "Press ENTER to continue..." << std::endl;

    std::cin.clear();
    std::cin.ignore(std::cin.rdbuf()->in_avail());
    std::cin.get();
    return;
}
