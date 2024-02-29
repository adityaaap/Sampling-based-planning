// #ifndef RRT_H
// #define RRT_H
// #include <vector>
// #include "Node.h"
// #include <chrono>
// #include "support.h"
// #endif

#ifndef RRT_PLANNER_H
#define RRT_PLANNER_H

#include <vector>
#include "Node.h"
#include <chrono>
#include "support.h"

using namespace std;
#define PI 3.141592654
typedef std::chrono::high_resolution_clock myclock;

class RRT_Planner
{
    public:

    vector<Node*> RRT_Graph;
    double* map;
    int x_size;
    int y_size;
    double* armstart_anglesV_rad;
    double* armgoal_anglesV_rad;
    int numdof;
    double**** plan;
    int** planlength;
    int maxSamples;
    double epsilon;
    double f_interpolate;
    int goal_index;
    
    //For testing
    vector<double> d;
    double cost;
    
    RRT_Planner(double* map, int x_size, int y_size, double* armstart_anglesV_rad, double* armgoal_anglesV_rad, int numdof, double**** plan, int** planlength, int maxSamples, double epsilon, double f_interpolate)
    {
        this->map = map;
        this->x_size = x_size;  
        this->y_size = y_size;
        this->armstart_anglesV_rad = armstart_anglesV_rad;
        this->armgoal_anglesV_rad = armgoal_anglesV_rad;
        this->numdof = numdof;
        this->plan = plan;
        this->planlength = planlength;
        this->maxSamples = maxSamples;
        this->epsilon = epsilon;
        this->f_interpolate = f_interpolate;
        goal_index = 0; //CHECK IF ANY ERROR

        vector<double> start;
        for(int i=0; i<numdof; i++){
            start.push_back(armstart_anglesV_rad[i]);
        }
        Node* root = new Node(start, numdof);
        RRT_Graph.push_back(root);

        goalSampleGenerator = std::uniform_real_distribution<double>(0.0, 1.0); 
        randomAngleGenerator = std::uniform_real_distribution<double>(0.0, 2 * PI); 
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        gen.seed(seed);

        
    }

    void run();
    vector<double> generateRandom();
    Node* nearestNeighbour(Node* qrand);
    void addVertex(Node* qnear, Node* qrand);
    vector<double*> getPath();
    double distance(Node* temp, Node* qrand);
    bool newConfig(Node* qnear, Node* qrand);
    bool isGoal(Node* qrand);
    double getCost();


    std::mt19937 gen;
    std::uniform_real_distribution<double> goalSampleGenerator; 
    std::uniform_real_distribution<double> randomAngleGenerator;

};

#endif //RRT_H
