// #ifndef RRT_H
// #define RRT_H
// #include <vector>
// #include "Node.h"
// #include <chrono>
// #include "support.h"
// #endif

#ifndef PRM_PLANNER_H
#define PRM_PLANNER_H

#include <iostream>
#include <vector>
#include "Node.h"
#include <chrono>
#include "support.h"
#include <unordered_map>



using namespace std;
#define PI 3.141592654
typedef std::chrono::high_resolution_clock myclock;

class PRM_Planner
{
    public:

    vector<Node*> PRM_Graph;
    unordered_map<Node*, vector<Node*>> Edge;
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
    // vector<double> d;
    // double cost;
    
    PRM_Planner(double* map, int x_size, int y_size, double* armstart_anglesV_rad, double* armgoal_anglesV_rad, int numdof, double**** plan, int** planlength, int maxSamples, double epsilon, double f_interpolate)
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

        // vector<double> start;
        // for(int i=0; i<numdof; i++){
        //     start.push_back(armstart_anglesV_rad[i]);
        // }
        // Node* root = new Node(start, numdof);
        // //root->cost = 0;
        // RRT_Graph.push_back(root);

        goalSampleGenerator = std::uniform_real_distribution<double>(0.0, 1.0); 
        randomAngleGenerator = std::uniform_real_distribution<double>(0.0, 2 * PI); 
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        gen.seed(seed);

        
    }

    void buildGraph();
    vector<double*> getPath();
    Node* nearestNode(Node* q);
    vector<double*> BFS(Node* start_in_graph, Node* end_in_graph, Node* start, Node* goal);
    //vector<double*> BFS(Node* q1, Node*q2, Node* start, Node* end);
    double distance(Node* temp, Node* qrand);
    vector<double> generateRandom();
    vector<Node*> neighbours(Node* qadd);
    bool isObstacleFree(Node* q1, Node* q2);

    // void run();
    // //void extend(Node* qrand);
    // //void add_vertex(Node* qnear,Node* qnew);
    
    // void rewire(Node* qend);
    
   
    // Node* nearestNeighbour(Node* qrand);
    // Node* addVertex(Node* qnear, Node* qrand);
    
    
    // bool newConfig(Node* qnear, Node* qrand);
    // bool isGoal(Node* qrand);
    // double getCost();


    std::mt19937 gen;
    std::uniform_real_distribution<double> goalSampleGenerator; 
    std::uniform_real_distribution<double> randomAngleGenerator;

};

#endif //RRT_H
