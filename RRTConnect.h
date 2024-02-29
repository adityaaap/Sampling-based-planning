#ifndef RRT_CONNECT_H
#define RRT_CONNECT_H

#include <vector>
#include "Node.h"
#include <chrono>
#include "support.h"

using namespace std;
#define PI 3.141592654
typedef std::chrono::high_resolution_clock myclock;

class RRTConnect_Planner{
    public:

    vector<Node*> RRT_start_graph;
    vector<Node*> RRT_goal_graph;
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

    RRTConnect_Planner(double* map, int x_size, int y_size, double* armstart_anglesV_rad, double* armgoal_anglesV_rad, int numdof, double**** plan, int** planlength, int maxSamples, double epsilon, double f_interpolate){
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
        this->goal_index = 0;

        vector<double> ang;
        for(int i=0; i<numdof; i++){
            ang.push_back(armstart_anglesV_rad[i]);
        }
        Node* start = new Node(ang, numdof);
        RRT_start_graph.push_back(start);

        vector<double> ang_end;
        for(int i=0; i<numdof; i++){
            ang_end.push_back(armgoal_anglesV_rad[i]);
        }
        Node* end = new Node(ang_end, numdof);
        RRT_goal_graph.push_back(end);

        goalSampleGenerator = std::uniform_real_distribution<double>(0.0, 1.0); 
        randomAngleGenerator = std::uniform_real_distribution<double>(0.0, 2 * PI); 
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        gen.seed(seed);

    }

    void run();
    Node* extend(Node* q1, int a);
    Node* addVertex(Node* qnear, Node* qrand, int a);
    void connect(Node* qstart, Node* qend);
    vector<double*> getPath();
    double getCost();
    double dist(Node* q1, Node* q2);
    Node* nearestNeighbour(Node* qrand, int a);
    vector<double> generateRandom();

    std::mt19937 gen;
    std::uniform_real_distribution<double> goalSampleGenerator; 
    std::uniform_real_distribution<double> randomAngleGenerator;
};

#endif //RRT_CONNECT_H
