#include "PRM.h"
#include <math.h>  
#include <random>
#include <cstdlib>
#include <time.h>
#include <stdio.h>
#include <iostream>
#include <bits/stdc++.h>
#include <limits>
#include <unordered_map>
#include <list>
//#include "support.h"


using namespace std;

void PRM_Planner::buildGraph(){
    int count = 0;
    while(count < maxSamples){
        count++;
        cout<<"Nodes in Graph "<<PRM_Graph.size()<<endl;
        vector<double> angles_random = generateRandom();
        Node* qrand = new Node(angles_random, numdof);

        if(IsValidArmConfiguration(&qrand->angles[0], numdof, map, x_size, y_size)){
            //if(!PRM_Graph.empty()){
                vector<Node*> neighbours = PRM_Planner::neighbours(qrand);
                for(Node* neighbour : neighbours){
                    if(isObstacleFree(neighbour, qrand)){
                        Edge[qrand].push_back(neighbour);
                        Edge[neighbour].push_back(qrand);
                    } 
                }
                PRM_Graph.push_back(qrand);
            //}
        }
    }
}

vector<double*> PRM_Planner::getPath(){
    Node* start = new Node();
    vector<double> start_angles;
    for(int i=0; i<numdof; i++){
        start_angles.push_back(armstart_anglesV_rad[i]);
    }
    start->angles = start_angles;
    
    Node* end = new Node();
    vector<double> end_angles;
    for(int i=0; i<numdof; i++){
        end_angles.push_back(armgoal_anglesV_rad[i]);
    }
    end->angles = end_angles;

    Node* start_in_graph = nearestNode(start);
    Node* end_in_graph = nearestNode(end);

    vector<double*> path = BFS(start_in_graph, end_in_graph, start, end);
    reverse(path.begin(), path.end());
    return path;
}

vector<double*> PRM_Planner::BFS(Node* start_in_graph, Node* end_in_graph, Node* start, Node* goal){
    unordered_map<Node*, Node*> parent;
    list<Node*> open;
    unordered_set<Node*> visited;

    open.push_back(start_in_graph);
    visited.insert(start_in_graph);

    vector<double*> path;
    double* goal_angles = &goal->angles[0];
    path.push_back(goal_angles);

    while(!open.empty()){
        Node* curr = open.front();
        open.pop_front();
        visited.insert(curr);
        //Node* abababa = Edge[curr][0];
        for(int i=0; i < Edge[curr].size(); i++){
            Node* curr_edge = Edge[curr][i];
            if((visited.find(curr_edge) == visited.end())){
                visited.insert(curr_edge);
                parent[curr_edge] = curr;
                open.push_back(curr_edge);
                if(distance(curr_edge, goal) < 0.01){
                    Node* temp = end_in_graph;
                    double* angles = &temp->angles[0];
                    path.push_back(angles);
                    while (parent.count(temp)>0) {
                        double* angle = &parent[temp]->angles[0];
                        path.push_back(angle);
                        temp = parent[temp];
                    }
                    angles = &start->angles[0];
                    path.push_back(angles);
                    **planlength = path.size();
                    return path;
                }
            }
        }
    }
}

Node* PRM_Planner::nearestNode(Node* q){
    Node* qnear;
    double dist = FLT_MAX;
    
    for(int i=0; i<PRM_Graph.size(); i++){
        if(isObstacleFree(q, PRM_Graph[i])){
            double dist_temp = PRM_Planner::distance(q, PRM_Graph[i]);
            if(dist_temp < dist){
                dist = dist_temp;
                qnear = PRM_Graph[i];
            }
        }
    }

    return qnear;
}

bool PRM_Planner::isObstacleFree(Node* q1, Node* q2){
    vector<double> angles1 = q1->angles;
    vector<double> angles2 = q2->angles;
    for(int i=1; i<= f_interpolate; i++){
        Node* temp = new Node();
        for(int j=0; j<numdof; j++){
            temp->angles.push_back(angles1[j] + (double)(i)/(f_interpolate) * (angles1[j] - angles2[j]));
        }
        double* angles = &temp->angles[0];
        if(!IsValidArmConfiguration(angles, numdof, map, x_size, y_size)){
            return false;
        }
    }

    return true;
}

vector<double> PRM_Planner::generateRandom(){
    vector<double> rand;
    double bias = goalSampleGenerator(gen);

    if(bias < 0.2){
        for(int i=0; i<numdof; i++){
            rand.push_back(armgoal_anglesV_rad[i]);
        }
        return rand;
    }

    for(int i=0; i<numdof; i++){
        double angle = randomAngleGenerator(gen);
        rand.push_back(angle);
    }
    return rand;
}

vector<Node*> PRM_Planner::neighbours(Node* qadd){
    double radius = epsilon;
    vector<Node*> neighbours;
    for(int i=0; i<PRM_Graph.size(); i++){
        if(distance(qadd, PRM_Graph[i]) <= epsilon){
            neighbours.push_back(PRM_Graph[i]);
        }
    }

    return neighbours;
}

double PRM_Planner::distance(Node* q1, Node* q2){
    vector<double> angle1 = q1->angles;
    vector<double> angle2 = q2->angles;

    double dist = 0;
    for(int i=0; i<numdof; i++){
        double dist_ = pow((angle1[i] - angle2[i]), 2);
        dist += dist_;
    }
    dist = sqrt(dist);
    return dist;
}