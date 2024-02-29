#include "RRT.h"
//#include "support.h"
#include <math.h>  
#include <random>
#include <cstdlib>
#include <time.h>
#include <stdio.h>
#include <iostream>
#include <bits/stdc++.h>
#include <limits>

using namespace std;

void RRT_Planner::run(){
    //unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    //gen.seed(seed);

    while(RRT_Graph.size() < maxSamples){
        cout<<"count "<<RRT_Graph.size()<<endl;
        vector<double> rand = generateRandom();
        Node* qrand = new Node(rand, numdof);
        Node* qnear = nearestNeighbour(qrand);
        if(newConfig(qnear, qrand)){
            addVertex(qnear, qrand);
            if(goal_index !=0){
                cout<<"Nodes in Graph "<<RRT_Graph.size()<<endl;
                break;
            }
        }
        
        //vector<double*> path = getPath();
        //return path;
    }
    //auto min = min_element(d.begin(), d.end());
    //double min_value = *min;
    //cout<<"Min Dist "<<min_value<< endl;
}

vector<double> RRT_Planner::generateRandom(){
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

Node* RRT_Planner::nearestNeighbour(Node* qrand){
    Node* qnear = new Node();
    double dist = FLT_MAX;
    for(int i=0; i<RRT_Graph.size(); i++){
        Node* temp = RRT_Graph[i];
        double dist_ = RRT_Planner::distance(temp, qrand);
        if(dist_ <= dist){
            dist = dist_;
            qnear = temp;
        }
    }
    d.push_back((double)dist);
    //Print angles
    cout<<"Angles ";
    for(int i=0; i<numdof; i++){
        cout<<qnear->angles[i]<<" ";
    }
    cout<<"end"<<endl;
    return qnear;
}

void RRT_Planner::addVertex(Node* qnear, Node* qrand){
    vector<double> start = qnear->angles;
    vector<double> end = qrand->angles;

    vector<double> unit_vect;
    double dist = RRT_Planner::distance(qnear, qrand);
    
    if(dist > epsilon){
        for(int i=0; i<numdof; i++){
            unit_vect.push_back((epsilon/dist) * (end[i] - start[i]));
        }
    }
    else{
        for(int i=0; i<numdof; i++){
            unit_vect.push_back((end[i] - start[i]));
        }
    }
    int i,j;
    int count = 0;
    Node* qadd = new Node();
    
    for(int i=1; i<=f_interpolate; i++){ // Progressing the angle in small increments and not all at once, f_interpolate is basically the number of intermediate steps
        Node* qtemp = new Node();
        for(int j=0; j<numdof; j++){
            qtemp->angles.push_back(start[j] + ((double)(i)/(f_interpolate))*unit_vect[j]);
        }

        double* config = &qtemp->angles[0];
        if(IsValidArmConfiguration(config, numdof, map, x_size, y_size)){
            count++;
            qadd = qtemp;
        }
        else{
            break;
        }

    }

    if(count > 0 && goal_index==0){
        RRT_Graph.push_back(qadd);
        qadd->parent = qnear;
        if(isGoal(qadd)){
            goal_index = RRT_Graph.size();
        }
    }
}

double RRT_Planner::distance(Node* q1, Node* q2){
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

bool RRT_Planner::newConfig(Node* q1, Node* q2){
    for(int i=0; i<numdof; i++){
        if(q1->angles[i] != q2->angles[i]){
            return true;
        }
    }
    return false;
}

bool RRT_Planner::isGoal(Node* q){
    vector<double> goal_angles;
    for(int i=0; i<numdof; i++){
        goal_angles.push_back(armgoal_anglesV_rad[i]);
    }

    for(int i=0; i<numdof; i++){
        if(q->angles[i] != goal_angles[i]){
            return false;
        }
    }
    return true;

    // //Node* goal = new Node(goal_angles, numdof);
    // double dist = 0;
    // for(int i=0; i<numdof; i++){
    //     double dist_ = pow((goal_angles[i] - q->angles[i]), 2);
    //     dist += dist_;
    // }
    // dist = sqrt(dist);

    // if(dist < 0.05){
    //     return true;
    // }
    // return false;
}

vector<double*> RRT_Planner::getPath(){
    vector<double*> path_temp;
    if(goal_index == 0){
        cout<< "No Path Found"<<endl;
        return path_temp;
    }

    Node* temp = RRT_Graph[goal_index-1];
    double* angles = NULL;
    int dist = 0;
    while(temp->parent != NULL){
        angles = &temp->angles[0];
        vector<double> ang = temp->angles;
        path_temp.push_back(angles);
        temp = temp->parent;
        double dist_ = 0;
        for(int i=0; i<numdof; i++){
            dist_ += pow((ang[i] - temp->angles[i]), 2);
            //dist += dist_;
        }
        //dist_ = sqrt(dist_);
        dist += dist_;
    }
    cost = dist;

    angles = &temp->angles[0];
    path_temp.push_back(angles);
    reverse(path_temp.begin(), path_temp.end());
    **planlength = path_temp.size();
    return path_temp;

}

double RRT_Planner::getCost(){
    return cost;
}


