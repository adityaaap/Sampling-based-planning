#include "RRTConnect.h"
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

void RRTConnect_Planner::run(){

    while(RRT_start_graph.size() + RRT_goal_graph.size() < maxSamples){
        vector<double> angles = generateRandom();
        Node* qrand = new Node(angles, numdof);
        Node* qstart = extend(qrand,1);
        Node* qend = extend(qrand,2);

        // vector<Node*>* start_graph = &this->RRT_start_graph;
        // vector<Node*>* end_graph = &this->RRT_goal_graph;
        // Node* qstart = extend (qrand, start_graph);
        
        if(dist(qstart, qend) < epsilon){
            connect(qstart, qend);
            cout<<"Nodes in Graph "<<RRT_start_graph.size()<<endl;
            break;
        }
    }
}

Node* RRTConnect_Planner::extend(Node* q1, int a){
    Node* qnear = nearestNeighbour(q1, a);
    Node* q = addVertex(qnear, q1, a);
    return q;
}

Node* RRTConnect_Planner::nearestNeighbour(Node* qrand, int a){
    if(a == 1){
        Node* qnear = new Node();
        double dist = FLT_MAX;
        for(int i=0; i<RRT_start_graph.size(); i++){
            Node* temp = RRT_start_graph[i];
            double dist_ = RRTConnect_Planner::dist(temp, qrand);
            if(dist_ <= dist){
                dist = dist_;
                qnear = temp;
            }
        }
        d.push_back((double)dist);
        //Print angles
        // cout<<"Angles ";
        // for(int i=0; i<numdof; i++){
        //     cout<<qnear->angles[i]<<" ";
        // }
        // cout<<"end"<<endl;
        return qnear;
    }
    if(a==2){
        Node* qnear = new Node();
        double dist = FLT_MAX;
        for(int i=0; i<RRT_goal_graph.size(); i++){
            Node* temp = RRT_goal_graph[i];
            double dist_ = RRTConnect_Planner::dist(temp, qrand);
            if(dist_ <= dist){
                dist = dist_;
                qnear = temp;
            }
        }
        d.push_back((double)dist);
        //Print angles
        // cout<<"Angles ";
        // for(int i=0; i<numdof; i++){
        //     cout<<qnear->angles[i]<<" ";
        // }
        // cout<<"end"<<endl;
        return qnear;
    }
}

Node* RRTConnect_Planner::addVertex(Node* qnear, Node* qrand, int a){
    vector<double> start = qnear->angles;
    vector<double> end = qrand->angles;

    vector<double> unit_vect;
    double dist = RRTConnect_Planner::dist(qnear, qrand);
    
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

    if(count > 0){// && goal_index==0){
        if(a == 1){
            RRT_start_graph.push_back(qadd);
            qadd->parent = qnear;
            return qadd;
        }
        if(a == 2){
            RRT_goal_graph.push_back(qadd);
            qadd->parent = qnear;
            return qadd;
        }
        // if(isGoal(qadd)){
        //     goal_index = RRT_Graph.size();
        // }
    }
    return qnear;
}

void RRTConnect_Planner::connect(Node* qstart, Node* qend){
    Node* temp1 = qend;
    Node* temp2 = qstart;
    while(temp1->parent != NULL){
        //temp = qend->parent;
        RRT_start_graph.push_back(temp1);
        Node* parent_temp1 = temp1->parent;
        temp1->parent = temp2;

        temp2 = temp1;
        temp1 = parent_temp1;
    }
    vector<double> goal_angle;
    for(int i=0; i<numdof; i++){
        goal_angle.push_back(armgoal_anglesV_rad[i]);
    }
    Node* goal = new Node(goal_angle, numdof);
    RRT_goal_graph.push_back(goal);
    //RRT_start_graph.push_back(temp1);
    goal_index = RRT_start_graph.size();
}


vector<double*> RRTConnect_Planner::getPath(){

    vector<double*> path_temp;
    
    if(goal_index == 0){
        cout<< "No Path Found"<<endl;
        return path_temp;
    }

    Node* temp = RRT_start_graph[goal_index-1];
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
            //cout<<"dist_ "<<dist_<<endl;
            //dist += dist_;
        }
        //cout<<"dist_ "<<dist_<<endl;
        dist_ = sqrt(dist_);
        dist += dist_;
        //cout<<"dist "<<dist<<endl;
    }
    cost = dist;

    angles = &temp->angles[0];
    path_temp.push_back(angles);
    reverse(path_temp.begin(), path_temp.end());
    **planlength = path_temp.size();
    return path_temp;
}

double RRTConnect_Planner::getCost(){
    return cost;
}

double RRTConnect_Planner::dist(Node* q1, Node* q2){
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



vector<double> RRTConnect_Planner::generateRandom(){
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




