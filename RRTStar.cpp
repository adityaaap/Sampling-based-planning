#include "RRTStar.h"
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

void RRTStar_Planner::run(){
    //unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    //gen.seed(seed);

    while(RRT_Graph.size() < maxSamples){
        cout<<"count "<<RRT_Graph.size()<<endl;
        vector<double> rand = generateRandom();
        Node* qrand = new Node(rand, numdof);
        // extend(qrand);
        // if(goal_index !=0){
        //     cout<<"Nodes in Graph "<<RRT_Graph.size()<<endl;
        //     break;
        // }

        Node* qnear = nearestNeighbour(qrand);
        Node* qend = addVertex(qnear, qrand);
        rewire(qend);
        if(goal_index !=0){
            cout<<"Nodes in Graph "<<RRT_Graph.size()<<endl;
            break;
        }

        ///////////////

        // if(newConfig(qnear, qrand)){
        //     qend = addVertex(qnear, qrand);
        //     if(goal_index !=0){
        //         cout<<"Nodes in Graph "<<RRT_Graph.size()<<endl;
        //         break;
        //     }
        // }
        // rewire(qend);
        //vector<double*> path = getPath();
        //return path;
    }
    //auto min = min_element(d.begin(), d.end());
    //double min_value = *min;
    //cout<<"Min Dist "<<min_value<< endl;
}


vector<Node*> RRTStar_Planner::neighbours(Node* qadd){
    double radius = epsilon;
    vector<Node*> neighbours;
    for(int i=0; i<RRT_Graph.size(); i++){
        if(distance(qadd, RRT_Graph[i]) <= epsilon){
            neighbours.push_back(RRT_Graph[i]);
        }
    }

    return neighbours;
}

void RRTStar_Planner::rewire(Node* qend){
    vector<Node*> neighbours = RRTStar_Planner::neighbours(qend);
    for(int i=0; i<neighbours.size(); i++){
        double dist = distance(qend, neighbours[i]);
        if(isObstacleFree(qend, neighbours[i])){
             if(neighbours[i]->cost + dist < qend->cost){ //optimal path to qend passes through q
                qend->cost = neighbours[i]->cost + dist;
                qend->parent = neighbours[i];
            }
        }
    }  

    for(int i=0; i<neighbours.size(); i++){
        double dist = distance(qend, neighbours[i]);
        if(isObstacleFree(qend, neighbours[i])){
            if(qend->cost + dist < neighbours[i]->cost){ //optimal path to q passes through qend
                neighbours[i]->cost = qend->cost + dist;
                neighbours[i]->parent = qend;
            }
        }
    }
    
}

bool RRTStar_Planner::isObstacleFree(Node* q1, Node* q2){
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

vector<double> RRTStar_Planner::generateRandom(){
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

Node* RRTStar_Planner::nearestNeighbour(Node* qrand){
    Node* qnear = new Node();
    double dist = FLT_MAX;
    for(int i=0; i<RRT_Graph.size(); i++){
        Node* temp = RRT_Graph[i];
        double dist_ = RRTStar_Planner::distance(temp, qrand);
        if(dist_ <= dist){
            dist = dist_;
            qnear = temp;
        }
    }
    //d.push_back((double)dist);
    //Print angles
    cout<<"Angles ";
    for(int i=0; i<numdof; i++){
        cout<<qnear->angles[i]<<" ";
    }
    cout<<"end"<<endl;
    return qnear;
}

Node* RRTStar_Planner::addVertex(Node* qnear, Node* qrand){
    vector<double> start = qnear->angles;
    vector<double> end = qrand->angles;

    vector<double> unit_vect;
    double dist = RRTStar_Planner::distance(qnear, qrand);
    
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
        // double cost = 0;
        // Node* temp = qadd;
        // while(temp->parent != NULL){
        //     double dist = distance(temp, temp->parent);
        //     temp = temp->parent;
        //     cost += dist;
        // }
        qadd->cost = qnear->cost + distance(qadd, qnear);

        if(isGoal(qadd)){
            goal_index = RRT_Graph.size();
        }
        return qadd;
    }

    return qnear;
}

double RRTStar_Planner::distance(Node* q1, Node* q2){
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

bool RRTStar_Planner::newConfig(Node* q1, Node* q2){
    for(int i=0; i<numdof; i++){
        if(q1->angles[i] != q2->angles[i]){
            return true;
        }
    }
    return false;
}

bool RRTStar_Planner::isGoal(Node* q){
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

vector<double*> RRTStar_Planner::getPath(){
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
    //cost = dist;

    angles = &temp->angles[0];
    path_temp.push_back(angles);
    reverse(path_temp.begin(), path_temp.end());
    **planlength = path_temp.size();
    return path_temp;

}

double RRTStar_Planner::getCost(){
    return 0;
}



// void RRTStar_Planner::extend(Node* qrand)
// {
//     Node* qnear = nearestNeighbour(qrand);
//     add_vertex(qnear,qrand);

// }

// void RRTStar_Planner::add_vertex(Node* qnear,Node* qnew)
// {
//     vector <double> start_angles = qnear->angles;
//     vector <double> end_angles = qnew->angles;
//     double config_dist = distance(qnear, qnew);
//     vector <double> eps_vec;
//     if (config_dist > epsilon)
//     {
//         for (int m=0;m<numdof;m++)
//         {
//             eps_vec.push_back((epsilon/config_dist)*(end_angles[m]-start_angles[m]));
//         }
//     }
//     else
//     {
//         for (int m=0;m<numdof;m++)
//         {
//             eps_vec.push_back((end_angles[m]-start_angles[m]));
//         }
//     }
//     int i,j;
//     int countNumvalid = 0;
//     Node* qadd = new Node();
//     for (i = 1; i <= f_interpolate; i++)
//     {
//         Node* qtemp = new Node(); 
//         for(j = 0; j < numdof; j++)
//         {
//             qtemp->angles.push_back(start_angles[j] + ((double)(i)/(f_interpolate))*eps_vec[j]);
        
//         }
        
//         double* config = &qtemp->angles[0];
//         if(IsValidArmConfiguration(config, numdof, map, x_size, y_size)) 
//         {
//             ++countNumvalid;
//             qadd = qtemp;
//         }
//         else{
//             break;
//         }
//     }
//     if (countNumvalid > 0 && goal_index==0)
//     {
//         RRT_Graph.push_back(qadd);
//         qadd->parent = qnear;
//         qadd->cost = qnear->cost + distance(qnear,qadd);
//         if (isGoal(qadd))
//         {
//             goal_index = RRT_Graph.size();
//         }
//     }
//     if (countNumvalid>0)
//     {
//         vector<Node*> Near = neighbours(qadd);
//         for (int i =0;i<Near.size();i++)
//         {
//             if (isObstacleFree(Near[i],qadd))
//             {
//                 double new_cost = Near[i]->cost + distance(Near[i],qadd);
//                 if (new_cost < qadd->cost)
//                 {
//                     qadd->parent = Near[i];
//                 }

//             }
//         }
//         for (int i =0;i<Near.size();i++)
//         {
//             if (isObstacleFree(Near[i],qadd))
//             {
//                 double new_cost = qadd->cost + distance(Near[i],qadd);
//                 if (new_cost < Near[i]->cost)
//                 {
//                     Near[i]->parent = qadd;
//                 }

//             }
//         }
//     }  
// }

