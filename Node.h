// #include<iostream>
// #include <vector>
// #include <bits/stdc++.h>
// using namespace std;

// class Node{
// 	public:
// 	vector<double> angles;
// 	double cost;
// 	Node* parent;

// 	Node(){
// 		this->cost = FLT_MAX;
// 	}
// 	Node(vector<double> angles, int dof){
// 		for(int i=0; i<dof; i++){
// 			this->angles.push_back(angles[i]);
// 		}

// 		this->parent=NULL;
// 		this->cost = FLT_MAX;
// 	}

// };

#include <vector>
#include <bits/stdc++.h>
using namespace std;
class Node
{
    public:
    vector <double> angles;
    Node* parent;
    double cost;

    Node()
    {
        cost = FLT_MAX;
    }
    Node(vector<double> angles,int numDOF)
    {
        for (int i = 0;i<numDOF;i++)
        {
            this->angles.push_back(angles[i]);
        }
        this->parent = NULL;
        cost = FLT_MAX;
    }
};