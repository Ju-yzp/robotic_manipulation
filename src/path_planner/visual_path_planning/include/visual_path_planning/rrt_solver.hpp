#ifndef  PATH_PLANNER_THREE_DIM_RRT_SOLVER_HPP_
#define  PATH_PLANNER_THREE_DIM_RRT_SOLVER_HPP_

#include<cmath>
#include<vector>
#include<iostream>

namespace rrt{
class Node{
public:
Node(){
    parent_ = nullptr;
}

Node(float x,float y,float z,Node *parent = nullptr)
:parent_(parent),
x_(x),y_(y),z_(z){}

// Node(Node &) = delete;

// Node& operator=(Node &) = delete;

~Node(){}

void set_position(float x,float y,float z)
{
    x_ = x;
    y_ = y;
    z_ = z;
}

float get_node_distance(Node * node)
{
    return sqrt(std::pow(this->x_-node->x_,2)+
                   std::pow(this->y_-node->y_,2)+
                   std::pow(this->z_-node->z_,2));
}

void set_parent(Node *parent){ parent_ = parent; }

void add_child(Node *child){ children_.push_back(child); }

Node* get_parent(){ return parent_;}

float get_x(){ return x_; }
float get_y(){ return y_; }
float get_z(){ return z_; }

private:

float x_,y_,z_;
Node *parent_;
std::vector<Node *> children_;
};

class Tree{
public:

Node* get_nearest_node(Node *random_node)
{
    Node * nearest_node = nodes_[0];
    float min_distance = random_node->get_node_distance(nearest_node);
    for(auto &node:nodes_)
    {
        float distance = random_node->get_node_distance(node);
        if(distance < min_distance){
           min_distance = distance;
           nearest_node = node; }
    }
    std::cout<<"The nearest node away "<<min_distance<<std::endl;
    return nearest_node;
}

std::vector<Node> get_path()
{
    std::vector<Node> plan_path;

    Node *node = nodes_[nodes_.size()-1];
    while(node->get_parent())
    {
        Node temp_node(node->get_x(),node->get_y(),node->get_z());
        plan_path.emplace_back(temp_node);
        node = node->get_parent();
    }
     Node temp_node(node->get_x(),node->get_y(),node->get_z());
     plan_path.emplace_back(temp_node);
    return plan_path;
}

void add_node(Node *new_node)
{
    std::cout<<"Add a new node into tree"<<std::endl;
    nodes_.push_back(new_node);
}

void reset()
{
    for(auto &node:nodes_)
        delete node;
}

private:

float pos_error_;

std::vector<Node *> nodes_;
};

}
#endif