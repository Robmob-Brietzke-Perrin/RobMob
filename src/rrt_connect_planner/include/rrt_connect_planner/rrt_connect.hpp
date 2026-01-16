#ifndef ROBMOB_RRT_CONNECT_HPP
#define ROBMOB_RRT_CONNECT_HPP

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <memory.h>
#include <memory>
#include <vector>
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace geometry_msgs::msg;

enum rrt_state
{
    TRAPPED,
    REACHED,
    ADVANCED
};

class TreeNode
{
    std::shared_ptr<TreeNode> parent;
    Point pose;

    public:
     void set_point(TreeNode point)
     {
        this->parent = point.parent;
        this->set_pos(point.get_pos().x,point.get_pos().y);
     }
    
     void set_pos(float x,float y)
     {
        this->pose.x = x;
        this->pose.y = y;   
     }

     Point get_pos()
     {
        return pose;
     }

     std::shared_ptr<TreeNode> get_parent()
    {
        return parent;
    }
};

class Tree 
{
    std::vector<TreeNode> l_Point;
    std::vector<Point> path;

    public:
    std::vector<Point> get_path()
    {
        return path;
    }

    std::vector<TreeNode> get_L_Point()
    {
        return l_Point;
    }   

    void set_path(TreeNode point)
    {
        std::shared_ptr<TreeNode> tmp_parent;
        tmp_parent->set_point(point);
        path.push_back(tmp_parent->get_pos());
        while(tmp_parent->get_parent() != NULL)
        {
            path.push_back(tmp_parent->get_pos());
            tmp_parent->set_point(*tmp_parent->get_parent());
        }
    }

    void add_paths(std::vector<Point> path_b)
    {
        std::reverse(path.begin(),path.end());
        path.pop_back();
        path.insert(path.end(),path_b.begin(),path_b.end());
    }
};

class RRT
{
    int K;


    TreeNode RandState()
    {
        float xmax = 100;
        float ymax = 100;
        TreeNode xrand;
        xrand.set_pos(rand()/xmax,rand()/ymax);
        return xrand;
    }

    // void BuildRRT(float* xinit)//initialise l'arbre puis loop sur un random x puis extend
    // {
    //     float*** Tree;
    //     Tree[0][0] = xinit;
    //     int k=0;
    //     while (k<K)
    //     {
    //         float* xrand = RandState();
    //         //extend
    //         k++;
    //     }
    // }

    TreeNode Nearest_Neighbor(TreeNode point,Tree tree) // Cherche le point le plus proche du point random donné
    {
        TreeNode xnear;
        std::vector<TreeNode> l_Point = tree.get_L_Point();
        long unsigned int i = 0;
        float dist = 1000000.0;
        float l_x,l_y,x,y,tmp;
        while(i<sizeof(l_Point))
        {
            tmp = dist;
            l_x = l_Point[i].get_pos().x;
            l_y = l_Point[i].get_pos().y;
            x = point.get_pos().x;
            y = point.get_pos().y;
            dist = std::min((l_x- x)/ (l_y-y),dist);
            if (tmp > dist)
                xnear.set_point(l_Point[i]);            
            i++;
        }
        return xnear;
    } 

    Point Get_goal(){Point goal; return goal;}

    bool New_State(TreeNode x,TreeNode xnear,TreeNode xnew,float unew) // Vérifie s'il atteint un point atteignable et envoie la commande au robot d'une distance constante donnée
    {
        float xdist = (x.get_pos().x-xnear.get_pos().x);
        float ydist = (x.get_pos().y-xnear.get_pos().y);
        xdist /= sqrt(pow(xdist, 2) + pow(ydist, 2));
        ydist /= sqrt(pow(xdist, 2)+pow(ydist, 2));
        
        xnew.set_pos(xdist*unew, ydist*unew);

        if (true)//TODO faire un check de la occupencygrid a voir avec jojo
            return true;
        else
            return true;
    }

    rrt_state Extend(Tree tree,TreeNode x)//Cherche à atteindre le point x donné par le random 
    {
        TreeNode xnear = Nearest_Neighbor(x, tree);
        TreeNode xnew;
        float unew;
        if (New_State(x,xnear,xnew,unew))
        {
            //Tree.add.vertex();
            //Tree.add.edge();
            if (xnew.get_pos().x == Get_goal().x && xnew.get_pos().y == Get_goal().y) // Vérifie s'il est possible d'atteindre le point final
                return REACHED;//reached;
            else 
                return ADVANCED;//advanced;// Sinon on a avancé
        }
        return TRAPPED;//trapped;// Sinon c'est qu'il est impossible d'atteindre le xrand donné
    }
};
#endif