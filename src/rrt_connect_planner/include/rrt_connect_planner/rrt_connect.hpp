#ifndef ROBMOB_RRT_CONNECT_HPP
#define ROBMOB_RRT_CONNECT_HPP

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <memory.h>
#include <memory>
#include <vector>

enum rrt_state
{
    TRAPPED,
    REACHED,
    ADVANCED
};

class pos{public: float x, y;};

class Point
{
    std::shared_ptr<Point> parent;
    pos pose;

    public:
     void set_point(Point point)
     {
        this->parent = point.parent;
        this->set_pos(point.get_pos().x,point.get_pos().y);
     }
    
     void set_pos(float x,float y)
     {
        this->pose = {x, y};
     }

     pos get_pos()
     {
        return pose;
     }

     std::shared_ptr<Point> get_parent()
    {
        return parent;
    }
};

class Tree 
{
    std::vector<Point> l_Point;
    std::vector<pos> path;

    public:
    std::vector<pos> get_path()
    {
        return path;
    }

    std::vector<Point> get_L_Point()
    {
        return l_Point;
    }   

    void set_path(Point point)
    {
        std::shared_ptr<Point> tmp_parent;
        tmp_parent->set_point(point);
        path.push_back(tmp_parent->get_pos());
        while(tmp_parent->get_parent() != NULL)
        {
            path.push_back(tmp_parent->get_pos());
            tmp_parent->set_point(*tmp_parent->get_parent());
        }
    }

    void add_paths(std::vector<pos> path_b)
    {
        std::reverse(path.begin(),path.end());
        path.pop_back();
        path.insert(path.end(),path_b.begin(),path_b.end());
    }
};

class RRT
{
    int K;


    Point RandState()
    {
        float xmax = 100;
        float ymax = 100;
        Point xrand;
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

    Point Nearest_Neighbor(Point point,Tree tree) // Cherche le point le plus proche du point random donné
    {
        Point xnear;
        std::vector<Point> l_Point = tree.get_L_Point();
        int i = 0;
        float dist = 1000000.0;
        while(i<sizeof(l_Point))
        {
            float tmp = dist;
            dist = std::min((l_Point[i].get_pos().x - point.get_pos().x)/ (l_Point[i].get_pos().y-point.get_pos().y),dist);
            if (tmp > dist)
                xnear.set_point(l_Point[i]);            
            i++;
        }
        return xnear;
    } 

    pos Get_goal(){pos goal; return goal;}

    bool New_State(Point x,Point xnear,Point xnew,float unew) // Vérifie s'il atteint un point atteignable et envoie la commande au robot d'une distance constante donnée
    {
        float xdist = (x.get_pos().x-xnear.get_pos().x);
        float ydist = (x.get_pos().y-xnear.get_pos().y);
        xdist /= sqrt(pow(xdist, 2) + pow(ydist, 2));
        ydist /= sqrt(pow(xdist, 2)+pow(ydist, 2));
        
        xnew.set_pos(xdist*unew, ydist*unew);

        if ()//TODO faire un check de la occupencygrid a voir avec jojo
            return true;
        else
            return true;
    }

    rrt_state Extend(Tree tree,Point x)//Cherche à atteindre le point x donné par le random 
    {
        Point xnear = Nearest_Neighbor(x, tree);
        Point xnew;
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