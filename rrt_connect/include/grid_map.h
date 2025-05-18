#ifndef map_MAP_H
#define map_MAP_H

#include <iostream>
#include <vector>
#include <random>



class Grid
{
public:
    Grid(unsigned int size_x, unsigned int size_y)
    : size_x_(size_x), size_y_(size_y)
    {
        map_ = new unsigned int[size_x_ * size_y_];
        initializeMap();
        // generateRandomObstacle(4);
    };

    ~Grid()
    {
        delete[] map_;
    };

    void initializeMap()
    {
        for (unsigned int i = 0; i != size_x_ * size_y_; ++i)
            map_[i] = 0;
    };

    unsigned int getCost(unsigned int mx, unsigned int my) const
    {
        return map_[getIndex(mx, my)];
    };

    unsigned int getCost(double wx, double wy) const
    {
        unsigned int mx = int(wx);
        unsigned int my = int(wy);
        return map_[getIndex(mx, my)];
    };

    void setCost(unsigned int mx, unsigned int my, unsigned int cost)
    {
        map_[getIndex(mx, my)] = cost;
    };

    unsigned int getIndex(unsigned int mx, unsigned int my) const
    {
        return my * size_x_ + mx;
    };

    void generateRandomObstacles(unsigned int num_obstacle)
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<int> random_x(100,300);
        std::uniform_int_distribution<int> random_y(100,300);

        unsigned int obstacle_size = 30;
        unsigned int obs = 0;

        while (obs < num_obstacle)
        {
            unsigned int pos_x = random_x(gen);
            unsigned int pos_y = random_y(gen);

            for (unsigned int mx = pos_x; mx < pos_x + obstacle_size; ++mx)
            {
                for (unsigned int my = pos_y; my < pos_y + obstacle_size; ++my)
                {
                    if (mx < size_x_ && my < size_y_ && map_[getIndex(mx, my)] == 0)
                    {
                        // assume that obstacle cost is 100
                        setCost(mx, my, 100);
                    }
                }
            }
            ++obs;
        }

        // // generate initial obstacle block
        // for (unsigned int mx = 0; mx < size_x_ * 1/2; ++mx)
        // {
        //     for (unsigned int my = size_y_ * 1/4; my < size_y_ * 1/4 + size_y_/50; ++my)
        //     {
        //         if (map_[getIndex(mx, my)] == 0)
        //             setCost(mx, my, 100);
        //     }
        // }

        // for (unsigned int mx = size_x_ * 1/2; mx < size_x_; ++mx)
        // {
        //     for (unsigned int my = size_y_ * 2/4; my < size_y_ * 2/4 + size_y_/50; ++my)
        //     {
        //         if (map_[getIndex(mx, my)] == 0)
        //             setCost(mx, my, 100);
        //     }
        // }

        // for (unsigned int mx = 0; mx < size_x_ * 1/2; ++mx)
        // {
        //     for (unsigned int my = size_y_ * 3/4; my < size_y_ * 3/4 + size_y_/50; ++my)
        //     {
        //         if (map_[getIndex(mx, my)] == 0)
        //             setCost(mx, my, 100);
        //     }
        // }
    };

private:
    unsigned int size_x_;
    unsigned int size_y_;
    unsigned int* map_;
};

#endif