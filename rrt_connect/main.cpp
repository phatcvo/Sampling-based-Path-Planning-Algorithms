#include <rrt_connect.h>
#include <iostream>
#include <chrono>



int main()
{
    // 400x400 size grid map
    // rrt(start_x, start_y, goal_x, goal_y, range_x, range_y)
    float start_x = 30.0;
    float start_y = 30.0;
    float goal_x = 350.0;
    float goal_y = 350.0;
    unsigned int max_sampling = 500;
    float step_length = 30.0;
    float termination_radius = 30.0;
    float range_x = 400.0;
    float range_y = 400.0;

    auto start_time = std::chrono::high_resolution_clock::now();

    RRTConnect::RRTConnect rrt_connect(start_x, start_y, goal_x, goal_y, max_sampling, step_length, termination_radius, range_x, range_y);
    rrt_connect.run();

    auto end_time = std::chrono::high_resolution_clock::now();
    std::cout << ">>> Execution time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() << " ms\n";

    rrt_connect.visualize();
}