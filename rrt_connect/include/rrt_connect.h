#ifndef RRT_H
#define RRT_H

#include <iostream>
#include <cmath>
#include <random>
#include <algorithm>
#include <list>
#include <set>
#include <opencv2/opencv.hpp>
#include <grid_map.h>


namespace RRTConnect{

class Node
{
public:
    Node(float x, float y) : x_(x), y_(y) {}

    float getX() const
    {
        return x_;
    }
    
    float getY() const
    {
        return y_;
    }

    void setX(float new_x)
    {
        x_ = new_x;
    }

    void setY(float new_y)
    {
        y_ = new_y;
    }

    Node* getParent() const
    {
        return parent_;
    }

    void setParent(Node* parent)
    {
        parent_ = parent;
    }

    float computeDistance(const Node* other) const
    {
        return std::hypot(x_ - other->getX(), y_ - other->getY());
    }

    float computeDistance(const Node other) const
    {
        return std::hypot(x_ - other.getX(), y_ - other.getY());
    }

    float computeDistance(float x, float y) const
    {
        return std::hypot(x_ - x, y_ - y);
    }

private:
    float x_;
    float y_;
    Node* parent_;
    Node* child_;
};

class RRTConnect
{
public:
    RRTConnect(float start_x, float start_y, float goal_x, float goal_y, unsigned int max_sampling, float step_length, float termination_radius, float range_x, float range_y)
        : start_x_(start_x), start_y_(start_y), goal_x_(goal_x), goal_y_(goal_y), max_sampling_(max_sampling), step_length_(step_length), termination_radius_(termination_radius)
    {
        grid_ = new Grid(range_x, range_y);
        grid_->generateRandomObstacles(20);
        success_ = false;
    }

    ~RRTConnect()
    {
        delete grid_;
        for (auto node : forward_node_list_)
            delete node;
        for (auto node : backward_node_list_)
            delete node;
    }

    // Bresenham's algorithm (http://members.chello.at/~easyfilter/bresenham.html)
    std::list<std::pair<unsigned int, unsigned int>> computeBresenhamLine(int x0, int y0, int x1, int y1) const
    {
        std::list<std::pair<unsigned int, unsigned int>> line;
        int dx =  abs(x1-x0), sx = x0<x1 ? 1 : -1;
        int dy = -abs(y1-y0), sy = y0<y1 ? 1 : -1; 
        int err = dx+dy, e2; /* error value e_xy */

        for(;;)
        {  /* loop */
            line.emplace_back(x0, y0);
            if (x0==x1 && y0==y1) break;
            e2 = 2*err;
            if (e2 >= dy) { err += dy; x0 += sx; } /* e_xy+e_x > 0 */
            if (e2 <= dx) { err += dx; y0 += sy; } /* e_xy+e_y < 0 */
        }
        return line;
    }

    bool isCollision(const Node* v, const Node* u) const
    {
        unsigned int m_vx, m_vy, m_ux, m_uy;
        m_vx = int(v->getX());
        m_vy = int(v->getY());
        m_ux = int(u->getX());
        m_uy = int(u->getY());

        auto bresenham_line = computeBresenhamLine(m_vx, m_vy, m_ux, m_uy);

        for (const auto& point : bresenham_line)
        {
            unsigned int cell_cost = grid_->getCost(point.first, point.second);
            if (cell_cost > 0.65)
                return true;
        }
        return false;
    }

    std::pair<float, float> computeDistanceAndAngle(const Node* node, const Node* nearest_node) const
    {
        float dx = node->getX() - nearest_node->getX();
        float dy = node->getY() - nearest_node->getY();

        return std::make_pair(std::hypot(dx, dy), std::atan2(dy, dx));
    }

    // Change the position of node with steer function
    Node* steer(Node* node, Node* nearest_node)
    {
        std::pair<float, float> distance_and_angle = computeDistanceAndAngle(node, nearest_node);

        float step_length = std::min(step_length_, distance_and_angle.first);

        node->setX(nearest_node->getX() + step_length * std::cos(distance_and_angle.second));
        node->setY(nearest_node->getY() + step_length * std::sin(distance_and_angle.second));

        return node;
    }

    // Generate new node with steer function!
    Node* steer2(Node* node, Node* nearest_node)
    {
        std::pair<float, float> distance_and_angle = computeDistanceAndAngle(node, nearest_node);

        float step_length = std::min(step_length_, distance_and_angle.first);

        Node* new_node = new Node(nearest_node->getX() + step_length * std::cos(distance_and_angle.second),
                                  nearest_node->getY() + step_length * std::sin(distance_and_angle.second));

        return new_node;
    }

    // Target biased sampling
    Node* sampleNode(const Node* root_node) const
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> distribution_x(0, 400 - 1);
        std::uniform_real_distribution<float> distribution_y(0, 400 - 1);
        std::uniform_real_distribution<float> bias_distribution(0, 1);

        // for biased sampling
        if (bias_distribution(gen) < bias_)
            return new Node(root_node->getX(), root_node->getY());
        
        // for uniform sampling
        float rand_x = distribution_x(gen);
        float rand_y = distribution_y(gen);

        return new Node(rand_x, rand_y);
    }

    // Random sampling
    Node* sampleNode() const
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> distribution_x(0, 400 - 1);
        std::uniform_real_distribution<float> distribution_y(0, 400 - 1);
        std::uniform_real_distribution<float> bias_distribution(0, 1);
        
        // for uniform sampling
        float rand_x = distribution_x(gen);
        float rand_y = distribution_y(gen);

        return new Node(rand_x, rand_y);
    }

    Node* nearest(const Node* node, const std::list<Node*> node_list) const
    {
        Node* nearest_node;
        double min_distance = std::numeric_limits<double>::infinity();
        for (const auto& u : node_list)
        {
            double distance = node->computeDistance(u);
            if (distance < min_distance)
            {
               min_distance = distance;
               nearest_node = u;
            }
        }
        return nearest_node;
    }

    void run()
    {
        start_ = new Node(start_x_, start_y_);
        start_->setParent(nullptr);
        forward_node_list_.push_back(start_);

        goal_ = new Node(goal_x_, goal_y_);
        goal_->setParent(nullptr);
        backward_node_list_.push_back(goal_);

        // Just for visualization
        visualize_node_list_.push_back(start_);
        visualize_node_list_.push_back(goal_);

        // Start with forward direction first!
        std::list<Node*> node_list = forward_node_list_;
        std::list<Node*> opposite_node_list = backward_node_list_;
        Node* root = start_;
        Node* opposite_root = goal_;

        int sampling_num = 0;
        while (sampling_num < max_sampling_)
        {
            sampling_num++;

            // Do random sampling
            Node* sample_node = sampleNode(opposite_root);
            // Node* sample_node = sampleNode();

            // Find nearest node from random node
            // Can be improved with another algorithm such like "KD-tree"
            Node* nearest_node = nearest(sample_node, node_list);

            // Apply steer function to random node
            Node* new_node = steer(sample_node, nearest_node);

            // If collision free then, add random node in node list
            // If new node is close to target node, then generate path from init to target
            if (isCollision(new_node, nearest_node))
            {
                delete new_node;
                continue;
            }
            else
            {
                new_node->setParent(nearest_node);
                node_list.push_back(new_node);
                // Just for visualization
                visualize_node_list_.push_back(new_node);

                // Find nearest node from opposite tree
                Node* nearest_node_prime = nearest(new_node, opposite_node_list);
                // Generate new node prime from opposite direction heading towards new node in current tree
                Node* new_node_prime = steer2(new_node, nearest_node_prime);

                // Extend function in paper
                if (isCollision(new_node_prime, nearest_node_prime))
                {
                    delete new_node_prime;
                    continue;
                }
                else
                {
                    new_node_prime->setParent(nearest_node_prime);
                    opposite_node_list.push_back(new_node_prime);
                    // Just for visualization
                    visualize_node_list_.push_back(new_node_prime);

                    // try to extend new node prime from opposite direction
                    while (true)
                    {
                        Node* new_node_prime_prime = steer2(new_node, new_node_prime);
                        if (isCollision(new_node_prime, new_node_prime_prime) == false)
                        {
                            new_node_prime_prime->setParent(new_node_prime);
                            opposite_node_list.push_back(new_node_prime_prime);
                            new_node_prime = new_node_prime_prime;
                            // Just for visualization
                            visualize_node_list_.push_back(new_node_prime_prime);
                        }
                        else
                        {
                            delete new_node_prime_prime;
                            break;
                        }

                        // Connect function in paper
                        if (new_node->computeDistance(new_node_prime) < termination_radius_
                            && isCollision(new_node, new_node_prime) == false)
                        {
                            extractPathNodes(new_node, new_node_prime);
                            success_ = true;
                            return;
                        }
                    }
                }
            }


            // Swap function in paper
            // Change tree
            std::list<Node*> temp_list = opposite_node_list;
            opposite_node_list = node_list;
            node_list = temp_list;

            // Change biased target
            Node* temp_root = opposite_root;
            opposite_root = root;
            root = temp_root;
        }

        // Success description
        if (success_) std::cout << ">>> target searching is successful" << std::endl;
        else std::cout << ">>> target searching is failed" << std::endl;
    }

    void extractPathNodes(Node* new_node, Node* new_node_prime)
    {
        std::vector<Node*> path_segment;
        std::vector<Node*> path_segment_prime;
        Node* node = new_node;
        bool reverse_prime = false;

        while (node->getParent() != nullptr)
        {
            path_segment.push_back(node);
            node = node->getParent();
        }
        path_segment.push_back(node);

        if (node == start_)
            std::reverse(path_segment.begin(), path_segment.end());

        node = new_node_prime;

        while (node->getParent() != nullptr)
        {
            path_segment_prime.push_back(node);
            node = node->getParent();
        }
        path_segment_prime.push_back(node);

        if (node == start_)
        {
            std::reverse(path_segment_prime.begin(), path_segment_prime.end());
            reverse_prime = true;
        }

        if (reverse_prime)
        {
            path_segment_prime.insert(path_segment_prime.end(), path_segment.begin(), path_segment.end());
            path_nodes_ = path_segment_prime;
        }
        else
        {
            path_segment.insert(path_segment.end(), path_segment_prime.begin(), path_segment_prime.end());
            path_nodes_ = path_segment;
        }
    }

    std::vector<std::pair<float,float>> getPathPoints()
    {
        std::vector<std::pair<float,float>> path_points;
    
        for (const auto& node : path_nodes_)
            path_points.emplace_back(std::make_pair(node->getX(), node->getY()));

        path_points_ = path_points;
        return path_points;
    }

    std::vector<Node*> getPathNodes()
    {
        return path_nodes_;
    }

    std::vector<std::pair<float,float>> postProcessPath()
    {
        std::vector<Node*> path = getPathNodes();
        std::vector<std::pair<float,float>> processed_points;

        processed_points.push_back(std::make_pair(start_->getX(), start_->getY()));

        Node* start_node = start_;
        Node* connect_node;

        int start_idx = 0;
        int end_idx = path.size();
        int i;

        while(true){
            start_node = path[start_idx];
            for (i = start_idx+1; i < end_idx; i++)
            {
                connect_node = path[i];
                if (isCollision(start_node, connect_node))
                {
                    Node* before_node = path[i-1];
                    processed_points.push_back(std::make_pair(before_node->getX(), before_node->getY()));
                    start_idx = i-1;
                    break;
                }
            }
            if(i == end_idx)
            {
                Node* goal_node = path[end_idx-1];
                processed_points.push_back(std::make_pair(goal_node->getX(), goal_node->getY()));
                break;
            }
        }
        return processed_points;
    }

    void visualize()
    {
        std::string canvas_name = "RRT algorithm";
        cv::Mat canvas = cv::Mat(400, 400, CV_8UC3, cv::Scalar(255, 255, 255));
        cv::Mat fliped_canvas = cv::Mat(400, 400, CV_8UC3, cv::Scalar(255, 255, 255));
        
        // map visualization (-1 for unknown, 100 for obstacle(black), 0 for freespace (grey))
        for(unsigned int i=0; i!=400; i++)
        {
            for (unsigned int j=0; j!=400; j++)
            {
                if (grid_->getCost(i,j) == 0)
                    cv::circle(canvas, cv::Point(i, j), 1, cv::Scalar(255,255,255), cv::FILLED, cv::LINE_8);
                else if (grid_->getCost(i,j) >= 0.65)
                    cv::circle(canvas, cv::Point(i, j), 0.1, cv::Scalar(0,0,0), cv::FILLED, cv::LINE_8);
            }
        }

        // RRT tree visualization
        for (const auto &node: visualize_node_list_)
        {
            float x1 = node->getX();
            float y1 = node->getY();

            // for root node
            if (node->getParent() == nullptr)
            {
                cv::circle(canvas, cv::Point(node->getY(), node->getX()), 2, cv::Scalar(177,127,29), cv::FILLED, cv::LINE_8);
                cv::flip(canvas, fliped_canvas, 0);
                cv::imshow(canvas_name, fliped_canvas);
                cv::waitKey(100);
                continue; 
            }
            float x2 = node->getParent()->getX();
            float y2 = node->getParent()->getY();
            cv::circle(canvas, cv::Point(x1, y1), 2, cv::Scalar(177,127,29), cv::FILLED, cv::LINE_8);
            cv::line(canvas, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(177,127,29), 1, cv::LINE_8);
            cv::flip(canvas, fliped_canvas, 0);
            cv::imshow(canvas_name, fliped_canvas);
            cv::waitKey(100);
        }

        // Shortest Path visualization
        if (success_)
        {
            std::vector<std::pair<float,float>> PathPoints = getPathPoints();
            float x1, y1, x2, y2;

            for (int i = 0; i < PathPoints.size(); i++)
            {
                if (i == 0)
                {
                    x1 = PathPoints[i].first;
                    y1 = PathPoints[i].second;
                    continue;
                }
                x2 = PathPoints[i].first;
                y2 = PathPoints[i].second;
                cv::line(canvas, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(255,0,0), 2, cv::LINE_8);
                x1 = x2;
                y1 = y2;
            }

            // smooth path
            std::vector<std::pair<float,float>> processed_points = postProcessPath();

            for (int i = 0; i < processed_points.size(); i++)
            {
                if (i == 0)
                {
                    x1 = processed_points[i].first;
                    y1 = processed_points[i].second;
                    continue;
                }
                x2 = processed_points[i].first;
                y2 = processed_points[i].second;
                cv::line(canvas, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(240,16,255), 2, cv::LINE_8);
                x1 = x2;
                y1 = y2;
            }

            cv::circle(canvas, cv::Point(start_x_, start_y_), 4, cv::Scalar(0,0,255), cv::FILLED, cv::LINE_8);
            cv::circle(canvas, cv::Point(goal_x_, goal_y_), 4, cv::Scalar(0,255,0), cv::FILLED, cv::LINE_8);
        }

        cv::flip(canvas, fliped_canvas, 0);
        cv::imshow(canvas_name, fliped_canvas);
        cv::imwrite("/home/kblee/Projects/planning_algorithms/rrt_connect/rrt_connect.png", fliped_canvas);
        cv::waitKey(0);
    }

    bool isSuccess()
    {
        return success_;
    }

private:
    // Parameters
    unsigned int max_sampling_;
    float step_length_;
    unsigned int termination_radius_;
    float bias_{0.2};

    Grid* grid_;
    Node* start_;
    Node* goal_;
    std::list<Node*> forward_node_list_;
    std::list<Node*> backward_node_list_;
    // Just for visualization
    std::list<Node*> visualize_node_list_;

    std::vector<Node*> path_nodes_;
    std::vector<std::pair<float,float>> path_points_;
    
    float start_x_, start_y_, goal_x_, goal_y_;
    bool success_;
};
}
#endif