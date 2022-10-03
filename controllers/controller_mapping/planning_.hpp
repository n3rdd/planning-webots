#include <queue>
#include <vector>
#include <utility>
#include <cmath>
#include <iostream>

using std::priority_queue, std::vector, std::pair;
using std::cout, std::endl;

const double MAX_COST = 9999.;

class Node {
public:
    int x_, y_;
    double cost_;
    Node* parent_;

    Node(int x, int y, double cost=MAX_COST): x_(x), y_(y) {
        this->cost_ = cost;
        this->parent_ = nullptr;
    }

    void setCost(double new_cost) {
        cost_ = new_cost;
    }

    bool operator> (const Node& node2) const {
        return this->cost_ > node2.cost_;
    }
};

class AstarPlanner {
public:
    const int width_, height_;
    priority_queue<Node, vector<Node>, std::greater<Node> > pq_;
    vector<vector<int> > occupancy_grid_map_;

    AstarPlanner(vector<vector<int> > occupancy_grid_map, int width, int height): 
        occupancy_grid_map_(occupancy_grid_map), width_(width), height_(height) {}
    
    // vector<Node> plan(Node start, Node goal);
    vector<Node> plan(int sx, int sy, int gx, int gy);
    vector<Node> getNeighbors(Node node);
    double getCost(Node node1, Node node2);
    double getHeuristic(Node node, Node goal);
};

double AstarPlanner::getCost(Node node1, Node node2) {
    if (occupancy_grid_map_[node1.x_][node1.y_] == 1 || occupancy_grid_map_[node2.x_][node2.y_] == 1) {
        return MAX_COST;
    }
    return sqrt(pow(node1.x_ - node2.x_, 2) + pow(node1.y_ - node2.y_, 2));
}

double AstarPlanner::getHeuristic(Node node, Node goal) {
    return sqrt(pow(node.x_ - goal.x_, 2) + pow(node.y_ - goal.y_, 2));

}

vector<Node> AstarPlanner::getNeighbors(Node node) {
    vector<Node> neighbors;
    vector<pair<int, int> > directions {
        pair<int, int> (1, 0),
        pair<int, int> (-1, 0),
        pair<int, int> (0, 1),
        pair<int, int> (0, -1)
    };

    for (auto direction: directions) {
        int nx = node.x_ + direction.first;
        int ny = node.y_ + direction.second;
        if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
            neighbors.push_back(Node(nx, ny));
        }
    }

    return neighbors;

}

// vector<Node> AstarPlanner::plan(Node start, Node goal) {
vector<Node> AstarPlanner::plan(int sx, int sy, int gx, int gy) {
    cout << "planning starts..." << endl;
    Node goal(gx, gy);
    Node start(sx, sy);
    start.cost_ = 0.;
    
    // const double start_cost = 1;
    // start.cost_ = 0.;
    cout << "start cost: " << start.cost_ << endl;
    cout << "goal cost: " << goal.cost_ << endl;
    // start.setCost(0.);
    pq_.push(start);
    Node node (-1, -1);  // 注意 neighbor.parent_ = &node;
    vector<Node> neighbors;
    double new_cost = 0.;
    while (!pq_.empty()) {
        node = pq_.top();
        pq_.pop();
        cout << "at node: " << node.x_ << " " << node.y_ << endl;
        if (node.x_ == goal.x_ && node.y_ == goal.y_) {
            break;
        }
        neighbors = getNeighbors(node);
        for (Node neighbor: neighbors) {
            cout << "  at neighbor: " << neighbor.x_ << " " << neighbor.y_ << endl;
            new_cost = getCost(neighbor, node) + node.cost_ + getHeuristic(neighbor, goal);
            cout << "  new_cost: " << new_cost << endl;
            cout << "  old cost: " << neighbor.cost_ << endl;
            // 此处有不明 bug 调不动.. 一旦改动 cost 该函数都不运行了
            if (new_cost < neighbor.cost_) {
                neighbor.cost_ = new_cost;
                neighbor.parent_ = &node;
                cout << "  modified-> " << neighbor.cost_ << endl;
                // push neighbor.neighbors to pq
                pq_.push(neighbor);
            }
        }
    }
    // // get the path
    vector<Node> path;
    // Node* p_node = &goal;
    // while (!p_node) {
    //     path.push_back(*p_node);
    //     p_node = p_node->parent_;
    // }
    return path;
}

