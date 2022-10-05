#include <queue>
#include <vector>
#include <utility>
#include <cmath>
#include <iostream>

using std::priority_queue, std::vector, std::pair;
using std::cout, std::endl;

const double MAX_COST = 999999.;
const double START_COST = 56.57;

class Node {
public:
    int x_;
    int y_;
    double cost_;
    // Node* parent_;

    Node(int x, int y, double cost): x_(x), y_(y) {
        cost_ = cost;
        // this->parent_ = parent;
    }

    bool operator> (const Node& node2) const {
        return cost_ > node2.cost_;
    }
};

class AstarPlanner {
public:
    const int width_, height_;
    priority_queue<Node, vector<Node>, std::greater<Node> > pq_;
    vector<vector<int> > occupancy_grid_map_;
    vector<vector<pair<int, int> > > parents_;
    vector<vector<double> > costs_;

    AstarPlanner(vector<vector<int> > occupancy_grid_map, int width, int height): 
        occupancy_grid_map_(occupancy_grid_map), width_(width), height_(height) {
            cout << "init AstarPlanner instance..." << endl;
            parents_ = vector<vector<pair<int, int> > >(
                width_, vector<pair<int, int> >(height_, pair<int, int>(-1, -1))
            );
            costs_ = vector<vector<double> >(
                width_, vector<double>(height_, MAX_COST)
            );
            cout << "init done ..." << endl;
        }
    
    // vector<Node> plan(Node start, Node goal);
    vector<pair<int, int> > plan(int sx, int sy, int gx, int gy);
    vector<Node> getNeighbors(Node node);
    double getCost(Node node1, Node node2);
    double getHeuristic(Node node, Node goal);
};

double AstarPlanner::getCost(Node node1, Node node2) {
    if (occupancy_grid_map_[node1.y_][node1.x_] == 1 || occupancy_grid_map_[node2.y_][node2.x_] == 1) {
        cout << node1.x_ << " " << node1.y_ << " " << occupancy_grid_map_[node1.y_][node1.x_] << endl;
        cout << node2.x_ << " " << node2.y_ << " " << occupancy_grid_map_[node2.y_][node2.x_] << endl;
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
            neighbors.push_back(Node(nx, ny, costs_[ny][nx]));
        }
    }

    return neighbors;

}

// vector<Node> AstarPlanner::plan(Node start, Node goal) {
vector<pair<int, int> > AstarPlanner::plan(int sx, int sy, int gx, int gy) {
    cout << "planning starts..." << endl;
    Node goal(gx, gy, MAX_COST);
    Node start(sx, sy, START_COST);
    costs_[sy][sx] = START_COST;
    
    cout << sx << " " << sy << endl;
    cout << gx << " " << gy << endl;
    cout << "start cost: " << start.cost_ << endl;
    cout << "goal cost: " << goal.cost_ << endl;
    // // start.setCost(0.);
    pq_.push(start);
    // pq_.push(Node(sx, sy, START_COST));
    // // Node node (-1, -1);  // 注意 neighbor.parent_ = &node;
    while (!pq_.empty()) {
        Node node = pq_.top();
        pq_.pop();
        cout << "at node: " << node.x_ << " " << node.y_ << endl;
        if (node.x_ == gx && node.y_ == gy) {
            break;
        }
        vector<Node> neighbors = getNeighbors(node);
        for (Node neighbor: neighbors) {
            cout << "  at neighbor: " << neighbor.x_ << " " << neighbor.y_ << endl;
            double new_cost = getCost(neighbor, node) + costs_[node.y_][node.x_];
            // cout << "  new_cost: " << new_cost << endl;
            // cout << "  old cost: " << neighbor.cost_ << endl;
            if (new_cost < costs_[neighbor.y_][neighbor.x_]) {
    //             // neighbor.cost_ = new_cost;
    //             // neighbor.parent_ = &node;
                parents_[neighbor.y_][neighbor.x_] = pair<int, int>(node.x_, node.y_);
                costs_[neighbor.y_][neighbor.x_] = new_cost;
                // cout << "  modified-> " << new_cost << endl;
    //             // push neighbor.neighbors to pq
                pq_.push(Node(neighbor.x_, neighbor.y_, new_cost + getHeuristic(neighbor, goal)));
            }
        }
    }
    // // // get the path
    vector<pair<int, int> > path;
    pair<int, int> next_pos (gx, gy);
    // // Node* p_node = &goal;
    // // while (!p_node) {
    // //     path.push_back(*p_node);
    // //     p_node = p_node->parent_;
    // // }
    cout << "printing the path.." << endl;
    while (true) {
        cout << next_pos.first << ", " << next_pos.second << endl;
        if (next_pos.first == sx && next_pos.second == sy) {
            break;
        }
        next_pos = parents_[next_pos.second][next_pos.first];
        path.push_back(next_pos);
    }
    return path;
}

