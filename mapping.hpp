// #include <cstring>
#include <cmath>
#include <utility>
#include <vector>

using std::cin;
using std::cout;
using std::endl;
using std::vector;

const double pi = 3.14;
const double eps = 1e-6;

int sign(double x)
{
    if (x > eps)
        return 1;
    if (x < -eps)
        return -1;
    return 0;
}

class Mapping {
public:
    // int **occupancy_count_;
    // int **occupancy_grid_map_;
    vector<vector<int>> occupancy_count_;
    vector<vector<int>> occupancy_grid_map_;

    const int width_, height_;
    // double st;
    const float floor_width_, floor_height_;
    const double lidar_max_range_;
    const int horizontal_resolution_;

    double unit_;  // 相邻激光射线的角度

    const int occupied_thresh_ = 1;

    Mapping(int horizontal_resolution, 
            double lidar_max_range, 
            int height,
            int width,
            double floor_height,
            double floor_width):

            horizontal_resolution_(horizontal_resolution),
            lidar_max_range_(lidar_max_range),
            height_(height),
            width_(width),
            floor_height_(floor_height),
            floor_width_(floor_width) {
        cout << "init Mapping instance..." << endl;
        cout << "horizontal res: " << horizontal_resolution_ << endl;
        unit_ = 2*pi / horizontal_resolution_;
        cout << "unit: " << unit_ << endl;
        cout << "height: " << height_ << endl;
        cout << "width: " << width_ << endl;
        
        occupancy_count_ = vector<vector<int> >(height_, vector<int>(width, 0));
        occupancy_grid_map_ = vector<vector<int> >(height_, vector<int>(width, 0));
        cout << "ogm init done.. " << endl;
        
        // for (int i=0; i < height_; ++i) {
            
        //     occupancy_count_[i] = new int[width_];

        //     cout << "occupancy count row init done" << endl;
        //     memset(occupancy_count_[i], 0, sizeof(int)*width_);
        //     cout << "occupancy count col init done" << endl;
            
        //     occupancy_grid_map_[i] = new int[width_];
        //     memset(occupancy_grid_map_[i], 0, sizeof(int) * width_);
        // }
    }

    ~Mapping() {
        // for (int i = 0; i < width_; ++i) {
        //     delete[] occupancy_count_[i];
        //     delete[] occupancy_grid_map_[i];
        // }
        // delete[] occupancy_count_;
        // delete[] occupancy_grid_map_;
    }

    std::pair<int, int> worldToMap(double x, double y);
    void updateOccupancyCount(const float* range_image, double x, double y);
    void updateMap();
};

std::pair<int, int> Mapping::worldToMap(double x, double y) {
    double ogm_resolution = width_ / floor_width_;
    double nx = std::ceil(x * ogm_resolution);
    double ny = std::ceil(y * ogm_resolution);
    // cout << "nx, ny: " << nx << " " << ny << endl;
    return std::make_pair(nx, ny);
}

void Mapping::updateOccupancyCount(const float* range_image, double x, double y) {
    
    for (int i = 0; i < horizontal_resolution_; ++i) {
        if (sign(range_image[i] - lidar_max_range_) == 0) continue;
        auto [nx, ny] = worldToMap(x + range_image[i] * cos(-i * unit_),
                                    y + range_image[i] * sin(-i * unit_));
        if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
            // cout << "nx, ny: " << nx << " " << ny << endl;
            occupancy_count_[ny][nx] += 1;
        }
    }
}

void Mapping::updateMap() {
    cout << "updating map.." << endl;
    for (int i = 0; i < height_; ++i) {
        for (int j = 0; j < width_; ++j) {
            if (occupancy_count_[i][j] >= occupied_thresh_) {
                cout << i << " " << j << "occupied" << endl;
                occupancy_grid_map_[i][j] = 1;
            }
        }
    }
    // for (int i = 0; i < height_; ++i) {
    //     memset(occupancy_count_[i], 0, sizeof(int) * width_);
    // }
}


