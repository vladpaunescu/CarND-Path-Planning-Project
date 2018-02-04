//
// Created by vlad on 04.02.2018.
//

#ifndef PATH_PLANNING_CONTROL_H
#define PATH_PLANNING_CONTROL_H

#include <vector>


/**
 * safety constants.
 */
struct Safety {
    static constexpr double TARGET_SPEED_MPH = 49.5;
    static const int TARGET_LANE = 1;

};

enum  {
    KEEP_LANE = 0,
    LANE_CHANGE
};


struct State {
    int id;

    int lane;
    double x;
    double y;
//    double vx, vy;
    double s;
    double d;
    double yaw;
    double speed;

    State(int id_, double car_x, double car_y,
          double car_s, double car_d, double car_yaw, double car_speed)
    : id(id_), x(car_x), y(car_y), s(car_s), d(car_d), yaw(car_yaw), speed(car_speed)
            {}

};

struct Path {
    std::vector<double> next_x_vals;
    std::vector<double> next_y_vals;

    Path(std::vector<double> x_v,  std::vector<double> y_v)
        : next_x_vals(x_v), next_y_vals(y_v)
    {}
};

struct Map {

    double max_s;

    std::vector<double> map_waypoints_x;
    std::vector<double> map_waypoints_y;
    std::vector<double> map_waypoints_s;
    std::vector<double> map_waypoints_dx;
    std::vector<double> map_waypoints_dy;

    Map(double max_s_,
        std::vector<double> map_waypoints_x_,
        std::vector<double> map_waypoints_y_,
        std::vector<double> map_waypoints_s_,
        std::vector<double> map_waypoints_dx_,
        std::vector<double> map_waypoints_dy_
    ) : max_s(max_s_), map_waypoints_x(map_waypoints_x_),
        map_waypoints_y(map_waypoints_y_), map_waypoints_s(map_waypoints_s_),
        map_waypoints_dx(map_waypoints_dx_), map_waypoints_dy(map_waypoints_dy_)
        {  }
};

/**
 * Module responsible for car planning
 */
class CarPlan {

    Map map;
    int lane;
    int lane_change_wp;


public:

    CarPlan(const Map& map, int lane, int lane_change_wp_);


    Path getOptimalPath(State car_state,
            std::vector<double> previous_path_x,
                        std::vector<double> previous_path_y,
                        double end_path_s, double end_path_d,
                        std::vector<std::vector<double>> sensor_fusion);

public:




};


#endif //PATH_PLANNING_CONTROL_H
