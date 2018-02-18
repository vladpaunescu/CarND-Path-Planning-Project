//
// Created by vlad on 04.02.2018.
//

#ifndef PATH_PLANNING_CONTROL_H
#define PATH_PLANNING_CONTROL_H

#include <vector>
#include "spline.h"


/**
 * safety constants.
 */
struct Safety {
    static constexpr double TARGET_SPEED_MPH = 49.5;
    static const int TARGET_LANE = 1;
    static const int LANE_WIDTH = 4;
    static constexpr double S_AHEAD_INCREMENT = 30.0;
    static constexpr double X_AHEAD_TARGET_METERS = 30.0;
    static const int CONTROL_WP_COUNT = 3;
    static const int PATH_WP_COUNT = 50;

    static constexpr double ACCELERATION = .224;
    static constexpr double SAMPLING_RATE_SECONDS = .02;
    static constexpr double MPH_TO_METERS_PER_SEC = 2.24;

};

enum {
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

    State() {}

    State(const State &st) :
            id(st.id),
            x(st.x),
            y(st.y),
            s(st.s),
            d(st.d),
            yaw(st.yaw),
            speed(st.speed) {}

    State &operator=(const State &other) {
        // check for self-assignment
        if (&other == this)
            return *this;

        id = other.id;
        x = other.x;
        y = other.y;
        s = other.s;
        d = other.d;
        yaw = other.yaw;
        speed = other.speed;

        return *this;
    }

    State(int id_, double car_x, double car_y,
          double car_s, double car_d, double car_yaw, double car_speed)
            : id(id_), x(car_x), y(car_y), s(car_s), d(car_d), yaw(car_yaw), speed(car_speed) {}

};


/* should respect command query separation pattern */
struct Path {

    std::vector<double> control_ptsx;
    std::vector<double> control_ptsy;
    bool initialized;
    tk::spline spl;

    std::vector<double> discretex;
    std::vector<double> discretey;

    State car_state;


private:
    std::vector<double> toWorldCoordinates(double x_, double y_) {
        double x_ref = x_;
        double y_ref = y_;
        std::vector<double> point;

        x_ = (x_ref * cos(car_state.yaw) - y_ref * sin(car_state.yaw));
        y_ = (x_ref * sin(car_state.yaw) + y_ref * cos(car_state.yaw));

        x_ += car_state.x;
        y_ += car_state.y;

        point.push_back(x_);
        point.push_back(y_);

        return point;
    }

    void toLocalCarCoordinates() {
        for (int i = 0; i < control_ptsx.size(); i++) {
            //shift to local car coordinates and rotate

            double shift_x = control_ptsx[i] - car_state.x;
            double shift_y = control_ptsy[i] - car_state.y;

            // align to car heading
            control_ptsx[i] = (shift_x * cos(0 - car_state.yaw) - shift_y * sin(0 - car_state.yaw));
            control_ptsy[i] = (shift_x * sin(0 - car_state.yaw) + shift_y * cos(0 - car_state.yaw));
        }
    }

    void adjustCarSpeed(double ref_vel) {
        if (ref_vel > car_state.speed) {
            car_state.speed += Safety::ACCELERATION;
        } else if (ref_vel < car_state.speed) {
            car_state.speed -= Safety::ACCELERATION;
        }
    }

public:


    Path() {
        initialized = false;
    }

    Path(std::vector<double> x_v, std::vector<double> y_v)
            : control_ptsx(x_v), control_ptsy(y_v), initialized(false) {}

    void addControlPoint(double x, double y) {
        control_ptsx.push_back(x);
        control_ptsy.push_back(y);
    }

    void addControlPoint(std::vector<double> point) {
        addControlPoint(point[0], point[1]);
    }

    void computeTrajectory() {
        toLocalCarCoordinates();
        spl.set_points(control_ptsx, control_ptsy);
        initialized = true;
    }

    void connectPrevPath(const Path &prevPath) {
        discretex = prevPath.discretex;
        discretey = prevPath.discretey;
//
//        for (int i = 0; i < prevPath.discretex.size(); i++) {
//            discretex.push_back(discretex[i]);
//            discretey.push_back(discretey[i]);
//        }
    }

    void setCarState(const State &car_state) {
        this->car_state = car_state;
    }


    void discretizePath(const Path &prevPath, double ref_vel) {
        connectPrevPath(prevPath);

        double target_x = Safety::X_AHEAD_TARGET_METERS;
        double target_y = spl(target_x);
        double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));

        double x_add_on = 0;
//        double ref_vel = Safety::TARGET_SPEED_MPH;

        // add only a difference from previous path
        int remainingWpCount = Safety::PATH_WP_COUNT - discretex.size();

        for (int i = 1; i <= remainingWpCount; i++) {

            adjustCarSpeed(ref_vel);

            double N = (target_dist / (Safety::SAMPLING_RATE_SECONDS * car_state.speed / Safety::MPH_TO_METERS_PER_SEC));
            double x_point = x_add_on + (target_x) / N;
            double y_point = spl(x_point);


            std::vector<double> point = toWorldCoordinates(x_point, y_point);

            discretex.push_back(point[0]);
            discretey.push_back(point[1]);

            x_add_on = x_point;
        }
    }

    double getValueForX(double target_x) {

        if (!initialized) {
            /* lazy initialization */
            computeTrajectory();
        }

        return spl(target_x);
    }




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
        map_waypoints_dx(map_waypoints_dx_), map_waypoints_dy(map_waypoints_dy_) {}
};

/**
 * Module responsible for car planning
 */
class CarPlan {

    Map map;
    int lane;
    int lane_change_wp;


public:

    CarPlan(const Map &map, int lane, int lane_change_wp_);


    Path getOptimalPath(State car_state,
                        std::vector<double> previous_path_x,
                        std::vector<double> previous_path_y,
                        double end_path_s, double end_path_d,
                        std::vector<std::vector<double>> sensor_fusion);

private:


    // static utility functions
    static bool carInSameLane(int lane, float d);

    static double getLaneMiddleD(int lane);


};


#endif //PATH_PLANNING_CONTROL_H
