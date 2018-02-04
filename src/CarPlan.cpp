//
// Created by vlad on 04.02.2018.
//


#include <math.h>
#include "spline.h"

#include "CarPlan.h"

using namespace std;



// For converting back and forth between radians and degrees.
static constexpr double pi() { return M_PI; }

static double deg2rad(double x) { return x * pi() / 180; }

static double rad2deg(double x) { return x * 180 / pi(); }


static double distance(double x1, double y1, double x2, double y2) {
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

static int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y) {

    double closestLen = 100000; //large number
    int closestWaypoint = 0;

    for (int i = 0; i < maps_x.size(); i++) {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = distance(x, y, map_x, map_y);
        if (dist < closestLen) {
            closestLen = dist;
            closestWaypoint = i;
        }

    }

    return closestWaypoint;

}

static int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y, vector<double> maps_dx,
                        vector<double> maps_dy) {

    int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];

    //heading vector
    double hx = map_x - x;
    double hy = map_y - y;

    //Normal vector:
    double nx = maps_dx[closestWaypoint];
    double ny = maps_dy[closestWaypoint];

    //Vector into the direction of the road (perpendicular to the normal vector)
    double vx = -ny;
    double vy = nx;

    //If the inner product of v and h is positive then we are behind the waypoint so we do not need to
    //increment closestWaypoint, otherwise we are beyond the waypoint and we need to increment closestWaypoint.

    double inner = hx * vx + hy * vy;
    if (inner < 0.0) {
        closestWaypoint++;
    }

    return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
static vector<double>
getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y, vector<double> maps_dx,
          vector<double> maps_dy) {
    int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y, maps_dx, maps_dy);

    int prev_wp;
    prev_wp = next_wp - 1;
    if (next_wp == 0) {
        prev_wp = maps_x.size() - 1;
    }

    double n_x = maps_x[next_wp] - maps_x[prev_wp];
    double n_y = maps_y[next_wp] - maps_y[prev_wp];
    double x_x = x - maps_x[prev_wp];
    double x_y = y - maps_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
    double proj_x = proj_norm * n_x;
    double proj_y = proj_norm * n_y;

    double frenet_d = distance(x_x, x_y, proj_x, proj_y);

    //see if d value is positive or negative by comparing it to a center point

    double center_x = 1000 - maps_x[prev_wp];
    double center_y = 2000 - maps_y[prev_wp];
    double centerToPos = distance(center_x, center_y, x_x, x_y);
    double centerToRef = distance(center_x, center_y, proj_x, proj_y);

    if (centerToPos <= centerToRef) {
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for (int i = 0; i < prev_wp; i++) {
        frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
    }

    frenet_s += distance(0, 0, proj_x, proj_y);

    return {frenet_s, frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
static vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y) {
    int prev_wp = -1;

    while (s > maps_s[prev_wp + 1] && (prev_wp < (int) (maps_s.size() - 1))) {
        prev_wp++;
    }

    int wp2 = (prev_wp + 1) % maps_x.size();

    double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s - maps_s[prev_wp]);

    double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
    double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

    double perp_heading = heading - pi() / 2;

    double x = seg_x + d * cos(perp_heading);
    double y = seg_y + d * sin(perp_heading);

    return {x, y};

}




CarPlan::CarPlan(const Map &map_, int lane_, int lane_change_wp_)
        : map(map_), lane(lane_), lane_change_wp(lane_change_wp_) {


};

Path CarPlan::getOptimalPath(State car_state, std::vector<double> previous_path_x, std::vector<double> previous_path_y,
                             double end_path_s, double end_path_d,
                             std::vector<std::vector<double>> sensor_fusion) {


    double ref_vel = Safety::TARGET_SPEED_MPH;

    int prev_size = previous_path_x.size();

    int next_wp = -1;
    double ref_x = car_state.x;
    double ref_y = car_state.y;
    double ref_yaw = deg2rad(car_state.yaw);

    if (prev_size < 2) {
        next_wp = NextWaypoint(ref_x, ref_y, ref_yaw, map.map_waypoints_x,
                               map.map_waypoints_y,
                               map.map_waypoints_dx,
                               map.map_waypoints_dy);
    } else {
        ref_x = previous_path_x[prev_size - 1];
        double ref_x_prev = previous_path_x[prev_size - 2];
        ref_y = previous_path_y[prev_size - 1];
        double ref_y_prev = previous_path_y[prev_size - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
        next_wp = NextWaypoint(ref_x, ref_y, ref_yaw,
                               map.map_waypoints_x,
                               map.map_waypoints_y,
                               map.map_waypoints_dx,
                               map.map_waypoints_dy);

        car_state.s = end_path_s;

        car_state.speed =
                (sqrt((ref_x - ref_x_prev) * (ref_x - ref_x_prev) + (ref_y - ref_y_prev) * (ref_y - ref_y_prev)) /
                 .02) * 2.237;
    }

    //find ref_v to use
    double closestDist_s = 100000;
    bool change_lanes = false;
    for (int i = 0; i < sensor_fusion.size(); i++) {
        //car is in my lane
        float d = sensor_fusion[i][6];
        if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2)) {
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx * vx + vy * vy);
            double check_car_s = sensor_fusion[i][5];
            check_car_s += ((double) prev_size * .02 * check_speed);
            //check s values greater than mine and s gap
            if ((check_car_s > car_state.s) && ((check_car_s - car_state.s) < 30) &&
                ((check_car_s - car_state.s) < closestDist_s)) {

                closestDist_s = (check_car_s - car_state.s);

                if ((check_car_s - car_state.s) > 20) {

                    //match that cars speed
                    ref_vel = check_speed * 2.237;
                    change_lanes = true;
                } else {
                    //go slightly slower than the cars speed
                    ref_vel = check_speed * 2.237 - 5;
                    change_lanes = true;

                }
            }


        }
    }

    //try to change lanes if too close to car in front
    if (change_lanes && ((next_wp - lane_change_wp) % map.map_waypoints_x.size() > 2)) {
        bool changed_lanes = false;
        //first try to change to left lane
        if (lane != 0 && !changed_lanes) {
            bool lane_safe = true;
            for (int i = 0; i < sensor_fusion.size(); i++) {
                //car is in left lane
                float d = sensor_fusion[i][6];
                if (d < (2 + 4 * (lane - 1) + 2) && d > (2 + 4 * (lane - 1) - 2)) {
                    double vx = sensor_fusion[i][3];
                    double vy = sensor_fusion[i][4];
                    double check_speed = sqrt(vx * vx + vy * vy);

                    double check_car_s = sensor_fusion[i][5];
                    check_car_s += ((double) prev_size * .02 * check_speed);
                    double dist_s = check_car_s - car_state.s;
                    if (dist_s < 20 && dist_s > -20) {
                        lane_safe = false;
                    }
                }
            }
            if (lane_safe) {
                changed_lanes = true;
                lane -= 1;
                lane_change_wp = next_wp;
            }
        }
        //next try to change to right lane
        if (lane != 2 && !changed_lanes) {
            bool lane_safe = true;
            for (int i = 0; i < sensor_fusion.size(); i++) {
                //car is in right lane
                float d = sensor_fusion[i][6];
                if (d < (2 + 4 * (lane + 1) + 2) && d > (2 + 4 * (lane + 1) - 2)) {
                    double vx = sensor_fusion[i][3];
                    double vy = sensor_fusion[i][4];
                    double check_speed = sqrt(vx * vx + vy * vy);

                    double check_car_s = sensor_fusion[i][5];
                    check_car_s += ((double) prev_size * .02 * check_speed);
                    double dist_s = check_car_s - car_state.s;
                    if (dist_s < 20 && dist_s > -10) {
                        lane_safe = false;
                    }
                }
            }
            if (lane_safe) {
                changed_lanes = true;
                lane += 1;
                lane_change_wp = next_wp;
            }

        }

    }


    vector<double> ptsx;
    vector<double> ptsy;

    if (prev_size < 2) {
        double prev_car_x = car_state.x - cos(car_state.yaw);
        double prev_car_y = car_state.y - sin(car_state.yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(car_state.x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(car_state.y);
    } else {
        ptsx.push_back(previous_path_x[prev_size - 2]);
        ptsx.push_back(previous_path_x[prev_size - 1]);

        ptsy.push_back(previous_path_y[prev_size - 2]);
        ptsy.push_back(previous_path_y[prev_size - 1]);


    }

    vector<double> next_wp0 = getXY(car_state.s + 30, (2 + 4 * lane), map.map_waypoints_s, map.map_waypoints_x,
                                    map.map_waypoints_y);
    vector<double> next_wp1 = getXY(car_state.s + 60, (2 + 4 * lane), map.map_waypoints_s, map.map_waypoints_x,
                                    map.map_waypoints_y);
    vector<double> next_wp2 = getXY(car_state.s + 90, (2 + 4 * lane), map.map_waypoints_s, map.map_waypoints_x,
                                    map.map_waypoints_y);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);


    for (int i = 0; i < ptsx.size(); i++) {

        //shift car reference angle to 0 degrees
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;

        ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
        ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));

    }


    tk::spline s;


    s.set_points(ptsx, ptsy);

    vector<double> next_x_vals;
    vector<double> next_y_vals;

    for (int i = 0; i < previous_path_x.size(); i++) {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));

    double x_add_on = 0;

    for (int i = 1; i <= 50 - previous_path_x.size(); i++) {

        if (ref_vel > car_state.speed) {
            car_state.speed += .224;
        } else if (ref_vel < car_state.speed) {
            car_state.speed -= .224;
        }


        double N = (target_dist / (.02 * car_state.speed / 2.24));
        double x_point = x_add_on + (target_x) / N;
        double y_point = s(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

        x_point += ref_x;
        y_point += ref_y;


        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);

    }


    Path path(next_x_vals, next_y_vals);
    return path;
}