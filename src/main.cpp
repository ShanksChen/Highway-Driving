#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "spline.h"
#include "json.hpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using namespace std;

// define the speed limit, saftey distance, maximum acceleration and velocity of car
double speed_limit = 49.0;
double safety_distance = 30.0;
double max_acceleration = 0.224;
double velocity = 0.0;

// define the functions
// behavior planner function return the target points in Frenet s,d coordinates and the speed of the car
vector<double> behavior_planner(vector<vector<double>> sensor_fusion, double car_s, double car_d);
// generate the trajectory to the target point
vector<vector<double>> generate_trajectory(vector<double> goal, double car_x, double car_y, double car_yaw,
                                            vector<double> previous_path_x, vector<double> previous_path_y,
                                            vector<double> map_waypoints_x, vector<double> map_waypoints_y,
                                            vector<double> map_waypoints_s);
// use the car_d parameter to detecte the lane where the car in
int detect_lane(double car_d);
// decide to lane change left, lane change right or keep lane
vector<double> change_lane(vector<vector<double>> car_from_left, vector<vector<double>> car_from_right,
                            double car_s, double car_d, double front_car_speed);
// check the lane is safe to change or not
bool check_lane_safe(vector<vector<double>> sensor_fusion, double car_s, double car_d);
// set the target points in Frenet s,d coordinates and the speed of the car
vector<double> set_goal(double s, double d, double speed);
// update the speed of the car
double change_speed(double speed_to_match = -1);
// choose the best lane to change when the car can change to right and left both
int best_lane_to_change(vector<vector<double>> car_from_left, vector<vector<double>> car_from_right, double car_s);

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }


  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          // cout << sensor_fusion <<endl;

          // DEBUG
					// cout << endl << "**************** ITERATION BEGIN ****************" << endl << endl;

          // set the goal point
          vector<double> goal = behavior_planner(sensor_fusion, car_s, car_d);
          /** 
          vector<double>::iterator it;
          cout<<" goal element(s,d,speed) = ";
          for(it = goal.begin();it!=goal.end();it++) {
            cout<<*it<<' ';
          }
          cout<<endl;
          **/

          // generate the trajectory to the goal
          vector<vector<double>> next_vals = generate_trajectory(goal, car_x, car_y, car_yaw, previous_path_x, previous_path_y, map_waypoints_x, map_waypoints_y, map_waypoints_s);

          json msgJson;

          msgJson["next_x"] = next_vals[0];
          msgJson["next_y"] = next_vals[1];

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}

vector<double> behavior_planner(vector<vector<double>> sensor_fusion, double car_s, double car_d){
  // cout<<"~~~~~~~~~~~~~~behavior planner start~~~~~~~~~~~~"<<endl;
  bool should_change_lane = false; // flag for lan change
  double front_car_speed; // record the front car speed
  double front_car_s = INFINITY; // record the front car s coordinate
  vector<vector<double>> car_from_left; //record the car data on the left lane
  vector<vector<double>> car_from_right; // record the car data on the right lane

  // find out car on which lane
  int lane = detect_lane(car_d);

  // cout << "car lane number ========" << lane << endl;
  // cout << "===================== loop start ================" << endl;

  // according to the sensor fusion data to decide the behavior
  for(int i = 0; i < sensor_fusion.size(); ++i) {
    // record the detected car data from sensor fusion
    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    double s = sensor_fusion[i][5];
    double d = sensor_fusion[i][6];

    double speed = sqrt(vx * vx + vy * vy);
    s += (double) speed * 0.02;

    // find out the car lane 
    int car_lane = detect_lane(d);

    // check the car lane
    bool is_same_lane = lane == car_lane;
    bool from_left = lane - 1 == car_lane;
    bool from_right = lane + 1 == car_lane;

    // is it close from our car?
    bool is_close = (s > car_s) && (s - car_s < safety_distance);

    // to change lane if the car is close from our car in the same lane
    if(is_same_lane && is_close) {
      should_change_lane = true;
      if(s < front_car_s){
        front_car_speed = speed;
        front_car_s = s;
      }
    }
    // save the data of the car on the left lane
    else if(from_left) {
      car_from_left.push_back(sensor_fusion[i]);
    }
    // save the data of the car on the right lane 
    else if(from_right) {
      car_from_right.push_back(sensor_fusion[i]);
    }
  }
  // cout << "===================== loop ended ================" << endl;
  // try to change lane, or slow down
  if(should_change_lane) {
    return change_lane(car_from_left, car_from_right, car_s, car_d, front_car_speed);
  }
  // keep lane 
  else {
    return set_goal(car_s + safety_distance, 2 + 4 * lane, change_speed());
  }
}

int detect_lane(double car_d) {
  if (car_d < 4) {
    return 0;
  } else if (car_d >= 4 && car_d < 8) {
    return 1;
  } else {
    return 2;
  }
}

vector<double> change_lane(vector<vector<double>> car_from_left, vector<vector<double>> car_from_right, double car_s, double car_d, double front_car_speed) {
  // detect current lane
  int lane = detect_lane(car_d);

  // check right or left lane is safe or not
  bool is_left_lane_safe = check_lane_safe(car_from_left, car_s, car_d);
  bool is_right_lane_safe = check_lane_safe(car_from_right, car_s, car_d);

  // our car on the lane 0 can't change to left lane and right lane is safe to go
  if((lane == 0 || (lane == 1 && !is_left_lane_safe)) && is_right_lane_safe) {
    return set_goal(car_s + 1.5 * safety_distance, 2 + 4 * (lane + 1), change_speed());
  } 
  // our car on the lane 1 and both right and left lane all can go, calculate the best lane to change
  else if (lane == 1 && is_right_lane_safe && is_left_lane_safe) {
    int best_lane = best_lane_to_change(car_from_left, car_from_right, car_s);
    return set_goal(car_s + 1.5 * safety_distance, 2 + 4 * best_lane, change_speed());
  } 
  // our car on the lane 2 can't change to right lane and left lane is safe to go
  else if((lane == 2 || (lane == 1 && !is_right_lane_safe)) && is_left_lane_safe) {
    return set_goal(car_s + 1.5 *  safety_distance, 2 + 4 * (lane - 1), change_speed());
  } 
  // slow dowm
  else {
    return set_goal(car_s + safety_distance, 2 + 4 * lane, change_speed(front_car_speed));
  }
}

bool check_lane_safe(vector<vector<double>> sensor_fusion, double car_s, double car_d){
  // check all cars from the lane
  for (int i = 0; i < sensor_fusion.size(); ++i) {
    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    double s = sensor_fusion[i][5];

    double speed = sqrt(vx * vx + vy * vy);
    s += (double) speed * 0.02;

    // if the car is in front of our car, false
    if((s >= car_s) && (s - car_s <= 2 * safety_distance)) {
      return false;
    }

    // if the cars behind our car, check the safe distance
    double min_distance = 0.25 * safety_distance;
    double max_distance = safety_distance;
    double distance = (1 - velocity/speed_limit) * max_distance + min_distance;
    if((s <= car_s) && (car_s - s <= distance)) {
      return false;
    }
  }
  return true;
}

vector<double> set_goal(double s, double d, double speed) {
  vector<double> goal;
  goal.push_back(s);
  goal.push_back(d);
  goal.push_back(speed);
  return goal;
}

double change_speed(double speed_to_match) {
  // speed up use the maximum acceleration
  double speed = velocity + max_acceleration;
  // if there a speed need to match
  if(speed_to_match != -1) {
    speed_to_match *= 2.24 * 0.95; // keep safe
    // slow down to match the speed
    if(max_acceleration <= velocity) {
      speed = max(speed_to_match, velocity - 1.5 * max_acceleration);
    }
    // acceleration to match the speed
    else {
      speed = min(speed_to_match, velocity + max_acceleration);
    }
  }
  // guarantee the speed doesn't exceed the speed limit
  velocity = min(speed_limit, speed);

  return velocity;
}

int best_lane_to_change(vector<vector<double>> car_from_left, vector<vector<double>> car_from_right, double car_s) {
  double closest_right_distance = INFINITY;
  double closest_left_distance = INFINITY;

  // find the closest car on the left lane
  for(int i = 0; i < car_from_left.size(); ++i) {
    double vx = car_from_left[i][3];
    double vy = car_from_left[i][4];
    double s = car_from_left[i][5];

    double speed = sqrt(vx * vx + vy * vy);
    s += (double) speed * 0.02;

    if((s > car_s) && (s - car_s < closest_left_distance)) {
      closest_left_distance = s - car_s;
    }
  }

  // find the closest car on the right lane
  for (int i = 0; i < car_from_right.size(); ++i) {
    double vx = car_from_right[i][3];
    double vy = car_from_right[i][4];
    double s = car_from_right[i][5];

    double speed = sqrt(vx*vx + vy*vy);
    s += (double) speed * .02;

    if ((s > car_s) && (s - car_s < closest_right_distance)) {
      closest_right_distance = s - car_s;
    }
  }
  // if the distance between the closest car on the other lanes is more than 3 times of the safety distance
  // and if left distance is large than right, go left
  if((closest_left_distance >= 3 * safety_distance && closest_right_distance >= 3 * safety_distance) || closest_left_distance >= closest_right_distance) {
    return 0;
  } 
  // and if right distance is large than left, go right
  else {
    return 2;
  }
}

vector<vector<double>> generate_trajectory(vector<double> goal, double car_x, double car_y, double car_yaw,
                                            vector<double> previous_path_x, vector<double> previous_path_y,
                                            vector<double> map_waypoints_x, vector<double> map_waypoints_y,
                                            vector<double> map_waypoints_s) {
  // cout<<"~~~~~~~~~~~~~~generate_trajectory start~~~~~~~~~~~~"<<endl;
  vector<double> ptsx;
  vector<double> ptsy;

  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw);

  int previous_size = previous_path_x.size();

  // use 2 points that make the path tangent to the car
  if (previous_size < 2){
    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);
  } 
  // use the previous path's end point as starting reference
  else {
    //redefine reference state as previous path end point
    ref_x = previous_path_x[previous_size -1];
    ref_y = previous_path_y[previous_size -1];

    double prev_ref_x = previous_path_x[previous_size - 2];
    double prev_ref_y = previous_path_y[previous_size - 2];
    ref_yaw = atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);

    // use two points that make the path tangent to the previous path's end point
    ptsx.push_back(prev_ref_x);
    ptsx.push_back(ref_x);

    ptsy.push_back(prev_ref_y);
    ptsy.push_back(ref_y);
  }

  vector<double> goal_points = getXY(goal[0], goal[1], map_waypoints_s, map_waypoints_x, map_waypoints_y);
  ptsx.push_back(goal_points[0]);
  ptsy.push_back(goal_points[1]);

  // shift car reference angle to 0 degrees
  for (size_t i = 0; i < ptsx.size(); ++i) {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
    ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
  }

  // create a spline
  tk::spline s;
  // set point to spline
  s.set_points(ptsx, ptsy);

  // define the acutal points we will use for the trajectory
  vector<double> next_x_vals;
  vector<double> next_y_vals;

  // start with all of the previous path points from last time
  for (size_t i = 0; i < previous_size; ++i) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  // calculate how to break up spline points so that we travel at our desired reference velocity
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_distance = sqrt(target_x * target_x + target_y * target_y);
  int N = target_distance / (0.02 * goal[2]/ 2.24);

  double x_add_on = 0.0;

  // fill up the rest of our path planner agter filling it with previous points, here we will always output 50 points
  for(size_t i = 0; i < 50 - previous_size; ++i)
  {
    double next_x = x_add_on + target_x / N;
    double next_y = s(next_x);

    x_add_on = next_x;

    double x_ref = next_x;
    double y_ref = next_y;
    
    // rotate back to normal after rotating it earlier 
    next_x = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
    next_y = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

    next_x += ref_x;
    next_y += ref_y;

    next_x_vals.push_back(next_x);
    next_y_vals.push_back(next_y);
  }
  
  vector<vector<double>> trajectory;
  trajectory.push_back(next_x_vals);
  trajectory.push_back(next_y_vals);

  /**
  vector<double>::iterator it;
  cout<<" next_x_vals  ===== ";
  for(it = next_x_vals.begin();it!=next_x_vals.end();it++) {
    cout<<*it<<' ';
  }
  cout<<endl;
  vector<double>::iterator itt;
  cout<<" next_y_vals  ===== ";
  for(itt = next_y_vals.begin();itt!=next_y_vals.end();itt++) {
    cout<<*itt<<' ';
  }
  cout<<endl;
  **/
  // cout<<"~~~~~~~~~~~~~~generate_trajectory end~~~~~~~~~~~~"<<endl;

  return trajectory;
}