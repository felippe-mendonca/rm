#ifndef __DRIVER_PIONEER_HPP__
#define __DRIVER_PIONEER_HPP__

#include <armadillo>
#include <atomic>
#include <chrono>
#include <cmath>
#include <msgpack.hpp>
#include <string>
#include <thread>
#include "msgs.hpp"
#include "zmq.hpp"

#ifndef isnan
#if __cplusplus <= 199711L  // c++98 or older
#define isnan(x) ::isnan(x)
#else
#define isnan(x) std::isnan(x)
#endif
#endif

#include <Aria/Aria.h>

constexpr double pi() {
  return std::atan(1) * 4;
}
auto deg2rad = [](double deg) { return deg * (pi() / 180.0); };
auto rad2deg = [](double rad) { return rad * (180.0 / pi()); };

using namespace std::chrono;
using namespace arma;

void assert_size(mat const& matrix, unsigned int const& n_elem, std::string const& who) {
  if (!(matrix.n_rows == n_elem && matrix.n_cols == 1) && !(matrix.n_rows == 1 && matrix.n_cols == n_elem)) {
    throw std::runtime_error("Invalid matrix size. " + who + " must be 1x" + std::to_string(n_elem) + " or " +
                             std::to_string(n_elem) + "x1.");
  }
}

namespace robot {
namespace local {

struct Pioneer {
  ArRobot robot;
  ArDeviceConnection* connection;
  static int port;

  Pioneer() { connect("localhost", this->port++); }
  Pioneer(std::string const& port) { connect(port); }

  virtual ~Pioneer() { disconnect(); }

  void disconnect() {
    if (robot.isConnected()) {
      robot.lock();
      robot.stopRunning();
      robot.unlock();
      robot.disconnect();
    }
    Aria::exit(0);
  }

  void connect(std::string const& port) {
    Aria::init();
    auto serial = new ArSerialConnection();
    connection = serial;
    serial->open(port.c_str());
    robot.setDeviceConnection(serial);
    if (!robot.blockingConnect()) {
      Aria::exit(0);
      throw std::runtime_error("Could not connect to robot.");
    }
    robot.runAsync(true);
    initialize();
  }

  void connect(std::string const& hostname, int const& port) {
    Aria::init();
    auto tcp = new ArTcpConnection;
    connection = tcp;
    tcp->open(hostname.c_str(), port);
    robot.setDeviceConnection(tcp);
    if (!robot.blockingConnect()) {
      Aria::exit(0);
      throw std::runtime_error("Could not connect to robot.");
    }
    robot.runAsync(true);
    initialize();
  }

  void set_speed(mat const& speed) {
    assert_size(speed, 2, "Speed");
    vec speed_vec(speed);
    robot.lock();
    robot.setVel(speed_vec(0));
    robot.setRotVel(rad2deg(speed_vec(1)));
    robot.unlock();
  }

  mat get_speed() {
    robot.lock();
    double linear = robot.getVel();
    double angular = robot.getRotVel();
    robot.unlock();
    mat speed_mat = {linear, deg2rad(angular)};
    return speed_mat;
  }

  mat get_pose() {
    robot.lock();
    ArPose arpose = robot.getPose();
    robot.unlock();
    mat pose_mat;
    pose_mat << arpose.getX() << endr << arpose.getY() << endr << deg2rad(arpose.getTh());
    return pose_mat;
  }

  void set_pose(mat pose) {
    assert_size(pose, 3, "Pose");
    vec pose_vec(pose);
    robot.lock();
    robot.moveTo(ArPose(pose_vec(0), pose_vec(1), rad2deg(pose_vec(2))));
    robot.unlock();
  }

 private:
  void initialize() {
    robot.lock();
    robot.moveTo(ArPose(0.0, 0.0, 0.0));
    robot.enableMotors();
    robot.setVel(0.0);
    robot.setRotVel(0.0);
    robot.unlock();
  }
};

int Pioneer::port = 8101;

}  // ::robot::local

namespace remote {

struct Pioneer {
  zmq::context_t context;
  zmq::socket_t requester;
  std::string address;

  Pioneer(std::string const& hostname, int const& port)
      : context(1), requester(context, ZMQ_REQ), address("tcp://" + hostname + ":" + std::to_string(port)) {
    requester.connect(this->address);
  }

  virtual ~Pioneer() { disconnect(); }

  void disconnect() {
    mat speed;
    speed << 0.0 << endr << 0.0;
    this->set_speed(speed);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  void set_speed(mat const& speed) {
    assert_size(speed, 2, "Speed");
    vec speed_vec(speed);
    request(RobotRequest::SET_SPEED, speed_vec);
  }

  mat get_speed() { return request(RobotRequest::GET_SPEED); }

  void set_pose(mat pose) {
    assert_size(pose, 3, "Pose");
    vec pose_vec(pose);
    request(RobotRequest::SET_POSE, pose_vec);
  }

  mat get_pose() { return request(RobotRequest::GET_POSE); }

 private:
  mat request(RobotRequest const& req_type, vec const& req_vec = vec()) {
    robot_request req(req_type, conv_to<std::vector<double>>::from(req_vec));
    std::stringstream stream;
    msgpack::pack(stream, req);
    std::string req_str{stream.str()};
    zmq::message_t request(req_str.begin(), req_str.end());
    this->requester.send(request);

    zmq::message_t response;
    this->requester.recv(&response);

    std::string rep_str(static_cast<char*>(response.data()), response.size());
    msgpack::object_handle handle = msgpack::unpack(rep_str.data(), rep_str.size());
    msgpack::object object = handle.get();

    robot_reply robot_rep;
    object.convert(robot_rep);
    // auto reply_type = std::get<0>(robot_rep);
    auto reply_data = std::get<1>(robot_rep);
    return conv_to<mat>::from(reply_data);
  }
};

}  // ::robot::remote
}  // ::robot

struct Loop {
  high_resolution_clock::time_point time;
  high_resolution_clock::time_point until_time;
  milliseconds period;

  Loop(uint64_t const& period_ms = 100)
      : time(high_resolution_clock::now()), until_time(high_resolution_clock::now()), period(period_ms) {}

  void wait() {
    time += period;
    std::this_thread::sleep_until(time);
  }

  bool until_ms(uint64_t ms) {
    auto diff = until_time + milliseconds(ms) - high_resolution_clock::now();
    std::cout << duration_cast<milliseconds>(diff).count() << std::endl;
    return duration_cast<milliseconds>(diff).count() > 0;
  }

  bool until_sec(uint64_t sec) { return until_ms(sec * 1000); }
};

#endif  // __DRIVER_PIONEER_HPP__