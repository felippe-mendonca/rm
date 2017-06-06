#include <iostream>
#include <msgpack.hpp>
#include <string>
#include <armadillo>
#include <zmq.hpp>
#include "pioneer.hpp"

using namespace std;
using namespace arma;
using namespace robot::local;

int main(int argc, char* argv[]) {
  if (argc != 3) {
    cout << "\t>> Usage: ./server <serial-port> <tcp-port>\n";
    exit(1);
  }

  zmq::context_t context(1);
  zmq::socket_t response(context, ZMQ_REP);
  response.bind("tcp://*:" + string(argv[2]));

  Pioneer robot(argv[1]);

  while (1) {
    zmq::message_t request;
    response.recv(&request);

    std::string req_str(static_cast<char*>(request.data()), request.size());
    msgpack::object_handle handle = msgpack::unpack(req_str.data(), req_str.size());
    msgpack::object object = handle.get();

    robot_request robot_req;
    object.convert(robot_req);
    auto request_type = std::get<0>(robot_req);
    auto request_data = std::get<1>(robot_req);

    robot_reply robot_rep;

    switch (request_type) {
      case RobotRequest::GET_POSE: {
        auto pose = robot.get_pose();
        robot_rep = std::make_tuple(RobotReply::SUCCESS, conv_to<std::vector<double>>::from(pose));
        cout << "[Request][GET_POSE]" << std::endl;
				break;
      }
      case RobotRequest::GET_SPEED: {
        auto speed = robot.get_speed();
        robot_rep = std::make_tuple(RobotReply::SUCCESS, conv_to<std::vector<double>>::from(speed));
				cout << "[Request][GET_SPEED]\n";
				break;
      }
			case RobotRequest::SET_POSE: {
				auto pose = conv_to<mat>::from(request_data);
        robot.set_pose(pose);
        robot_rep = std::make_tuple(RobotReply::SUCCESS, std::vector<double>());
        cout << "[Request][SET_POSE]\n";
				break;
			}
			case RobotRequest::SET_SPEED: {
        auto speed = conv_to<mat>::from(request_data);
        robot.set_speed(speed);
        robot_rep = std::make_tuple(RobotReply::SUCCESS, std::vector<double>());
				cout << "[Request][SET_SPEED]\n";
				break;
			}
    }
    
    std::stringstream stream;
    msgpack::pack(stream, robot_rep);
    std::string rep_str { stream.str() }; 
    
    zmq::message_t reply(rep_str.begin(), rep_str.end());
    response.send(reply);
  }
}