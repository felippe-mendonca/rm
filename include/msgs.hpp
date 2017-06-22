#ifndef _ROBOT_MSG_
#define _ROBOT_MSG_

#include <msgpack.hpp>

enum class RobotRequest { GET_POSE = 0, GET_SPEED, SET_POSE, SET_SPEED, GET_TELEMETRY };
enum class RobotReply { SUCCESS = 0, INVALID_POSE, INVALID_SPEED };

typedef std::tuple<RobotRequest, std::vector<double>> robot_request;
typedef std::tuple<RobotReply, std::vector<double>> robot_reply;

MSGPACK_ADD_ENUM(RobotRequest);
MSGPACK_ADD_ENUM(RobotReply);

#endif  // _ROBOT_MSG_