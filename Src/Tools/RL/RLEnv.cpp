#include "RLEnv.h"
#include <algorithm>

Environment::Environment (const FieldPositions field_positions, const int observation_size, const int action_size) :
  observation_length ((unsigned int)(observation_size)),
  action_length ((unsigned int)(action_size)),
  field_positions (field_positions),
  observation_vector (observation_size) {}
bool Environment::shouldReset(GroundTruthRobotPose pose) {
  if (RLConfig::mode == "push_ball_to_goal" || RLConfig::mode == "dummy_defenders" ||
      RLConfig::mode == "goalie") {
    float target_x = field_positions.virtual_ball_X_position;
    float target_y = field_positions.virtual_ball_Y_position;

    return (target_x < -4500 && target_x > -5000 && target_y < 800 && target_y > -800);
  } else if (RLConfig::mode == "ball_targeting") {
    float target_x = field_positions.virtual_ball_X_position;
    float target_y = field_positions.virtual_ball_Y_position;

    float x = pose.translation[0];
    float y = pose.translation[1];

    return sqrt(pow(x - target_x, 2) + pow(y - target_y, 2)) <= 190;
  } else {
    return false;
  }
}


float clamp(float number, float low, float high)
{
  return number <= low ? low : number >= high ? high : number;
}


std::vector<float> Environment::getPredictedPosition(RobotPose theRobotPose, std::vector<float> action)
{

float robotX = theRobotPose.translation.x();
float robotY = theRobotPose.translation.y();
float angle = theRobotPose.rotation;        
if (angle < 0) {
  angle += 2 * PI;
}

 float policy_target_x = robotX + (
      (
          (cos(angle) * clamp(action[1], -1, 1))
          + (cos(angle + PI / 2) * clamp(action[2], -1.0, 1.0))
      )
      * 200
  ); 
  float policy_target_y = robotY + (
      (
          (sin(angle) * clamp(action[1], -1, 1))
          + (sin(angle + PI / 2) * clamp(action[2], -1.0, 1.0))
      )
      * 200
  );

  robotX = (
      robotX * (1 - 0.2)
      + policy_target_x * 0.2
  ); 
  robotY = (
      robotY * (1 - 0.2)
      + policy_target_y * 0.2
  );  

  std::vector<float> result;
  result.push_back(robotX);
  result.push_back(robotY);
  return result;

}


std::vector<float> Environment::getObservation(RobotPose theRobotPose, FieldBall theFieldBall) {

  const float goal_x = 4800;
  const float goal_y = 0;

  float x = theRobotPose.translation.x();
  float y = theRobotPose.translation.y();
  float angle = theRobotPose.rotation;
  
  if (angle < 0) {
    angle += 2 * PI;
  }

  float target_x = theFieldBall.positionOnField.x();
  float target_y = theFieldBall.positionOnField.y();
  
  float relative_angle;
  float delta_x = target_x - x;
  float delta_y = target_y - y;
  float theta_radians = atan2(delta_y, delta_x);

  if (theta_radians >= 0) {
    relative_angle = theta_radians;
  } else {
    relative_angle = theta_radians + (2*PI);
  }
  
  float goal_delta_x = goal_x - x;
  float goal_delta_y = goal_y - y;
  float goal_theta_radians = atan2(goal_delta_y, goal_delta_x);
  float goal_relative_angle;

  if (goal_theta_radians >= 0) {
    goal_relative_angle = goal_theta_radians;
  } else {
    goal_relative_angle = goal_theta_radians + (2*PI);
  }

  if (RLConfig::debug_print == true) {
    std::cout << "position" << std::endl;
    std::cout << x << std::endl;
    std::cout << y << std::endl;
    std::cout << "target position" << std::endl;
    std::cout << target_x << std::endl;
    std::cout << target_y << std::endl;
    std::cout << "ball relative angle" << std::endl;
    std::cout << relative_angle << std::endl;
    std::cout << "goal relative angle" << std::endl;
    std::cout << goal_relative_angle << std::endl;
    std::cout << "robot angle" << std::endl;
    std::cout << angle << std::endl;
  }
  
  observation_vector[0] = (field_positions.dummy_defender_1_X_position - x) / 9000.0;
  observation_vector[1] = (field_positions.dummy_defender_1_Y_position - y) / 6000.0;
  observation_vector[2] = (field_positions.dummy_defender_2_X_position - x) / 9000.0;
  observation_vector[3] = (field_positions.dummy_defender_2_Y_position - y) / 6000.0;
  observation_vector[4] = (target_x - x) / 9000.0;
  observation_vector[5] = (target_y - y) / 6000.0;
  observation_vector[6] = (goal_x - target_x) / 9000.0;
  observation_vector[7] = (goal_y - target_y) / 6000.0;
  observation_vector[8] = sin(relative_angle - angle);
  observation_vector[9] = cos(relative_angle - angle);
  observation_vector[10] = sin(goal_relative_angle - angle);
  observation_vector[11] = cos(goal_relative_angle - angle);
  return observation_vector;
}

//deprecated
std::vector<float> Environment::getObservation(GroundTruthRobotPose pose) {
  const float goal_x = -4500.0;
  const float goal_y = 0;

  float x = pose.translation[0];
  float y = pose.translation[1];
  float angle = pose.rotation;
  
  if (angle < 0) {
    angle += 2 * PI;
  }

  float target_x = field_positions.virtual_ball_X_position;
  float target_y = field_positions.virtual_ball_Y_position;
  
  float relative_angle;
  float delta_x = target_x - x;
  float delta_y = target_y - y;
  float theta_radians = atan2(delta_y, delta_x);

  if (theta_radians >= 0) {
    relative_angle = theta_radians;
  } else {
    relative_angle = theta_radians + (2*PI);
  }
  
  float goal_delta_x = goal_x - x;
  float goal_delta_y = goal_y - y;
  float goal_theta_radians = atan2(goal_delta_y, goal_delta_x);
  float goal_relative_angle;

  if (goal_theta_radians >= 0) {
    goal_relative_angle = goal_theta_radians;
  } else {
    goal_relative_angle = goal_theta_radians + (2*PI);
  }

  if (RLConfig::debug_print == true) {
    std::cout << "position" << std::endl;
    std::cout << x << std::endl;
    std::cout << y << std::endl;
    std::cout << "target position" << std::endl;
    std::cout << target_x << std::endl;
    std::cout << target_y << std::endl;
    std::cout << "ball relative angle" << std::endl;
    std::cout << relative_angle << std::endl;
    std::cout << "goal relative angle" << std::endl;
    std::cout << goal_relative_angle << std::endl;
    std::cout << "robot angle" << std::endl;
    std::cout << angle << std::endl;
  }

  observation_vector[0] = (field_positions.dummy_defender_1_X_position - x) / 9000.0;
  observation_vector[1] = (field_positions.dummy_defender_1_Y_position - y) / 6000.0;
  observation_vector[2] = (field_positions.dummy_defender_2_X_position - x) / 9000.0;
  observation_vector[3] = (field_positions.dummy_defender_2_Y_position - y) / 6000.0;
  observation_vector[4] = (target_x - x) / 9000.0;
  observation_vector[5] = (target_y - y) / 6000.0;
  observation_vector[6] = (goal_x - target_x) / 9000.0;
  observation_vector[7] = (goal_y - target_y) / 6000.0;
  observation_vector[8] = sin(relative_angle - angle);
  observation_vector[9] = cos(relative_angle - angle);
  observation_vector[10] = sin(goal_relative_angle - angle);
  observation_vector[11] = cos(goal_relative_angle - angle);

  return observation_vector;
}



std::vector<float> Environment::getFloatVectorFromJSONArray(const json::value &json_value) {
  std::vector<float> result;
  const json::array &json_array = as_array(json_value);
  for(auto entry = json_array.begin(); entry != json_array.end(); ++entry) {
     const json::value &entry_string = *entry;
     result.push_back(std::stof(to_string(entry_string)));
  }
  return result;
}

void Environment::setFieldPositions(const float virtual_ball_X_position, const float virtual_ball_Y_position,
                       const float dummy_defender_1_X_position, const float dummy_defender_1_Y_position,
                       const float dummy_defender_2_X_position, const float dummy_defender_2_Y_position) {
  field_positions.virtual_ball_X_position = virtual_ball_X_position;
  field_positions.virtual_ball_Y_position = virtual_ball_Y_position;
  field_positions.dummy_defender_1_X_position = dummy_defender_1_X_position;
  field_positions.dummy_defender_1_Y_position = dummy_defender_1_Y_position;
  field_positions.dummy_defender_2_X_position = dummy_defender_2_X_position;
  field_positions.dummy_defender_2_Y_position = dummy_defender_2_Y_position;
}
