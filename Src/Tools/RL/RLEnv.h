#ifndef RLEnv_h
#define RLEnv_h

#include "Representations/Modeling/RobotPose.h"
#include "Representations/BehaviorControl/FieldBall.h"

#include "Tools/json.h"
#include "Tools/RLConfig.h"

#define PI 3.14159265

extern bool RLConfig::debug_print;
extern std::string RLConfig::mode;

struct FieldPositions {
  float virtual_ball_X_position;
  float virtual_ball_Y_position;
  float dummy_defender_1_X_position;
  float dummy_defender_1_Y_position;
  float dummy_defender_2_X_position;
  float dummy_defender_2_Y_position;
  
  FieldPositions(const float virtual_ball_X_position, const float virtual_ball_Y_position,
                  const float dummy_defender_1_X_position, const float dummy_defender_1_Y_position,
                  const float dummy_defender_2_X_position, const float dummy_defender_2_Y_position):
  virtual_ball_X_position (virtual_ball_X_position), virtual_ball_Y_position (virtual_ball_Y_position),
  dummy_defender_1_X_position (dummy_defender_1_X_position), dummy_defender_1_Y_position (dummy_defender_1_Y_position),
  dummy_defender_2_X_position (dummy_defender_2_X_position), dummy_defender_2_Y_position (dummy_defender_2_Y_position) {}
  
  FieldPositions(const FieldPositions &field_positions) {
    virtual_ball_X_position = field_positions.virtual_ball_X_position;
    virtual_ball_Y_position = field_positions.virtual_ball_Y_position;
    dummy_defender_1_X_position = field_positions.dummy_defender_1_X_position;
    dummy_defender_1_Y_position = field_positions.dummy_defender_1_Y_position;
    dummy_defender_2_X_position = field_positions.dummy_defender_2_X_position;
    dummy_defender_2_Y_position = field_positions.dummy_defender_2_Y_position;
  }
};    // struct FieldPositions

class Environment {
private:
  unsigned int observation_length;
  unsigned int action_length;
  
  FieldPositions field_positions;
  
  std::vector<float> observation_vector;
  std::vector<float> normalized_vector;
  std::vector<float> current_action;
  
  float normalization_clip;
  float normalization_epsilon;
  std::vector<float> normalization_mean;
  std::vector<float> normalization_var;
  
public:
  Environment (const json::value metadata, const FieldPositions field_positions, const int observation_size);
  
  std::vector<float> getFloatVectorFromJSONArray(const json::value &json_value);
  bool shouldReset(GroundTruthRobotPose pose);
  
  std::vector<float> getObservation(GroundTruthRobotPose pose);
  std::vector<float> getObservation(RobotPose pose, FieldBall ball);

  std::vector<float> normalizeObservation();
  
  void setFieldPositions(const float virtual_ball_X_position, const float virtual_ball_Y_position,
                         const float dummy_defender_1_X_position, const float dummy_defender_1_Y_position,
                         const float dummy_defender_2_X_position, const float dummy_defender_2_Y_position);
  
  unsigned int getActionLength() { return action_length; }
  
  void setCurrentAction(std::vector<float> currentAction) {
    current_action = std::vector<float>(currentAction);
  }
  
  std::vector<float> getCurrentAction() { return current_action; }
  std::vector<float> getCurrentObservation() { return observation_vector; }
  std::vector<float> getNormalizedObservation() { return normalized_vector; }
};    // class Environment

#endif /* RLEnv_h */
