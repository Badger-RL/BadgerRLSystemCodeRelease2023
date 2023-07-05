/**
 * @file FreeKickWall.h
 *
 * This file declares a behavior to position between the ball and the own penalty mark.
 *
 * @author Arne Hasselbring
 * @author Sina Schreiber
 */

#pragma once

#include "Tools/BehaviorControl/Strategy/ActiveRole.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/GlobalTeammatesModel.h"

MODULE(FreeKickWall,
       {,
    REQUIRES(RobotPose),
    REQUIRES(GlobalTeammatesModel),
    
});

struct ObstacleVector{
    float x;
    float y;
    bool isteammate;
};

class FreeKickWall : public ActiveRole
{
    virtual bool onSegment(Vector2f p, Vector2f q, Vector2f r);
    virtual int orientation(Vector2f p, Vector2f q, Vector2f r);
    virtual bool doIntersect(Vector2f p1, Vector2f q1, Vector2f p2, Vector2f q2);
    virtual bool preCollision(std::vector<ObstacleVector>& Obstacle, float predictedPosX, float predictedPosY, bool obstacle[]);
    virtual void addObstaclesSimRobot(std::vector<ObstacleVector>& Obstacle);
    virtual std::pair<int, int> startIndexOfLongestConsecutive0s(const bool data[], int length);
    virtual Vector2f rotate_point(float cx,float cy,float angle, Vector2f p);
    const float normalizeValue = 1300.f; // threshold for normalize the old Vector
    const float maxXValue = 500.f; // threshold for x Value on orthogonal line between penalty mark and ball
    const float maxYValue = 300.f; // threshold for y Value on orthogonal line between penalty mark and ball
    Vector2f currentTarget = Vector2f::Zero(); // current Target
    std::vector<ObstacleVector> simRobot;
    bool robotPreCollision = false;
    
    SkillRequest execute(const Agent& self, const Agents& teammates) override;
};
