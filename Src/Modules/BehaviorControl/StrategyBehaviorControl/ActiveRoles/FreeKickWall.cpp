/**
 * @file FreeKickWall.cpp
 *
 * This file implements a behavior to position between the ball and the own penalty mark.
 *
 * @author Arne Hasselbring
 * @author Sina Schreiber
 * @author Ike Nguyen
 */

#include "FreeKickWall.h"
#include "Representations/Modeling/RobotPose.h"
#include <iostream>

#define PI 3.14159265

Vector2f newPosition(float temp, double multiplier, Vector2f ballPosition, Vector2f targetPosition, float normalizeValue) {
    const Vector2f shift =  (multiplier * ballPosition).normalized(normalizeValue);
    targetPosition = ballPosition + shift;
    temp = sqrt(pow(ballPosition.x() - targetPosition.x(), 2)
                + pow(ballPosition.y() - targetPosition.y(), 2));
    if (temp >= 750.f) {
        return targetPosition;
    } else {
        return newPosition(temp, multiplier - 0.1, ballPosition, targetPosition, normalizeValue);
    }
}

SkillRequest FreeKickWall::execute(const Agent&, const Agents&)
{
    
    //  const Vector2f defenderVector = Vector2f(theFieldDimensions.xPosOwnPenaltyMark, 0.f) - ballPosition; // change of direction
    //  const Vector2f newTarget = ballPosition + defenderVector.normalized(normalizeValue); // calculated new target
    //  const Angle rotationAngle = defenderVector.angle(); // angle for the rotation of the direction
    //  const Vector2f rotatedNewTargetPosition = newTarget.rotated(-rotationAngle); // rotated new target
    //  const Vector2f rotatedOldTargetPosition = currentTarget.rotated(-rotationAngle); // rotated old target
    
    const Vector2f ballPosition = theFieldBall.recentBallPositionOnField();
    double multiplier = -1;
    Vector2f targetPosition;
    targetPosition = newPosition(0.f, multiplier, ballPosition, targetPosition, normalizeValue);
    
    /**
     * if the new target is within the threshold keep the position otherwise walk to the new target.
     */
    if(std::abs(targetPosition.x() - ballPosition.x()) > maxXValue ||
       (std::abs(targetPosition.y() - ballPosition.y()) > maxYValue))
        currentTarget = targetPosition;
    
    // Check if robot is about to collide. make sure Robot only walk backwords by putting obstacles in the 4 regions that points towards the ball
    bool obstacles[8] = {false, false, false, false,false, false, false,false};
    addObstaclesSimRobot(simRobot);
    robotPreCollision = preCollision(simRobot,  currentTarget.x(), currentTarget.y(), obstacles);
    if(robotPreCollision){
        double angleRelativeToRobot = atan2(ballPosition.y() - theRobotPose.translation.y(), ballPosition.x() - theRobotPose.translation.x());
        obstacles[(int)(angleRelativeToRobot/(PI/4))] = true;
        obstacles[(int) (((angleRelativeToRobot + PI/4))/(PI/4)) % 8] = true;
        if((angleRelativeToRobot - PI/4) < 0){
            obstacles[(int)((2 * PI -(angleRelativeToRobot - PI/4))/(PI/4))] = true;
        }
        if((angleRelativeToRobot - PI/4 - PI/4) < 0){
            obstacles[(int)((2 * PI -(angleRelativeToRobot - PI/4 - PI/4))/(PI/4))] = true;
        }
        std::pair<int, int> index = startIndexOfLongestConsecutive0s(obstacles, sizeof(obstacles)/sizeof(obstacles[0]));
        double angle = ((index.first + index.second)/2 + 1) * (PI/4) - PI/8;
        float x = 100.f * cos(angle);
        float y = 100.f * sin(angle);
        std::cout << "Collision" << std::endl;
        return SkillRequest::Builder::walkTo(Pose2f(angle ,(Vector2f(x,y) + currentTarget)));
    }
    return SkillRequest::Builder::walkTo(Pose2f((ballPosition - currentTarget).angle(), currentTarget));
}


// Given three collinear points p, q, r, the function checks if
// point q lies on line segment 'pr'
bool FreeKickWall::onSegment(Vector2f p, Vector2f q, Vector2f r)
{
    if (q.x() <= fmax(p.x(), r.x()) && q.x() >= fmin(p.x(), r.x()) &&
        q.y() <= fmax(p.y(), r.y()) && q.y() >= fmin(p.y(), r.y()))
        return true;
    
    return false;
}

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are collinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int FreeKickWall::orientation(Vector2f p, Vector2f q, Vector2f r)
{
    // for details of below formula.
    float val = (q.y() - p.y()) * (r.x() - q.x()) -
    (q.x() - p.x()) * (r.y() - q.y());
    
    if (val == 0) return 0;  // collinear
    
    return (val > 0)? 1: 2; // clock or counterclock wise
}


// The main function that returns true if line segment 'p1q1'
// and 'p2q2' intersect.
bool FreeKickWall::doIntersect(Vector2f p1, Vector2f q1, Vector2f p2, Vector2f q2)
{
    // Find the four orientations needed for general and
    // special cases
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);
    
    // General case
    if (o1 != o2 && o3 != o4)
        return true;
    
    // Special Cases
    // p1, q1 and p2 are collinear and p2 lies on segment p1q1
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;
    
    // p1, q1 and q2 are collinear and q2 lies on segment p1q1
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;
    
    // p2, q2 and p1 are collinear and p1 lies on segment p2q2
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;
    
    // p2, q2 and q1 are collinear and q1 lies on segment p2q2
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;
    
    return false; // Doesn't fall in any of the above cases
}

bool FreeKickWall::preCollision(std::vector<ObstacleVector>& Obstacle, float predictedPosX, float predictedPosY, bool obstacles[8]){
    bool intersect = false;
    bool intersect2 = false;
    bool intersect3 = false;
    bool withInRobotSquare = false;
    for(unsigned int i = 0; i < Obstacle.size(); i ++){
        Vector2f stl(Obstacle[i].x + 250.f, Obstacle[i].y + 250.f);
        Vector2f sbl(Obstacle[i].x - 250.f, Obstacle[i].y + 250.f);
        Vector2f str(Obstacle[i].x + 250.f, Obstacle[i].y - 250.f);
        Vector2f sbr(Obstacle[i].x - 250.f, Obstacle[i].y - 250.f);
        Vector2f PredictedPoseVector(predictedPosX, predictedPosY);
        Vector2f obstaclePose(Obstacle[i].x, Obstacle[i].y);
        
        double dist = (theRobotPose.translation - PredictedPoseVector).norm();
        Vector2f unitVector = Vector2f((PredictedPoseVector - theRobotPose.translation).x()/dist,(PredictedPoseVector - theRobotPose.translation).y()/dist);
        Vector2f newVector = Vector2f(theRobotPose.translation.x() + unitVector.x()*100.f, theRobotPose.translation.y() + unitVector.y()*100.f);
        Vector2f rotateCounterClockwise = rotate_point(theRobotPose.translation.x(), theRobotPose.translation.y(), 10, newVector);
        Vector2f rotateClockwise = rotate_point(theRobotPose.translation.x(), theRobotPose.translation.y(), -10, newVector);
        
        intersect = (doIntersect(theRobotPose.translation, newVector, sbl, stl) || doIntersect(theRobotPose.translation, newVector, sbl, sbr) || doIntersect(theRobotPose.translation, newVector, sbr, str) || doIntersect(theRobotPose.translation, newVector, stl, str));
        intersect2 = (doIntersect(theRobotPose.translation, rotateCounterClockwise, sbl, stl) || doIntersect(theRobotPose.translation, rotateCounterClockwise, sbl, sbr) || doIntersect(theRobotPose.translation, rotateCounterClockwise, sbr, str) || doIntersect(theRobotPose.translation, rotateCounterClockwise, stl, str));
        intersect3 = (doIntersect(theRobotPose.translation, rotateClockwise, sbl, stl) || doIntersect(theRobotPose.translation, rotateClockwise, sbl, sbr) || doIntersect(theRobotPose.translation, rotateClockwise, sbr, str) || doIntersect(theRobotPose.translation, rotateClockwise, stl, str));
        //std::cout << "Distance between obstacle and Robot: " <<(theRobotPose.translation - obstaclePose).norm() << std::endl;
        
        double angleRelativeToRobot = atan2( Obstacle[i].y - theRobotPose.translation.y(), Obstacle[i].x - theRobotPose.translation.x());
        if(angleRelativeToRobot<0){
            angleRelativeToRobot += 2*PI;
        }
        if((theRobotPose.translation - obstaclePose).norm() < 500.f){
            obstacles[(int)(angleRelativeToRobot / (PI/4))] = true;
        }
        
        withInRobotSquare = theRobotPose.translation.x() <= stl.x() && theRobotPose.translation.x() >= sbl.x() && theRobotPose.translation.y() <= stl.y() && theRobotPose.translation.y() >= str.y();
        if(intersect || withInRobotSquare ){
            break;
        }
        while (Obstacle.size() > 15)
        {
            Obstacle.erase(Obstacle.begin());
        }
    }
    return intersect || intersect2 || intersect3 || withInRobotSquare;
}

void FreeKickWall::addObstaclesSimRobot(std::vector<ObstacleVector>& Obstacle){
    
    for(auto& teammate: theGlobalTeammatesModel.teammates){
        //std::cout << "teammate number: " << teammate.number << std::endl;
        ObstacleVector o{teammate.pose.translation.x(), teammate.pose.translation.y(), true};
        Obstacle.push_back(o);
    }
}


// Calculate longest sub array with all 1's index
std::pair<int, int> FreeKickWall::startIndexOfLongestConsecutive0s(const bool data[], int length) {
    int startIndex{}, counter{}, previousValue{},maxCounter{}, startIndexMax{}, endIndex{}, endIndexMax{};
    std::pair<int, int> result;
    // Go through all elements of the array
    for (int i{}; i < length; ++i) {
        
        // Get the current element. Special Handling for last element
        const bool value = data[i];
        // If we see a 1, then we are in a open sliding window
        if (value == false) {
            
            // If the window was just opened, then remember the start index
            if (previousValue == true) startIndex = i;
            if(data[i+1] == true || data[i] == false){ endIndex = i;}
            // Count the number of 1's in the open window. This will be the width at the end
            ++counter;
        }
        else {
            // If the last window size is bigger than whatever we had before
            // Now, the value is 0. So the window is closed.
            if ((previousValue == false) && (counter > maxCounter)) {
                // Store new max window size and new start index for max.                endIndex =
                maxCounter = counter;
                startIndexMax = startIndex;
                endIndexMax = endIndex;
            }
            // Optimization: If the maximum size of a found window is bigger
            // than the rest of the remaining value, we can stop all activities
            if (maxCounter >= length / 2) break;
            
            // Counter is 0 again
            counter = 0;
        }
        previousValue = value;
        
        
    }
    // Now, the value is 0. So the window is closed.
    if ((data[length-1] == false) && (counter > maxCounter)) {
        // Store new max window size and new start index for max.                endIndex =
        maxCounter = counter;
        startIndexMax = startIndex;
        endIndexMax = endIndex;
    }
    
    result.first = startIndexMax;
    result.second = endIndexMax;
    return result;
}

Vector2f FreeKickWall::rotate_point(float cx,float cy,float angleDegree, Vector2f p)
{
    float angle = angleDegree * PI / 180.0;
    float s = sin(angle);
    float c = cos(angle);
    
    float px = p.x();
    float py = p.y();
    // translate point back to origin:
    px -= cx;
    py -= cy;
    
    // rotate point
    float xnew = px * c - py * s;
    float ynew = px * s + py * c;
    
    // translate point back:
    px = xnew + cx;
    py = ynew + cy;
    return Vector2f(px, py);
}
