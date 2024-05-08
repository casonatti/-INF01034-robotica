#ifndef ACTION_H
#define ACTION_H

#include <vector>
#include <iostream>

enum MotionMode {MANUAL, WANDER, WALLFOLLOW};
enum MovingDirection {STOP, FRONT, BACK, LEFT, RIGHT, AUTO};

typedef struct
{
    MotionMode mode;
    MovingDirection direction;
} MotionControl;

class Action
{
public:
    Action();
    
    void manualRobotMotion(MovingDirection direction);
    void avoidObstacles(std::vector<float> lasers, std::vector<float> sonars);
    void wallFollowing(std::vector<float> lasers, std::vector<float> sonars);

    MotionControl handlePressedKey(char key);

    void correctVelocitiesIfInvalid();
    float getLinearVelocity();
    float getAngularVelocity();

private:
    std::vector<float>cteTotal {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    float ctePrev = 0;
    float linVel;
    float angVel;
};

#endif // ACTION_H
