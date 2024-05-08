#include "Action.h"

#include <cmath>

Action::Action()
{
    linVel = 0.0;
    angVel = 0.0;
}

void Action::avoidObstacles(std::vector<float> lasers, std::vector<float> sonars)
{
    int distanceThreshold = 1.0;
    int lasersSize = lasers.size();
    int lowerBoundary = lasersSize/2 - 80;
    int higherBoundary = lasersSize/2 + 80;
    int minValueIndex;
    float minValue = 32;

    for(int theta = lowerBoundary; theta < higherBoundary; ++theta) {
        if(lasers[theta] < minValue) {
            minValueIndex = theta;
            minValue = lasers[theta];
        }
    }

    // Debug
    // std::cout << "Index[" << minValueIndex << "], MinValue = " << minValue << std::endl;

    // Most critical wall encounter (almost front encounter)
    if(minValue < distanceThreshold + 0.3 && (minValueIndex >= 60 && minValueIndex <= 120)) {
        linVel = 0.1;
        if(lasers[0] > lasers[lasersSize - 1]) {
            angVel = 0.5; // Turn left
        } else {
            angVel = -0.5; // Turn right
        }
    } else {
        if(minValue < distanceThreshold/2) {
            linVel = 0;
        } else {
            if(minValue < distanceThreshold) {
                if(minValueIndex < 60 && minValueIndex >= 20) {
                    linVel = 0.35;
                    angVel = -0.4;
                } else if(minValueIndex < 20 && minValueIndex >= lowerBoundary) {
                    linVel = 0.5;
                    angVel = -0.25;
                } else if(minValueIndex > 120 && minValueIndex <= 160) {
                    linVel = 0.35;
                    angVel = 0.4;
                } else if(minValueIndex > 160 && minValueIndex <= higherBoundary) {
                    linVel = 0.5;
                    angVel = 0.25;
                }
            } else {
                linVel = 0.5;
                angVel = 0;
            }
        }   
    }
}

void Action::wallFollowing(std::vector<float> lasers, std::vector<float> sonars)
{   
    int distanceThreshold = 1;
    int lowerBoundary = 0;
    int higherBoundary = lasers.size() - 1;
    int minValueIndex;
    float minValue = 32;
    float integral = 0;

    float tp = 0.15;
    float td = 4.5;
    float ti = 0.00003;

    float cte = 1 - lasers[0];
    float diffcte = cte - ctePrev;

    ctePrev = cte;
    if(cteTotal.size() == 10) {
        cteTotal.pop_back();
        cteTotal.insert(cteTotal.begin(), cte);
    } else {
        cteTotal.push_back(cte);
    }

    for(int i = 0; i < 10; ++i) {   
        integral += cteTotal[i];
    }

    integral = integral/10; // Scaling integral

    for(int theta = lowerBoundary; theta < higherBoundary; ++theta) {
        if(lasers[theta] < minValue) {
            minValueIndex = theta;
            minValue = lasers[theta];
        }
    }


    if((minValueIndex >= 70 && minValueIndex <= 140) && minValue < distanceThreshold + 0.5) {
        linVel = 0;
        angVel = -0.5;
    } else if(minValue < distanceThreshold + 0.5) {
        if(minValueIndex >= 0 && minValueIndex <= 20) {
            linVel = 0.5;
            angVel = -tp * cte -td * diffcte -ti * integral;
        } else if(minValueIndex > 20 && minValueIndex <= 60) {
            linVel = 0.1;
            angVel = -0.5;
        } else {
            linVel = 0.2;
        }
    } else {
        linVel = 0.1;
        angVel = 0.5;
    }

    // Debug
    // std::cout << "CTE: " << cte << " Laser[0]: " << lasers[0] << " Laser[" << minValueIndex << "] Value: " << minValue << std::endl;
    // std::cout << "Integral: " << integral << std::endl;
}

void Action::manualRobotMotion(MovingDirection direction)
{
    if(direction == FRONT){
        linVel= 0.5; angVel= 0.0;
    }else if(direction == BACK){
        linVel=-0.5; angVel= 0.0;
    }else if(direction == LEFT){
        linVel= 0.0; angVel= 0.5;
    }else if(direction == RIGHT){
        linVel= 0.0; angVel=-0.5;
    }else if(direction == STOP){
        linVel= 0.0; angVel= 0.0;
    }
}

void Action::correctVelocitiesIfInvalid()
{
    float b=0.38;

    float leftVel  = linVel - angVel*b/(2.0);
    float rightVel = linVel + angVel*b/(2.0);

    float VELMAX = 0.5;

    float absLeft = fabs(leftVel);
    float absRight = fabs(rightVel);

    if(absLeft>absRight){
        if(absLeft > VELMAX){
            leftVel *= VELMAX/absLeft;
            rightVel *= VELMAX/absLeft;
        }
    }else{
        if(absRight > VELMAX){
            leftVel *= VELMAX/absRight;
            rightVel *= VELMAX/absRight;
        }
    }
    
    linVel = (leftVel + rightVel)/2.0;
    angVel = (rightVel - leftVel)/b;
}

float Action::getLinearVelocity()
{
    return linVel;
}

float Action::getAngularVelocity()
{
    return angVel;
}

MotionControl Action::handlePressedKey(char key)
{
    MotionControl mc;
    mc.mode=MANUAL;
    mc.direction=STOP;

    if(key=='1'){
        mc.mode=MANUAL;
        mc.direction=STOP;
    }else if(key=='2'){
        mc.mode=WANDER;
        mc.direction=AUTO;
    }else if(key=='3'){
        mc.mode=WALLFOLLOW;
        mc.direction=AUTO;
    }else if(key=='w' or key=='W'){
        mc.mode=MANUAL;
        mc.direction = FRONT;
    }else if(key=='s' or key=='S'){
        mc.mode=MANUAL;
        mc.direction = BACK;
    }else if(key=='a' or key=='A'){
        mc.mode=MANUAL;
        mc.direction = LEFT;
    }else if(key=='d' or key=='D'){
        mc.mode=MANUAL;
        mc.direction = RIGHT;
    }else if(key==' '){
        mc.mode=MANUAL;
        mc.direction = STOP;
    }
    
    return mc;
}

