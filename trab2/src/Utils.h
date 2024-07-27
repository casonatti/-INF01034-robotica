#ifndef UTILS_H
#define UTILS_H

#include <cmath>

#define DEG2RAD(x) x*M_PI/180.0
#define RAD2DEG(x) x*180.0/M_PI

float normalizeAngleDEG(float a);
float normalizeAngleRAD(float a);

float getLikelihoodFromLogOdds(float logodds);
float getLogOddsFromLikelihood(float likelihood);

int map(int value, int from_low, int from_high, int to_low, int to_high);

float sonarMean(float R, float r, float beta, float alpha);

float sonarOcc(float occ_update, float occ);

typedef struct{
    float x, y, theta;
} Pose2D;

#endif // UTILS_H
