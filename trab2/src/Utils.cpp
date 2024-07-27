#include "Utils.h"


float normalizeAngleDEG(float a)
{
    while(a>180.0)
        a -= 360.0;
    while(a<=-180.0)
        a += 360.0;
    return a;
}

float normalizeAngleRAD(float a)
{
    while(a>M_PI)
        a -= 2*M_PI;
    while(a<=-M_PI)
        a += 2*M_PI;
    return a;
}

float getLikelihoodFromLogOdds(float logodds)
{
    return 1.0 - 1.0/(1.0+exp(logodds));
}

float getLogOddsFromLikelihood(float likelihood)
{
    return log(likelihood/(1.0-likelihood));
}

int map(int value, int from_low, int from_high, int to_low, int to_high) {
    return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low;
}

float sonarMean(float R, float r, float beta, float alpha) {
    return ((R - r)/R + (beta - alpha)/beta) / 2;
}

float sonarOcc(float occ_update, float occ) {
    return (occ_update * occ) / ((occ_update * occ) + ((1.0 - occ_update) * (1.0 - occ)));
}