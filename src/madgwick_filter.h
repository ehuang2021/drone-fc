#ifndef MADGWICK_FILTER_H
#define MADGWICK_FILTER_H

typedef struct {
    float q1;
    float q2;
    float q3;
    float q4;
} Quanterion;

void filterUpdate(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z, Quanterion *q, float delta);


#endif
