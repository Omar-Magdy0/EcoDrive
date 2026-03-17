#ifndef PMSM_MODEL_H
#define PMSM_MODEL_H

#include <math.h>

typedef struct {
    float i[3];
    float v[3];
   float vn;
   float theta;
    float omega;
} MotorState;

typedef struct {
    float vthev[3];
    float rthev[3];
    float T_load;
} MotorInput;

typedef struct {
    float Ld, Lq, Rs, pp, Ke, J, B;
} MotorParams;

typedef struct {
    float T;
} MotorOutput;

static inline void pmsm_step(const MotorState* state, const MotorInput* in, const MotorParams* p, float dt, MotorState* next, MotorOutput* out) {
    const float x0 = 1.0/dt;
    const float x1 = p->pp*state->theta;
    const float x2 = 2*x1;
    const float x3 = (1.0/2.0)*p->Ld;
    const float x4 = (1.0/2.0)*p->Lq;
    const float x5 = x3 - x4;
    const float x6 = x3 + x4;
    const float x7 = -x5*cos(x2) + x6;
    const float x8 = x0*x7;
    const float x9 = p->Rs + x8 + in->rthev[0];
    const float x10 = 1.0/x9;
    const float x11 = cos(x1);
    const float x12 = p->Ke*p->pp;
    const float x13 = state->omega*x12;
    const float x14 = x11*x13;
    const float x15 = (1.0/3.0)*M_PI;
    const float x16 = x15 + x2;
    const float x17 = x0*(x5*cos(x16) + x6);
    const float x18 = p->Rs + x17 + in->rthev[1];
    const float x19 = 1.0/x18;
    const float x20 = (1.0/6.0)*M_PI;
    const float x21 = x2 + x20;
    const float x22 = x0*(x5*sin(x21) + x6);
    const float x23 = p->Rs + x22 + in->rthev[2];
    const float x24 = 1.0/x23;
    const float x25 = cos(x1 + x15);
    const float x26 = x13*x25 + x17*state->i[1] + in->vthev[1];
    const float x27 = sin(x1 + x20);
    const float x28 = x13*x27 + x22*state->i[2] + in->vthev[2];
    const float x29 = (x10*(-x14 + x8*state->i[0] + in->vthev[0]) + x19*x26 + x24*x28)/(x10 + x19 + x24);
    const float x30 = x0*x7*state->i[0] - x14 - x29 + in->vthev[0];
    const float x31 = x10*x30;
    const float x32 = -x29;
    const float x33 = x26 + x32;
    const float x34 = x19*x33;
    const float x35 = x28 + x32;
    const float x36 = x24*x35;
    const float x37 = p->Ld - p->Lq;

    next->i[0] = x31;
    next->i[1] = x34;
    next->i[2] = x36;
    next->v[0] = -x31*in->rthev[0] + in->vthev[0];
    next->v[1] = -x34*in->rthev[1] + in->vthev[1];
    next->v[2] = -x36*in->rthev[2] + in->vthev[2];
    next->vn = x29;
    out->T = p->Ke*p->pp*x10*x11*x30 + (1.0/2.0)*p->pp*pow(x30, 2)*x37*sin(x2)/pow(x9, 2) + (1.0/2.0)*p->pp*pow(x35, 2)*x37*cos(x21)/pow(x23, 2) - 1.0/2.0*p->pp*pow(x33, 2)*x37*sin(x16)/pow(x18, 2) - x12*x25*x34 - x12*x27*x36;
}

#endif // PMSM_MODEL_H
