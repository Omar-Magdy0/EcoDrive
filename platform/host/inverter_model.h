#ifndef INVERTER_MODEL_H
#define INVERTER_MODEL_H

#include <math.h>
#include <stdbool.h>

typedef struct {
    float i[3];
    float v[3];
} InverterState;

typedef struct {
    float vbus;
    float duty[3];
    float drive[3];
} InverterInput;

typedef struct {
    float Rdson, RdsOff, FWmin;
} InverterParams;

typedef struct {
    float vthev[3];
    float rthev[3];
} InverterOutput;

static inline void inverter_step(const InverterState* state, const InverterInput* in, const InverterParams* p, InverterOutput* out) {
    // --- Optimized Intermediate Logic ---
    const float x0 = in->drive[0] == 1;
    const float x1 = in->vbus + 0.69999999999999996;
    const float x2 = p->FWmin < -state->i[0];
    const float x3 = p->FWmin < state->i[0];
    const float x4 = (1.0/2.0)*in->vbus;
    const float x5 = in->drive[1] == 1;
    const float x6 = p->FWmin < -state->i[1];
    const float x7 = p->FWmin < state->i[1];
    const float x8 = in->drive[2] == 1;
    const float x9 = p->FWmin < -state->i[2];
    const float x10 = p->FWmin < state->i[2];

    // --- Final Assignments ---
    out->vthev[0] = ((x0) ? (
   in->vbus*in->duty[0]
)
: ((x2) ? (
   x1
)
: ((x3) ? (
   -0.69999999999999996
)
: (
   x4
))));
    out->vthev[1] = ((x5) ? (
   in->vbus*in->duty[1]
)
: ((x6) ? (
   x1
)
: ((x7) ? (
   -0.69999999999999996
)
: (
   x4
))));
    out->vthev[2] = ((x8) ? (
   in->vbus*in->duty[2]
)
: ((x9) ? (
   x1
)
: ((x10) ? (
   -0.69999999999999996
)
: (
   x4
))));
    out->rthev[0] = ((x0 || x2 || x3) ? (
   p->Rdson
)
: (
   p->RdsOff
));
    out->rthev[1] = ((x5 || x6 || x7) ? (
   p->Rdson
)
: (
   p->RdsOff
));
    out->rthev[2] = ((x10 || x8 || x9) ? (
   p->Rdson
)
: (
   p->RdsOff
));
}

#endif // INVERTER_MODEL_H
