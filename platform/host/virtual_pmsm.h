#pragma once

#ifdef __cplusplus
extern "C"{
#endif
#include <stdint.h>
#include <stdbool.h>


typedef enum{
    SINUSOIDAL = 0,
    TRAPEZOIDAL = 1,
}bemf_type_t;


#define MAX_STATE 10
typedef void (*DerivFunc)(const float* x, float t, const void* params, float *dxdt);
void euf_step(float *x, int n, float dt, DerivFunc f, float t, const void* params);
void rk4_step(float *x, int n, float dt, DerivFunc f, float t, const void* params);




#ifdef __cplusplus
}
#endif


#ifdef __cplusplus

#include "ElcoreScopeStream.hpp"
#define VPMSM_ELEC_CHANNELS 3
class SimHelper{
    public:
    float *elec_buf;
    int elec_scope_size;
    ElcoreScopeStream<float> elec_scope;

    struct MechanicalData {
        float speed = 0.0f;
        float torque = 0.0f;
    } mechanical;

    float dc_link_voltage = 0.0f;

    SimHelper(unsigned int elec_buf_depth):
    elec_buf(new float[VPMSM_ELEC_CHANNELS * elec_buf_depth]),
    elec_scope(elec_buf, VPMSM_ELEC_CHANNELS, 10, elec_buf_depth/10)
    {
        elec_scope_size = elec_buf_depth;
        elec_scope.trigger_level = 0;
    }
    ~SimHelper()
    {
        delete[] elec_buf;
    }

    void start() {
        elec_scope.triggered = false;
        elec_scope.frozen = false;
    }

    void stop() {
        elec_scope.frozen = true;
    }
};
#endif
