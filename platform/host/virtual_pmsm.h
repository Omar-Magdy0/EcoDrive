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


typedef struct{
    struct{
        float i[3];
        float v[3];
        float vn;
        float id;
        float iq;
        float e[3];
        float L[3];
    }elec;

    struct{
        float torque_l;
        float torque_e;
        float theta;
        float omega;
    }mech;

    elec_bus *e_bus;

    float Ra;
    float Ld;
    float Lq;
    float Ke;
    float Kt;
    float J;
    float B;
    float pole_pairs;
    bemf_type_t bemf_type;
}pmsm_model;




void pmsm_write(pmsm_model *m, float duty[3], bool drive[3], float );
void pmsm_step(pmsm_model *m, float dt, float t);
void pmsm_init(pmsm_model *m, elec_bus *bus);


#ifdef __cplusplus
}
#endif


#ifdef __cplusplus

#include "ElcoreScopeStream.hpp"
#define VPMSM_ELEC_CHANNELS 3
class VpmsmHelper{

    public:
    float *elec_buf;
    int elec_scope_size;
    ElcoreScopeStream<float> elec_scope;

    VpmsmHelper(unsigned int elec_buf_depth):
    elec_buf(new float[VPMSM_ELEC_CHANNELS * elec_buf_depth]),
    elec_scope(elec_buf, VPMSM_ELEC_CHANNELS, 10, elec_buf_depth/10)
    {
        elec_scope_size = elec_buf_depth;
        elec_scope.trigger_level = 0;
    }
    ~VpmsmHelper()
    {
        delete elec_buf;
    }
};
#endif
