#include "virtual_pmsm.h"
#include <math.h>



void euf_step(float *x, int n, float dt, DerivFunc f, float t, const void *params)
{
    float dxdt[MAX_STATE];
    f(x, t, params, dxdt);
    for(int i = 0; i < n; i++)
    {
        x[i] = x[i] + dxdt[i]*dt;
    }
}

void rk4_step(float *x, int n, float dt, DerivFunc f, float t, const void *params)
{
    float k1[MAX_STATE];
    float k2[MAX_STATE];
    float k3[MAX_STATE];
    float k4[MAX_STATE];
    float xtmp[MAX_STATE];

    // k1
    f(x, t, params, k1);

    // k2
    for(int i = 0; i < n; i++)
        xtmp[i] = x[i] + 0.5f * dt * k1[i];

    f(xtmp, t + 0.5f * dt, params, k2);

    // k3
    for(int i = 0; i < n; i++)
        xtmp[i] = x[i] + 0.5f * dt * k2[i];

    f(xtmp, t + 0.5f * dt, params, k3);

    // k4
    for(int i = 0; i < n; i++)
        xtmp[i] = x[i] + dt * k3[i];

    f(xtmp, t + dt, params, k4);

    // final update
    for(int i = 0; i < n; i++)
    {
        x[i] += dt * (
            k1[i]
          + 2.0f * k2[i]
          + 2.0f * k3[i]
          + k4[i]
        ) / 6.0f;
    }
}

struct{
    float v[3];
    float i[3];
    float torque;
    float theta;
    float omega;
}vpmsm_sample;


#include "virtual_pmsm_gui.h"


    
VpmsmHelper vpmsmHelper = VpmsmHelper(1000);


#include "elcore.h"
void pmsm_didt(const float* i, float t, const void* params, float *didt)
{
    pmsm_model* ctx = (pmsm_model*)(params);
    for(int j = 0; j < 3; j++){
        didt[j] = (ctx->e_bus->v[j] - ctx->elec.vn - ctx->Ra*i[j] - ctx->elec.e[j])/ctx->elec.L[j]; 
    }
}

void pmsm_dmechdt(const float* mech, float t, const void* params, float *dmechdt)
{
    pmsm_model* ctx = (pmsm_model*)(params);
    dmechdt[0] = (ctx->mech.torque_e - ctx->mech.torque_l - ctx->B * mech[0])/ctx->J;
    dmechdt[1] = ctx->mech.omega;
}

void pmsm_update_mech(pmsm_model *m)
{
    float theta_e = m->pole_pairs * m->mech.theta;
    m->elec.id=   cos(theta_e)*m->e_bus->i[0] + cos(theta_e - 2*M_PI/3)*m->e_bus->i[1] + cos(theta_e + 2*M_PI/3)*m->e_bus->i[2];
    m->elec.iq= - sin(theta_e)*m->e_bus->i[0] - sin(theta_e - 2*M_PI/3)*m->e_bus->i[1] - sin(theta_e + 2*M_PI/3)*m->e_bus->i[2];
    m->mech.torque_e = (3.0/2)*m->pole_pairs*(m->Kt*m->elec.iq + (m->Ld - m->Lq)*m->elec.id*m->elec.iq);
}

void pmsm_update_elec(pmsm_model *m)
{
    //Back Emf update
    float omega_e = m->pole_pairs * m->mech.omega;
    float theta_e = m->pole_pairs * m->mech.theta;
    if(m->bemf_type == SINUSOIDAL){
        m->elec.e[0] = m->Ke * omega_e * sinf(theta_e);
        m->elec.e[1] = m->Ke * omega_e * sinf(theta_e - 2.0f * (M_PI / 3.0f));
        m->elec.e[2] = m->Ke * omega_e * sinf(theta_e + 2.0f * (M_PI / 3.0f)); 
    }else if(m->bemf_type == TRAPEZOIDAL){
        m->elec.e[0] = m->Ke * omega_e * 1;
        m->elec.e[1] = m->Ke * omega_e * 1;
        m->elec.e[2] = m->Ke * omega_e * 1;
    }

    //average and saliency
    float L0 = (2.0f * m->Ld + m->Lq) / 3.0f;
    float L2 = (m->Ld - m->Lq) / 2.0f;

    // phase offsets
    m->elec.L[0] = L0 + L2 * cosf(2.0f * theta_e);                    // phase A
    m->elec.L[1] = L0 + L2 * cosf(2.0f * (theta_e - 2.0f*M_PI/3.0f)); // phase B
    m->elec.L[2] = L0 + L2 * cosf(2.0f * (theta_e + 2.0f*M_PI/3.0f)); // phase C


    m->elec.vn = (m->e_bus->v[0] + m->e_bus->v[1] + m->e_bus->v[2])/3;
    //check for floating phases for voltage update
    for(int i = 0; i < 3; i++)
    {
        if(!m->e_bus->driven[i])
        {
            m->e_bus->v[i] = m->elec.e[i] + m->elec.vn;
        }
    }
}

void pmsm_step(pmsm_model *m, float dt, float t)
{
    pmsm_update_elec(m);
    //step through the electrical model;
    rk4_step(m->e_bus->i, 3, dt, pmsm_didt, t, m);
    pmsm_update_mech(m);
    //step through the mechanical model;
    float mech[2] = {m->mech.omega, m->mech.theta};
    rk4_step(mech, 2, dt, pmsm_dmechdt, t, m);
    m->mech.omega = mech[0]; m->mech.theta = mech[1];
    
    vpmsmHelper.elec_scope.write(m->e_bus->v);
}

void pmsm_init(pmsm_model *m, elec_bus *bus)
{
    m->e_bus = bus;
}

void pmsm_deinit()
{
}


