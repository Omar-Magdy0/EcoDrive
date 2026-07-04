// For trap sectors specifically (1-6)
/*
struct Trap
{
    struct
    {
        q31_t Vcomm_q31 = 0;
        eldriver_mc3p_sector_t sector;
        q31_t I_feedforward = 0;
        q31_t EC_sp_q31 = 0; //electrical setpoint
        arm_pid_instance_q31 I_pid;
        PmsmControlTypes::Direction comm_dir;
    } state;
    Olstup olstup;
} trap;
//======================================================
//  CALLBACK DEFINITIONS
//======================================================
void Trap_bemfzc_ComCallback();
void Trap_hall_ComCallback();

//======================================================
// CORE MODE API
//======================================================
void Trap_olstup_start();
void Trap_init();
void Trap_onEnter(PmsmControlTypes::MCMode prev_mct);
void Trap_onExit();
void Trap_pwmLoop();
void Trap_xmcLoop();

*/



/*
PmsmControlTypes::ERR PmsmControl::configTrapPid(PmsmControlTypes::Pid Ibus_pid)
{
    //Sanity checks
    if(Ibus_pid.Kp > 1 || Ibus_pid.Kp < -1){return ERR::CONFIG_BAD;}
    if(Ibus_pid.Ki > 1 || Ibus_pid.Ki < -1){return ERR::CONFIG_BAD;}
    if(Ibus_pid.Kd > 1 || Ibus_pid.Kd < -1){return ERR::CONFIG_BAD;}
    //fixed point scaling and conversion
    mc.trap.state.I_pid.Kp = Ibus_pid.Kp * INT32_MAX;
    mc.trap.state.I_pid.Ki = Ibus_pid.Ki * INT32_MAX;
    mc.trap.state.I_pid.Kd = Ibus_pid.Kd * INT32_MAX;
    //Atomic operation , We do this quick
    uint32_t mask = eldriver_atomic_start();
    arm_pid_init_q31(&mc.trap.state.I_pid, 0);
    eldriver_atomic_end(mask);
    return ERR::OK;
}
*/