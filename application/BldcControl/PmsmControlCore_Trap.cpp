/*
#include "PmsmControlCore.h"
using namespace PmsmControlTypes;
void PmsmControlCore::Trap_init()
{

}
void PmsmControlCore::Trap_onEnter(MCMode prev_mct){
    Trap_olstup_start();
}
void PmsmControlCore::Trap_onExit(){}
//==========================================
// OPEN LOOP CODE
//==========================================
void PmsmControlCore::Trap_olstup_start()
{

}

void PmsmControlCore::Trap_xmcLoop()
{
    posDriver.xTickUpdateTrap();
    int32_t Theta_q15p16 = posDriver.getMechAng_q31();
    state.mechAngv_RPXT_q7p24 = (Theta_q15p16 - state.mechTheta_q15p16);
    state.mechTheta_q15p16 = Theta_q15p16;
    int32_t ep_q15p16 = 0;
    int32_t es_q7p24 = 0;
    if(control.run_mode == RunMode::ClosedLoop)
    {   //Closed loop here , Controllers are active
        if(control.mech_mode == MechMode::Position)
        {
            ep_q15p16 = state.mechTheta_sp_q15p16 - state.mechTheta_q15p16;
        }else
        {
            ep_q15p16 = 0;
            control.position_pid.state[2] = state.mechAngv_RPXT_sp_q7p24;
        }
        state.mechAngv_RPXT_sp_q7p24 = arm_pid_q31(&control.position_pid, ep_q15p16);
        //Clamping logic here for speed
        if(control.mech_mode == MechMode::Speed)
        {
            es_q7p24 = state.mechAngv_RPXT_sp_q7p24 - state.mechAngv_RPXT_q7p24;
        }else
        {
            es_q7p24 = 0;
            control.speed_pid.state[2] = trap.state.EC_sp_q31;
        }
        trap.state.EC_sp_q31 = arm_pid_q31(&control.speed_pid, es_q7p24);  
        //Clamping logic here for Torque based on active controller 
    }
    state.eAngv_RPT_q31 = q31_t( (state.eAngv_RPS / pwm_freq_hz) * (INT32_MAX/M_PI) );
}


//==========================================
// Closed LOOP CODE
//==========================================
void PmsmControlCore::Trap_pwmLoop()
{
    state.Vbus_q31 = (mc3p_sync_meas.trap.vbus_q31);
    state.Ibus_q31 = (mc3p_sync_meas.trap.cbus_q31);
    posDriver.getElecAng_q31();
    state.eTheta_q31 += state.eAngv_RPT_q31;
    q15_t duty;
    q31_t e = 0;
    posDriver.pTickUpdateTrap();
    if(control.elec_mode == ElecMode::Current)
    {
        //calculate error normally
        e = (trap.state.EC_sp_q31 - mc3p_sync_meas.trap.cbus_q31);
    }else
    {
        e = 0;
        trap.state.I_pid.state[2] = trap.state.EC_sp_q31;
    }
    //Always divide error by 4 to avoid overflow of pid accumulator (cmsis_pid reference)
    e = e>>2;
    trap.state.Vcomm_q31 = arm_pid_q31(&trap.state.I_pid, e) + trap.state.I_feedforward;
    //Saturate output
    int32_t vbus_lim_q31 = ((int64_t)state.Vbus_q31 * mc3p.duty_max_q15)>>15;
    if(trap.state.Vcomm_q31 > vbus_lim_q31){trap.state.Vcomm_q31 = vbus_lim_q31; trap.state.I_pid.state[2] = trap.state.Vcomm_q31;}
    duty = (int32_t)(trap.state.Vcomm_q31)/(state.Vbus_q31 >> 15);
    eldriver_mc3p_write_trap(&mc3p, trap.state.sector, duty);
}

*/

/*



PmsmControlTypes::ERR PmsmControl::configTrapOlstup(PmsmControlTypes::ConfigOlstup cfg)
{
    if(mc.control.mc_mode != MCMode::Idle){return ERR::CONFIG_NOT_ALLOWED_MOTOR_RUNNING;}
    //Table sanity checks
    if(cfg.time_ms_tb[0] != 0){return ERR::CONFIG_BAD;}
    for(int i = 0; i < olstup_tb_size() - 1; i++)
    {
        if(cfg.time_ms_tb[i + 1] < cfg.time_ms_tb[i]){return ERR::CONFIG_BAD;}
    }
    for(int i = 0; i < olstup_tb_size(); i++)
    {
        if(cfg.ec_tb[i] < 0){return ERR::CONFIG_BAD;}
        if(cfg.rpm_tb[i] < 0){return ERR::CONFIG_BAD;}
    }
    if(cfg.elec_mode != ElecMode::Current && cfg.elec_mode != ElecMode::Voltage){return ERR::CONFIG_BAD;}
    if(cfg.vbus_init <= 0){return ERR::CONFIG_BAD;}
    if(cfg.init_sector < ELDRIVER_MC3P_SECTOR_TRAP1 || cfg.init_sector > ELDRIVER_MC3P_SECTOR_TRAP6){return ERR::CONFIG_BAD;}
    mc.trap.olstup.cfg = cfg;
    return ERR::OK;
}


*/

/*

void PosOpen::pTickUpdateTrap_impl()
{
    // Detect Commutation event
    if (pTicks_till_comm == 0)
    {
        pTicks_till_comm = comm_pTicks;
        mc.trap.state.sector = TrapIncrement(mc.trap.state.sector, mc.trap.state.comm_dir);
    }
    pTicks_till_comm = pTicks_till_comm - 1;
}

void PosOpen::xTickUpdateTrap_impl()
{
    PmsmControlTypes::Direction dir_sp = (PmsmControlTypes::Direction)((uint32_t)mc.state.mechAngv_RPXT_sp_q7p24>>31);
    PmsmControlTypes::Direction dir = (PmsmControlTypes::Direction)((uint32_t)drive_rpxt_q7p24>>31);
    bool dir_reverse = dir_sp != dir;
    bool resync = (drive_rpxt_q7p24 < rpxt_min_thresh)&&(drive_rpxt_q7p24 > -rpxt_min_thresh) && (mc.state.mechAngv_RPXT_sp_q7p24 != 0); 
    switch(state_fsm)
    {
        case PosDriverFsm::Unsync:
            if(resync)//scan for direction change or movement from stop
            {
                state_fsm = PmsmControlTypes::PosDriverFsm::Align;
            }else{
                break;
            }
        case PosDriverFsm::Align://Apply align vector
            mc.trap.olstup.tb_index = 0;
            mc.trap.olstup.time_start_ms = mc.xTicks_to_ms(mc.xTicks);
            mc.trap.state.EC_sp_q31 = ELDRIVER_MC3P_FLOAT_TO_VS(mc.trap.olstup.cfg.ec_tb[0]);
            comm_pTicks = (float)(mc.trap.olstup.cfg.time_ms_tb[mc.trap.olstup.tb_index + 1] - mc.trap.olstup.cfg.time_ms_tb[mc.trap.olstup.tb_index]) * mc.pwm_freq_hz / 1000;
            mc.state.eTheta_q31 = 0;
            mc.trap.olstup.is_complete = false;
            pTicks_till_comm = comm_pTicks;
            mc.trap.state.sector = mc.trap.olstup.cfg.init_sector;
            mc.trap.olstup.elec_mode_temp = mc.control.elec_mode;
            mc.control.elec_mode = mc.trap.olstup.cfg.elec_mode;
            //Apply alignment sector
            state_fsm = PmsmControlTypes::PosDriverFsm::Ramp;
            mc.trap.state.comm_dir = dir_sp;
            drive_rpxt_q7p24 = 0;
            break;
        case PosDriverFsm::Ramp:
            //Fsm and interpolation
            //Interpolation part
            //Apply ramp of openloop startup table (olstups)
            if(!mc.trap.olstup.is_complete)
            {
                float rps; q31_t ec_sp_q31;
                ec_sp_q31 = mc.trap.state.EC_sp_q31;
                rps = mc.rpxt_to_rps(drive_rpxt_q7p24);
                rps = rps*(dir == Direction::Forward?1:-1);
                mc.olstup_lut_run(mc.trap.olstup, ec_sp_q31, rps);
                mc.trap.state.EC_sp_q31 = ec_sp_q31;
                if(rps > 0)comm_pTicks = ((((M_PI / 3))) / (float)(rps*mc.model.pole_pairs)) * mc.pwm_freq_hz;
                drive_rpxt_q7p24 = mc.rps_to_rpxt(rps);
                break;
            }else{
                state_fsm = PmsmControlTypes::PosDriverFsm::OpenLocked;
            }
        case PosDriverFsm::OpenLocked:
            if(resync)//scan for direction change or stop from movement
            {
                state_fsm = PmsmControlTypes::PosDriverFsm::Unsync;
            }else{

            }
            break;
        default:
            break;
    }
}

*/