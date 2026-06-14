// For trap sectors specifically (1-6)
static inline eldriver_mc3p_sector_t TrapIncrement(eldriver_mc3p_sector_t sector, Direction dir)
{
    return static_cast<eldriver_mc3p_sector_t>(
        (dir == Direction::Forward)
            ? elmath_increment_roll(sector, ELDRIVER_MC3P_SECTOR_TRAP1, ELDRIVER_MC3P_SECTOR_TRAP6)
            : elmath_decrement_roll(sector, ELDRIVER_MC3P_SECTOR_TRAP1, ELDRIVER_MC3P_SECTOR_TRAP6));
}
static inline constexpr std::array<eldriver_mc3p_sector_t, 8> HALL_TRAP_TABLE = /** Hall->trap sector LUT (idx=hall 0..7). */ {
    ELDRIVER_MC3P_SECTOR_FLOAT,
    ELDRIVER_MC3P_SECTOR_TRAP5,
    ELDRIVER_MC3P_SECTOR_TRAP3,
    ELDRIVER_MC3P_SECTOR_TRAP4,
    ELDRIVER_MC3P_SECTOR_TRAP1,
    ELDRIVER_MC3P_SECTOR_TRAP6,
    ELDRIVER_MC3P_SECTOR_TRAP2,
    ELDRIVER_MC3P_SECTOR_FLOAT};
static inline constexpr size_t STUP_TABLE_SIZE = 6; /** STUP profile table size. */

struct Trap
{
    enum class RunMode
    {
        NOMODE = 0,
        OL,
        CL
    };
    struct
    {
        volatile Trap::RunMode run_mode = Trap::RunMode::NOMODE;
        uint32_t applied_eAngv_mRPS = 0;
        volatile q15_t duty_q15;
        eldriver_mc3p_sector_t sector;
    } state;

    struct
    {
        bool complete = false;
        volatile uint32_t pwmTicks_till_comm = 0;
        uint32_t comm_period_pwmTicks = 0;
        uint32_t time_start_us = 0;
        uint32_t est_eAngv_mRPS = 0;
        uint8_t tb_index = 0;
        eldriver_mc3p_sector_t init_sector = ELDRIVER_MC3P_SECTOR_TRAP1;
        float time_tb[STUP_TABLE_SIZE] = {0, 2, 4, 6, 8, 10};
        float voltage_tb[STUP_TABLE_SIZE] = {3, 8, 12.5, 16, 21.5, 25};
        float eAngv_RPS_tb[STUP_TABLE_SIZE] = {RPM_TO_RPS(0),  RPM_TO_RPS(100*DEFAULT_POLE_PAIRS), 
                                            RPM_TO_RPS(200*DEFAULT_POLE_PAIRS), RPM_TO_RPS(300*DEFAULT_POLE_PAIRS), 
                                            RPM_TO_RPS(400*DEFAULT_POLE_PAIRS),  RPM_TO_RPS(500*DEFAULT_POLE_PAIRS) };
    } olstup;
} trap;
//======================================================
//  CALLBACK DEFINITIONS
//======================================================
static inline void Trap_bemfzc_ComCallback()
{
}
static inline void Trap_hall_ComCallback()
{
}
//======================================================
// CORE MODE API
//======================================================
inline void Trap_olstup_cfg(float vbus_init, float time_tb[STUP_TABLE_SIZE], float voltage_tb[STUP_TABLE_SIZE], float rpm_tb[STUP_TABLE_SIZE], eldriver_mc3p_sector_t init_sector = ELDRIVER_MC3P_SECTOR_TRAP1)
{
    if (state.vbus_mV == 0)
    {
        state.vbus_mV = vbus_init * 1000;
    }
    trap.olstup.init_sector = init_sector;
    for (int i = 0; i < STUP_TABLE_SIZE; i++)
    {
        trap.olstup.time_tb[i] = time_tb[i];
        trap.olstup.voltage_tb[i] = voltage_tb[i];
        trap.olstup.eAngv_RPS_tb[i] = (rpm_tb[i] * 2 * M_PI / 60) * model.pole_pairs;
    }
}
inline void Trap_olstup_start()
{
    trap.olstup.tb_index = 0;
    trap.olstup.time_start_us = ticks_to_us(pwmTicks);
    trap.state.duty_q15 = (((uint64_t)trap.olstup.voltage_tb[0] * (1 << 15)) / state.vbus_mV);
    trap.olstup.comm_period_pwmTicks = (trap.olstup.time_tb[trap.olstup.tb_index + 1] - trap.olstup.time_tb[trap.olstup.tb_index]) * pwm_freq_hz;
    trap.olstup.pwmTicks_till_comm = trap.olstup.comm_period_pwmTicks;
    trap.state.sector = trap.olstup.init_sector;
    eldriver_mc3p_write_trap(&mc3p, trap.state.sector, trap.state.duty_q15);
    trap.olstup.complete = false;
    trap.state.run_mode = Trap::RunMode::OL;
    control.mctype = MCType::Trap;
}
inline void Trap_ol_pwmLoop()
{
    // Detect Commutation event
    if (trap.olstup.pwmTicks_till_comm == 0)
    {
        // Calculate ticks till next commutation event based on frequency
        trap.olstup.pwmTicks_till_comm = trap.olstup.comm_period_pwmTicks;
        trap.state.sector = TrapIncrement(trap.state.sector, state.dir);
    }
    trap.olstup.pwmTicks_till_comm--;
    eldriver_mc3p_write_trap(&mc3p, trap.state.sector, trap.state.duty_q15);
}
inline void Trap_ol_xmcLoop()
{
    if (!trap.olstup.complete)
    {
        /* STARTUP SECTION START */
        // Interpolate angular velocity and duty cycle and do appropiate updates
        float et = (ticks_to_us(pwmTicks) - trap.olstup.time_start_us)/1'000'000;
        if (trap.olstup.tb_index < (STUP_TABLE_SIZE - 1) && et > trap.olstup.time_tb[trap.olstup.tb_index + 1])
        {
            trap.olstup.tb_index++;
        }
        bool complete = et >= trap.olstup.time_tb[STUP_TABLE_SIZE - 1];
        if (!complete)
        {
            // We interpolate here for duty cycle and angular velocity
            float ret_slp = (float)(et - trap.olstup.time_tb[trap.olstup.tb_index]) / (trap.olstup.time_tb[trap.olstup.tb_index + 1] - trap.olstup.time_tb[trap.olstup.tb_index]);
            float voltage = trap.olstup.voltage_tb[trap.olstup.tb_index] + ret_slp * (trap.olstup.voltage_tb[trap.olstup.tb_index + 1] - trap.olstup.voltage_tb[trap.olstup.tb_index]);
            float eAngv_RPS = trap.olstup.eAngv_RPS_tb[trap.olstup.tb_index] + ret_slp * (trap.olstup.eAngv_RPS_tb[trap.olstup.tb_index + 1] - trap.olstup.eAngv_RPS_tb[trap.olstup.tb_index]);
            trap.state.applied_eAngv_mRPS = eAngv_RPS*1000;
            trap.state.duty_q15 = (voltage * 1000 * (1 << 15)) / state.vbus_mV;
            if(trap.state.applied_eAngv_mRPS > 0)trap.olstup.comm_period_pwmTicks = ((((M_PI / 3))*1000) / (float)trap.state.applied_eAngv_mRPS) * pwm_freq_hz;
            state.eAngv_mRPS = trap.state.applied_eAngv_mRPS;
        }
        trap.olstup.complete = complete;
        /* STARTUP SECTION END */
    }else{
        /* OPEN LOOP DRIVE SECTION V/I - F control */
        // TODO : SUPPORT COMMANDING VOLTAGE , CURRENT AND FREQUENCY AND DOING NECCESSARY UPDATES


    }
}
inline void Trap_cl_pwmLoop()
{
}
inline void Trap_cl_xmcLoop()
{
}
inline void Trap_init()
{
    Trap_olstup_start();
}
inline void Trap_onEnter(MCType prev_mct)
{
}
inline void Trap_pwmLoop()
{
    state.vbus_mV = (((int64_t)mc3p_sync_meas.trap.vbus_q31 * 1000) * ELDRIVER_MC3P_VS_SCALE) >> 31;
    switch (trap.state.run_mode)
    {
    case Trap::RunMode::OL:
        Trap_ol_pwmLoop();
        break;
    case Trap::RunMode::CL:
        Trap_cl_pwmLoop();
        break;
    default:
        break;
    }
}
inline void Trap_xmcLoop()
{
    switch (trap.state.run_mode)
    {
    case Trap::RunMode::OL:
        Trap_ol_xmcLoop();
        break;
    case Trap::RunMode::CL:
        Trap_cl_xmcLoop();
        break;
    default:
        break;
    }
}
inline void Trap_onExit()
{
}
