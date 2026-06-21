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
        volatile q31_t Vcomm_q31 = 0;
        volatile eldriver_mc3p_sector_t sector;
        arm_pid_instance_q31 I_pid;
        q31_t I_feedforward = 0;
        q31_t EC_sp_q31 = 0; //electrical setpoint
    } state;

    static constexpr size_t OLSTUP_TABLE_SIZE = 6; /** STUP profile table size. */

    struct
    {
        bool complete = false;
        volatile uint32_t pwmTicks_till_comm = 0;
        uint32_t comm_period_pwmTicks = 0;
        uint32_t time_start_us = 0;
        uint8_t tb_index = 0;
        eldriver_mc3p_sector_t init_sector = ELDRIVER_MC3P_SECTOR_TRAP1;
        float time_tb[OLSTUP_TABLE_SIZE] = {0, 2, 4, 6, 8, 10};
        float EC_tb[OLSTUP_TABLE_SIZE] = {3, 6, 12, 16, 20, 24};
        float eAngv_RPS_tb[OLSTUP_TABLE_SIZE] = {RPM_TO_RPS(0),  RPM_TO_RPS(100*1), 
                                            RPM_TO_RPS(200*1), RPM_TO_RPS(300*1), 
                                            RPM_TO_RPS(400*1),  RPM_TO_RPS(500*1) };
    } olstup;

    float I_Kp;
    float I_Kd;
    float I_Ki;
    volatile RunMode run_mode = RunMode::NOMODE;
} trap;
//======================================================
//  CALLBACK DEFINITIONS
//======================================================
void Trap_bemfzc_ComCallback();
void Trap_hall_ComCallback();

//======================================================
// CORE MODE API
//======================================================
void Trap_olstup_cfg(float vbus_init,const float time_tb[Trap::OLSTUP_TABLE_SIZE],const float ec_tb[Trap::OLSTUP_TABLE_SIZE],const float rpm_tb[Trap::OLSTUP_TABLE_SIZE], eldriver_mc3p_sector_t init_sector = ELDRIVER_MC3P_SECTOR_TRAP1);
void Trap_olstup_start();
void Trap_init();
void Trap_onEnter(MCType prev_mct);
void Trap_onExit();
void Trap_pwmLoop();
void Trap_xmcLoop();

