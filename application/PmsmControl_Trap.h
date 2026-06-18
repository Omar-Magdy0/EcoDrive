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
        volatile q31_t Vcomm_q31 = 0;
        volatile eldriver_mc3p_sector_t sector;
        arm_pid_instance_q15 I_pid;
        q15_t I_feedforward = 0;
        q31_t EC_sp_q31 = 0; //electrical setpoint
    } state;

    struct
    {
        bool complete = false;
        volatile uint32_t pwmTicks_till_comm = 0;
        uint32_t comm_period_pwmTicks = 0;
        uint32_t time_start_us = 0;
        uint8_t tb_index = 0;
        eldriver_mc3p_sector_t init_sector = ELDRIVER_MC3P_SECTOR_TRAP1;
        float time_tb[STUP_TABLE_SIZE] = {0, 2, 4, 6, 8, 10};
        float voltage_tb[STUP_TABLE_SIZE] = {3, 8, 16, 20, 24, 30};
        float eAngv_RPS_tb[STUP_TABLE_SIZE] = {RPM_TO_RPS(0),  RPM_TO_RPS(100*DEFAULT_POLE_PAIRS), 
                                            RPM_TO_RPS(200*DEFAULT_POLE_PAIRS), RPM_TO_RPS(300*DEFAULT_POLE_PAIRS), 
                                            RPM_TO_RPS(400*DEFAULT_POLE_PAIRS),  RPM_TO_RPS(500*DEFAULT_POLE_PAIRS) };
    } olstup;
} trap;
//======================================================
//  CALLBACK DEFINITIONS
//======================================================
void Trap_bemfzc_ComCallback();
void Trap_hall_ComCallback();

//======================================================
// CORE MODE API
//======================================================
void Trap_olstup_cfg(float vbus_init, float time_tb[STUP_TABLE_SIZE], float voltage_tb[STUP_TABLE_SIZE], float rpm_tb[STUP_TABLE_SIZE], eldriver_mc3p_sector_t init_sector = ELDRIVER_MC3P_SECTOR_TRAP1);
void Trap_olstup_start();
void Trap_init();
void Trap_onEnter(MCType prev_mct);
void Trap_onExit();
void Trap_pwmLoop();
void Trap_xmcLoop();

