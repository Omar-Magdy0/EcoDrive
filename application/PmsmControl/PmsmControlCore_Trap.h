// For trap sectors specifically (1-6)
static inline eldriver_mc3p_sector_t TrapIncrement(eldriver_mc3p_sector_t sector, PmsmControlTypes::Direction dir)
{
    return static_cast<eldriver_mc3p_sector_t>(
        (dir == PmsmControlTypes::Direction::Forward)
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
        q31_t Vcomm_q31 = 0;
        eldriver_mc3p_sector_t sector;
        q31_t I_feedforward = 0;
        q31_t EC_sp_q31 = 0; //electrical setpoint
        arm_pid_instance_q31 I_pid;
    } state;

    struct
    {
        bool complete = false;
        volatile uint32_t pwmTicks_till_comm = 0;
        uint32_t comm_period_pwmTicks = 0;
        uint32_t time_start_ms = 0;
        uint8_t tb_index = 0;
        PmsmControlTypes::ElecMode ecmode_temp;
        PmsmControlTypes::ConfigOlstup cfg;
    } olstup;
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
void Trap_olstup_start();
void Trap_init();
void Trap_onEnter(PmsmControlTypes::MCMode prev_mct);
void Trap_onExit();
void Trap_pwmLoop();
void Trap_xmcLoop();