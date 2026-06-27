
struct Foc
{
    enum class RunMode
    {
        NOMODE = 0,
        OL,
        CL
    };
    struct
    {
        arm_pid_instance_q31 Id_pid;
        q31_t Id_feedforward = 0;
        arm_pid_instance_q31 Iq_pid;
        q31_t Iq_feedforward = 0;
        q31_t Vd_q31 = 0;
        q31_t Vq_q31 = 0;
        q31_t Id_q31 = 0;
        q31_t Iq_q31 = 0;
        q31_t ECq_sp_q31 = 0;
        q31_t ECd_sp_q31 = 0;
        volatile int16_t mod_idx_max_q3p12 = ((int16_t)(1 * INT16_MAX) >> 3);
    } state;

    struct
    {
        bool complete = false;
        uint32_t time_start_ms = 0;
        uint32_t est_eAngv_mRPS = 0;
        uint8_t tb_index = 0;
        PmsmControlTypes::ElecMode ecmode_temp;
        PmsmControlTypes::ConfigOlstup cfg;
    } olstup;
    volatile RunMode run_mode = RunMode::NOMODE;
} foc;
//======================================================
//  CALLBACK DEFINITIONS
//======================================================
void Foc_bemfzc_ComCallback();
void Foc_hall_ComCallback();

//======================================================
// CORE MODE API
//======================================================
void Foc_olstup_start();
void Foc_init();
void Foc_onEnter(PmsmControlTypes::MCMode prev_mct);
void Foc_onExit();
void Foc_pwmLoop();
void Foc_xmcLoop();

