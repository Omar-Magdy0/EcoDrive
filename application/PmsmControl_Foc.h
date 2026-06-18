
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
        volatile Foc::RunMode run_mode = Foc::RunMode::NOMODE;
        arm_pid_instance_q15 Id_pid;
        q15_t Id_feedforward = 0;
        arm_pid_instance_q15 Iq_pid;
        q15_t Iq_feedforward = 0;
        q31_t vd_q31 = 0;
        q31_t vq_q31 = 0;
        q31_t Id_q31 = 0;
        q31_t Iq_q31 = 0;
        volatile q31_t ECq_sp_q31 = 0;
        volatile q31_t ECd_sp_q31 = 0;
    } state;

    struct
    {
        bool complete = false;
        uint32_t time_start_us = 0;
        uint32_t est_eAngv_mRPS = 0;
        uint8_t tb_index = 0;
        eldriver_mc3p_sector_t init_sector = ELDRIVER_MC3P_SECTOR_TRAP1;
        float time_tb[STUP_TABLE_SIZE] = {0, 2, 4, 6, 8, 10};
        float voltage_tb[STUP_TABLE_SIZE] = {3, 8, 16, 20, 24, 29};
        float eAngv_RPS_tb[STUP_TABLE_SIZE] = {RPM_TO_RPS(0),  RPM_TO_RPS(100*DEFAULT_POLE_PAIRS), 
                                            RPM_TO_RPS(200*DEFAULT_POLE_PAIRS), RPM_TO_RPS(300*DEFAULT_POLE_PAIRS), 
                                            RPM_TO_RPS(400*DEFAULT_POLE_PAIRS),  RPM_TO_RPS(500*DEFAULT_POLE_PAIRS) };
    } olstup;
} foc;
//======================================================
//  CALLBACK DEFINITIONS
//======================================================
void Foc_bemfzc_ComCallback();
void Foc_hall_ComCallback();

//======================================================
// CORE MODE API
//======================================================
void Foc_olstup_cfg(float vbus_init, float time_tb[STUP_TABLE_SIZE], float voltage_tb[STUP_TABLE_SIZE], float rpm_tb[STUP_TABLE_SIZE]);
void Foc_olstup_start();
void Foc_init();
void Foc_onEnter(MCType prev_mct);
void Foc_onExit();
void Foc_pwmLoop();
void Foc_xmcLoop();

