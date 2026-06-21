
struct Foc
{
    static constexpr size_t OLSTUP_TABLE_SIZE = 6; /** STUP profile table size. */

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
        q31_t vd_q31 = 0;
        q31_t vq_q31 = 0;
        q31_t Id_q31 = 0;
        q31_t Iq_q31 = 0;
        q31_t ECq_sp_q31 = 0;
        q31_t ECd_sp_q31 = 0;
        volatile int16_t mod_idx_max_q3p12 = ((int16_t)(1 * INT16_MAX) >> 3);
    } state;

    struct
    {
        bool complete = false;
        uint32_t time_start_us = 0;
        uint32_t est_eAngv_mRPS = 0;
        uint8_t tb_index = 0;
        float time_tb[OLSTUP_TABLE_SIZE] = {0, 2, 4, 6, 8, 10};
        float EC_tb[OLSTUP_TABLE_SIZE] = {3, 5, 7, 9, 10.5, 11.8};
        float eAngv_RPS_tb[OLSTUP_TABLE_SIZE] = {RPM_TO_RPS(0),  RPM_TO_RPS(100*1), 
                                            RPM_TO_RPS(200*1), RPM_TO_RPS(300*1), 
                                            RPM_TO_RPS(400*1),  RPM_TO_RPS(500*1) };
    } olstup;

    float Id_Kp = 0;
    float Id_Ki = 0;
    float Id_Kd = 0;
    float Iq_Kp = 0;
    float Iq_Ki = 0;
    float Iq_Kd = 0;
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
void Foc_olstup_cfg(float vbus_init,const float time_tb[Foc::OLSTUP_TABLE_SIZE],const float ec_tb[Foc::OLSTUP_TABLE_SIZE],const float rpm_tb[Foc::OLSTUP_TABLE_SIZE]);
void Foc_olstup_start();
void Foc_init();
void Foc_onEnter(MCType prev_mct);
void Foc_onExit();
void Foc_pwmLoop();
void Foc_xmcLoop();

