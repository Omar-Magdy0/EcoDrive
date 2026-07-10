struct Foc
{
    struct
    {
        arm_pid_instance_q31 Id_pid;
        arm_pid_instance_q31 Iq_pid;
        q31_t Vd_q31 = 0;
        q31_t Vq_q31 = 0;
        q31_t Id_q31 = 0;
        q31_t Iq_q31 = 0;
        q31_t ECq_sp_q31 = 0;
        q31_t ECd_sp_q31 = 0;
        volatile int16_t mod_idx_max_q3p12 = static_cast<int16_t>((1 * INT16_MAX) >> 3);
    } state;

    Olstup olstup;
} foc;

//======================================================
// CORE MODE API
//======================================================
void Foc_olstup_start();
void Foc_init();
void Foc_onEnter(PmsmControlTypes::MCMode prev_mct);
void Foc_onExit();
void Foc_pwmLoop();
void Foc_xmcLoop();
