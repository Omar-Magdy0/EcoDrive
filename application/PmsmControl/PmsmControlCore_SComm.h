

struct SComm
{
    enum class IDStage
    {
        IDLE,
        RS_ID,           /** Stator resistance identification. */
        LD_ID,           /** D-axis inductance identification. */
        REST0,
        LQ_ID            /** Q-axis inductance identification. */
    };
    enum class IDSubStage
    {
        ESettle_Start,
        ESettle_Wait,
        Active_Sampling
    };
    static constexpr uint16_t oversample = 1 << PmsmControlConf::OVERSAMPLE_BITS; //using a power of two, for shift based division and averaging (nearly free)

    //Config runtime
    uint8_t hfi_N = 20;
    q31_t dc_vinj_q31;
    q31_t hfi_vinj_q31;

    //States
    int64_t accumulate0;
    int64_t accumulate1;
    q31_t hfi_Angv_RPT_q31 = ((float)2.0/hfi_N)*INT32_MAX;
    q31_t hfi_angle_q31;              /** High-frequency injection angle */
    volatile uint32_t eSettle_start_tick = 0;
    volatile IDStage idstage;
    volatile IDSubStage idsub;
    volatile uint16_t samples_counter; /** Remaining samples for current step. */
    
    
    //Results
    PmsmControlTypes::Model model;
} scomm;

void SComm_init();    /** Initialize self-commissioning. */
void SComm_onEnter(PmsmControlTypes::MCMode prev_mct);    /** Initialize self-commissioning. */
void SComm_onExit();
void SComm_pwmLoop(); /** Self-commissioning PWM loop. */
void SComm_xmcLoop(); /** Self-commissioning slow loop. */