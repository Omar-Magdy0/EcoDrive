

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
    static constexpr uint8_t oversample_bits = 15;
    static constexpr uint16_t oversample = 1 << oversample_bits; //using a power of two, for shift based division and averaging (nearly free)
    static constexpr uint8_t hfi_N = 20;

    uint16_t eSettle_min_ticks = 5000;
    volatile uint32_t eSettle_start_tick = 0;
    volatile IDStage idstage;
    volatile IDSubStage idsub;
    volatile uint16_t samples_counter; /** Remaining samples for current step. */
    
    int64_t accumulate0;
    int64_t accumulate1;
    q31_t hfi_Angv_RPT_q31 = ((float)2.0/hfi_N)*INT32_MAX;
    q31_t hfi_angle_q31;              /** High-frequency injection angle */
    q31_t dc_vinj_q31 = ELDRIVER_MC3P_FLOAT_TO_VS(2.5);
    q31_t hfi_vinj_q31 = ELDRIVER_MC3P_FLOAT_TO_VS(12);

    float R;
    float Ld;
    float Lq;
} scomm;

void SComm_init();    /** Initialize self-commissioning. */
void SComm_onEnter(MCType prev_mct);    /** Initialize self-commissioning. */
void SComm_onExit();
void SComm_pwmLoop(); /** Self-commissioning PWM loop. */
void SComm_xmcLoop(); /** Self-commissioning slow loop. */