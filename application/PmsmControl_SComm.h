

struct
{
    enum class IDStage
    {
        RESET = 0,       /** Initial / reset stage. */
        DAXIS_ALIGN,     /** D-axis alignment stage. */
        RS_ID,           /** Stator resistance identification. */
        LD_ID,           /** D-axis inductance identification. */
        LQ_ID,           /** Q-axis inductance identification. */
        ELEC_POSTPROCESS /** Final post-processing stage. */
    };

    volatile IDStage IDstage;
    IDStage IDstage_last;
    elcore_swttimer_t timer{};    /** Stage timing for self-commissioning. */
    uint32_t remaining_id_samples; /** Remaining samples for current step. */
    float id_sin_prod_2;          /** D-axis correlation accumulator. */
    float id_cos_prod_2;          /** D-axis correlation accumulator. */
    float iq_sin_prod_2;          /** Q-axis correlation accumulator. */
    float iq_cos_prod_2;          /** Q-axis correlation accumulator. */
    float rs_dc;                  /** Estimated DC resistance. */
    uint16_t iir_filtered_cnt;    /** IIR filter counter for smoothing. */
    float hfi_angle;              /** High-frequency injection angle estimate. */
} SComm;

void SComm_init();    /** Initialize self-commissioning. */
void SComm_onEnter(MCType prev_mct);    /** Initialize self-commissioning. */
void SComm_onExit();
void SComm_pwmLoop(); /** Self-commissioning PWM loop. */
void SComm_xmcLoop(); /** Self-commissioning slow loop. */
