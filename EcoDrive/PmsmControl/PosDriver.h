#pragma once
#include "eldriver_conf.h"
#include "eldriver_hall.h"
#include "PmsmControlTypes.h"
#include "elmath.h"
#include "eldriver_mc3p.h"
#include <cstdint>

#ifndef ELDRIVER_HALL1_ENABLED
#define ELDRIVER_BEMFZC_ENABLED
#endif

class PmsmControlCore;
template <typename Impl>
class PosDriverBase
{
protected:
    PmsmControlCore &mc;
    Impl &self() { return static_cast<Impl &>(*this); }
    const Impl &self() const { return static_cast<const Impl &>(*this); }
    PmsmControlTypes::PosDriverFsm state_fsm;
    uint32_t commAngle_q31;
    void align();
    void olramp();
public:
    explicit PosDriverBase(PmsmControlCore &mc) : mc(mc) {}

    bool init() {return self().init_impl();}
    int64_t getMechAng_q31p32() const { return self().getMechAng_q31p32_impl(); }
    int32_t getElecAng_q31() const { return self().getElecAng_q31_impl(); }
    PmsmControlTypes::PosDriverType getType()const { return self().getType_impl(); }
    void pTickUpdate(){self().pTickUpdate_impl();}
    void xTickUpdate(){self().xTickUpdate_impl();}
    static constexpr uint8_t modeSupport(){return self().modeSupport_impl();}
    void setCommAngle(int32_t commAngle_q31) { self().setCommAngle_impl(commAngle_q31);}
    void reset() { self().reset_impl(); }
};

class PosOpen : public PosDriverBase<PosOpen>
{
    volatile uint32_t pTicks_till_comm = 0;
    volatile int32_t drive_rpxt_q7p24 = 0;
    volatile uint32_t comm_pTicks = 0;
    volatile int32_t rpxt_min_thresh = 0;
    public:
    explicit PosOpen(PmsmControlCore &mc) : PosDriverBase<PosOpen>(mc) {}
    bool init_impl();
    PmsmControlTypes::PosDriverType getType_impl()
    {
        return PmsmControlTypes::PosDriverType::Open;
    }
    void pTickUpdate_impl();
    void xTickUpdate_impl();
    int64_t getMechAng_q31p32_impl() const{return 0;};
    int32_t getElecAng_q31_impl() const{return 0;};
    void setCommAngle_impl(q31_t  _commAngle_q31)
    {
        commAngle_q31 = _commAngle_q31;
    }
    void reset_impl()
    {

    }
};

#ifdef ELDRIVER_HALL1_ENABLED
using PosDriver = PosOpen;
#else
using PosDriver = PosOpen;
#endif
