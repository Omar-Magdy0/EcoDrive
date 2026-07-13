#include "PosDriver.h"
#include "PmsmControlCore.h"

using namespace PmsmControlTypes;
bool PosOpen::init_impl()
{
    rpxt_min_thresh = mc.rps_to_rpxt(100*(2*M_PI/60.0));
    state_fsm = PmsmControlTypes::PosDriverFsm::Unsync;
    mc.control.run_mode = RunMode::Override;
    return true;
}

void PosOpen::pTickUpdate_impl()
{
}

void PosOpen::xTickUpdate_impl()
{
}
