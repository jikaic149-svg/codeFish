// Offline harness only: direct typed heading commands; no transport, board or PWM.
#include "ControlState.h"
#include "APP/Algorithm/AttitudeControl.h"

namespace {
ControlState control;
robofish::attitude::AngleFilter filter;
robofish::attitude::HeadingState heading;
robofish::motion::State gait;
robofish::ActuatorFrame frame;
double out[6];
}
extern "C" __declspec(dllexport) void reset_run() {
  control={}; control.stopPending=false; filter={}; heading={}; gait={}; frame={};
}
extern "C" __declspec(dllexport) const double* actuate(unsigned ms, double refCCW, double yawCCW,
    double rateCCW, double power, double split, int shareTravel, int stopped) {
  namespace a=robofish::attitude;
  if (stopped) {control.stop(); frame={}; gait={}; std::fill(out,out+6,0.); return out;}
  control.startHeading(-refCCW,power,100,ms);
  auto settings=robofish::config::attitude::makeSettings();
  a::Sample sample{{-180,0,float(yawCCW)},{0,0,float(-rateCCW)},ms,ms,true,true,true};
  sample=a::filterAngles(sample,filter,robofish::config::attitude::kAngleFilterStageHz,
                         robofish::config::attitude::kMaximumSampleAgeMs);
  auto result=a::evaluate(settings,sample,ms,true,control.attitudeMotionAllowed(),false);
  a::holdHeading(settings,result,heading,false,control.attitudeGeneration,ms,true,-refCCW);
  auto req=control.target;
  req.yawPercent=result.yawCorrectionPercent;
  auto recipe=robofish::config::motion::makeRecipe(req.mode);
  recipe.tail.amplitudeDegrees*=control.tailAmplitudeGain();
  control.applyHeadingAllocation(req,recipe);
  // Experimental split: keep tail-drive + common-prop command constant (NOT constant watts).
  req.tailDrivePercent*=split; req.propulsionDrivePercent*=2.-split;
  if (shareTravel) req.preserveTailBias=false;
  const auto mix=robofish::motion::allocate(req,recipe,robofish::config::motion::makeGeometry(),
                                            frame,gait,ms,20);
  frame=mix.candidateFrame;
  out[0]=.5*(frame.leftPropulsionPercent+frame.rightPropulsionPercent);
  out[1]=mix.effectiveTailBiasDegrees; // positive geometric bias = positive world CCW in this recipe
  out[2]=mix.effectiveTailAmplitudeDegrees; out[3]=mix.tailFrequencyHz;
  out[4]=mix.tailAmplitudeDegrees; out[5]=frame.leftPropulsionPercent-frame.rightPropulsionPercent;
  return out;
}
