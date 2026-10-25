// Host-only model links the production protocol, control state, PID and mixer.
// The Python driver supplies a labelled synthetic water response, never hardware.
#include "ControlState.h"
#include "APP/Algorithm/AttitudeControl.h"
#include "Service/LoRa/LoraProtocol.h"
#include "config/ControlConfig.h"

namespace {
ControlState control;
robofish::attitude::HeadingState heading;
robofish::attitude::AngleFilter filter;
robofish::motion::State gait;
robofish::ActuatorFrame written;
double output[16]{};
}
#define EXPORT extern "C" __declspec(dllexport)
EXPORT void rf_reset() {
  control={};heading={};filter={};gait={};written={};
}
EXPORT int rf_command(const char *wire,unsigned now) {
  robofish::lora::Command command;
  if (!robofish::lora::parseLine(wire,command)) return -1;
  return control.accept(command,now)?0:1+static_cast<int>(control.loraResult.code);
}
EXPORT const double *rf_tick(unsigned now,double bodyCw,double rateCw,int valid) {
  namespace a=robofish::attitude;
  auto settings=robofish::config::attitude::makeSettings();
  settings.yawKp=control.yawPid.kp;settings.yawKi=control.yawPid.ki;settings.yawKd=control.yawPid.kd;
  a::Sample sample{{-180,0,static_cast<float>(-bodyCw)},{0,0,static_cast<float>(rateCw)},now,now,
                   valid!=0,valid!=0,valid!=0};
  sample=a::filterAngles(sample,filter,robofish::config::attitude::kAngleFilterStageHz,
                         robofish::config::attitude::kMaximumSampleAgeMs);
  auto result=a::evaluate(settings,sample,now,control.attitudeEnabled,control.attitudeMotionAllowed(),false);
  control.headingFeedbackValid=result.headingValid && result.bodyValid;control.headingSampleMs=result.sampleMs;
  a::holdHeading(settings,result,heading,control.target.yawPercent!=0,control.attitudeGeneration,now,
                 control.headingTargetActive,control.headingTargetCwDegrees);
  if (control.stopPending || control.maintenance || control.outputFault || control.outputRecovery) {
    written={};gait={};
  } else if (control.leaseActive || result.yawActive) {
    auto request=control.target;
    if (result.yawActive) { request.enabled=true;request.yawPercent+=result.yawCorrectionPercent; }
    auto recipe=robofish::config::motion::makeRecipe(request.mode);
    control.applyHeadingAllocation(request,recipe);
    written=robofish::motion::allocate(request,recipe,
        robofish::config::motion::makeGeometry(),written,gait,now,20).candidateFrame;
  } else { written.leftPropulsionPercent=written.rightPropulsionPercent=0; }
  control.observeOutput(63);
  if (written.leftPropulsionPercent==0 && written.rightPropulsionPercent==0) control.stopPending=false;
  output[0]=result.yawDegrees;output[1]=result.yawErrorDegrees;output[2]=result.yawCorrectionPercent;
  output[3]=result.headingTargetDegrees;output[4]=result.headingValid;output[5]=result.yawActive;
  output[6]=written.leftPropulsionPercent;output[7]=written.rightPropulsionPercent;
  output[8]=written.tailFrontDegrees;output[9]=written.tailRearDegrees;
  output[10]=control.target.surgePercent;output[11]=control.leaseActive;output[12]=control.headingTargetActive;
  output[13]=result.yawI;output[14]=result.yawD;output[15]=result.yawP;
  return output;
}
