#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include "APP/Algorithm/AttitudeControl.h"
#include "config/AttitudeConfig.h"
#include "Mode/Motion/MotionMode.h"
#include "ControlState.h"

using namespace robofish;
static unsigned checks = 0;
static void check(bool value, const char *message) {
  ++checks;
  if (!value) { std::fprintf(stderr, "FAIL: %s\n", message); std::exit(1); }
}
static bool near(float a, float b, float epsilon = .002f) { return std::abs(a-b) < epsilon; }
static attitude::Settings settings() {
  auto s = config::attitude::makeSettings();
  s.sensorToBody[0] = 1; s.sensorToBody[1] = -2; s.sensorToBody[2] = -3;
  s.pitchCommandSign = s.rollCommandSign = 1; // Synthetic, calibrated model only.
  return s;
}
static attitude::Sample sample(uint32_t now = 100) {
  return {{12, 8, 35}, {4, 2, 0}, now, now, true, true, true};
}
static void mountingChecks() {
  constexpr double radians = .017453292519943295;
  // Independent quaternion -> rotation matrix, then sensor Euler encoding.
  const double cr=std::cos(12*radians/2), sr=std::sin(12*radians/2);
  const double cp=std::cos(-8*radians/2), sp=std::sin(-8*radians/2);
  const double cy=std::cos(35*radians/2), sy=std::sin(35*radians/2);
  const double w=cr*cp*cy+sr*sp*sy, x=sr*cp*cy-cr*sp*sy;
  const double y=cr*sp*cy+sr*cp*sy, z=cr*cp*sy-sr*sp*cy;
  const double bodyToNed[3][3] = {
    {1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)},
    {2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)},
    {2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)}};
  int axes[] = {1,2,3};
  unsigned proper=0, reflected=0;
  do {
    unsigned inversions=0;
    for(unsigned i=0;i<3;++i) for(unsigned j=i+1;j<3;++j) inversions += axes[i]>axes[j];
    for(unsigned bits=0;bits<8;++bits) {
      auto s=settings(); int signProduct=1; double map[3][3]{};
      for(unsigned row=0;row<3;++row) {
        const int sign=bits&(1u<<row)?-1:1;
        s.sensorToBody[row]=sign*axes[row]; signProduct*=sign;
        map[row][axes[row]-1]=sign;
      }
      const bool valid=signProduct*(inversions%2?-1:1)==1;
      check(attitude::validFrame(s)==valid,"only right-handed signed permutations are mounts");
      if(!valid) {
        ++reflected;
        check(attitude::evaluate(s,sample(),100,true,true,false).reason==attitude::Reason::Uncalibrated,
              "reflection never becomes feedback");
        continue;
      }
      ++proper;
      double sensorToWorld[3][3]{};
      for(unsigned row=0;row<3;++row) for(unsigned col=0;col<3;++col)
        for(unsigned k=0;k<3;++k) sensorToWorld[row][col]+=(row? -1:1)*bodyToNed[row][k]*map[k][col];
      auto input=sample();
      input.anglesDegrees={static_cast<float>(std::atan2(sensorToWorld[2][1],sensorToWorld[2][2])/radians),
        static_cast<float>(std::asin(std::clamp(-sensorToWorld[2][0],-1.0,1.0))/radians),
        static_cast<float>(std::atan2(sensorToWorld[1][0],sensorToWorld[0][0])/radians)};
      const float bodyRates[]={4,-2,0}; float sensorRates[3]{};
      for(unsigned row=0;row<3;++row) for(unsigned col=0;col<3;++col) sensorRates[col]+=map[row][col]*bodyRates[row];
      input.ratesDps={sensorRates[0],sensorRates[1],sensorRates[2]};
      const auto result=attitude::evaluate(s,input,100,true,true,false);
      check(result.bodyValid && near(result.rollDegrees,12) && near(result.pitchDegrees,-8),"all mounts recover body attitude");
      check(near(result.rollRateDps,4) && near(result.pitchRateDps,-2),"rates rotate with the sensor frame");
      check(result.headingValid && near(result.yawDegrees,35),"all mounts recover local body heading");
      const double forwardDotX=-bodyToNed[0][2]*bodyRates[1];
      const double forwardDotY=-bodyToNed[1][2]*bodyRates[1];
      const double headingRate=(bodyToNed[0][0]*forwardDotY-bodyToNed[1][0]*forwardDotX)/
          (bodyToNed[0][0]*bodyToNed[0][0]+bodyToNed[1][0]*bodyToNed[1][0]);
      check(near(result.yawRateDps,static_cast<float>(headingRate)),"heading rate matches independent matrix derivative");
      check(near(result.rollCorrectionPercent,-12.4f) && near(result.pitchCorrectionPercent,8.2f),"PD opposes attitude and rate");
    }
  } while(std::next_permutation(axes,axes+3));
  check(proper==24 && reflected==24,"all 24 mounts and 24 reflections covered");
}
static void inputChecks() {
  auto s=settings(); auto input=sample();
  auto evaluate=[&](bool requested=true,bool motion=true,bool manual=false) { return attitude::evaluate(s,input,100,requested,motion,manual); };
  check(evaluate(false).reason==attitude::Reason::Disabled,"disabled by operator");
  check(evaluate(false).bodyValid && !evaluate(false).rollActive && evaluate(false).rollCorrectionPercent==0,
        "body pose visible with corrections off for bench calibration");
  input.ratesMs=90;
  check(evaluate().hasSample && evaluate().sampleMs==90,"feedback age references the oldest sample used by the control tick");
  input=sample();
  check(evaluate(true,false).reason==attitude::Reason::WaitingMotion,"no motion lease means no gyro output");
  auto result=evaluate(true,true,true);
  check(!result.pitchActive && result.pitchCorrectionPercent==0 && result.rollActive,"manual heave retains pitch authority");
  s.rollCommandSign=0;
  check(evaluate(true,true,true).reason==attitude::Reason::ManualPitch,"uncalibrated roll cannot fill in for manual pitch");
  s=settings(); input.hasRates=false;
  check(evaluate().reason==attitude::Reason::NoImu,"angle alone cannot provide rate damping");
  input=sample(); input.ratesMs=uint32_t(100-s.maximumAgeMs-1);
  check(evaluate().reason==attitude::Reason::Stale,"stale rate rejected even with fresh angle");
  input=sample(); input.anglesMs=uint32_t(100-s.maximumAgeMs-1);
  check(evaluate().reason==attitude::Reason::Stale,"stale attitude rejected");
  input=sample(); input.anglesMs=input.ratesMs=UINT32_MAX-4;
  check(attitude::evaluate(s,input,5,true,true,false).reason==attitude::Reason::Active,"freshness handles millisecond wrap");
  input=sample(); input.anglesDegrees.x=std::numeric_limits<float>::quiet_NaN();
  check(evaluate().reason==attitude::Reason::Invalid,"nonfinite sensor data never commands fins");
  input=sample(); input.ratesDps.z=2001;
  check(evaluate().reason==attitude::Reason::Invalid,"invalid rate range rejected");
  input=sample(); input.anglesDegrees.y=91;
  check(evaluate().reason==attitude::Reason::Invalid,"invalid WIT pitch rejected");
  input=sample(); input.ratesDps.x=1900;
  check(evaluate().rollCorrectionPercent==-20,"correction is independently bounded");
  s.rollCommandSign=-1;
  check(evaluate().rollCorrectionPercent==20,"actuator calibration sign is explicit");
  s=settings(); s.pitchKp=std::numeric_limits<float>::max();
  check(evaluate().reason==attitude::Reason::Uncalibrated,"invalid gains cannot overflow control");
  check(attitude::validSettings(config::attitude::makeSettings()),"production mounting follows logged roll near +/-180");
}
static void confirmedDirectionsAndWrapChecks() {
  const auto production=config::attitude::makeSettings();
  auto logged=sample(); logged.anglesDegrees={-175.748291f,7.020264f,113.049316f}; logged.ratesDps={0,0,0};
  const auto pose=attitude::evaluate(production,logged,100,false,false,false);
  check(pose.bodyValid && near(pose.rollDegrees,4.251709f) && near(pose.pitchDegrees,-7.020264f) &&
        near(pose.yawDegrees,-113.049316f),"logged sensor pose maps to small body tilt instead of a 176-degree roll error");
  check(!pose.pitchActive && !pose.rollActive && !pose.yawActive &&
        production.rollTrimDegrees==0 && production.pitchTrimDegrees==0,
        "log reading cannot silently enable assistance or declare an arbitrary pose level");
  auto s=settings(); s.yawCommandSign=1; s.rollCommandSign=0;
  check(config::attitude::kPitchCommandSign==1 && config::attitude::kYawCommandSign==1,
        "existing UP and right-turn command signs are recorded");
  auto input=sample(); input.anglesDegrees=input.ratesDps={0,0,0};
  auto result=attitude::evaluate(s,input,100,true,true,false);
  check(result.bodyValid && result.pitchCalibrated && result.yawCalibrated && !result.rollCalibrated,
        "synthetic level-zero mount enables pitch/yaw while unknown roll fin response stays disabled");
  check(near(result.rollDegrees,0) && near(result.pitchDegrees,0) && near(result.yawDegrees,0),
        "sensor zero is level with no half-turn offset");
  for(float angle : {-.1f,.1f,-10.0f,10.0f}) {
    input.anglesDegrees={0,angle,0};
    result=attitude::evaluate(s,input,100,true,true,false);
    check(near(result.pitchDegrees,-angle) && near(result.pitchCorrectionPercent,angle),
          "nose-up decreases sensor pitch and produces a restoring DOWN demand through zero");
    input.anglesDegrees={angle,0,0};
    result=attitude::evaluate(s,input,100,true,true,false);
    check(near(result.rollDegrees,angle) && !result.rollActive,
          "sensor roll increases to the right through zero without uncalibrated fin output");
    input.anglesDegrees={0,0,angle};
    result=attitude::evaluate(s,input,100,true,true,false);
    check(near(result.yawDegrees,-angle),"sensor yaw increases to the left through zero");
  }
  for(float heading : {-179.0f,-.1f,0.0f,.1f,179.0f}) {
    for(float drift : {-2.0f,2.0f}) {
      attitude::HeadingState held;
      input.anglesDegrees={0,0,heading}; input.ratesDps={0,0,0};
      result=attitude::evaluate(s,input,100,true,true,false);
      attitude::holdHeading(s,result,held,false,0);
      input.anglesDegrees.z=std::remainder(heading+drift,360.0f);
      result=attitude::evaluate(s,input,100,true,true,false);
      attitude::holdHeading(s,result,held,false,0);
      check(result.yawActive && near(result.yawCorrectionPercent,drift),
            "left yaw drift commands right restoring demand across zero and +/-180");
    }
  }
  input.anglesDegrees={0,0,0}; input.ratesDps={0,-10,10};
  result=attitude::evaluate(s,input,100,true,true,false);
  attitude::HeadingState held;
  attitude::holdHeading(s,result,held,false,0);
  check(result.pitchCorrectionPercent<0 && result.yawCorrectionPercent>0,
        "measured rate damping opposes nose-up and left turn even at zero angle");
  for(float origin : {-180.0f,0.0f,180.0f}) {
    attitude::HeadingState held;
    input.anglesDegrees={0,0,origin}; input.ratesDps={0,0,0};
    result=attitude::evaluate(s,input,100,true,true,false);
    attitude::holdHeading(s,result,held,false,0);
    input.anglesDegrees.z=std::remainder(origin+180.0f,360.0f);
    result=attitude::evaluate(s,input,100,true,true,false);
    attitude::holdHeading(s,result,held,false,0);
    check(result.yawCorrectionPercent==-s.maximumCorrectionPercent,
          "exact half-turn uses one consistent direction regardless of angle encoding");
  }
  // Both possible level readings agree with the user's observed angle signs.
  // Test each without choosing an unmeasured mounting for production.
  for(int yz : {-1,1}) {
    s.sensorToBody[1]=2*yz; s.sensorToBody[2]=3*yz;
    s.rollCommandSign=1; // Synthetic fin response, never used in production.
    for(float tilt : {-.1f,0.0f,.1f}) {
      input.anglesDegrees={std::remainder((yz>0?180.0f:0.0f)+tilt,360.0f),-tilt,0};
      input.ratesDps={0,0,0};
      result=attitude::evaluate(s,input,100,true,true,false);
      check(result.bodyValid && near(result.rollDegrees,tilt) && near(result.pitchDegrees,tilt),
            "both level-zero and level-half-turn mounts cross upright continuously");
      check(near(result.rollCorrectionPercent,-tilt) && near(result.pitchCorrectionPercent,-tilt),
            "both mounts restore pitch and roll on each side of upright");
    }
  }
  for(float halfTurn : {-540.0f,-180.0f,180.0f,540.0f})
    check(attitude::wrapDegrees(halfTurn)==-180,"equivalent half-turn encodings have one signed result");
}
static void actuatorChecks() {
  auto s=settings(); const auto feedback=attitude::evaluate(s,sample(),100,true,true,false);
  motion::Request base; base.enabled=true; base.mode=DriveMode::Hybrid;
  base.surgePercent=35; base.yawPercent=10; base.allowFlowAssist=false;
  auto corrected=base; corrected.pitchPercent=feedback.pitchCorrectionPercent; corrected.rollPercent=feedback.rollCorrectionPercent;
  auto recipe=config::motion::makeRecipe(base.mode);
  recipe.pectoral.leftRollDegrees=recipe.pectoral.rightRollDegrees=30; // Synthetic calibrated fin geometry.
  const auto geometry=config::motion::makeGeometry();
  ActuatorFrame plainFrame,stabilizedFrame; motion::State plain,stabilized;
  for(uint32_t now=20;now<=6000;now+=20) {
    const auto original=motion::allocate(base,recipe,geometry,plainFrame,plain,now,20);
    const auto result=motion::allocate(corrected,recipe,geometry,stabilizedFrame,stabilized,now,20);
    check(result.candidateFrame.leftPropulsionPercent==original.candidateFrame.leftPropulsionPercent &&
          result.candidateFrame.rightPropulsionPercent==original.candidateFrame.rightPropulsionPercent,
          "pitch/roll assist cannot create or alter propeller demand");
    check(near(result.candidateFrame.tailFrontDegrees,original.candidateFrame.tailFrontDegrees) &&
          near(result.candidateFrame.tailRearDegrees,original.candidateFrame.tailRearDegrees),"manual turning and gait preserved");
    check(result.candidateFrame.pectoralLeftDegrees>=geometry.pectoralLeft.minimumDegrees &&
          result.candidateFrame.pectoralLeftDegrees<=geometry.pectoralLeft.maximumDegrees &&
          result.candidateFrame.pectoralRightDegrees>=geometry.pectoralRight.minimumDegrees &&
          result.candidateFrame.pectoralRightDegrees<=geometry.pectoralRight.maximumDegrees,"existing fin limits preserved");
    check(std::abs(result.candidateFrame.pectoralLeftDegrees-stabilizedFrame.pectoralLeftDegrees)<=1.801f &&
          std::abs(result.candidateFrame.pectoralRightDegrees-stabilizedFrame.pectoralRightDegrees)<=1.801f,
          "existing 90-degree/second fin slew preserved");
    plainFrame=original.candidateFrame; stabilizedFrame=result.candidateFrame;
  }
  check(!near(plainFrame.pectoralRightDegrees,stabilizedFrame.pectoralRightDegrees),"feedback reaches fin allocation");
}
static void syntheticResponse() {
  auto s=settings(); float angle=20,rate=0;
  for(unsigned tick=0;tick<300;++tick) {
    auto input=sample(tick*20); input.anglesDegrees={angle,0,0}; input.ratesDps={rate,0,0};
    const auto output=attitude::evaluate(s,input,tick*20,true,true,false);
    check(std::isfinite(output.rollCorrectionPercent) && std::abs(output.rollCorrectionPercent)<=20,"model output stays bounded");
    // A simple positive-authority second-order plant, not a measured fish model.
    rate+=(30*output.rollCorrectionPercent-5*rate)*.02f; angle+=rate*.02f;
  }
  check(std::abs(angle)<.1f && std::abs(rate)<.1f,"PD converges in synthetic model only");
}
static void authorizationChecks() {
  ControlState state; state.stopPending=false;
  wifi::Command command; std::strcpy(command.session,"attitude-test");
  command.action=wifi::Action::Acquire;
  check(state.accept(command,0).code==wifi::Code::Accepted,"test owner acquires session");
  state.stopPending=false; // The real owner confirms a neutral BSP write after acquire.
  command.action=wifi::Action::Target;
  const auto target=[&](float surge,float yaw,uint32_t now) {
    ++command.sequence; command.receivedMs=now; command.axes={surge,yaw,0};
    check(state.accept(command,now).code==wifi::Code::Accepted,"fresh manual target accepted");
  };
  target(35,0,20); check(state.attitudeMotionAllowed(),"forward target authorizes feedback eligibility");
  target(-35,0,40); check(!state.attitudeMotionAllowed(),"reverse remains manual");
  target(0,40,60); check(!state.attitudeMotionAllowed(),"stationary yaw remains manual");
  target(35,10,80); check(state.attitudeMotionAllowed(),"forward turn retains feedback eligibility");
  command.action=wifi::Action::Stop; command.receivedMs=100; ++command.sequence;
  check(state.accept(command,100).code==wifi::Code::Accepted && !state.attitudeMotionAllowed(),"explicit STOP removes feedback authority");
  state.stopPending=false; command.action=wifi::Action::Target;
  target(35,0,120);
  check(!state.expire(30120) && state.attitudeMotionAllowed(),"forward feedback remains eligible without repeated motion frames");
  target(35,0,3140);
  for(unsigned i=0;i<5;++i) state.observeOutput(31);
  check(state.outputRecovery && !state.attitudeMotionAllowed(),"output recovery keeps gyro inactive");
}
static void headingChecks() {
  auto s=settings(); s.yawCommandSign=1;
  attitude::HeadingState state;
  auto input=sample(); input.anglesDegrees={0,0,-179}; input.ratesDps={0,0,0};
  const auto step=[&](bool manual=false,uint32_t generation=1,bool motion=true) {
    auto result=attitude::evaluate(s,input,100,true,motion,false);
    attitude::holdHeading(s,result,state,manual,generation); return result;
  };
  auto result=step();
  check(result.headingLocked && near(result.headingTargetDegrees,179) && near(result.yawCorrectionPercent,0),"first forward segment captures current heading");
  input.anglesDegrees.z=179; result=step();
  check(near(result.yawCorrectionPercent,-2),"heading correction takes shortest path across +/-180");
  result=step(true);
  check(!result.yawActive && !result.headingLocked && !state.locked && result.yawCorrectionPercent==0,"manual yaw drops hold reference and correction");
  input.anglesDegrees.z=-80; result=step();
  check(near(result.headingTargetDegrees,80) && near(result.yawCorrectionPercent,0),"release captures new heading, never returns to pre-turn course");
  input.anglesDegrees.z=-100; input.ratesDps.z=-20; result=step();
  check(result.yawCorrectionPercent==-20,"heading error and rate damping are bounded");
  input.anglesMs=uint32_t(100-s.maximumAgeMs-1); result=step();
  check(result.reason==attitude::Reason::Stale && !state.locked && !result.yawActive,"stale sensor releases heading hold");
  input=sample(); input.anglesDegrees={0,0,-120}; input.ratesDps={0,0,0}; result=step();
  check(near(result.headingTargetDegrees,120),"fresh sensor after a gap captures current course");
  input.anglesDegrees.z=-130; result=step(false,2);
  check(near(result.headingTargetDegrees,130) && near(result.yawCorrectionPercent,0),"new enable generation reanchors within one control tick");
  result=step(false,2,false);
  check(!result.headingLocked && !state.locked && result.yawCorrectionPercent==0,"STOP/no authorization drops heading hold");
  input.anglesDegrees={0,90,0}; result=step(false,2);
  check(!result.headingValid && !result.yawActive,"vertical forward axis has no defined heading");
  input.anglesDegrees={0,0,0}; result=step(false,2);
  attitude::holdHeading(s,result,state,true,2);
  check(!result.yawActive && result.yawCorrectionPercent==0,"a reused result cannot retain stale yaw correction");
  float heading=20,rate=0;
  state={0,3,true};
  for(unsigned tick=0;tick<300;++tick) {
    input=sample(tick*20); input.anglesDegrees={0,0,-heading}; input.ratesDps={0,0,-rate};
    auto output=attitude::evaluate(s,input,tick*20,true,true,false);
    attitude::holdHeading(s,output,state,false,3);
    check(std::isfinite(output.yawCorrectionPercent) && std::abs(output.yawCorrectionPercent)<=20,"synthetic yaw remains bounded");
    rate+=(30*output.yawCorrectionPercent-5*rate)*.02f; heading+=rate*.02f;
  }
  check(std::abs(heading)<.1f && std::abs(rate)<.1f,"relative heading converges in synthetic model only");
  ControlState owner; check(!owner.attitudeEnabled,"control loop defaults off");
  owner.setAttitude(true); const auto generation=owner.attitudeGeneration;
  owner.setAttitude(false); owner.setAttitude(true);
  check(owner.attitudeGeneration!=generation && !owner.leaseActive,"OFF/ON invalidates heading but never starts motion");
}
int main() {
  mountingChecks(); inputChecks(); confirmedDirectionsAndWrapChecks(); actuatorChecks(); syntheticResponse(); authorizationChecks(); headingChecks();
  std::printf("PASS: %u attitude checks; 24 mounts, three-axis PD, heading wrap/reanchor, stale/STOP/manual priority, fin limits, synthetic convergence; Result=%zu, HeadingState=%zu bytes\n",checks,sizeof(attitude::Result),sizeof(attitude::HeadingState));
}
