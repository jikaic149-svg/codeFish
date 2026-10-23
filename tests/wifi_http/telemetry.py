"""Compile the real telemetry builder on the PC and check its fixed JSON buffer."""
import os
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
COMPILER = Path('C:/msys64/ucrt64/bin/g++.exe')
CJSON = Path('D:/ESP32/ESP-IDF-v5.3.2/components/json/cJSON')


def function(source, signature):
    start = source.index(signature)
    opening = source.index('{', start)
    depth = 0
    for position in range(opening, len(source)):
        depth += (source[position] == '{') - (source[position] == '}')
        if not depth:
            return source[start:position+1]
    raise ValueError('Incomplete telemetry function')


def main():
    source = (ROOT / 'main/network.cpp').read_text(encoding='utf-8')
    capacity = int(re.search(r'std::array<char,\s*(\d+)> responseBuffer', source)[1])
    prefix = r'''
#include <array>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include "cJSON.h"
#include "network.h"
#include "config/AttitudeConfig.h"
#include "config/BoardConfig.h"
#include "Service/LoRa/LoraProtocol.h"
using esp_err_t=int;
struct httpd_req_t {};
constexpr int HTTPD_RESP_USE_STRLEN=-1;
static MotionSnapshot fixture;
static QueueHandle_t snapshots=nullptr;
static cJSON *captured=nullptr;
int xQueuePeek(QueueHandle_t,void *out,unsigned) { *static_cast<MotionSnapshot *>(out)=fixture; return pdTRUE; }
int httpd_resp_set_status(httpd_req_t *,const char *) { return 0; }
int httpd_resp_send(httpd_req_t *,const char *,int) { return 1; }
int json(httpd_req_t *,cJSON *body) { captured=body; return 0; }
'''
    suffix = r'''
// All optional groups are populated together: a deliberately conservative bound.
void inflate(cJSON *node) {
  for(cJSON *entry=node->child;entry;entry=entry->next) {
    if(cJSON_IsNumber(entry)) cJSON_SetNumberValue(entry,
        std::floor(entry->valuedouble)==entry->valuedouble?4294967295.0:-1.2345678901234567e-20);
    else if(cJSON_IsBool(entry)) cJSON_SetBoolValue(entry, false);
    else if(entry->child) inflate(entry);
  }
}
int main() {
  fixture.control.outputRecovery=true; fixture.control.loRaSeen=true;
  fixture.control.loraResult.available=true;
  fixture.control.loraResult.code=robofish::lora::ControlCode::Unsupported;
  fixture.control.target.surgePercent=fixture.control.target.yawPercent=fixture.control.target.pitchPercent=13.123456f;
  fixture.control.target.propulsionScale=.1234567f;
  std::memset(fixture.wifi.session,'x',64); fixture.wifi.session[64]=0;
  fixture.lora.accepted=UINT32_MAX; fixture.lora.lastSource=fixture.lora.lastSequence=65535;
  fixture.lora.lastCode=24;
  fixture.imu.hasAngles=fixture.imu.hasAngularVelocity=fixture.imu.hasAcceleration=true;
  fixture.imu.baud=9600;
  fixture.imu.angles=fixture.imu.angularVelocity=fixture.imu.acceleration={13.123456f,-17.123456f,19.123456f};
  fixture.attempted.tailFrontDegrees=fixture.attempted.tailRearDegrees=13.123456f;
  fixture.attempted.pectoralLeftDegrees=fixture.attempted.pectoralRightDegrees=13.123456f;
  fixture.written=fixture.attempted;
  fixture.allocation.phaseRad=fixture.allocation.commonPropulsionPercent=fixture.allocation.differentialPropulsionPercent=.123456789;
  fixture.attitude.bodyValid=fixture.attitude.headingValid=fixture.attitude.headingLocked=fixture.attitude.hasSample=true;
  fixture.attitude.reason=robofish::attitude::Reason::HeadingUnavailable;
  fixture.attitude.rollDegrees=fixture.attitude.pitchDegrees=fixture.attitude.yawDegrees=13.123456f;
  fixture.attitude.pitchCorrectionPercent=fixture.attitude.rollCorrectionPercent=fixture.attitude.yawCorrectionPercent=13.123456f;
  fixture.attitude.headingTargetDegrees=13.123456f;
  if(telemetry(nullptr)!=0 || !captured) return 2;
  if(cJSON_GetObjectItemCaseSensitive(cJSON_GetObjectItemCaseSensitive(captured,"imu"),"baud")->valuedouble!=9600) return 4;
  inflate(captured);
  char *encoded=cJSON_PrintUnformatted(captured);
  if(!encoded) return 3;
  const size_t length=std::strlen(encoded);
  std::array<char, RESPONSE_CAPACITY> output{};
  const bool fits=cJSON_PrintPreallocated(captured,output.data(),output.size(),false);
  std::printf("%s: telemetry worst-case bound %zu bytes / %zu-byte buffer; snapshot=%zu bytes\n",fits?"PASS":"FAIL",length,output.size(),sizeof(MotionSnapshot));
  cJSON_free(encoded); cJSON_Delete(captured);
  return fits?0:1;
}
'''
    cache = ROOT / 'build/host-telemetry'
    cache.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, PATH=str(COMPILER.parent) + os.pathsep + os.environ.get('PATH', ''))
    with tempfile.TemporaryDirectory(prefix='packet-', dir=cache) as directory:
        work = Path(directory).resolve()
        if not work.is_relative_to(ROOT.resolve()):
            raise ValueError('Test directory escaped workspace')
        test = work / 'telemetry.cpp'
        test.write_text(prefix + function(source, 'cJSON *frameJson(') + '\n' +
            function(source, 'esp_err_t telemetry(') + '\n' + suffix.replace('RESPONSE_CAPACITY', str(capacity)), encoding='utf-8')
        common = [f'-I{ROOT / "tests/control/stubs"}', f'-I{ROOT / "main"}', f'-I{CJSON}']
        obj, binary = work / 'cJSON.o', work / 'telemetry.exe'
        subprocess.run([str(COMPILER.with_name('gcc.exe')), '-std=c99', '-O1', f'-I{CJSON}', '-c', str(CJSON / 'cJSON.c'), '-o', str(obj)], check=True, env=env)
        subprocess.run([str(COMPILER), '-std=c++17', '-O1', '-Wall', '-Wextra', '-Werror', *common,
            str(test), str(ROOT / 'main/Service/LoRa/LoraProtocol.cpp'), str(obj), '-o', str(binary)], check=True, env=env)
        subprocess.run([str(binary)], check=True, env=env)


if __name__ == '__main__':
    main()
