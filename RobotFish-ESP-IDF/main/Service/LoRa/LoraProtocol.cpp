#include "Service/LoRa/LoraProtocol.h"

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace robofish::lora {

bool parseLine(const char *line, Command &command) {
  if (!line || !*line) return false;
  long fields[11]{};
  size_t count = 0;
  const char *cursor = line;
  while (*cursor) {
    if (count == 11) return false;
    errno = 0;
    char *end = nullptr;
    const long value = std::strtol(cursor, &end, 10);
    if (end == cursor || errno == ERANGE || value < INT32_MIN || value > INT32_MAX) return false;
    fields[count++] = value;
    if (*end == '\0') break;
    if (*end != ',' || end[1] == '\0') return false;
    cursor = end + 1;
  }
  if (count < 5 ||
      fields[1] != config::lora::kFishNode ||
      fields[2] < 0 || fields[2] > UINT16_MAX || fields[3] < 0 ||
      fields[3] > UINT16_MAX) {
    return false;
  }
  // 先完整验证，再更新输出。非法命令不得通过 knob 字段修改控制状态。
  Command parsed;
  parsed.source = static_cast<uint16_t>(fields[2]);
  parsed.sequence = static_cast<uint16_t>(fields[3]);
  if (fields[0] == 101 || fields[0] == 102) {
    if ((fields[0] == 101 && count != 11) || (fields[0] == 102 && count != 5)) return false;
    const char *end = std::strrchr(line, ',');
    uint16_t crc = 0xffff;
    for (const char *p = line; p < end; ++p) {
      crc ^= static_cast<uint16_t>(static_cast<uint8_t>(*p)) << 8;
      for (unsigned bit = 0; bit < 8; ++bit)
        crc = static_cast<uint16_t>((crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0));
    }
    if (fields[count - 1] < 0 || fields[count - 1] != crc) return false;
    if (fields[0] == 102) parsed.code = kHeartbeat;
    else {
      if (fields[4] != kTarget && fields[4] != kMode && fields[4] != 17 &&
          fields[4] != kAttitudeOn && fields[4] != kAttitudeOff && fields[4] != kYawPid) return false;
      if (fields[5] < 1 || fields[5] > 3 || fields[9] < 0 || fields[9] > 1000) return false;
      for (unsigned i = 6; i < 9; ++i) {
        if (fields[4] == kYawPid) { if (fields[i] < 0 || fields[i] > 100000) return false; }
        else if (fields[i] < -1000 || fields[i] > 1000) return false;
      }
      if (fields[4] == kYawPid && fields[9] != 1000) return false;
      if ((fields[4] == kAttitudeOn || fields[4] == kAttitudeOff) &&
          (fields[6] || fields[7] || fields[8])) return false;
      parsed.code = static_cast<uint8_t>(fields[4]);
      parsed.mode = static_cast<DriveMode>(fields[5]);
      if (parsed.code == kYawPid)
        parsed.yawPid = {fields[6] / 1000.0f, fields[7] / 1000.0f, fields[8] / 1000.0f};
      else parsed.axes = {fields[6] * 0.1f, fields[7] * 0.1f, fields[8] * 0.1f};
      parsed.propulsionScalePercent = fields[9] * 0.1f;
    }
    command = parsed;
    return true;
  }
  if (fields[0] != 100 || count > 6 || fields[4] < 1 || fields[4] > 19) return false;
  parsed.code = static_cast<uint8_t>(fields[4]);
  parsed.hasKnob = count == 6 && fields[5] >= 0;
  if (parsed.hasKnob) {
    parsed.knob = static_cast<uint16_t>(std::clamp(fields[5], 0L, 1000L));
  }
  command = parsed;
  return true;
}

size_t formatAck(char *buffer, size_t capacity, const Command &command,
                 uint32_t acceptedCount) {
  if (!buffer || capacity == 0) return 0;
  const int length = std::snprintf(
      buffer, capacity, "910,%u,%d,%u,%u,%u,%lu\n",
      static_cast<unsigned>(command.source), config::lora::kFishNode,
      static_cast<unsigned>(command.sequence), static_cast<unsigned>(command.source),
      static_cast<unsigned>(command.code),
      static_cast<unsigned long>(acceptedCount));
  if (length < 0 || static_cast<size_t>(length) >= capacity) {
    buffer[0] = '\0';
    return 0;
  }
  return static_cast<size_t>(length);
}

ParseResult LineDecoder::consume(uint8_t byte, Command &command) {
  if (byte == '\r' || byte == 0xff) return ParseResult::Incomplete;
  if (byte == '\n') {
    if (discarding_) {
      reset();
      return ParseResult::Rejected;
    }
    if (length_ == 0) return ParseResult::Incomplete;
    line_[length_] = '\0';
    const bool accepted = parseLine(line_, command);
    reset();
    return accepted ? ParseResult::Accepted : ParseResult::Rejected;
  }
  if (discarding_) return ParseResult::Incomplete;
  if (byte < ' ' || byte > '~' || length_ == config::lora::kMaxLineLength) {
    discarding_ = true;
    return ParseResult::Incomplete;
  }
  line_[length_++] = static_cast<char>(byte);
  return ParseResult::Incomplete;
}

void LineDecoder::reset() {
  length_ = 0;
  discarding_ = false;
}

}  // namespace robofish::lora
