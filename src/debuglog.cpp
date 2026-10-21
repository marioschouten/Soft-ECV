#include "debuglog.h"
#include <stdarg.h>

//Ring buffer for the live log on the web page (about 3.5 KB)
#define LOG_LINES     32
#define LOG_LINE_SIZE 100

struct LogLine {
  uint32_t seq;
  uint32_t ms;
  uint8_t tag;
  char text[LOG_LINE_SIZE];
};

static LogLine lines[LOG_LINES];
static uint32_t log_seq = 0;

static const char* const tag_names[] = {"ot", "mqtt", "mqtt-in", "range", "update", "convert", "1wire", "decode", "sys"};

const char* dbg_tag_name(uint8_t tag) {
  return tag <= LOG_SYS ? tag_names[tag] : "?";
}

bool dbg_on(uint8_t flag) {
  return flag < DBG_COUNT && (config.debug_flags & (1 << flag));
}

void dbg(uint8_t tag, const char* fmt, ...) {
  if (tag != LOG_SYS && !dbg_on(tag)) { return; }

  char text[LOG_LINE_SIZE];
  va_list args;
  va_start(args, fmt);
  vsnprintf(text, sizeof(text), fmt, args);
  va_end(args);

  //System messages always go to the serial port, debug messages only when enabled
  if (tag == LOG_SYS || config.debug_serial) {
    Serial.print('[');
    Serial.print(dbg_tag_name(tag));
    Serial.print("] ");
    Serial.println(text);
  }

  if (tag == LOG_SYS || config.debug_web) {
    LogLine& line = lines[log_seq % LOG_LINES];
    line.seq = ++log_seq;
    line.ms  = millis();
    line.tag = tag;
    strncpy(line.text, text, LOG_LINE_SIZE);
  }
}

void json_string(Print& out, const char* s) {
  out.print('"');
  for (; *s; s++) {
    char c = *s;
    if (c == '"' || c == '\\') { out.print('\\'); out.print(c); }
    else if ((uint8_t)c < 0x20) { out.printf("\\u%04x", c); }
    else { out.print(c); }
  }
  out.print('"');
}

void dbg_write_json(Print& out, uint32_t after) {
  out.print('[');
  bool first = true;
  uint32_t oldest = log_seq > LOG_LINES ? log_seq - LOG_LINES + 1 : 1;
  for (uint32_t seq = max(after + 1, oldest); seq <= log_seq; seq++) {
    LogLine& line = lines[(seq - 1) % LOG_LINES];
    if (!first) { out.print(','); }
    first = false;
    out.printf("{\"s\":%u,\"t\":%u,\"g\":\"%s\",\"m\":", line.seq, line.ms, dbg_tag_name(line.tag));
    json_string(out, line.text);
    out.print('}');
  }
  out.print(']');
}
