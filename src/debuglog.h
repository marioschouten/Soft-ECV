//Debug and system messages, written to the USB serial port and kept for the live log on the web page.
#pragma once
#include <Arduino.h>
#include "config.h"

//Message tags: the debug message types of config.h, plus system messages which are always shown
const uint8_t LOG_SYS = DBG_COUNT;

bool dbg_on(uint8_t flag);
void dbg(uint8_t tag, const char* fmt, ...) __attribute__((format(printf, 2, 3)));
const char* dbg_tag_name(uint8_t tag);

//Write the kept messages newer than sequence number "after" as a JSON array
void dbg_write_json(Print& out, uint32_t after);

//Write a string as a JSON string value, with escaping
void json_string(Print& out, const char* s);
