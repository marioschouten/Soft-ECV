#!/usr/bin/env python3
"""OpenTherm conformance checker for the Soft E-CV (OpenTherm Protocol Specification v2.2).

  selftest  Send test frames to the D1 and check every reply. The replies are not sent on the
            OpenTherm line. Meant for the desk: it also sets test values (fault flag, outside
            temperature) that stay until the next MQTT message or restart.
              ot_check.py selftest --port /dev/ttyUSB0
              ot_check.py selftest --host 192.168.11.40 --password ...
  live      Follow the real conversations with the Elga, check them and save them to a capture file.
              ot_check.py live --host 192.168.11.40 [--minutes 60]
  analyse   Check a saved capture again.
              ot_check.py analyse captures/ot-20261021-1015.jsonl

Serial access needs pyserial, for example via PlatformIO's Python: ~/.platformio/penv/bin/python tools/ot_check.py ...
"""
import argparse
import datetime
import http.cookiejar
import json
import os
import sys
import time
import urllib.parse
import urllib.request

READ_DATA, WRITE_DATA, INVALID_DATA, RESERVED, READ_ACK, WRITE_ACK, DATA_INVALID, UNKNOWN_DATAID = range(8)
TYPES = ["READ-DATA", "WRITE-DATA", "INVALID-DATA", "RESERVED", "READ-ACK", "WRITE-ACK", "DATA-INVALID", "UNKNOWN-DATAID"]

# Data-ids of the v2.2 specification: name, access, kind, range (f8.8 only)
SPEC = {
    0: ("Status", "R", "flag8", None), 1: ("Control setpoint", "W", "f8.8", (0, 100)),
    2: ("Master configuration", "W", "flag8", None), 3: ("Slave configuration", "R", "flag8", None),
    4: ("Remote command", "W", "u8", None), 5: ("Fault flags", "R", "flag8", None),
    6: ("Remote parameter flags", "R", "flag8", None), 7: ("Cooling control", "W", "f8.8", (0, 100)),
    8: ("Control setpoint CH2", "W", "f8.8", (0, 100)), 9: ("Remote override room setpoint", "R", "f8.8", (0, 30)),
    10: ("Number of TSPs", "R", "u8", None), 11: ("TSP index/value", "RW", "u8", None),
    12: ("Fault buffer size", "R", "u8", None), 13: ("Fault buffer entry", "R", "u8", None),
    14: ("Max relative modulation", "W", "f8.8", (0, 100)), 15: ("Max capacity / min modulation", "R", "u8", None),
    16: ("Room setpoint", "W", "f8.8", (-40, 127)), 17: ("Relative modulation", "R", "f8.8", (0, 100)),
    18: ("CH water pressure", "R", "f8.8", (0, 5)), 19: ("DHW flow rate", "R", "f8.8", (0, 16)),
    20: ("Day and time", "RW", "special", None), 21: ("Date", "RW", "u8", None), 22: ("Year", "RW", "u16", None),
    23: ("Room setpoint CH2", "W", "f8.8", (-40, 127)), 24: ("Room temperature", "W", "f8.8", (-40, 127)),
    25: ("Boiler flow temperature", "R", "f8.8", (-40, 127)), 26: ("DHW temperature", "R", "f8.8", (-40, 127)),
    27: ("Outside temperature", "R", "f8.8", (-40, 127)), 28: ("Return water temperature", "R", "f8.8", (-40, 127)),
    29: ("Solar storage temperature", "R", "f8.8", (-40, 127)), 30: ("Solar collector temperature", "R", "s16", None),
    31: ("Flow temperature CH2", "R", "f8.8", (-40, 127)), 32: ("DHW2 temperature", "R", "f8.8", (-40, 127)),
    33: ("Exhaust temperature", "R", "s16", None), 48: ("DHW setpoint bounds", "R", "s8", None),
    49: ("Max CH setpoint bounds", "R", "s8", None), 50: ("OTC heat curve bounds", "R", "s8", None),
    56: ("DHW setpoint", "RW", "f8.8", (0, 127)), 57: ("Max CH water setpoint", "RW", "f8.8", (0, 127)),
    58: ("OTC heat curve ratio", "RW", "f8.8", (0, 40)), 100: ("Remote override function", "R", "flag8", None),
    115: ("OEM diagnostic code", "R", "u16", None),
    **{i: (n, "RW", "u16", None) for i, n in zip(range(116, 124), ["Burner starts", "CH pump starts", "DHW pump starts", "DHW burner starts",
                                                                  "Burner hours", "CH pump hours", "DHW pump hours", "DHW burner hours"])},
    124: ("Master OpenTherm version", "W", "f8.8", (0, 127)), 125: ("Slave OpenTherm version", "R", "f8.8", (0, 127)),
    126: ("Master product version", "W", "u8", None), 127: ("Slave product version", "R", "u8", None),
}
# Data-ids the slave must support (section 5.2) and the allowed reply types
MANDATORY = {
    (0, READ_DATA): {READ_ACK}, (1, WRITE_DATA): {WRITE_ACK, DATA_INVALID}, (3, READ_DATA): {READ_ACK},
    (14, WRITE_DATA): {WRITE_ACK, DATA_INVALID}, (17, READ_DATA): {READ_ACK, DATA_INVALID}, (25, READ_DATA): {READ_ACK, DATA_INVALID},
}
# Data-ids implemented by the E-CV firmware (src/boiler.cpp)
# Deviations kept on purpose: the Elga sends the room temperature as READ-DATA, the firmware echoes it as READ-ACK
TOLERATED = {(24, READ_DATA)}
FIRMWARE_IDS = {0, 1, 2, 3, 5, 14, 15, 16, 17, 18, 19, 24, 25, 26, 27, 28, 48, 49, 56, 57, 124, 125, 126, 127}
FRAME_REPLIED, FRAME_INJECTED, FRAME_PARITY, FRAME_BAD_TYPE, FRAME_DAMAGED = 1, 2, 4, 8, 16


def parity_odd(frame):
    return bin(frame).count("1") % 2 == 1


def build(msg_type, data_id, data):
    frame = (msg_type << 28) | (data_id << 16) | (data & 0xFFFF)
    if parity_odd(frame):
        frame |= 1 << 31
    return frame


def mtype(frame): return (frame >> 28) & 7
def did(frame): return (frame >> 16) & 0xFF
def data(frame): return frame & 0xFFFF
def f88(value): return int(round(value * 256)) & 0xFFFF
def from_f88(d): return (d - 0x10000 if d & 0x8000 else d) / 256


def describe(frame):
    name = SPEC.get(did(frame), ("data-id %d" % did(frame),))[0]
    return "%08x %s id %d (%s) data %04X" % (frame, TYPES[mtype(frame)], did(frame), name, data(frame))


class Results:
    def __init__(self, quiet_pass=True):
        self.counts = {"PASS": 0, "WARN": 0, "FAIL": 0}
        self.quiet_pass = quiet_pass
        self.messages = []

    def add(self, level, text):
        self.counts[level] += 1
        if level != "PASS" or not self.quiet_pass:
            line = "%-4s %s" % (level, text)
            self.messages.append(line)
            print(line, flush=True)

    def summary(self):
        return "PASS %(PASS)d  WARN %(WARN)d  FAIL %(FAIL)d" % self.counts


def check_conversation(res, request, reply, flags=0, latency=None, gap=None, wire=False):
    """Check one request/reply pair against the specification. reply is None when the device did not answer."""
    where = "T-%08x" % request
    if parity_odd(request):
        res.add("FAIL" if reply is not None else "PASS", "%s parity error: %s" % (where, "replied, must be rejected (4.5)" if reply is not None else "rejected"))
        return
    if mtype(request) not in (READ_DATA, WRITE_DATA, INVALID_DATA):
        res.add("FAIL" if reply is not None else "PASS", "%s message type %s: %s" % (where, TYPES[mtype(request)], "replied" if reply is not None else "ignored"))
        return
    if reply is None:
        if flags & FRAME_DAMAGED:
            res.add("WARN", "%s reported damaged by the line receiver, not answered" % where)
        else:
            res.add("FAIL", "%s valid request without reply" % where)
        return

    rid, rtype, req_type, req_id = did(reply), mtype(reply), mtype(request), did(request)
    label = "%s %s id %d -> B-%08x %s" % (where, TYPES[req_type], req_id, reply, TYPES[rtype])
    ok = True

    def fail(text):
        nonlocal ok
        ok = False
        res.add("FAIL", "%s: %s" % (label, text))

    if parity_odd(reply): fail("reply parity wrong")
    if reply & 0x0F000000: fail("spare bits not zero")
    if rid != req_id: fail("reply data-id %d differs from request" % rid)
    allowed = {READ_DATA: {READ_ACK, DATA_INVALID, UNKNOWN_DATAID}, WRITE_DATA: {WRITE_ACK, DATA_INVALID, UNKNOWN_DATAID},
               INVALID_DATA: {DATA_INVALID, UNKNOWN_DATAID}}[req_type]
    if rtype not in allowed: fail("reply type not allowed for %s (4.4)" % TYPES[req_type])

    must = MANDATORY.get((req_id, req_type))
    if must and rtype not in must:
        fail("mandatory data-id must reply %s (5.2)" % "/".join(TYPES[t] for t in sorted(must)))
    if req_id == 0 and rtype == READ_ACK and (data(reply) >> 8) != (data(request) >> 8):
        fail("status reply must return the master status in HB")
    spec = SPEC.get(req_id)
    if spec and rtype in (READ_ACK, WRITE_ACK) and spec[2] == "f8.8" and spec[3]:
        value = from_f88(data(reply))
        if not spec[3][0] <= value <= spec[3][1]:
            fail("value %.2f outside range %s" % (value, spec[3]))
    if spec and req_type in (READ_DATA, WRITE_DATA):
        access = "R" if req_type == READ_DATA else "W"
        if access not in spec[1] and rtype in (READ_ACK, WRITE_ACK):
            if (req_id, req_type) in TOLERATED and rtype == READ_ACK and data(reply) == data(request):
                res.add("WARN", "%s: %s is %s-only, READ accepted for the Elga (sends it as READ-DATA)" % (label, spec[0], spec[1]))
            else:
                fail("%s is %s-only in the specification" % (spec[0], spec[1]))
    if req_id not in FIRMWARE_IDS and rtype != UNKNOWN_DATAID:
        res.add("WARN", "%s: data-id not implemented in firmware but not UNKNOWN-DATAID" % label)

    if wire and latency is not None:
        if latency < 20 or latency > 800:
            fail("reply after %d ms, allowed 20-800 ms (4.3)" % latency)
    if wire and gap and gap > 1150:
        res.add("WARN", "%s: %d ms since the previous request, master must send at least every 1.15 s" % (where, gap))
    if ok:
        res.add("PASS", label)


# ---------------------------------------------------------------- links to the D1
class SerialLink:
    def __init__(self, port):
        import serial  # pyserial
        self.ser = serial.Serial()
        self.ser.port, self.ser.baudrate, self.ser.timeout = port, 115200, 0.2
        # DTR/RTS are wired to reset and GPIO0 on the D1 mini: keep them released
        self.ser.dtr = False
        self.ser.rts = False
        self.ser.open()
        time.sleep(0.3)
        self.ser.reset_input_buffer()

    def command(self, line):
        self.ser.write((line + "\n").encode())
        end = time.time() + 3
        while time.time() < end:
            text = self.ser.readline().decode(errors="replace").strip()
            if text.startswith("OTJ "):
                return json.loads(text[4:])
        raise RuntimeError("no answer from the D1 on '%s'" % line)

    def info(self): return self.command("info")
    def inject(self, frame): return self.command("inject %08x" % frame)
    def sim(self, topic, payload): return self.command("sim %s %s" % (topic, payload))


class HttpLink:
    def __init__(self, host, password=None):
        self.base = "http://%s" % host
        self.opener = urllib.request.build_opener(urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()))
        if password:
            self.post("/api/login", {"pass": password})

    def get(self, path):
        with self.opener.open(self.base + path, timeout=10) as r:
            return json.loads(r.read())

    def post(self, path, fields):
        body = urllib.parse.urlencode(fields).encode()
        with self.opener.open(self.base + path, data=body, timeout=10) as r:
            return json.loads(r.read())

    def info(self):
        s = self.get("/api/status")
        return {"fw": s["fw"], "modulating": s["ecv"]["modulating"], "delay": s["ot"]["delay"], "heap": s["heap"], "heap_block": s["heap_block"]}

    def inject(self, frame): return self.post("/api/inject", {"q": "%08x" % frame})
    def sim(self, topic, payload): return self.post("/api/sim", {"topic": topic, "payload": payload})


def reply_of(answer):
    if "error" in answer:
        raise RuntimeError(answer["error"])
    return int(answer["r"], 16) if answer.get("r") else None


# ---------------------------------------------------------------- selftest
def selftest(link):
    res = Results()
    info = link.info()
    print("Soft E-CV firmware %s, boiler type %s, reply delay %d ms" % (info["fw"], "modulating" if info["modulating"] else "on/off", info["delay"]))
    if "heap" in info:
        print("Free memory %.1f KB, largest block %.1f KB" % (info["heap"] / 1024, info["heap_block"] / 1024))
    print("\n== Every data-id 0-127, as READ-DATA and WRITE-DATA")

    def send(frame):
        reply = reply_of(link.inject(frame))
        check_conversation(res, frame, reply)
        return reply

    for i in range(128):
        send(build(READ_DATA, i, 0x0300 if i == 0 else 0))
        sample = {1: f88(45), 14: f88(80), 16: f88(20.5), 24: f88(19.8), 56: f88(60), 57: f88(70), 124: f88(2.2)}.get(i, f88(20))
        send(build(WRITE_DATA, i, sample))

    print("\n== Rejected and INVALID-DATA frames")
    send(build(READ_DATA, 25, 0) ^ (1 << 31))            # parity error
    send(build(READ_ACK, 25, 0))                         # slave message type as request
    send(build(INVALID_DATA, 1, f88(45)))
    send(build(INVALID_DATA, 200, 0))

    print("\n== Values")

    def expect(name, frame, want_type, want_data=None, mask=0xFFFF):
        reply = send(frame)
        if reply is None:
            res.add("FAIL", "%s: no reply" % name)
        elif mtype(reply) != want_type or (want_data is not None and (data(reply) & mask) != want_data):
            res.add("FAIL", "%s: got %s data %04X, expected %s%s" % (name, TYPES[mtype(reply)], data(reply), TYPES[want_type],
                                                                   "" if want_data is None else " data %04X" % want_data))
        else:
            res.add("PASS", name)
            print("PASS %s" % name)

    expect("ID 3 boiler type %s" % ("modulating" if info["modulating"] else "on/off"), build(READ_DATA, 3, 0), READ_ACK, 0x0000 if info["modulating"] else 0x0200)
    expect("ID 15 capacity 9 kW, minimum modulation 14 %", build(READ_DATA, 15, 0), READ_ACK, 0x090E)
    expect("ID 48 DHW setpoint bounds 70/30", build(READ_DATA, 48, 0), READ_ACK, 0x461E)
    expect("ID 49 max CH setpoint bounds 80/20", build(READ_DATA, 49, 0), READ_ACK, 0x5014)
    expect("ID 125 OpenTherm version 2.2", build(READ_DATA, 125, 0), READ_ACK, f88(2.2))
    expect("ID 1 setpoint 120 C is out of range", build(WRITE_DATA, 1, f88(120)), DATA_INVALID)
    expect("ID 24 room temperature as READ-DATA (Elga) echoed", build(READ_DATA, 24, 0x11B0), READ_ACK, 0x11B0)
    expect("ID 14 max modulation 50 % accepted", build(WRITE_DATA, 14, f88(50)), WRITE_ACK, f88(50))
    link.sim("ecv/sensors/outside_temperature", "-5.5")
    expect("ID 27 negative outside temperature -5.5 C", build(READ_DATA, 27, 0), READ_ACK, f88(-5.5))
    link.sim("ecv/sensors/dhw_temperature", "48.25")
    expect("ID 26 DHW temperature 48.25 C (was always 0 in 1.2.3)", build(READ_DATA, 26, 0), READ_ACK, f88(48.25))
    link.sim("ecv/command/max_ch_water_setpoint", "65")
    expect("ID 57 write 75 C acknowledged with E-CV value 65 C", build(WRITE_DATA, 57, f88(75)), WRITE_ACK, f88(65))
    #No flow temperature (no sensors, no MQTT value): fault bit and OEM code 25, if enabled
    flow_left = None
    reply = send(build(READ_DATA, 25, 0))
    if reply is not None and mtype(reply) == DATA_INVALID:
        if info.get("sensor_fault", False):
            expect("ID 0 fault bit without flow temperature", build(READ_DATA, 0, 0x0100), READ_ACK, 0x0101)
            expect("ID 5 OEM fault code 25 without flow temperature", build(READ_DATA, 5, 0), READ_ACK, 0x0019)
        link.sim("ecv/sensors/heater_flow_temperature", "40")
        flow_left = 40
    expect("ID 5 no fault flags", build(READ_DATA, 5, 0), READ_ACK, 0x0000)
    link.sim("ecv/status/fault", "1")
    expect("ID 0 fault flag set, parity correct", build(READ_DATA, 0, 0x0100), READ_ACK, 0x0101)
    expect("ID 5 service request flag on fault", build(READ_DATA, 5, 0), READ_ACK, 0x0100)
    link.sim("ecv/status/fault", "0")
    link.sim("ecv/status/ch_mode", "1")
    link.sim("ecv/status/flame", "1")
    expect("ID 0 CH mode and flame", build(READ_DATA, 0, 0x0100), READ_ACK, 0x010A)
    link.sim("ecv/status/ch_mode", "0")
    link.sim("ecv/status/flame", "0")
    reply = send(build(READ_DATA, 25, 0))
    if reply is not None:
        if mtype(reply) == READ_ACK:
            print("INFO ID 25 flow temperature %.2f C" % from_f88(data(reply)))
        else:
            print("INFO ID 25 flow temperature not available (no sensor reading and no MQTT value): %s" % TYPES[mtype(reply)])
    reply = send(build(READ_DATA, 28, 0))
    if reply is not None and mtype(reply) == READ_ACK:
        print("INFO ID 28 return temperature %.2f C" % from_f88(data(reply)))

    print("\nTest values left on the D1: outside temperature -5.5, DHW temperature 48.25, max CH setpoint 65%s (until MQTT or restart)"
          % (", flow temperature 40" if flow_left else ""))
    print("Selftest: %s" % res.summary())
    return res.counts["FAIL"] == 0


# ---------------------------------------------------------------- live and analyse
class LiveStats:
    def __init__(self):
        self.ids = {}
        self.latency = []
        self.gaps = []
        self.rejected = 0

    def add(self, frame):
        request = int(frame["q"], 16)
        key = (did(request), TYPES[mtype(request)])
        reply = TYPES[mtype(int(frame["r"], 16))] if frame.get("r") else "no reply"
        self.ids.setdefault(key, {}).setdefault(reply, 0)
        self.ids[key][reply] += 1
        if frame.get("r") and not frame["f"] & FRAME_INJECTED:
            self.latency.append(frame["l"])
        if frame.get("g"):
            self.gaps.append(frame["g"])
        if not frame.get("r"):
            self.rejected += 1

    def report(self):
        lines = ["Data-ids asked by the Elga:"]
        for (i, t), replies in sorted(self.ids.items()):
            name = SPEC.get(i, ("not in v2.2",))[0]
            lines.append("  %3d %-11s %-30s %s" % (i, t, name, ", ".join("%s x%d" % kv for kv in sorted(replies.items()))))
        if self.latency:
            lines.append("Reply delay: min %d, avg %.0f, max %d ms (allowed 20-800)" % (min(self.latency), sum(self.latency) / len(self.latency), max(self.latency)))
        if self.gaps:
            lines.append("Time between requests: min %d, avg %.0f, max %d ms (master limit 1150)" % (min(self.gaps), sum(self.gaps) / len(self.gaps), max(self.gaps)))
        lines.append("Rejected frames: %d" % self.rejected)
        return "\n".join(lines)


def check_frame(res, stats, frame):
    stats.add(frame)
    wire = not frame["f"] & FRAME_INJECTED
    check_conversation(res, int(frame["q"], 16), int(frame["r"], 16) if frame.get("r") else None,
                       frame["f"], frame.get("l"), frame.get("g"), wire)


def live(link, minutes, out_path):
    res, stats = Results(), LiveStats()
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    after, started, last_report = 0, time.time(), time.time()
    print("Capturing to %s, press Ctrl+C to stop" % out_path, flush=True)
    with open(out_path, "a") as out:
        try:
            while minutes is None or time.time() - started < minutes * 60:
                answer = link.get("/api/frames?after=%d" % after)
                for frame in answer["frames"]:
                    if after and frame["s"] > after + 1:
                        res.add("WARN", "%d frames missed between polls" % (frame["s"] - after - 1))
                    after = frame["s"]
                    frame["pc_time"] = datetime.datetime.now().isoformat(timespec="seconds")
                    out.write(json.dumps(frame) + "\n")
                    check_frame(res, stats, frame)
                out.flush()
                if time.time() - last_report > 300:
                    print("-- %s  %s" % (datetime.datetime.now().strftime("%H:%M"), res.summary()), flush=True)
                    last_report = time.time()
                time.sleep(0.5 if len(answer["frames"]) == 40 else 2)
        except KeyboardInterrupt:
            pass
    print("\n" + stats.report())
    print("Live check: %s" % res.summary())


def analyse(path):
    res, stats = Results(), LiveStats()
    with open(path) as f:
        for line in f:
            if line.strip():
                check_frame(res, stats, json.loads(line))
    print(stats.report())
    print("Analysis: %s" % res.summary())
    return res.counts["FAIL"] == 0


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("selftest")
    p.add_argument("--port")
    p.add_argument("--host")
    p.add_argument("--password", default=os.environ.get("ECV_PASSWORD"))
    p = sub.add_parser("live")
    p.add_argument("--host", required=True)
    p.add_argument("--minutes", type=float)
    p.add_argument("--out", default="captures/ot-%s.jsonl" % datetime.datetime.now().strftime("%Y%m%d-%H%M"))
    p = sub.add_parser("analyse")
    p.add_argument("file")
    args = parser.parse_args()

    if args.cmd == "selftest":
        if not args.port and not args.host:
            parser.error("selftest needs --port or --host")
        link = SerialLink(args.port) if args.port else HttpLink(args.host, args.password)
        sys.exit(0 if selftest(link) else 1)
    if args.cmd == "live":
        live(HttpLink(args.host), args.minutes, args.out)
    if args.cmd == "analyse":
        sys.exit(0 if analyse(args.file) else 1)


if __name__ == "__main__":
    main()
