/* Soft E-CV demo: answers the D1's web API in the browser with a simulated heating cycle.
   tools/build_demo.py puts this script in front of the page script of web/index.html and writes docs/demo.html.
   The simulation follows the firmware: the Elga's request cycle, the modulation calculation every 60 s
   (setpoint - flow, 2..20 C), the OpenHAB power levels (7 steps, 9 kW) and a simple heat model. */
(() => {
  'use strict';
  const FW = '__FW_VERSION__';
  const T0 = Date.now();
  const UP0 = 3 * 3600 + 17 * 60;            // Device uptime when the page opens (s)
  const DEMAND_AGO = 26 * 60;                // The Elga asked for heat 26 minutes before the page opened

  // ---------- OpenTherm frames ----------
  const RD = 0, WR = 1, RACK = 4, WACK = 5, DINV = 6;
  const TYPES = ['READ-DATA', 'WRITE-DATA', 'INVALID-DATA', 'RESERVED', 'READ-ACK', 'WRITE-ACK', 'DATA-INVALID', 'UNKNOWN-DATAID'];
  const NAMES = {
    0: ['Status flags', 0], 1: ['Control setpoint CH water temperature (C)', 1], 5: ['Application-specific and OEM fault flags', 0],
    16: ['Room setpoint (C)', 1], 17: ['Relative modulation level (Percent)', 1], 18: ['Water pressure in CH circuit (bar)', 1],
    19: ['Water flow rate in DHW circuit (litres/minute)', 1], 24: ['Room temperature (C)', 1], 25: ['Boiler flow water temperature (C)', 1],
    26: ['DHW temperature (C)', 1], 56: ['DHW setpoint (C)', 1]
  };
  const f88 = v => Math.round(v * 256) & 0xFFFF;
  const from88 = d => (d & 0x8000 ? d - 0x10000 : d) / 256;
  function frame(type, id, data) {
    let f = ((type & 7) << 28) | ((id & 0xFF) << 16) | (data & 0xFFFF);
    let ones = 0; for (let x = f >>> 0; x; x >>>= 1) ones += x & 1;
    if (ones % 2) f |= 0x80000000;
    return f >>> 0;
  }
  const hex = f => f.toString(16).padStart(8, '0');
  const describe = (prefix, f) => {
    const id = (f >>> 16) & 0xFF, d = f & 0xFFFF, n = NAMES[id];
    return `${prefix}-${hex(f)} ${TYPES[(f >>> 28) & 7]} ${n[0]}: ${n[1] ? from88(d).toFixed(2) : d.toString(16).toUpperCase().padStart(4, '0')}`;
  };

  // ---------- Simulation, one step per second ----------
  const ELGA_CYCLE = [18, 19, 56, 24, 5, 1, 0, 16, 17, 25, 26];
  const sim = {
    sec: 0, flow: 27.4, ret: 26.1, room: 19.1, roomSp: 20.5, sp: 0, mod: 0, level: 0, ch: false, lastStatus: 0,
    lastModCalc: -999, seq: 0, frames: [], hist: { flow: [], ret: [], sp: [], mod: [] }, log: [], logSeq: 0,
    flags: 1                                                          // OpenTherm traffic in the debug log
  };
  const cfg = {
    wifi_ssid: 'HomeWiFi', wifi_pass_set: true, ap_fallback_min: 5, mqtt_host: '192.168.1.10', mqtt_port: 1883, mqtt_user: 'soft-ecv',
    mqtt_pass_set: true, mqtt_client_id: 'soft-ecv', hostname: 'soft-ecv', modulating: true, report_sensor_fault: true
  };
  const dbg = { serial: true, web: true, keep: false };
  let signedIn = true, busyCount = 0;

  // Small repeatable noise, so the readings look like real DS18B20 values (1/16 C steps)
  let seed = 7;
  const rnd = () => ((seed = (seed * 16807) % 2147483647) / 2147483647) - 0.5;
  const ds = v => Math.round((v + rnd() * 0.08) * 16) / 16;

  function log(tag, m, ms = 300) {
    sim.log.push({ s: ++sim.logSeq, t: (UP0 - DEMAND_AGO + sim.sec) * 1000 + ms, g: tag, m });
    if (sim.log.length > 32) sim.log.shift();
  }
  const on = bit => (sim.flags >> bit) & 1;
  const levelOf = m => m < 0.01 ? 0 : m <= 15 ? 7 : m <= 30 ? 6 : m <= 43 ? 5 : m <= 57 ? 4 : m <= 71 ? 3 : m <= 86 ? 2 : 1;

  function step() {
    const s = sim, t = s.sec;
    s.ch = t >= 20;                                                  // Heat demand starts 20 s into the simulation
    s.sp = s.ch ? 45.5 + 1.2 * Math.sin(t / 700) : 46.0;              // Control setpoint written by the Elga (ID 1)
    // Firmware: modulation from (setpoint - flow) every 60 s, 2..20 C maps to 0..100 %
    if (t - s.lastModCalc >= 60) {
      s.lastModCalc = t;
      const diff = s.sp - s.flow;
      s.mod = !s.ch ? 0 : diff > 20 ? 100 : diff < 2 ? 0 : Math.min(100, diff / 18 * 100);
      if (on(1)) log('mqtt', `Publish ecv/thermostat/rawdata/modulation Request: ${s.sp.toFixed(2)} Heater flow: ${s.flow.toFixed(2)} Difference:${diff.toFixed(2)} Set Modulation: ${s.mod.toFixed(2)}`);
    }
    // OpenHAB: power level from the modulation, level 1 = 9 kW
    s.level = s.ch ? levelOf(s.mod) : 0;
    const kw = s.level ? (8 - s.level) / 7 * 9 : 0;
    // Heat model: element power in, losses to the radiators out
    s.flow += kw * 0.0034 - (s.flow - 21) * 0.00062;
    const dT = kw ? 2.5 + kw * 0.55 : 1.2;
    const target = s.flow - dT;                                      // The return follows with delay and hardly drops
    s.ret += (target - s.ret) * (target > s.ret ? 0.02 : 0.002);
    s.room += s.ch ? 0.00045 : -0.0001;

    // One Elga request per second
    const id = ELGA_CYCLE[t % ELGA_CYCLE.length];
    const flowR = ds(s.flow), status = (s.level ? 0x02 | 0x08 : 0);   // CH mode and flame from OpenHAB
    let q, r;
    const fill = f88(s.roomSp);                                      // The Elga puts the room setpoint in READ requests
    switch (id) {
      case 0: q = frame(RD, 0, (s.ch ? 0x0300 : 0x0200) | s.lastStatus); r = frame(RACK, 0, (q & 0xFF00) | status); s.lastStatus = status; break;
      case 1: q = frame(WR, 1, f88(s.sp)); r = frame(WACK, 1, q & 0xFFFF); break;
      case 5: q = frame(RD, 5, 0); r = frame(RACK, 5, 0); break;
      case 16: q = frame(WR, 16, fill); r = frame(WACK, 16, fill); break;
      case 17: q = frame(RD, 17, fill); r = frame(RACK, 17, f88(s.mod)); break;
      case 18: q = frame(RD, 18, fill); r = frame(RACK, 18, f88(1.8)); break;
      case 19: q = frame(RD, 19, fill); r = frame(RACK, 19, 0); break;
      case 24: q = frame(RD, 24, f88(Math.round(s.room * 100) / 100)); r = frame(RACK, 24, q & 0xFFFF); break;
      case 25: q = frame(RD, 25, fill); r = frame(RACK, 25, f88(flowR)); break;
      case 26: q = frame(RD, 26, fill); r = frame(RACK, 26, 0); break;
      case 56: q = frame(RD, 56, fill); r = frame(RACK, 56, f88(65)); break;
    }
    const lat = 250 + (rnd() > 0.45 ? 1 : 0) + (rnd() > 0.48 ? 1 : 0);
    s.frames.push({ s: ++s.seq, t: (UP0 - DEMAND_AGO + t) * 1000 + 412, q: hex(q), r: hex(r), g: 1004 + Math.round(rnd() * 6), l: lat, f: 1 });
    if (s.frames.length > 128) s.frames.shift();
    if (on(0)) { log('ot', describe('T', q), 412); log('ot', describe('B', r) + ` Replied after: ${lat}ms.`, 412 + lat); }
    if (on(5) && (r >>> 28 & 7) === RACK && NAMES[id][1]) log('convert', `ID ${id}: value ${from88(r & 0xFFFF).toFixed(2)} to f8.8 ${(r & 0xFFFF).toString(16).toUpperCase().padStart(4, '0')}`);
    if (on(6) && t % 5 === 0) log('1wire', `Temperature heater is: ${flowR.toFixed(2)} and return water is: ${ds(s.ret).toFixed(2)} celcius.`);

    // Trend: one point every 10 s, 30 minutes
    if (t % 10 === 0) {
      const h = s.hist;
      h.flow.push(+flowR.toFixed(2)); h.ret.push(+ds(s.ret).toFixed(2)); h.sp.push(+s.sp.toFixed(2)); h.mod.push(+s.mod.toFixed(2));
      for (const k in h) if (h[k].length > 180) h[k].shift();
    }
    s.sec++;
  }

  // Boot messages, then run the simulation up to the moment the page opens
  log('sys', `Soft E-CV ${FW}, reset reason: Power On`);
  log('sys', 'Connecting to WiFi network HomeWiFi');
  log('sys', 'HTTP server started');
  log('sys', '1-Wire: 2 sensors found on D3');
  log('sys', 'WiFi connected to HomeWiFi, IP address 192.168.1.50, signal -61 dBm');
  log('sys', 'Reachable at http://soft-ecv.local');
  log('sys', 'MQTT connected to 192.168.1.10:1883');
  for (const l of sim.log) l.t = l.s * 900;                          // Boot lines at the start of the uptime
  while (sim.sec < DEMAND_AGO) step();
  const advance = () => { const target = DEMAND_AGO + Math.floor((Date.now() - T0) / 1000); while (sim.sec < target) step(); };
  const upNow = () => UP0 + Math.floor((Date.now() - T0) / 1000);

  // ---------- API ----------
  const FLOW_ADDR = '28FF4A1C631603D2', RET_ADDR = '2818CD79A200034A';
  function status() {
    const s = sim, flow = ds(s.flow), ret = ds(s.ret);
    return {
      fw: FW, up: upNow(), heap: 22800 + Math.round(rnd() * 600), heap_block: 19800 + Math.round(rnd() * 400), busy: busyCount,
      reset: 'Power On', core: '3.1.2', name: cfg.hostname,
      wifi: { ok: true, ssid: cfg.wifi_ssid, rssi: -61 + Math.round(rnd() * 4), ip: '192.168.1.50', mac: '84:CC:A8:00:12:34', ap: false, ap_ssid: 'SoftECV-Setup-1234', ap_clients: 0 },
      mqtt: { host: cfg.mqtt_host, port: cfg.mqtt_port, ok: true, state: 0, rx_age: 3 + (s.sec % 7) },
      ot: { frames: s.seq, invalid: 0, timeouts: 0, unknown: 0, age_ms: Math.round(300 + rnd() * 400), fpm: 60, delay: 250, seq: s.seq },
      leader: { seen: true, ch: s.ch, dhw: true, sp: +s.sp.toFixed(1), room_sp: s.roomSp, room_t: +s.room.toFixed(2), max_mod: 100 },
      ecv: {
        flow, flow_src: 'sensor', ret, ret_src: 'sensor', mod: +s.mod.toFixed(2), pres: 1.8, out: 6.5, dhw_t: 0, dhw_flow: 0,
        max_ch: 70, dhw_sp: 65, max_mod: 100, fault: false, sensor_fault: false, ch_mode: s.level > 0, flame: s.level > 0, modulating: cfg.modulating
      },
      auth: { set: true, ok: signedIn }, setup: false
    };
  }
  function get(path, query) {
    switch (path) {
      case '/api/status': return status();
      case '/api/history': return { step: 10, ...sim.hist };
      case '/api/frames': {
        const after = +query.get('after') || 0;
        return { now: upNow() * 1000, delay: 250, frames: sim.frames.filter(f => f.s > after).slice(-40) };
      }
      case '/api/log': { const after = +query.get('after') || 0; return sim.log.filter(l => l.s > after); }
      case '/api/sensors':
        return { found: [{ a: FLOW_ADDR, t: ds(sim.flow) }, { a: RET_ADDR, t: ds(sim.ret) }], flow: FLOW_ADDR, ret: RET_ADDR, flow_t: ds(sim.flow), ret_t: ds(sim.ret), scanning: false };
      case '/api/debug': return { flags: sim.flags, ...dbg };
      case '/api/settings': return signedIn ? cfg : null;
      case '/api/wifi/scan':
        return { busy: false, nets: [{ ssid: 'HomeWiFi', rssi: -61, secure: true }, { ssid: 'HomeWiFi-Guest', rssi: -67, secure: true }, { ssid: 'Neighbours', rssi: -82, secure: true }] };
    }
    return undefined;
  }
  function post(path, body) {
    switch (path) {
      case '/api/login': signedIn = true; return {};
      case '/api/logout': signedIn = false; return {};
      case '/api/debug':
        sim.flags = +body.get('flags') || 0;
        for (const k of ['serial', 'web', 'keep']) dbg[k] = body.get(k) === '1';
        return {};
      case '/api/settings/device':
        cfg.hostname = body.get('hostname') || cfg.hostname; cfg.modulating = body.get('modulating') === '1';
        cfg.report_sensor_fault = body.get('sensor_fault') === '1'; return {};
      case '/api/settings/mqtt':
        cfg.mqtt_host = body.get('host') || cfg.mqtt_host; cfg.mqtt_port = +body.get('port') || 1883;
        cfg.mqtt_user = body.get('user') || ''; if (body.get('client_id')) cfg.mqtt_client_id = body.get('client_id'); return {};
      case '/api/sensors/scan':
        log('sys', '1-Wire: 2 sensors found on D3'); return {};
      case '/api/restart': case '/api/erase':
        return { error: 'Not available in the demo.', status: 400 };
    }
    return {};                                                       // Other settings: accepted, nothing to do
  }

  const json = (obj, status = 200) => new Response(JSON.stringify(obj), { status, headers: { 'Content-Type': 'application/json' } });
  const realFetch = window.fetch.bind(window);
  window.fetch = async (input, opt = {}) => {
    const url = new URL(typeof input === 'string' ? input : input.url, location.href);
    if (!url.pathname.startsWith('/api/')) return realFetch(input, opt);
    advance();
    await new Promise(r => setTimeout(r, 15 + Math.random() * 40));   // A little network delay
    if ((opt.method || 'GET') === 'POST') {
      const res = post(url.pathname, opt.body instanceof URLSearchParams ? opt.body : new URLSearchParams());
      return res && res.error ? json({ error: res.error }, res.status) : json({ ok: true });
    }
    const res = get(url.pathname, url.searchParams);
    if (res === null) return json({ error: 'Sign in first.' }, 401);
    return res === undefined ? json({ error: 'Not found.' }, 404) : json(res);
  };

  // Firmware upload uses XMLHttpRequest: answer that it is not available in the demo
  const RealXHR = window.XMLHttpRequest;
  window.XMLHttpRequest = function () {
    const x = new RealXHR(); let fake = false;
    const open = x.open.bind(x);
    x.open = (m, u, ...a) => { fake = String(u).startsWith('/api/'); if (!fake) open(m, u, ...a); };
    const send = x.send.bind(x);
    x.send = b => {
      if (!fake) return send(b);
      setTimeout(() => {
        Object.defineProperty(x, 'status', { value: 400 });
        Object.defineProperty(x, 'responseText', { value: JSON.stringify({ error: 'Firmware updates are not available in the demo.' }) });
        x.onload && x.onload();
      }, 400);
    };
    return x;
  };

  // demo.html#settings or #debug opens that tab
  addEventListener('load', () => {
    const tab = document.querySelector(`[data-view="view-${location.hash.slice(1)}"]`);
    if (tab) setTimeout(() => tab.click(), 100);
  });

  // Demo notice above the page, left out with ?screenshot (which also switches off animations for headless screenshots)
  if (new URLSearchParams(location.search).has('screenshot')) {
    addEventListener('DOMContentLoaded', () => {
      const st = document.createElement('style');
      st.textContent = '*, *::before, *::after { transition: none !important; animation: none !important; }';
      document.head.append(st);
    });
  } else {
    addEventListener('DOMContentLoaded', () => {
      const r = document.createElement('div');
      r.className = 'ribbon';
      r.textContent = 'Demo with simulated data: a heating cycle of the Elga and the E-CV, computed in your browser. Nothing is sent to a real D1.';
      document.body.prepend(r);
    });
  }
})();
