#!/usr/bin/env node
/*
 * MQTT activity indicator on MAX7219 (4 x 8x8, mounted vertically -> 8 x 32).
 * Node.js port of mqtt_display.py, same behaviour and settings.
 *
 * Each MQTT message = one row at the bottom of the screen; the number of lit
 * LEDs depends on the payload length (see SCALE). Older rows scroll up like
 * a console log.
 *
 * Run:  node mqtt_display.js                 (real MAX7219 via SPI)
 *       node mqtt_display.js --console       (no hardware, ASCII output)
 *       node mqtt_display.js --test          (orientation test, see orientationTest)
 *       node mqtt_display.js --test --once   (show test with current settings)
 */

'use strict';

// ============================================================
// CONFIGURATION
// ============================================================

// MQTT
const MQTT_HOST = 'localhost';
const MQTT_PORT = 1883;
const MQTT_USERNAME = 'admin';
const MQTT_PASSWORD = 'password';
const MQTT_TOPIC = '#';
const MQTT_CLIENT_ID = 'mqtt-display';
const MQTT_KEEPALIVE = 60;
const MQTT_RECONNECT_MIN_DELAY = 1;    // seconds
const MQTT_RECONNECT_MAX_DELAY = 30;   // seconds

// Retained messages are sent by the broker in one burst right after
// subscribing: Home Assistant discovery configs, bridge/devices, etc.
// They are old state, not activity.
//   false -> do not show them on the display (only their count is logged)
//   true  -> show them like normal messages
const SHOW_RETAINED = false;

// Blink command. A message to this topic does not add a row to the screen.
//   payload "true"           -> blink for BLINK_DEFAULT_SECONDS seconds
//   payload number, e.g. 5   -> blink for 5 seconds (max BLINK_MAX_SECONDS)
//   payload "false" or 0     -> stop blinking
// Blinks at maximum brightness, then restores the previous picture and brightness.
const BLINK_TOPIC = '/mqtt-log-indicator/blink';
const BLINK_DEFAULT_SECONDS = 1;
const BLINK_MAX_SECONDS = 60;
const BLINK_HZ = 10;          // flashes per second
const BLINK_MODE = 'full';    // "full" = all LEDs, "content" = current picture

// DISPLAY
const WIDTH = 8;
const HEIGHT = 32;

// Scale: "linear", "log" or "adaptive"
const SCALE = 'log';

// linear: number of characters per LED
//   leds = ceil(length / CHARS_PER_LED)
const CHARS_PER_LED = 20;

// log: each next LED needs LOG_BASE times more characters
//   1 LED  if length >= 1 (any non-empty payload)
//   2 LEDs if length >= LOG_MIN_CHARS * LOG_BASE
//   3 LEDs if length >= LOG_MIN_CHARS * LOG_BASE^2 ...
// LOG_BASE = 3, LOG_MIN_CHARS = 1:
//   1-2 | 3-8 | 9-26 | 27-80 | 81-242 | 243-728 | 729-2186 | 2187+
// LOG_BASE = 2, LOG_MIN_CHARS = 1:
//   1 | 2-3 | 4-7 | 8-15 | 16-31 | 32-63 | 64-127 | 128+
const LOG_BASE = 3;
const LOG_MIN_CHARS = 1;

// adaptive: the scale adapts to the actual traffic.
//   Remembers the lengths of the last ADAPTIVE_WINDOW non-empty messages;
//   LEDs = share of messages in the window shorter than this one (percentile) * 8.
//   Shortest -> 1 LED, longest -> 8 LEDs, the rest spread evenly
//   in between, whatever the lengths on the network are.
//   Until there are ADAPTIVE_MIN_SAMPLES messages, the "log" scale is used.
const ADAPTIVE_WINDOW = 500;
const ADAPTIVE_MIN_SAMPLES = 30;
// How often (in messages) to log the current thresholds; 0 = never
const ADAPTIVE_LOG_EVERY = 200;

// Maximum LEDs per row
const MAX_LEDS_PER_ROW = 8;

// Brightness 0..15
const BRIGHTNESS = 5;

// SPI
const SPI_BUS = 0;
const SPI_DEVICE = 0;
const SPI_SPEED = 1_000_000;

// MAX7219
const NUM_MODULES = 4;

// ---- Physical mapping (only affects updateDisplay) ----------
// Logical screen: row 0 = top, column 0 = left.
//
// Mirror the WHOLE picture (simplest fix, try these first):
//   SCREEN_FLIP_X = true  -> LEDs grow from the right edge become from the left
//   SCREEN_FLIP_Y = true  -> new rows appear at the top become at the bottom
const SCREEN_FLIP_X = true;
const SCREEN_FLIP_Y = false;
// Module 0 = the module nearest to DIN (first in the chain).
//
// MODULE_TOP_IS_FIRST: true  -> module 0 shows logical rows 0..7 (top),
//                      false -> module 0 shows logical rows 24..31 (bottom).
let MODULE_TOP_IS_FIRST = true;
//
// Rotation of each 8x8 block before it is sent: 0, 90, 180 or 270 degrees
// (clockwise). Typical FC-16 "4-in-1" boards turned vertical need 90 or 270.
let MODULE_ROTATION = 90;
//
// Mirror each 8x8 block horizontally after rotation.
let MODULE_FLIP_X = true;
//
// Per-module overrides, e.g. {2: 180} if one matrix is soldered upside down.
const MODULE_ROTATION_OVERRIDE = {};
//
// Bit order inside a MAX7219 digit register:
//   true  -> column 0 = bit 7 (most FC-16 modules)
//   false -> column 0 = bit 0
const MSB_IS_LEFT = true;

// ============================================================
// END OF CONFIGURATION
// ============================================================

// MAX7219 registers
const REG_DIGIT0 = 0x01;
const REG_DECODE_MODE = 0x09;
const REG_INTENSITY = 0x0A;
const REG_SCAN_LIMIT = 0x0B;
const REG_SHUTDOWN = 0x0C;
const REG_DISPLAY_TEST = 0x0F;

const argv = process.argv.slice(2);

function log(msg) {
  process.stdout.write(msg + '\n');
}

function clamp(v, lo, hi) {
  return Math.max(lo, Math.min(hi, v));
}

function blankScreen() {
  return Array.from({ length: HEIGHT }, () => new Array(WIDTH).fill(0));
}

// ------------------------------------------------------------
// Screen buffer (logical)
// ------------------------------------------------------------

const screen = blankScreen();

/** Scroll everything up by one row and add a new row at the bottom. */
function pushRow(leds) {
  screen.shift();
  screen.push(Array.from({ length: WIDTH }, (_, c) => (c < leds ? 1 : 0)));
}

// ------------------------------------------------------------
// Scales
// ------------------------------------------------------------

/** Rolling window of recent payload lengths; LEDs by percentile rank. */
class AdaptiveScale {
  constructor(size) {
    this.window = [];
    this.sorted = [];
    this.size = size;
    this.seen = 0;
  }

  static bisectLeft(a, x) {
    let lo = 0, hi = a.length;
    while (lo < hi) { const m = (lo + hi) >> 1; if (a[m] < x) lo = m + 1; else hi = m; }
    return lo;
  }

  static bisectRight(a, x) {
    let lo = 0, hi = a.length;
    while (lo < hi) { const m = (lo + hi) >> 1; if (a[m] <= x) lo = m + 1; else hi = m; }
    return lo;
  }

  add(length) {
    this.window.push(length);
    this.sorted.splice(AdaptiveScale.bisectRight(this.sorted, length), 0, length);
    if (this.window.length > this.size) {
      const old = this.window.shift();
      this.sorted.splice(AdaptiveScale.bisectLeft(this.sorted, old), 1);
    }
    this.seen += 1;
  }

  ready() {
    return this.sorted.length >= ADAPTIVE_MIN_SAMPLES;
  }

  leds(length, limit) {
    const n = this.sorted.length;
    const below = AdaptiveScale.bisectLeft(this.sorted, length);
    const upto = AdaptiveScale.bisectRight(this.sorted, length);
    const rank = (below + upto) / 2 / n;          // mid-rank, 0..1
    return clamp(Math.ceil(rank * limit), 1, limit);
  }
}

const adaptive = new AdaptiveScale(ADAPTIVE_WINDOW);

/** learn=true: count this message in adaptive statistics (real messages only). */
function ledsForLength(length, learn = false) {
  const limit = Math.min(MAX_LEDS_PER_ROW, WIDTH);
  if (length <= 0) return 0;
  if (SCALE === 'adaptive') {
    if (learn) adaptive.add(length);
    if (adaptive.ready()) return adaptive.leds(length, limit);
    // not enough data yet -> log scale
  }
  if (SCALE === 'log' || SCALE === 'adaptive') {
    // integer thresholds, no float log() rounding issues
    let leds = 1;
    let threshold = LOG_MIN_CHARS * LOG_BASE;
    while (leds < limit && length >= threshold) {
      leds += 1;
      threshold *= LOG_BASE;
    }
    return leds;
  }
  return Math.min(limit, Math.ceil(length / CHARS_PER_LED));
}

/** Human-readable table 'LEDs: length range' for the log. */
function scaleDescription() {
  const limit = Math.min(MAX_LEDS_PER_ROW, WIDTH);

  // ledsForLength is monotonic -> exponential + binary search
  const firstLengthWith = (k) => {
    let hi = 1;
    while (ledsForLength(hi) < k) hi *= 2;
    let lo = hi > 1 ? Math.floor(hi / 2) + 1 : 1;
    while (lo < hi) {
      const mid = Math.floor((lo + hi) / 2);
      if (ledsForLength(mid) >= k) hi = mid; else lo = mid + 1;
    }
    return lo;
  };

  const starts = [];
  for (let k = 1; k <= limit; k++) starts.push(firstLengthWith(k));
  const parts = [];
  for (let k = 1; k <= limit; k++) {
    const lo = starts[k - 1];
    if (k === limit) {
      parts.push(`${k} LED: ${lo}+`);
    } else {
      const hi = starts[k] - 1;
      if (hi < lo) parts.push(`${k} LED: -`);   // skipped (many equal lengths)
      else parts.push(`${k} LED: ${hi === lo ? lo : `${lo}-${hi}`}`);
    }
  }
  let name = SCALE;
  if (SCALE === 'adaptive') {
    name = adaptive.ready()
      ? `adaptive, ${adaptive.sorted.length} msgs in window`
      : `adaptive (warming up ${adaptive.sorted.length}/${ADAPTIVE_MIN_SAMPLES}, using log)`;
  }
  return `SCALE=${name} | ${parts.join(' | ')}`;
}

// ------------------------------------------------------------
// MAX7219 output
// ------------------------------------------------------------

/** Rotate an 8x8 block (array of rows) clockwise. */
function rotateBlock(block, degrees) {
  const turns = ((Math.floor(degrees / 90) % 4) + 4) % 4;
  for (let t = 0; t < turns; t++) {
    const rev = block.slice().reverse();
    block = rev[0].map((_, c) => rev.map((row) => row[c]));
  }
  return block;
}

class Max7219 {
  constructor() {
    // Loaded here so --console works without the native module installed.
    let spi;
    try {
      spi = require('spi-device');
    } catch (e) {
      log('DISPLAY | module "spi-device" is not installed: run `npm install` '
        + '(needs build tools: sudo apt install build-essential), or use --console');
      process.exit(1);
    }
    this.device = spi.openSync(SPI_BUS, SPI_DEVICE, {
      mode: spi.MODE0,
      maxSpeedHz: SPI_SPEED,
    });
    this.initChips();
  }

  /** values[m] goes to module m (module 0 = nearest to DIN). */
  writeAll(register, values) {
    const data = [];
    // First bytes shifted in end up in the farthest module.
    for (let m = NUM_MODULES - 1; m >= 0; m--) data.push(register, values[m] & 0xFF);
    const sendBuffer = Buffer.from(data);
    this.device.transferSync([{ sendBuffer, byteLength: sendBuffer.length, speedHz: SPI_SPEED }]);
  }

  writeSame(register, value) {
    this.writeAll(register, new Array(NUM_MODULES).fill(value));
  }

  initChips() {
    this.writeSame(REG_DISPLAY_TEST, 0);
    this.writeSame(REG_SCAN_LIMIT, 7);
    this.writeSame(REG_DECODE_MODE, 0);
    this.writeSame(REG_INTENSITY, clamp(BRIGHTNESS, 0, 15));
    this.clear();
    this.writeSame(REG_SHUTDOWN, 1);
  }

  clear() {
    for (let d = 0; d < 8; d++) this.writeSame(REG_DIGIT0 + d, 0);
  }

  setBrightness(value) {
    this.writeSame(REG_INTENSITY, clamp(Math.trunc(value), 0, 15));
  }

  showBuffer(buf) {
    const digits = screenToModules(buf);
    for (let d = 0; d < 8; d++) {
      this.writeAll(REG_DIGIT0 + d, digits.map((m) => m[d]));
    }
  }

  close() {
    try {
      this.clear();
      this.writeSame(REG_SHUTDOWN, 0);
    } finally {
      this.device.closeSync();
    }
  }
}

/** Convert logical screen[HEIGHT][WIDTH] into per-module digit bytes. */
function screenToModules(buf) {
  if (SCREEN_FLIP_X) buf = buf.map((row) => row.slice().reverse());
  if (SCREEN_FLIP_Y) buf = buf.slice().reverse();
  const modules = [];
  for (let m = 0; m < NUM_MODULES; m++) {
    const blockIndex = MODULE_TOP_IS_FIRST ? m : NUM_MODULES - 1 - m;
    let block = [];
    for (let r = 0; r < 8; r++) block.push(buf[blockIndex * 8 + r].slice(0, 8));
    const rot = m in MODULE_ROTATION_OVERRIDE ? MODULE_ROTATION_OVERRIDE[m] : MODULE_ROTATION;
    block = rotateBlock(block, rot);
    if (MODULE_FLIP_X) block = block.map((row) => row.slice().reverse());
    const digits = [];
    for (let r = 0; r < 8; r++) {
      let byte = 0;
      for (let c = 0; c < 8; c++) {
        if (block[r][c]) byte |= MSB_IS_LEFT ? (0x80 >> c) : (1 << c);
      }
      digits.push(byte);
    }
    modules.push(digits);
  }
  return modules;
}

/** Stand-in for MAX7219 when run with --console. */
class ConsoleDisplay {
  constructor() {
    this.quiet = false;  // suppress frames while blinking (20 frames/s is unreadable)
  }

  showBuffer(buf) {
    if (this.quiet) return;
    log(buf.map((row) => row.map((v) => (v ? '●' : '.')).join('')).join('\n'));
    log('-'.repeat(WIDTH));
  }

  setBrightness() {}

  close() {}
}

let display = null;

/** Checkerboard pattern; phase 0/1 swaps lit and dark cells. */
function checkerboard(phase) {
  return Array.from({ length: HEIGHT }, (_, r) =>
    Array.from({ length: WIDTH }, (_, c) => (r + c + phase) % 2));
}

/** Send a logical buffer to the hardware. */
function render(buf) {
  try {
    display.showBuffer(buf);
  } catch (e) {  // SPI glitch must not kill the MQTT loop
    log(`DISPLAY | error: ${e.message}`);
  }
}

/**
 * Only converts the buffer and sends it to the hardware.
 * override: show this buffer instead of the log screen (screen is kept).
 * While blinking, does nothing: the blink timer redraws when it ends.
 */
function updateDisplay(override = null) {
  if (blinkTimer) return;
  render(override || screen);
}

// ------------------------------------------------------------
// Blink (command topic)
// ------------------------------------------------------------

let blinkUntil = 0;
let blinkTimer = null;
let blinkOn = true;

/** Return seconds to blink (0 = stop), or null if payload is not understood. */
function parseBlinkPayload(text) {
  const t = text.trim().toLowerCase();
  if (['true', 'on', 'yes'].includes(t)) return BLINK_DEFAULT_SECONDS;
  if (['false', 'off', 'no', 'stop'].includes(t)) return 0;
  if (!/^[+-]?(\d+([.,]\d*)?|[.,]\d+)(e[+-]?\d+)?$/.test(t)) return null;
  const value = Number(t.replace(',', '.'));
  if (!Number.isFinite(value)) return null;
  return clamp(value, 0, BLINK_MAX_SECONDS);
}

/** Start blinking, or extend/shorten a running blink; 0 stops it. */
function startBlink(seconds) {
  blinkUntil = Date.now() + seconds * 1000;
  if (seconds <= 0) {
    if (blinkTimer) blinkTick();  // running blink ends on this tick
    return;
  }
  if (blinkTimer) return;  // running timer picks up the new deadline
  blinkOn = true;
  if (display instanceof ConsoleDisplay) display.quiet = true;
  blinkTimer = setInterval(blinkTick, 1000 / (2 * BLINK_HZ));
  blinkTick();
}

function blinkTick() {
  if (Date.now() >= blinkUntil) {
    clearInterval(blinkTimer);
    blinkTimer = null;
    try {
      display.setBrightness(BRIGHTNESS);
    } catch (e) {
      log(`DISPLAY | error: ${e.message}`);
    }
    if (display instanceof ConsoleDisplay) display.quiet = false;
    // back to the previous state (log, or checkerboard if auth fails)
    updateDisplay(authFailed ? checkerboard(authFailPhase ^ 1) : null);
    log('BLINK | done');
    return;
  }
  let frame;
  if (blinkOn) {
    frame = BLINK_MODE === 'content'
      ? screen
      : Array.from({ length: HEIGHT }, () => new Array(WIDTH).fill(1));
  } else {
    frame = blankScreen();
  }
  try {
    display.setBrightness(15);
  } catch (e) {
    log(`DISPLAY | error: ${e.message}`);
  }
  render(frame);
  blinkOn = !blinkOn;
}

// ------------------------------------------------------------
// MQTT
// ------------------------------------------------------------

// CONNACK codes meaning "wrong login/password" / "not authorized"
// (MQTT 3.1.1: 4, 5; MQTT 5: 134, 135)
const AUTH_FAIL_CODES = [4, 5, 134, 135];
let authFailPhase = 0;
let authFailed = false;
let retainedSkipped = 0;

function onConnect(client) {
  log(`MQTT | connected to ${MQTT_HOST}:${MQTT_PORT}, subscribing to ${MQTT_TOPIC}`);
  client.subscribe(MQTT_TOPIC);  // re-subscribe after every reconnect
  if (authFailed) {
    authFailed = false;
    updateDisplay();  // checkerboard -> back to the message log
  }
}

function onAuthFailed(code) {
  authFailed = true;
  log(`MQTT | auth failed (rc=${code}): check MQTT_USERNAME / MQTT_PASSWORD`);
  updateDisplay(checkerboard(authFailPhase));
  authFailPhase ^= 1;  // invert on every attempt
}

function onMessage(topic, payload, packet) {
  try {
    if (topic === BLINK_TOPIC) {
      if (packet.retain) {
        // an old retained command must not blink on every reconnect
        log('BLINK | ignored retained command');
        return;
      }
      const text = payload.toString('utf8');
      const seconds = parseBlinkPayload(text);
      if (seconds === null) {
        log(`BLINK | unknown payload ${JSON.stringify(text.slice(0, 40))}, expected true/false/number`);
      } else {
        log(`BLINK | ${seconds === 0 ? 'stop' : `${seconds} s`}`);
        startBlink(seconds);
      }
      return;
    }
    if (packet.retain && !SHOW_RETAINED) {
      retainedSkipped += 1;
      return;
    }
    if (retainedSkipped) {
      log(`MQTT | skipped ${retainedSkipped} retained messages`);
      retainedSkipped = 0;
    }
    // Invalid UTF-8 is replaced with U+FFFD; length is counted in characters
    // (code points), not bytes and not UTF-16 units.
    const text = payload.toString('utf8');
    const length = Array.from(text).length;
    const leds = ledsForLength(length, true);
    pushRow(leds);
    updateDisplay();
    log(`MQTT | topic=${topic} | length=${length} | LEDs=${leds}`);
    if (SCALE === 'adaptive' && ADAPTIVE_LOG_EVERY > 0 && length > 0
        && (adaptive.seen === ADAPTIVE_MIN_SAMPLES || adaptive.seen % ADAPTIVE_LOG_EVERY === 0)) {
      log(scaleDescription());
    }
  } catch (e) {
    log(`MQTT | error handling message: ${e.message}`);
  }
}

function makeClient() {
  const mqtt = require('mqtt');
  const client = mqtt.connect({
    host: MQTT_HOST,
    port: MQTT_PORT,
    protocol: 'mqtt',
    protocolVersion: 4,
    clientId: MQTT_CLIENT_ID,
    username: MQTT_USERNAME || undefined,
    password: MQTT_USERNAME ? MQTT_PASSWORD : undefined,
    keepalive: MQTT_KEEPALIVE,
    clean: true,
    resubscribe: false,  // we subscribe ourselves in onConnect
    reconnectPeriod: MQTT_RECONNECT_MIN_DELAY * 1000,
    // mqtt.js stops reconnecting after a refused CONNACK (e.g. wrong password)
    // unless this is set; we want to keep retrying (and show the checkerboard)
    reconnectOnConnackError: true,
    connectTimeout: 10 * 1000,
  });

  client.on('connect', () => {
    client.options.reconnectPeriod = MQTT_RECONNECT_MIN_DELAY * 1000;
    onConnect(client);
  });

  client.on('error', (err) => {
    if (AUTH_FAIL_CODES.includes(err.code)) {
      onAuthFailed(err.code);
    } else {
      log(`MQTT | connect failed: ${err.message}`);
    }
  });

  client.on('close', () => {
    // exponential backoff between reconnect attempts
    client.options.reconnectPeriod = Math.min(
      client.options.reconnectPeriod * 2, MQTT_RECONNECT_MAX_DELAY * 1000);
  });

  client.on('offline', () => log('MQTT | disconnected, will reconnect'));
  client.on('message', onMessage);
  return client;
}

// ------------------------------------------------------------
// Main
// ------------------------------------------------------------

/**
 * Draw a reference pattern and cycle through all mapping variants.
 * Correct picture: one triangle over the whole height, pressed to the LEFT
 * edge, 1 LED wide at the TOP, growing by 1 LED every 4 rows,
 * 8 LEDs wide at the BOTTOM.
 */
function orientationTest() {
  for (let r = 0; r < HEIGHT; r++) {
    const n = Math.min(WIDTH, Math.floor(r / 4) + 1);
    screen[r] = Array.from({ length: WIDTH }, (_, c) => (c < n ? 1 : 0));
  }
  let variants = [];
  for (const top of [true, false]) {
    for (const rot of [0, 90, 180, 270]) {
      for (const flip of [false, true]) variants.push([rot, flip, top]);
    }
  }
  if (argv.includes('--once')) variants = [[MODULE_ROTATION, MODULE_FLIP_X, MODULE_TOP_IS_FIRST]];
  let i = 0;
  const step = () => {
    const [rot, flip, top] = variants[i % variants.length];
    MODULE_ROTATION = rot; MODULE_FLIP_X = flip; MODULE_TOP_IS_FIRST = top;
    log(`TEST ${String((i % variants.length) + 1).padStart(2)}/${variants.length} | `
      + `MODULE_ROTATION = ${rot} | MODULE_FLIP_X = ${flip} | MODULE_TOP_IS_FIRST = ${top}`);
    updateDisplay();
    i += 1;
  };
  step();
  setInterval(step, 6000);
}

function main() {
  display = argv.includes('--console') ? new ConsoleDisplay() : new Max7219();

  let client = null;
  const shutdown = (signal) => {
    log(`Stopping (signal ${signal})`);
    try {
      if (client) client.end(true);
    } finally {
      try { display.close(); } finally { process.exit(0); }
    }
  };
  process.on('SIGTERM', () => shutdown('SIGTERM'));
  process.on('SIGINT', () => shutdown('SIGINT'));

  if (argv.includes('--test')) {
    orientationTest();
    return;
  }

  updateDisplay();
  log(scaleDescription());
  client = makeClient();
}

if (require.main === module) {
  main();
}

module.exports = {
  ledsForLength, scaleDescription, parseBlinkPayload, screenToModules,
  checkerboard, AdaptiveScale, pushRow, screen,
};
