#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MQTT activity indicator on MAX7219 (4 x 8x8, mounted vertically -> 8 x 32).

Each MQTT message = one row at the bottom of the screen; the number of lit
LEDs is ceil(len(payload) / CHARS_PER_LED), capped at MAX_LEDS_PER_ROW.
Older rows scroll up like a console log.

Run:  python3 mqtt_display.py            (real MAX7219 via SPI)
      python3 mqtt_display.py --console  (no hardware, ASCII output)
      python3 mqtt_display.py --test     (orientation test, see orientation_test)
      python3 mqtt_display.py --test --once  (show test with current settings)
"""

# ============================================================
# CONFIGURATION
# ============================================================

# MQTT
MQTT_HOST = "localhost"
MQTT_PORT = 1883
MQTT_USERNAME = "admin"
MQTT_PASSWORD = "password"
MQTT_TOPIC = "#"
MQTT_CLIENT_ID = "mqtt-display"
MQTT_KEEPALIVE = 60
MQTT_RECONNECT_MIN_DELAY = 1     # seconds
MQTT_RECONNECT_MAX_DELAY = 30    # seconds

# Retained messages are sent by the broker in one burst right after
# subscribing: Home Assistant discovery configs, bridge/devices, etc.
# They are old state, not activity.
#   False -> do not show them on the display (only their count is logged)
#   True  -> show them like normal messages
SHOW_RETAINED = False

# Blink command. A message to this topic does not add a row to the screen.
#   payload "true"           -> blink for BLINK_DEFAULT_SECONDS seconds
#   payload number, e.g. 5   -> blink for 5 seconds (max BLINK_MAX_SECONDS)
#   payload "false" or 0     -> stop blinking
# Blinks at maximum brightness, then restores the previous picture and brightness.
BLINK_TOPIC = "/mqtt-log-indicator/blink"
BLINK_DEFAULT_SECONDS = 1
BLINK_MAX_SECONDS = 60
BLINK_HZ = 10          # flashes per second
BLINK_MODE = "full"    # "full" = all LEDs, "content" = current picture

# DISPLAY
WIDTH = 8
HEIGHT = 32

# Scale: "linear", "log" or "adaptive"
SCALE = "log"

# linear: number of characters per LED
#   leds = ceil(length / CHARS_PER_LED)
CHARS_PER_LED = 20

# log: each next LED needs LOG_BASE times more characters
#   1 LED  if length >= 1 (any non-empty payload)
#   2 LEDs if length >= LOG_MIN_CHARS * LOG_BASE
#   3 LEDs if length >= LOG_MIN_CHARS * LOG_BASE^2 ...
# LOG_BASE = 3, LOG_MIN_CHARS = 1:
#   1-2 | 3-8 | 9-26 | 27-80 | 81-242 | 243-728 | 729-2186 | 2187+
# LOG_BASE = 2, LOG_MIN_CHARS = 1:
#   1 | 2-3 | 4-7 | 8-15 | 16-31 | 32-63 | 64-127 | 128+
LOG_BASE = 3
LOG_MIN_CHARS = 1

# adaptive: the scale adapts to the actual traffic.
#   Remembers the lengths of the last ADAPTIVE_WINDOW non-empty messages;
#   LEDs = share of messages in the window shorter than this one (percentile) * 8.
#   Shortest -> 1 LED, longest -> 8 LEDs, the rest spread evenly
#   in between, whatever the lengths on the network are.
#   Until there are ADAPTIVE_MIN_SAMPLES messages, the "log" scale is used.
ADAPTIVE_WINDOW = 500
ADAPTIVE_MIN_SAMPLES = 30
# How often (in messages) to log the current thresholds; 0 = never
ADAPTIVE_LOG_EVERY = 200

# Maximum LEDs per row
MAX_LEDS_PER_ROW = 8

# Brightness 0..15
BRIGHTNESS = 5

# SPI
SPI_BUS = 0
SPI_DEVICE = 0
SPI_SPEED = 1_000_000

# MAX7219
NUM_MODULES = 4

# ---- Physical mapping (only affects update_display) ---------
# Logical screen: row 0 = top, column 0 = left.
#
# Mirror the WHOLE picture (simplest fix, try these first):
#   SCREEN_FLIP_X = True  -> LEDs grow from the right edge become from the left
#   SCREEN_FLIP_Y = True  -> new rows appear at the top become at the bottom
SCREEN_FLIP_X = True
SCREEN_FLIP_Y = False
# Module 0 = the module nearest to DIN (first in the chain).
#
# MODULE_TOP_IS_FIRST: True  -> module 0 shows logical rows 0..7 (top),
#                      False -> module 0 shows logical rows 24..31 (bottom).
MODULE_TOP_IS_FIRST = True
#
# Rotation of each 8x8 block before it is sent: 0, 90, 180 or 270 degrees
# (clockwise). Typical FC-16 "4-in-1" boards turned vertical need 90 or 270.
MODULE_ROTATION = 90
#
# Mirror each 8x8 block horizontally after rotation.
MODULE_FLIP_X = True
#
# Per-module overrides, e.g. {2: 180} if one matrix is soldered upside down.
MODULE_ROTATION_OVERRIDE = {}
#
# Bit order inside a MAX7219 digit register:
#   True  -> column 0 = bit 7 (most FC-16 modules)
#   False -> column 0 = bit 0
MSB_IS_LEFT = True

# ============================================================
# END OF CONFIGURATION
# ============================================================

import bisect
import collections
import math
import signal
import sys
import threading
import time

import paho.mqtt.client as mqtt

# MAX7219 registers
REG_NOOP = 0x00
REG_DIGIT0 = 0x01
REG_DECODE_MODE = 0x09
REG_INTENSITY = 0x0A
REG_SCAN_LIMIT = 0x0B
REG_SHUTDOWN = 0x0C
REG_DISPLAY_TEST = 0x0F


def log(msg):
    print(msg, flush=True)


# ------------------------------------------------------------
# Screen buffer (logical)
# ------------------------------------------------------------

screen = [[0] * WIDTH for _ in range(HEIGHT)]
screen_lock = threading.Lock()


class AdaptiveScale:
    """Rolling window of recent payload lengths; LEDs by percentile rank."""

    def __init__(self, window):
        self.window = collections.deque()
        self.sorted = []
        self.size = window
        self.seen = 0

    def add(self, length):
        self.window.append(length)
        bisect.insort(self.sorted, length)
        if len(self.window) > self.size:
            old = self.window.popleft()
            del self.sorted[bisect.bisect_left(self.sorted, old)]
        self.seen += 1

    def ready(self):
        return len(self.sorted) >= ADAPTIVE_MIN_SAMPLES

    def leds(self, length, limit):
        n = len(self.sorted)
        below = bisect.bisect_left(self.sorted, length)
        upto = bisect.bisect_right(self.sorted, length)
        rank = (below + upto) / 2.0 / n          # mid-rank, 0..1
        return max(1, min(limit, math.ceil(rank * limit)))


adaptive = AdaptiveScale(ADAPTIVE_WINDOW)


def leds_for_length(length, learn=False):
    """learn=True: count this message in adaptive statistics (real messages only)."""
    limit = min(MAX_LEDS_PER_ROW, WIDTH)
    if length <= 0:
        return 0
    if SCALE == "adaptive":
        if learn:
            adaptive.add(length)
        if adaptive.ready():
            return adaptive.leds(length, limit)
        # not enough data yet -> log scale
    if SCALE in ("log", "adaptive"):
        # integer thresholds, no float log() rounding issues
        leds = 1
        threshold = LOG_MIN_CHARS * LOG_BASE
        while leds < limit and length >= threshold:
            leds += 1
            threshold *= LOG_BASE
        return leds
    return min(limit, math.ceil(length / CHARS_PER_LED))


def scale_description():
    """Human-readable table 'LEDs: length range' for the startup log."""
    limit = min(MAX_LEDS_PER_ROW, WIDTH)

    def first_length_with(k):
        # leds_for_length is monotonic -> exponential + binary search
        hi = 1
        while leds_for_length(hi) < k:
            hi *= 2
        lo = hi // 2 + 1 if hi > 1 else 1
        while lo < hi:
            mid = (lo + hi) // 2
            if leds_for_length(mid) >= k:
                hi = mid
            else:
                lo = mid + 1
        return lo

    starts = [first_length_with(k) for k in range(1, limit + 1)]
    parts = []
    for k in range(1, limit + 1):
        lo = starts[k - 1]
        if k == limit:
            parts.append("%d LED: %d+" % (k, lo))
        else:
            hi = starts[k] - 1
            if hi < lo:
                parts.append("%d LED: -" % k)   # skipped (many equal lengths)
            else:
                parts.append("%d LED: %s" % (k, lo if hi == lo else "%d-%d" % (lo, hi)))
    name = SCALE
    if SCALE == "adaptive":
        name = ("adaptive, %d msgs in window" % len(adaptive.sorted)
                if adaptive.ready() else
                "adaptive (warming up %d/%d, using log)"
                % (len(adaptive.sorted), ADAPTIVE_MIN_SAMPLES))
    return "SCALE=%s | %s" % (name, " | ".join(parts))


def push_row(leds):
    """Scroll everything up by one row and add a new row at the bottom."""
    row = [1 if c < leds else 0 for c in range(WIDTH)]
    with screen_lock:
        screen.pop(0)
        screen.append(row)


# ------------------------------------------------------------
# MAX7219 output
# ------------------------------------------------------------

def _rotate_block(block, degrees):
    """Rotate an 8x8 block (list of rows) clockwise."""
    turns = (degrees // 90) % 4
    for _ in range(turns):
        block = [list(r) for r in zip(*block[::-1])]
    return block


class Max7219:
    def __init__(self):
        import spidev
        self.spi = spidev.SpiDev()
        self.spi.open(SPI_BUS, SPI_DEVICE)
        self.spi.max_speed_hz = SPI_SPEED
        self.spi.mode = 0
        self.init_chips()

    def _write_all(self, register, values):
        """values[m] goes to module m (module 0 = nearest to DIN)."""
        data = []
        # First bytes shifted in end up in the farthest module.
        for m in reversed(range(NUM_MODULES)):
            data += [register, values[m] & 0xFF]
        self.spi.xfer2(data)

    def _write_same(self, register, value):
        self._write_all(register, [value] * NUM_MODULES)

    def init_chips(self):
        self._write_same(REG_DISPLAY_TEST, 0)
        self._write_same(REG_SCAN_LIMIT, 7)
        self._write_same(REG_DECODE_MODE, 0)
        self._write_same(REG_INTENSITY, max(0, min(15, BRIGHTNESS)))
        self.clear()
        self._write_same(REG_SHUTDOWN, 1)

    def clear(self):
        for d in range(8):
            self._write_same(REG_DIGIT0 + d, 0)

    def set_brightness(self, value):
        self._write_same(REG_INTENSITY, max(0, min(15, int(value))))

    def show(self, digits):
        """digits[m][d] = byte for module m, digit register d."""
        for d in range(8):
            self._write_all(REG_DIGIT0 + d, [digits[m][d] for m in range(NUM_MODULES)])

    def close(self):
        try:
            self.clear()
            self._write_same(REG_SHUTDOWN, 0)
        finally:
            self.spi.close()


def screen_to_modules(buf):
    """Convert logical screen[HEIGHT][WIDTH] into per-module digit bytes."""
    if SCREEN_FLIP_X:
        buf = [row[::-1] for row in buf]
    if SCREEN_FLIP_Y:
        buf = buf[::-1]
    modules = []
    for m in range(NUM_MODULES):
        block_index = m if MODULE_TOP_IS_FIRST else NUM_MODULES - 1 - m
        block = [list(buf[block_index * 8 + r][:8]) for r in range(8)]
        block = _rotate_block(block, MODULE_ROTATION_OVERRIDE.get(m, MODULE_ROTATION))
        if MODULE_FLIP_X:
            block = [r[::-1] for r in block]
        digits = []
        for r in range(8):
            byte = 0
            for c in range(8):
                if block[r][c]:
                    byte |= (0x80 >> c) if MSB_IS_LEFT else (1 << c)
            digits.append(byte)
        modules.append(digits)
    return modules


class ConsoleDisplay:
    """Stand-in for MAX7219 when run with --console."""

    quiet = False  # suppress frames while blinking (20 frames/s is unreadable)

    def show_screen(self, buf):
        if self.quiet:
            return
        print("\n".join("".join("●" if v else "." for v in row) for row in buf))
        print("-" * WIDTH, flush=True)

    def set_brightness(self, value):
        pass

    def close(self):
        pass


display = None
display_lock = threading.RLock()   # MQTT thread and blink thread share SPI


def checkerboard(phase):
    """Checkerboard pattern; phase 0/1 swaps lit and dark cells."""
    return [[(r + c + phase) % 2 for c in range(WIDTH)] for r in range(HEIGHT)]


def _render(buf):
    """Send a logical buffer to the hardware."""
    try:
        with display_lock:
            if isinstance(display, ConsoleDisplay):
                display.show_screen(buf)
            else:
                display.show(screen_to_modules(buf))
    except Exception as e:  # SPI glitch must not kill the MQTT loop
        log("DISPLAY | error: %s" % e)


def update_display(override=None):
    """Only converts the buffer and sends it to the hardware.
    override: show this buffer instead of the log screen (screen is kept).
    While blinking, does nothing: the blink thread redraws when it ends."""
    if blink_active():
        return
    if override is not None:
        snapshot = override
    else:
        with screen_lock:
            snapshot = [row[:] for row in screen]
    _render(snapshot)


# ------------------------------------------------------------
# Blink (command topic)
# ------------------------------------------------------------

blink_lock = threading.Lock()
blink_until = 0.0
blink_thread = None


def blink_active():
    with blink_lock:
        return blink_thread is not None


def parse_blink_payload(text):
    """Return seconds to blink (0 = stop), or None if payload is not understood."""
    t = text.strip().lower()
    if t in ("true", "on", "yes"):
        return float(BLINK_DEFAULT_SECONDS)
    if t in ("false", "off", "no", "stop"):
        return 0.0
    try:
        value = float(t.replace(",", "."))
    except ValueError:
        return None
    if not math.isfinite(value):
        return None
    return max(0.0, min(float(BLINK_MAX_SECONDS), value))


def start_blink(seconds):
    """Start blinking, or extend/shorten a running blink; 0 stops it."""
    global blink_until, blink_thread
    with blink_lock:
        blink_until = time.monotonic() + seconds
        if seconds <= 0 or blink_thread is not None:
            return  # running worker picks up the new deadline
        blink_thread = threading.Thread(target=_blink_worker, daemon=True)
        blink_thread.start()


def _blink_worker():
    global blink_thread
    full = [[1] * WIDTH for _ in range(HEIGHT)]
    dark = [[0] * WIDTH for _ in range(HEIGHT)]
    half_period = 1.0 / (2.0 * BLINK_HZ)
    on = True
    try:
        while True:
            with blink_lock:
                if time.monotonic() >= blink_until:
                    # atomic with start_blink(): a new command after this
                    # point starts a fresh worker instead of being lost
                    blink_thread = None
                    break
            if on:
                if BLINK_MODE == "content":
                    with screen_lock:
                        frame = [row[:] for row in screen]
                else:
                    frame = full
            else:
                frame = dark
            with display_lock:
                if isinstance(display, ConsoleDisplay):
                    display.quiet = True
                display.set_brightness(15)
                _render(frame)
            on = not on
            time.sleep(half_period)
    except Exception as e:
        log("BLINK | error: %s" % e)
        with blink_lock:
            blink_thread = None
    with display_lock:
        if blink_active():
            return  # a newer blink already took over the display
        try:
            display.set_brightness(BRIGHTNESS)
        except Exception as e:
            log("DISPLAY | error: %s" % e)
        if isinstance(display, ConsoleDisplay):
            display.quiet = False
        # back to the previous state (log, or checkerboard if auth fails)
        update_display(checkerboard(auth_fail_phase ^ 1) if auth_failed else None)
    log("BLINK | done")


# ------------------------------------------------------------
# MQTT
# ------------------------------------------------------------

# CONNACK codes meaning "wrong login/password" / "not authorized"
# (MQTT 3.1.1: 4, 5; MQTT 5: 134, 135)
AUTH_FAIL_CODES = (4, 5, 134, 135)
auth_fail_phase = 0
auth_failed = False


def on_connect(client, userdata, flags, rc, *args):
    global auth_fail_phase, auth_failed
    rc_val = getattr(rc, "value", rc)
    if rc_val == 0:
        log("MQTT | connected to %s:%s, subscribing to %s" % (MQTT_HOST, MQTT_PORT, MQTT_TOPIC))
        client.subscribe(MQTT_TOPIC)  # re-subscribe after every reconnect
        if auth_failed:
            auth_failed = False
            update_display()  # checkerboard -> back to the message log
    elif rc_val in AUTH_FAIL_CODES:
        auth_failed = True
        log("MQTT | auth failed (rc=%s): check MQTT_USERNAME / MQTT_PASSWORD" % rc)
        update_display(checkerboard(auth_fail_phase))
        auth_fail_phase ^= 1  # invert on every attempt
    else:
        log("MQTT | connect failed, rc=%s" % rc)


def on_disconnect(client, userdata, *args):
    log("MQTT | disconnected (%s), will reconnect" % (args[-1] if args else "?"))


retained_skipped = 0


def on_message(client, userdata, msg):
    global retained_skipped
    try:
        if msg.topic == BLINK_TOPIC:
            if msg.retain:
                # an old retained command must not blink on every reconnect
                log("BLINK | ignored retained command")
                return
            text = msg.payload.decode("utf-8", errors="replace")
            seconds = parse_blink_payload(text)
            if seconds is None:
                log("BLINK | unknown payload %r, expected true/false/number" % text[:40])
            else:
                log("BLINK | %s" % ("stop" if seconds == 0 else "%g s" % seconds))
                start_blink(seconds)
            return
        if msg.retain and not SHOW_RETAINED:
            retained_skipped += 1
            return
        if retained_skipped:
            log("MQTT | skipped %d retained messages" % retained_skipped)
            retained_skipped = 0
        text = msg.payload.decode("utf-8", errors="replace")
        length = len(text)
        leds = leds_for_length(length, learn=True)
        push_row(leds)
        update_display()
        log("MQTT | topic=%s | length=%d | LEDs=%d" % (msg.topic, length, leds))
        if (SCALE == "adaptive" and ADAPTIVE_LOG_EVERY > 0 and length > 0
                and (adaptive.seen == ADAPTIVE_MIN_SAMPLES
                     or adaptive.seen % ADAPTIVE_LOG_EVERY == 0)):
            log(scale_description())
    except Exception as e:
        log("MQTT | error handling message: %s" % e)


def make_client():
    # paho-mqtt 2.x requires an explicit callback API version; 1.x has no such
    # argument. VERSION1 keeps the same callback signatures on both.
    if hasattr(mqtt, "CallbackAPIVersion"):
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, client_id=MQTT_CLIENT_ID)
    else:
        client = mqtt.Client(client_id=MQTT_CLIENT_ID)
    if MQTT_USERNAME:
        client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message
    client.reconnect_delay_set(min_delay=MQTT_RECONNECT_MIN_DELAY,
                               max_delay=MQTT_RECONNECT_MAX_DELAY)
    return client


# ------------------------------------------------------------
# Main
# ------------------------------------------------------------

def orientation_test():
    """
    Draw a reference pattern and cycle through all mapping variants.
    Correct picture: one triangle over the whole height, pressed to the LEFT
    edge, 1 LED wide at the TOP, growing by 1 LED every 4 rows,
    8 LEDs wide at the BOTTOM.
    """
    global MODULE_ROTATION, MODULE_FLIP_X, MODULE_TOP_IS_FIRST
    with screen_lock:
        for r in range(HEIGHT):
            n = min(WIDTH, r // 4 + 1)
            screen[r] = [1 if c < n else 0 for c in range(WIDTH)]
    variants = [(rot, flip, top) for top in (True, False)
                for rot in (0, 90, 180, 270) for flip in (False, True)]
    if "--once" in sys.argv:
        variants = [(MODULE_ROTATION, MODULE_FLIP_X, MODULE_TOP_IS_FIRST)]
    while True:
        for i, (rot, flip, top) in enumerate(variants, 1):
            MODULE_ROTATION, MODULE_FLIP_X, MODULE_TOP_IS_FIRST = rot, flip, top
            log("TEST %2d/%d | MODULE_ROTATION = %d | MODULE_FLIP_X = %s | "
                "MODULE_TOP_IS_FIRST = %s" % (i, len(variants), rot, flip, top))
            update_display()
            time.sleep(6)


def main():
    global display
    display = ConsoleDisplay() if "--console" in sys.argv else Max7219()

    if "--test" in sys.argv:
        try:
            orientation_test()
        except KeyboardInterrupt:
            display.close()
        return
    update_display()

    client = make_client()

    def shutdown(signum, frame):
        log("Stopping (signal %s)" % signum)
        try:
            client.disconnect()
        finally:
            display.close()
            sys.exit(0)

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)

    log(scale_description())
    client.connect_async(MQTT_HOST, MQTT_PORT, MQTT_KEEPALIVE)
    while True:
        try:
            # retry_first_connection: keep trying if broker is down at startup
            client.loop_forever(retry_first_connection=True)
        except SystemExit:
            raise
        except Exception as e:
            log("MQTT | loop error: %s, restarting in 5 s" % e)
            time.sleep(5)


if __name__ == "__main__":
    main()
