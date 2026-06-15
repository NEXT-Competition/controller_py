#!/usr/bin/env python3

# Sequence: wait for trigger (PWM == 2000) -> spin shooter to full speed
#           -> wait for spin-up -> open gate for X sec -> close gate -> stop shooter

import os
import socket
import tempfile
import threading
from time import monotonic, sleep

# lgpio creates its notify pipe (.lgd-nfy*) in the current working directory at
# import time, so move to a writable dir first -- the package runs from /opt,
# which the service user can't write to.
os.chdir(tempfile.gettempdir())

import cv2  # noqa: E402
import lgpio  # noqa: E402
import numpy as np  # noqa: E402
from fusion_hat.servo import Servo  # noqa: E402
from picamera2 import Picamera2  # noqa: E402

# ---------------------------------------------------------------
# SETTINGS TO TUNE
# ---------------------------------------------------------------
SHOOTER_CHANNEL = 0  # PWM channel the shooter motor/ESC is on
GATE_CHANNEL = 1  # PWM channel the gate (feeder) servo is on

# All setpoints are in MICROSECONDS (how an ESC/RC servo is actually driven).
# The Fusion HAT Servo maps angle -90..+90 -> 500..2500 us linearly, so we
# convert us -> angle with us_to_angle() below.
#   1500 us = neutral / ESC arm / stop   (servo center)
#   2000 us = full throttle / one extreme
#   1000 us = other extreme
SHOOTER_FULL_US = 1650  # full throttle
SHOOTER_STOP_US = 1550  # neutral / stop / ESC arm point (bidirectional ESC)

# Gate is a 180 (positional) servo: it holds an angle. We use a 0..180 mental
# model where 0 = closed, and convert to the Fusion HAT's -90..+90 range via
# gate_deg_to_angle() below. The gate starts closed.
GATE_CLOSED_DEG = 120  # closed position
GATE_OPEN_DEG = 0  # open position

# Timing (seconds)
ARM_TIME = 3  # hold neutral at startup so the ESC initializes/arms (it beeps)
SPINUP_TIME = 3  # wait after starting motor before opening gate ( “3 sec")
GATE_OPEN_TIME = 3.0  # how long the gate stays open  ("amount of time")

# Trigger
TRIGGER_VALUE = 2000  # fire when the PWM signal reaches this (microseconds)
TRIGGER_MARGIN = 50  # allow a little jitter (fires at >= 1950)

# BCM GPIO the incoming RC/PWM signal is wired to.
# NOTE: this is GPIO25 == physical pin 22 on the 40-pin header.
PWM_PIN = 22

# FPV streaming over the point-to-point Wi-Fi dongle link.
# We use UNICAST, not multicast: on Wi-Fi, multicast/broadcast is sent at the
# lowest basic rate with no ACK/retransmit -> slow and lossy. Unicast gets rate
# adaptation + ACKs, which is far lower latency for a two-Pi link. The receiver
# announces itself with a small beacon so we know which address to unicast to.
STREAM_IFACE_IP = "10.42.0.1"  # this Pi's USB-dongle/hotspot IP (egress interface)
STREAM_PORT = 5005  # shooter -> receiver: video
DISCOVERY_PORT = 5006  # receiver -> shooter: "I'm here" beacons
WIDTH = 640
HEIGHT = 480

# ---------------------------------------------------------------
# HARDWARE OBJECTS
# ---------------------------------------------------------------
shooter = Servo(SHOOTER_CHANNEL)
gate = Servo(GATE_CHANNEL)


def us_to_angle(microseconds):
    # Fusion HAT Servo: angle -90..+90 maps to 500..2500 us, so
    # 1500 us -> 0 deg, 2000 us -> +45 deg, 1000 us -> -45 deg.
    return (microseconds - 1500) * 0.09


def gate_deg_to_angle(deg):
    # Gate uses a 0..180 model (0 = closed); the Fusion HAT Servo wants -90..+90.
    # 0 deg -> -90, 90 deg -> 0, 120 deg -> +30, 180 deg -> +90.
    return deg - 90


# ---------------------------------------------------------------
# INCOMING PWM MEASUREMENT (lgpio on the Fusion HAT digital pin)
# ---------------------------------------------------------------
# Measure the width (in microseconds) of the incoming pulse on PWM_PIN
# (BCM GPIO22, a Fusion HAT digital pin). lgpio's alert callback hands us
# the exact edge level (0/1) and a hardware nanosecond timestamp captured
# AT the edge -- so there's no level-race and no debounce needed, unlike
# the RPi.GPIO callback which only gives the channel number.


def _open_gpiochip():
    # gpiochip0 is the 40-pin header on Pi 4; fall back to 4 just in case.
    for chip in (0, 4):
        try:
            return lgpio.gpiochip_open(chip)
        except lgpio.error:
            continue
    raise RuntimeError("Could not open a GPIO chip (tried 0 and 4)")


_h = _open_gpiochip()
lgpio.gpio_claim_alert(_h, PWM_PIN, lgpio.BOTH_EDGES)

_start_tick = None
_pulse_width = 0  # last measured pulse width in microseconds
_last_pulse_time = 0.0  # monotonic() time of the last completed pulse


def _pwm_callback(chip, gpio, level, tick):
    global _start_tick, _pulse_width, _last_pulse_time

    if level == 1:  # Rising edge
        _start_tick = tick

    elif level == 0 and _start_tick is not None:  # Falling edge
        _pulse_width = (tick - _start_tick) / 1000.0  # ns -> us
        _last_pulse_time = monotonic()


_cb = lgpio.callback(_h, PWM_PIN, lgpio.BOTH_EDGES, _pwm_callback)

# ---------------------------------------------------------------
# FPV STREAMING (Pi Camera ball tracking, blasted over UDP)
# ---------------------------------------------------------------
# Video-out socket: bind to the dongle IP so frames egress that interface.
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
try:
    sock.bind((STREAM_IFACE_IP, 0))
except OSError as e:
    print(f"FPV: could not bind to {STREAM_IFACE_IP} ({e}); using default route")

# Discovery-in socket: learn the receiver's address from its beacons.
disco = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
disco.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
try:
    disco.bind((STREAM_IFACE_IP, DISCOVERY_PORT))
except OSError:
    disco.bind(("", DISCOVERY_PORT))
disco.setblocking(False)

_peer = None  # (ip, port) of the receiver, learned from its beacons


def _update_peer():
    # Drain any beacons; keep the most recent sender as the stream target.
    global _peer
    try:
        while True:
            _data, addr = disco.recvfrom(64)
            _peer = (addr[0], STREAM_PORT)
    except BlockingIOError:
        pass

picam2 = Picamera2()
picam2.configure(
    picam2.create_preview_configuration(
        main={"size": (WIDTH, HEIGHT), "format": "RGB888"}
    )
)
picam2.start()
sleep(1)

_streaming = True


def stream_fpv():
    # Capture frames, track a green ball, draw overlays and blast the
    # processed frame over UDP. Runs in a background thread so the fire
    # sequence (which blocks on sleeps) doesn't freeze the video.
    while _streaming:
        frame = picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # ---- Ball tracking ----
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = np.array([65, 40, 80])
        upper = np.array([110, 255, 255])
        mask = cv2.inRange(hsv, lower, upper)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_ball, best_area = None, 0

        for c in contours:
            area = cv2.contourArea(c)
            if area < 2000:
                continue
            perimeter = cv2.arcLength(c, True)
            if perimeter == 0:
                continue
            circularity = 4 * np.pi * area / (perimeter * perimeter)
            if circularity < 0.7:
                continue
            if area > best_area:
                best_ball = c
                best_area = area

        if best_ball is not None:
            (x, y), radius = cv2.minEnclosingCircle(best_ball)
            if radius > 15:
                center = (int(x), int(y))
                radius = int(radius)
                cv2.circle(frame, center, radius, (0, 255, 0), 3)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
                cv2.putText(
                    frame,
                    f"Ball x={center[0]} y={center[1]}",
                    (max(center[0] - 120, 0), max(center[1] - 25, 25)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                )
        # -----------------------

        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]
        ret, jpeg = cv2.imencode(".jpg", frame, encode_param)

        # Unicast the frame to whoever last beaconed us.
        _update_peer()
        if ret and _peer is not None:
            data = jpeg.tobytes()
            if len(data) <= 65507:
                sock.sendto(data, _peer)


_stream_thread = threading.Thread(target=stream_fpv, daemon=True)


# ---------------------------------------------------------------
# Return the current incoming PWM value (pulse width in microseconds).
# ---------------------------------------------------------------
def read_trigger_pwm():
    return _pulse_width


SIGNAL_TIMEOUT = 0.1  # seconds; ignore the latched value if no recent pulse


def is_triggered():
    # The pulse width is latched from the last completed pulse. If pulses have
    # stopped (signal lost), that value is stale -- don't treat it as a trigger.
    if monotonic() - _last_pulse_time > SIGNAL_TIMEOUT:
        return False
    return read_trigger_pwm() >= (TRIGGER_VALUE - TRIGGER_MARGIN)


# ---------------------------------------------------------------
# THE FIRE SEQUENCE
# ---------------------------------------------------------------
def fire():
    print("Trigger detected -> starting shooter")
    shooter.angle(us_to_angle(SHOOTER_FULL_US))  # motor ON at full speed
    sleep(SPINUP_TIME)  # let it spin up ("2-3 sec after motor starting")

    print("Opening gate")
    gate.angle(gate_deg_to_angle(GATE_OPEN_DEG))  # move to open position
    sleep(GATE_OPEN_TIME)  # keep open for X seconds

    print("Closing gate")
    gate.angle(gate_deg_to_angle(GATE_CLOSED_DEG))  # move back to closed

    print("Stopping shooter")
    shooter.angle(us_to_angle(SHOOTER_STOP_US))  # motor OFF


def cleanup():
    global _streaming
    gate.angle(gate_deg_to_angle(GATE_CLOSED_DEG))  # return gate to closed
    shooter.angle(us_to_angle(SHOOTER_STOP_US))  # stop the motor on exit (safety)
    sleep(0.1)
    _streaming = False
    if _stream_thread.is_alive():
        _stream_thread.join(timeout=2)
    _cb.cancel()
    lgpio.gpiochip_close(_h)
    picam2.stop()
    sock.close()
    disco.close()


# ---------------------------------------------------------------
# MAIN LOOP
# ---------------------------------------------------------------
def main():
    # Arm the ESC: hold neutral (1500 us) at startup so it initializes.
    # ESCs refuse to arm if they see high throttle at power-on, so this MUST
    # be the neutral pulse, held for a couple seconds (the ESC usually beeps).
    print("Arming ESC: holding neutral (%d us)..." % SHOOTER_STOP_US)
    shooter.angle(us_to_angle(SHOOTER_STOP_US))
    gate.angle(gate_deg_to_angle(GATE_CLOSED_DEG))  # start with the gate closed
    sleep(ARM_TIME)
    print("ESC armed.")

    # start FPV streaming in the background
    _stream_thread.start()
    print("FPV streaming started.")
    print("Ready. Waiting for trigger (PWM == %d)..." % TRIGGER_VALUE)

    # Edge-triggered: fire once on each low->high transition. This avoids
    # re-firing while the trigger is held, and -- because we re-sample the
    # CURRENT state after fire() returns -- it won't fire again on the stale
    # value latched during the long fire() sequence.
    was_triggered = False
    while True:
        triggered = is_triggered()
        if triggered and not was_triggered:
            fire()
            print("Ready. Waiting for trigger...")
        was_triggered = triggered
        sleep(0.02)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopping safely.")
    finally:
        cleanup()
