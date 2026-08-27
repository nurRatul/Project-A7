"""
neom8n.py

Driver + background tracker for u-blox NEO-M8N GPS modules (this covers
the common "Picksok" NEO-M8N breakout, and should work unmodified on any
NEO-6M/7M/8M module too — they all speak the same NMEA 0183 sentences by
default over UART).

Wiring (module -> Pi):
    VCC -> 3.3V or 5V (check your specific board's regulator before wiring)
    GND -> GND
    TX  -> Pi RXD  (GPIO15 / physical pin 10)
    RX  -> Pi TXD  (GPIO14 / physical pin 8)

Before use on the Pi:
    sudo raspi-config      # Interface Options -> Serial Port ->
                            #   "login shell over serial"    = No
                            #   "serial port hardware enabled" = Yes
                            # then reboot
    pip install pyserial pynmea2
    # sanity check the module is talking before trusting this driver:
    #   cat /dev/serial0        (NMEA text like $GPGGA/$GPRMC should scroll by)

------------------------------------------------------------------
Honest limits of what this can tell you — read before trusting output
------------------------------------------------------------------
* Time-to-first-fix is not instant. A cold start (module just powered,
  no saved almanac) commonly takes 25-30+ seconds outdoors under open
  sky, and can take much longer indoors or near tall buildings. Don't
  assume read()/read_once() will return quickly the first time you run it.
* A civilian NEO-M8N is typically accurate to a few meters under open
  sky, worse near buildings/trees/indoors ("multipath"). Use hdop (below
  ~2 is good, above ~5 is poor) and satellites alongside has_fix to
  judge whether a reading is trustworthy enough to act on.
* has_fix only means the module believes it has a position — not that
  the position is accurate. Treat a fix right after has_fix first flips
  True, or with very few satellites, with suspicion.
* speed/course are the module's own Doppler-based estimate. At low or
  zero speed they get noisy — don't expect a stationary bot to report a
  stable course.
"""

import time
from datetime import datetime
from threading import Lock, Thread

import pynmea2
import serial

from logger.logger_manager import LoggerManager

logger = LoggerManager.get_logger("car_controller.neom8n")


def _to_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _to_int(value):
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _empty_fix():
    return {
        "latitude": None,
        "longitude": None,
        "altitude_m": None,
        "speed_kmh": None,
        "course_deg": None,
        "satellites": None,
        "fix_quality": None,   # 0=no fix, 1=GPS, 2=DGPS, 4=RTK fixed, 5=RTK float, 6=dead reckoning
        "fix_type": None,      # 1=no fix, 2=2D, 3=3D (from GSA)
        "hdop": None,
        "pdop": None,
        "vdop": None,
        "has_fix": False,
        "fix_time_utc": None,  # UTC datetime, from RMC's date + time
        "last_update": None,   # time.time() this dict was last touched
    }


class NEOM8NGPS:
    """Low-level serial access + NMEA parsing for a NEO-M8N GPS module."""

    def __init__(self, port="/dev/ttyAMA0", baudrate=9600, timeout=1.0):
        self._serial = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        self._serial.reset_input_buffer()

    def read_raw(self):
        """
        Read and return the next raw NMEA line from the module, completely
        unparsed (e.g. "$GPGGA,123519,4807.038,N,...*47"). Returns "" if
        nothing arrived within the serial timeout — that's normal, not an
        error, just call it again.
        """
        line = self._serial.readline()
        if not line:
            return ""
        return line.decode("ascii", errors="replace").strip()

    @staticmethod
    def parse_sentence(line):
        """
        Parse one raw NMEA 0183 line into a dict covering whatever that
        sentence type provides (a *partial* fix — merge several together
        for the full picture, which is what read() and GPSTracker do).
        Returns None for blank/corrupted lines (bad checksum) or sentence
        types this driver doesn't use (GSV, GLL, ...).
        """
        if not line or not line.startswith("$"):
            return None
        try:
            msg = pynmea2.parse(line)
        except pynmea2.ParseError:
            return None

        data = {}

        if isinstance(msg, pynmea2.types.talker.GGA):
            quality = msg.gps_qual or 0
            data["fix_quality"] = quality
            data["satellites"] = _to_int(msg.num_sats)
            hdop = _to_float(msg.horizontal_dil)
            if hdop is not None:
                data["hdop"] = hdop
            data["has_fix"] = quality > 0
            if quality > 0:
                data["latitude"] = msg.latitude
                data["longitude"] = msg.longitude
                data["altitude_m"] = msg.altitude

        elif isinstance(msg, pynmea2.types.talker.RMC):
            active = msg.status == "A"
            data["has_fix"] = active
            if active:
                data["latitude"] = msg.latitude
                data["longitude"] = msg.longitude
                if msg.spd_over_grnd is not None:
                    data["speed_kmh"] = float(msg.spd_over_grnd) * 1.852
                if msg.true_course is not None:
                    data["course_deg"] = float(msg.true_course)
            if msg.datestamp and msg.timestamp:
                data["fix_time_utc"] = datetime.combine(msg.datestamp, msg.timestamp)

        elif isinstance(msg, pynmea2.types.talker.GSA):
            fix_type = _to_int(msg.mode_fix_type)
            if fix_type is not None:
                data["fix_type"] = fix_type
            pdop = _to_float(msg.pdop)
            vdop = _to_float(msg.vdop)
            hdop = _to_float(msg.hdop)
            if pdop is not None:
                data["pdop"] = pdop
            if vdop is not None:
                data["vdop"] = vdop
            if hdop is not None:
                data["hdop"] = hdop

        elif isinstance(msg, pynmea2.types.talker.VTG):
            if msg.spd_over_grnd_kmph is not None:
                data["speed_kmh"] = float(msg.spd_over_grnd_kmph)
            if msg.true_track is not None:
                data["course_deg"] = float(msg.true_track)

        else:
            return None

        return data or None

    def read(self, timeout=None, raw=False):
        """
        Block until a usable position fix has been read, merging in
        whatever else (satellites, hdop, speed, course...) showed up
        along the way. Returns a *fresh* fix dict every call — nothing is
        remembered from a previous read() call.

        timeout: seconds to wait before giving up (raises TimeoutError).
                 None waits forever.
        raw:     if True, adds fix["raw"] = list of raw NMEA lines that
                 were read while building this fix.
        """
        fix = _empty_fix()
        raw_lines = []
        deadline = None if timeout is None else time.monotonic() + timeout

        while True:
            if deadline is not None and time.monotonic() > deadline:
                raise TimeoutError(
                    f"No GPS fix after {timeout:.1f}s — check wiring/baud "
                    "rate, and that the antenna has a clear view of the sky"
                )
            line = self.read_raw()
            if not line:
                continue
            if raw:
                raw_lines.append(line)
            parsed = self.parse_sentence(line)
            if parsed:
                fix.update(parsed)
                fix["last_update"] = time.time()
                if fix["has_fix"] and fix["latitude"] is not None:
                    break

        if raw:
            fix["raw"] = raw_lines
        return fix

    def close(self):
        self._serial.close()


class GPSTracker:
    """
    Background daemon thread that continuously drains the serial port and
    keeps a thread-safe snapshot of the latest GPS fix. Use this when your
    main program needs live GPS data without blocking on serial reads
    itself (e.g. inside a control loop) — start it once, read snapshot()
    as often as you like, stop it when you're done.
    """

    def __init__(self, gps):
        self.gps = gps
        self._lock = Lock()
        self._thread = None
        self._running = False
        self._fix = _empty_fix()
        self._last_raw = None

    @property
    def is_running(self):
        return self._running

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        self._thread = None

    def _loop(self):
        while self._running:
            try:
                self._update()
            except Exception:
                logger.exception("GPS read/update failed")
                time.sleep(0.1)

    def _update(self):
        line = self.gps.read_raw()
        if not line:
            return
        parsed = self.gps.parse_sentence(line)
        with self._lock:
            self._last_raw = line
            if parsed:
                self._fix.update(parsed)
                self._fix["last_update"] = time.time()

    def snapshot(self, raw=False):
        """Thread-safe copy of the latest known fix."""
        with self._lock:
            data = dict(self._fix)
            if raw:
                data["raw"] = self._last_raw
            return data