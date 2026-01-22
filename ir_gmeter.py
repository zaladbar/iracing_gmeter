#!/usr/bin/env python3
# iRacing G‑meter (G‑ball) overlay
# Dependencies: pyirsdk, PyQt5
# MIT License

import math
import sys
import time
import json
from pathlib import Path
from collections import deque

# --- Telemetry ---
try:
    import irsdk
except Exception as e:
    print("pyirsdk is required. Install with: pip install pyirsdk")
    raise

# --- UI ---
from PyQt5 import QtCore, QtGui, QtWidgets

G0 = 9.80665  # m/s^2

def ema(prev, new, alpha):
    if prev is None:
        return new
    return alpha * new + (1 - alpha) * prev

class IRacingReader(QtCore.QObject):
    """Thin wrapper around pyirsdk for safe periodic reads."""
    telemetry = QtCore.pyqtSignal(float, float, float, float)  # long[m/s2], lat[m/s2], pitch[rad], roll[rad]

    def __init__(self, parent=None):
        super().__init__(parent)
        self.ir = irsdk.IRSDK()
        self.connected = False
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(16)  # ~60 Hz UI tick; pyirsdk blocks for a new frame on freeze_var_buffer_latest()

    def _ensure_connected(self):
        if not self.connected:
            try:
                self.ir.startup()
                self.connected = self.ir.is_initialized and self.ir.is_connected
            except Exception:
                self.connected = False

    def _tick(self):
        try:
            self._ensure_connected()
            if not (self.connected and self.ir.is_connected):
                self.telemetry.emit(float('nan'), float('nan'), 0.0, 0.0)
                return

            # Get the latest consistent frame
            self.ir.freeze_var_buffer_latest()

            # Prefer high‑rate _ST variables if present; fall back otherwise
            def getvar(name, fallback=None):
                if name in self.ir:
                    return float(self.ir[name])
                if fallback and fallback in self.ir:
                    return float(self.ir[fallback])
                return float('nan')

            long_ms2 = getvar('LongAccel_ST', 'LongAccel')
            lat_ms2  = getvar('LatAccel_ST',  'LatAccel')
            pitch    = getvar('Pitch')  # rad
            roll     = getvar('Roll')   # rad

            self.telemetry.emit(long_ms2, lat_ms2, pitch, roll)
        except Exception:
            # If anything goes wrong (e.g., sim closed), mark as disconnected and keep trying
            try:
                self.ir.shutdown()
            except Exception:
                pass
            self.connected = False
            self.telemetry.emit(float('nan'), float('nan'), 0.0, 0.0)

class GMeterOverlay(QtWidgets.QWidget):
    def __init__(self, config: dict):
        super().__init__()
        # Window flags: frameless, always on top, transparent background
        self.setWindowFlags(
            QtCore.Qt.FramelessWindowHint |
            QtCore.Qt.WindowStaysOnTopHint |
            QtCore.Qt.Tool
        )
        self.setAttribute(QtCore.Qt.WA_TranslucentBackground, bool(config.get('transparent', False)))
        self.setFixedSize(int(config.get('size', 280)), int(config.get('size', 280)))
        self.setFocusPolicy(QtCore.Qt.StrongFocus)

        # State (from config)
        self.g_scale = float(config.get('g_scale', 2.0))     # outer ring = ±g_scale
        self.alpha = float(max(0.05, min(0.95, config.get('alpha', 0.25))))  # EMA smoothing
        self.use_comp = bool(config.get('use_comp', False))  # gravity compensation
        self.show_trail = bool(config.get('show_trail', True))
        self.show_help = bool(config.get('show_help', False))
        self.enable_hotkeys = bool(config.get('enable_hotkeys', False))
        self.bg_color = tuple(config.get('bg_rgba', [0, 0, 0, 200]))

        self._s_long_g = None
        self._s_lat_g = None
        self._last_pos = None
        self._trail = deque(maxlen=120)  # ~2s tail at 60Hz
        self._drag_origin = None

        # Telemetry reader
        self.reader = IRacingReader()
        self.reader.telemetry.connect(self._on_telemetry)

        # UI refresh
        self._ui_timer = QtCore.QTimer(self)
        self._ui_timer.timeout.connect(self.update)
        self._ui_timer.start(16)

    # ----- Input -----
    def mousePressEvent(self, e):
        if e.button() == QtCore.Qt.LeftButton:
            self._drag_origin = (e.globalPos(), self.frameGeometry().topLeft())
        elif e.button() == QtCore.Qt.RightButton:
            QtWidgets.QApplication.quit()

    def mouseMoveEvent(self, e):
        if self._drag_origin:
            gpos, top_left = self._drag_origin
            delta = e.globalPos() - gpos
            self.move(top_left + delta)

    def mouseReleaseEvent(self, e):
        self._drag_origin = None

    def keyPressEvent(self, e):
        if not self.enable_hotkeys:
            return
        k = e.key()
        if k == QtCore.Qt.Key_G:
            self.use_comp = not self.use_comp
        elif k == QtCore.Qt.Key_T:
            self.show_trail = not self.show_trail
        elif k == QtCore.Qt.Key_H:
            self.show_help = not self.show_help
        elif k in (QtCore.Qt.Key_Plus, QtCore.Qt.Key_Equal):
            self.g_scale = min(5.0, self.g_scale + 0.25)
        elif k in (QtCore.Qt.Key_Minus, QtCore.Qt.Key_Underscore):
            self.g_scale = max(0.5, self.g_scale - 0.25)
        elif k == QtCore.Qt.Key_S:
            self.alpha = min(0.95, self.alpha + 0.05)
        elif k == QtCore.Qt.Key_A:
            self.alpha = max(0.05, self.alpha - 0.05)
        self.update()

    # ----- Telemetry handling -----
    def _on_telemetry(self, long_ms2, lat_ms2, pitch, roll):
        if math.isnan(long_ms2) or math.isnan(lat_ms2):
            # No sim / no data
            self._s_long_g = None
            self._s_lat_g = None
            self._last_pos = None
            return

        # Optional gravity compensation.
        # iRacing’s LatAccel/LongAccel include gravity (docs & community lists).
        # Approximate removal of gravity component using body‑frame pitch/roll.
        # g_body = [-g*sin(pitch), -g*cos(pitch)*sin(roll), -g*cos(pitch)*cos(roll)]
        if self.use_comp:
            long_ms2 = long_ms2 + G0 * math.sin(pitch)
            lat_ms2  = lat_ms2  + G0 * math.cos(pitch) * math.sin(roll)

        # Convert to g
        long_g = long_ms2 / G0   # + forward, - braking
        lat_g  = lat_ms2  / G0   # + left, - right

        # Smooth
        self._s_long_g = ema(self._s_long_g, long_g, self.alpha)
        self._s_lat_g  = ema(self._s_lat_g,  lat_g,  self.alpha)

        # Store trail point in normalized coordinates (screen will map them)
        if self._s_long_g is not None and self._s_lat_g is not None:
            self._last_pos = (self._s_lat_g, self._s_long_g)  # (x=lat, y=long)
            self._trail.append(self._last_pos)

    # ----- Drawing -----
    def paintEvent(self, _):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)

        # Background
        w, h = self.width(), self.height()
        center = QtCore.QPointF(w/2, h/2)
        radius = min(w, h) * 0.45
        if not self.testAttribute(QtCore.Qt.WA_TranslucentBackground):
            r, g, b, a = self.bg_color
            painter.fillRect(self.rect(), QtGui.QColor(r, g, b, a))

        # Outer halo
        pen_outer = QtGui.QPen(QtGui.QColor(255, 255, 255, 180), 2)
        painter.setPen(pen_outer)
        painter.drawEllipse(center, radius, radius)

        # Grid rings at 0.5g increments
        painter.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255, 90), 1))
        tick = 0.5
        r_step = radius * (tick / self.g_scale)
        g_ticks = int(self.g_scale / tick)
        for i in range(1, g_ticks):
            rr = r_step * i
            painter.drawEllipse(center, rr, rr)

        # Crosshairs (use QPointF to avoid float->int overload issues)
        painter.drawLine(
            QtCore.QPointF(center.x() - radius, center.y()),
            QtCore.QPointF(center.x() + radius, center.y())
        )  # lateral
        painter.drawLine(
            QtCore.QPointF(center.x(), center.y() - radius),
            QtCore.QPointF(center.x(), center.y() + radius)
        )  # longitudinal

        # Labels
        font = QtGui.QFont("Segoe UI", 9)
        painter.setFont(font)
        label = f"±{self.g_scale:.2f} g   α={self.alpha:.2f}   comp={'on' if self.use_comp else 'off'}"
        painter.setPen(QtGui.QColor(255, 255, 255, 200))
        painter.drawText(10, int(h - 10), label)

        # Help
        if self.show_help:
            if self.enable_hotkeys:
                help_lines = [
                    "G = grav. comp, T = trail, +/- = scale, S/A = smoothing, H = help, Right‑click = quit",
                    "Drag to move. Dot = (Lat g, Long g). Up = accel, Down = brake, Left = left‑G.",
                ]
            else:
                help_lines = [
                    "Drag to move. Right‑click = quit.",
                    "Edit gmeter_config.json to change scale/smoothing/comp/trail.",
                ]
            y = 20
            for line in help_lines:
                painter.drawText(10, y, line)
                y += 16

        # Dot and trail
        if self._last_pos is not None:
            # Map from (lat_g, long_g) to pixels; positive lat -> left, positive long -> up
            lat_g, long_g = self._last_pos
            x = center.x() - (lat_g / self.g_scale) * radius
            y = center.y() - (long_g / self.g_scale) * radius

            # Trail
            if self.show_trail and len(self._trail) > 1:
                path = QtGui.QPainterPath()
                for i, (lat_i, long_i) in enumerate(self._trail):
                    xi = center.x() - (lat_i / self.g_scale) * radius
                    yi = center.y() - (long_i / self.g_scale) * radius
                    if i == 0:
                        path.moveTo(xi, yi)
                    else:
                        path.lineTo(xi, yi)
                painter.setPen(QtGui.QPen(QtGui.QColor(0, 200, 255, 120), 2))
                painter.drawPath(path)

            # Dot
            painter.setBrush(QtGui.QColor(0, 200, 255, 220))
            painter.setPen(QtGui.QPen(QtGui.QColor(0, 0, 0, 120), 1))
            painter.drawEllipse(QtCore.QPointF(x, y), 6, 6)

            # Numeric readout
            painter.setPen(QtGui.QColor(255, 255, 255, 220))
            painter.drawText(int(10), int(20),
                             f"Long: {long_g:.2f} g   Lat: {lat_g:.2f} g")
        else:
            # No telemetry yet: show a hint
            painter.setPen(QtGui.QColor(255, 255, 255, 160))
            painter.drawText(int(10), int(20), "Waiting for iRacing telemetry…")

def _default_config() -> dict:
    return {
        "g_scale": 2.0,
        "alpha": 0.25,
        "use_comp": False,
        "show_trail": True,
        "show_help": False,
        "enable_hotkeys": False,
        "transparent": False,  # set True for glassy background; False avoids black box on some GPUs
        "bg_rgba": [16, 16, 20, 200],
        "size": 280,
    }


def _load_config(path: Path) -> dict:
    if not path.exists():
        cfg = _default_config()
        try:
            path.write_text(json.dumps(cfg, indent=2))
        except Exception:
            pass
        return cfg
    try:
        return json.loads(path.read_text())
    except Exception:
        return _default_config()


def main():
    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName("iRacing G‑meter")
    cfg_path = Path(__file__).with_name('gmeter_config.json')
    cfg = _load_config(cfg_path)
    w = GMeterOverlay(cfg)
    # Center on primary screen
    screen = app.primaryScreen().availableGeometry()
    w.move(int(screen.center().x() - w.width()/2), int(screen.center().y() - w.height()/2))
    w.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
