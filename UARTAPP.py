
import sys
import time
import csv
import struct
from collections import deque

import serial
import serial.tools.list_ports
import numpy as np

from PySide6.QtCore import QThread, Signal, Slot, QTimer, Qt
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QComboBox, QSpinBox, QDoubleSpinBox,
    QTabWidget, QFormLayout, QFileDialog, QMessageBox, QGroupBox,
    QTextEdit, QCheckBox, QLineEdit
)
import pyqtgraph as pg


# =========================
# MSP430 GUI binary protocol
# =========================
FRAME_HEAD = b"\xAA\x55"

CMD_PING      = 0x01
CMD_SET_DAC   = 0x10
CMD_READ_ADC  = 0x11
CMD_ZERO_OFFSET = 0x12
CMD_START_CA  = 0x20
CMD_START_CV  = 0x21
CMD_START_EIS = 0x22
CMD_STOP      = 0x23
CMD_START_DPV = 0x24
CMD_START_SWV = 0x25

CMD_DATA      = 0x30
CMD_ACK       = 0x7F
CMD_ERROR     = 0x7E


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def make_frame(cmd: int, payload: bytes = b"") -> bytes:
    body = bytes([cmd]) + struct.pack("<H", len(payload)) + payload
    crc = crc16_modbus(body)
    return FRAME_HEAD + body + struct.pack("<H", crc)


def dac_code_from_mv(mv: float, vref_mv: float = 2500.0) -> int:
    return int(np.clip(mv / vref_mv * 65535.0, 0, 65535))


def mv_i32(value_mv: float) -> int:
    return int(round(value_mv))


class SerialWorker(QThread):
    data_packet = Signal(dict)
    ack_packet = Signal(int)
    error_packet = Signal(int)
    text_line = Signal(str)
    status = Signal(str)

    def __init__(self):
        super().__init__()
        self.ser = None
        self.running = False
        self.tx_queue = deque()

    def open_port(self, port: str, baud: int):
        self.ser = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.02,
            write_timeout=0.2,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.running = True
        self.start()

    def close_port(self):
        self.running = False
        self.wait(800)
        if self.ser and self.ser.is_open:
            self.ser.close()

    def send_frame(self, cmd: int, payload: bytes = b""):
        self.tx_queue.append(make_frame(cmd, payload))

    def send_raw(self, data: bytes):
        self.tx_queue.append(data)

    def run(self):
        buf = bytearray()
        last_text_flush = time.time()
        self.status.emit("Serial thread started")

        while self.running:
            try:
                while self.tx_queue:
                    self.ser.write(self.tx_queue.popleft())

                incoming = self.ser.read(512)
                if incoming:
                    buf.extend(incoming)

                # Parse mixed stream:
                # 1) Binary protocol frames begin with AA 55.
                # 2) Firmware may also output plain CSV/text before protocol is active.
                while True:
                    idx = buf.find(FRAME_HEAD)

                    # No binary frame header: try to parse complete text lines.
                    if idx < 0:
                        self._consume_text_lines(buf, keep_tail=True)
                        break

                    # Text before binary header.
                    if idx > 0:
                        text_part = bytes(buf[:idx])
                        del buf[:idx]
                        self._emit_text_bytes(text_part)

                    # Need minimum frame size: AA55 + CMD + LEN(2) + CRC(2)
                    if len(buf) < 7:
                        break

                    cmd = buf[2]
                    length = struct.unpack("<H", buf[3:5])[0]
                    total = 2 + 1 + 2 + length + 2

                    if length > 256:
                        # Bad sync. Drop first byte and try again.
                        del buf[0]
                        continue

                    if len(buf) < total:
                        break

                    frame = bytes(buf[:total])
                    del buf[:total]

                    body = frame[2:-2]
                    rx_crc = struct.unpack("<H", frame[-2:])[0]
                    calc_crc = crc16_modbus(body)

                    if rx_crc != calc_crc:
                        self.status.emit("CRC error")
                        continue

                    payload = frame[5:-2]
                    self._handle_binary_frame(cmd, payload)

                # Periodically flush complete text lines if firmware is streaming CSV.
                now = time.time()
                if now - last_text_flush > 0.25:
                    self._consume_text_lines(buf, keep_tail=True)
                    last_text_flush = now

            except Exception as e:
                self.status.emit(f"Serial error: {e}")
                time.sleep(0.1)

        self.status.emit("Serial thread stopped")

    def _consume_text_lines(self, buf: bytearray, keep_tail: bool = True):
        while True:
            nl_positions = [p for p in (buf.find(b"\n"), buf.find(b"\r")) if p >= 0]
            if not nl_positions:
                # Avoid unlimited growth if only text without newline.
                if not keep_tail and buf:
                    self._emit_text_bytes(bytes(buf))
                    buf.clear()
                elif len(buf) > 512:
                    self._emit_text_bytes(bytes(buf[:512]))
                    del buf[:512]
                return

            p = min(nl_positions)
            line = bytes(buf[:p + 1])
            del buf[:p + 1]

            # Remove additional CR/LF bytes.
            while buf and buf[0] in (10, 13):
                del buf[0]

            self._emit_text_bytes(line)

    def _emit_text_bytes(self, data: bytes):
        text = data.decode(errors="ignore").strip()
        if not text:
            return
        self.text_line.emit(text)

        parsed = self._parse_firmware_csv(text)
        if parsed is not None:
            self.data_packet.emit(parsed)

    def _parse_firmware_csv(self, text: str):
        # Firmware text format:
        # old: sample,dac_mv,mode,BME_text,ch0_raw,ch0_mv,ch1_raw,ch1_mv,ch2_raw,ch2_mv
        # new: old fields plus ch0_uv,ch1_uv,ch2_uv. Use uV when present.
        parts = [p.strip() for p in text.split(",")]
        if len(parts) < 10:
            return None

        try:
            sample = int(parts[0])
            dac_mv = float(parts[1])
            ch0_raw = int(parts[4])
            ch0_mv = float(parts[5])
            ch1_raw = int(parts[6])
            ch1_mv = float(parts[7])
            ch2_raw = int(parts[8])
            ch2_mv = float(parts[9])

            if len(parts) >= 13:
                ch0_uv = float(parts[10])
                ch1_uv = float(parts[11])
                ch2_uv = float(parts[12])
                e_meas_v = (ch0_uv + ch2_uv) / 1e6
                tia_mv = ch1_uv / 1000.0
            else:
                e_meas_v = (ch0_mv + ch2_mv) / 1000.0
                tia_mv = ch1_mv

            # Firmware loop period is LOOP_PERIOD_MS = 100 ms.
            t_ms = sample * 100

            # CH1 is the TIA/current-channel voltage. The GUI converts tia_mv
            # to current using the user-entered feedback resistor.
            return {
                "t_ms": t_ms,
                "e_set_v": dac_mv / 1000.0,
                "e_meas_v": e_meas_v,
                "tia_mv": tia_mv,
                "i_a": (tia_mv * 1000.0) / 1e12,  # temporary 1 MOhm conversion; corrected in MainWindow.on_data
                "adc_raw": ch1_raw,
                "source": "csv",
                "raw_text": text,
            }
        except Exception:
            return None

    def _handle_binary_frame(self, cmd: int, payload: bytes):
        if cmd == CMD_ACK:
            if len(payload) >= 1:
                self.ack_packet.emit(payload[0])
                self.status.emit(f"ACK 0x{payload[0]:02X}")
            else:
                self.status.emit("ACK")

        elif cmd == CMD_ERROR:
            if len(payload) >= 1:
                self.error_packet.emit(payload[0])
                self.status.emit(f"ERROR for cmd 0x{payload[0]:02X}")
            else:
                self.status.emit("ERROR")

        elif cmd == CMD_DATA:
            # payload:
            # uint32 timestamp_ms
            # int32  e_set_uV
            # int32  e_meas_uV
            # int32  current_pA
            # int32  adc_raw
            if len(payload) >= 20:
                t_ms, e_set_uv, e_meas_uv, current_pa, adc_raw = struct.unpack("<Iiiii", payload[:20])
                d = {
                    "t_ms": t_ms,
                    "e_set_v": e_set_uv / 1e6,
                    "e_meas_v": e_meas_uv / 1e6,
                    # Firmware current_pA is the equivalent current for Rf = 1 MOhm.
                    # With the 24-bit firmware this is derived from TIA uV, so it
                    # preserves sub-mV resolution. Convert it back to equivalent
                    # TIA voltage so the GUI can rescale for the entered Rf.
                    "tia_mv": current_pa / 1000.0,
                    "i_a": current_pa / 1e12,  # temporary 1 MOhm conversion; corrected in MainWindow.on_data
                    "adc_raw": adc_raw,
                    "source": "binary",
                }
                # Extended firmware appends environment data after the first 20 bytes:
                # int32 temp_centi_C, uint32 humidity_milli_percent, uint32 pressure_Pa, uint8 valid.
                if len(payload) >= 33:
                    temp_centi, hum_milli, pressure_pa = struct.unpack("<iII", payload[20:32])
                    env_valid = payload[32] != 0
                    if env_valid:
                        d["temp_c"] = temp_centi / 100.0
                        d["humidity_pct"] = hum_milli / 1000.0
                        d["pressure_hpa"] = pressure_pa / 100.0
                self.data_packet.emit(d)

        else:
            self.status.emit(f"Unknown frame cmd 0x{cmd:02X}, len={len(payload)}")


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("MSP430 Electrochemical Workstation GUI")
        self.resize(1250, 780)

        self.worker = SerialWorker()
        self.worker.data_packet.connect(self.on_data)
        self.worker.text_line.connect(self.on_text_line)
        self.worker.status.connect(self.set_status)
        self.worker.ack_packet.connect(self.on_ack)
        self.worker.error_packet.connect(self.on_error)

        self.t_data = []
        self.i_data = []
        self.tia_mv_data = []
        self.e_data = []
        self.e_set_data = []
        self.source_data = []

        self.csv_file = None
        self.csv_writer = None
        self.auto_y = True

        self.latest_data = None
        self.pending_zero_snapshot = None
        self.last_zero_offset = None

        # PC-side current gain calibration. Original current is preserved in CSV;
        # calibrated current is added as a separate column and used for display when enabled.
        self.current_gain_cal_enabled = False
        self.current_gain_factor = 1.026
        self.current_offset_a = 0.0  # additive display/CSV soft offset in ampere

        # Each test mode owns an independent data buffer. This prevents data from
        # one scan appearing in another scan's plots.
        self.current_mode = "manual"
        self.mode_buffers = {name: self.new_data_buffer() for name in ("manual", "ca", "cv", "pulse", "eis")}
        self.ca_avg_buffer = []
        self.env_poll_pending = False

        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)

        layout.addWidget(self.build_connection_bar())
        self.recalculate_current_from_rf()

        self.tabs = QTabWidget()
        self.tabs.addTab(self.build_manual_tab(), "Manual")
        self.tabs.addTab(self.build_ca_tab(), "CA")
        self.tabs.addTab(self.build_cv_tab(), "CV / LSV")
        self.tabs.addTab(self.build_pulse_tab(), "DPV / SWV")
        self.tabs.addTab(self.build_eis_tab(), "EIS")
        self.tabs.addTab(self.build_terminal_tab(), "Raw UART")
        layout.addWidget(self.tabs)

        bottom_row = QHBoxLayout()
        self.status_label = QLabel("Disconnected")
        self.env_label = QLabel("T: -- °C   H: -- %   P: -- hPa")
        self.env_label.setMinimumWidth(260)
        self.env_label.setAlignment(Qt.AlignRight)
        bottom_row.addWidget(self.status_label, 1)
        bottom_row.addWidget(self.env_label, 0)
        layout.addLayout(bottom_row)

        for plot in self.all_plots():
            try:
                plot.getViewBox().sigRangeChangedManually.connect(self.disable_auto_y)
            except Exception:
                pass

        self.plot_timer = QTimer()
        self.plot_timer.timeout.connect(self.refresh_plots)
        self.plot_timer.start(100)

        # Poll a single ADC/environment packet periodically after connection so
        # temperature/humidity/pressure update even when no electrochemical scan is running.
        self.env_timer = QTimer()
        self.env_timer.timeout.connect(self.poll_environment)
        self.env_timer.start(1000)

    def build_connection_bar(self):
        box = QGroupBox("Connection")
        outer = QVBoxLayout(box)
        row = QHBoxLayout()
        info_row = QHBoxLayout()

        self.port_box = QComboBox()
        self.refresh_ports()

        refresh_btn = QPushButton("Refresh")
        refresh_btn.clicked.connect(self.refresh_ports)

        self.baud_box = QComboBox()
        self.baud_box.addItems(["9600", "57600", "115200", "230400", "460800", "921600"])
        self.baud_box.setCurrentText("115200")

        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)

        self.ping_btn = QPushButton("Ping")
        self.ping_btn.clicked.connect(lambda: self.send(CMD_PING))

        self.stop_btn = QPushButton("Stop")
        self.stop_btn.clicked.connect(self.stop_scan)

        self.zero_btn = QPushButton("Zero SD24B Offset")
        self.zero_btn.setToolTip("Short the TIA/current SD24B channel, then run 0-offset calibration")
        self.zero_btn.clicked.connect(self.zero_current)

        self.auto_y_btn = QPushButton("Return Auto Y")
        self.auto_y_btn.clicked.connect(self.return_auto_y)

        self.save_btn = QPushButton("Start CSV")
        self.save_btn.clicked.connect(self.toggle_csv)

        self.rf_mohm = QDoubleSpinBox()
        self.rf_mohm.setRange(0.001, 10000.0)
        self.rf_mohm.setDecimals(6)
        self.rf_mohm.setValue(10.0)
        self.rf_mohm.setSuffix(" MΩ")
        self.rf_mohm.setToolTip("TIA feedback resistor. Current is calculated as I = V_TIA / Rf.")
        self.rf_mohm.valueChanged.connect(self.recalculate_current_from_rf)

        self.rf_info_label = QLabel("Current scale: 1.000 mV → 1000.000 pA")
        self.rf_info_label.setMinimumWidth(260)

        self.zero_offset_label = QLabel("Zero offset: --")
        self.zero_offset_label.setMinimumWidth(360)
        self.zero_offset_label.setToolTip(
            "Shows the offset estimated by the PC from the latest shorted reading before CMD_ZERO_OFFSET. "
            "Current firmware ACK does not return the exact averaged calibration value."
        )

        self.gain_cal_btn = QPushButton("Apply Gain Cal")
        self.gain_cal_btn.setCheckable(True)
        self.gain_cal_btn.setToolTip(
            "Enable PC-side gain correction: I_cal = I_raw / gain. "
            "CSV keeps original current_a and adds current_gain_cal_a."
        )
        self.gain_cal_btn.clicked.connect(self.toggle_gain_calibration)

        self.gain_factor = QDoubleSpinBox()
        self.gain_factor.setRange(0.100000, 10.000000)
        self.gain_factor.setDecimals(6)
        self.gain_factor.setValue(1.065000)
        self.gain_factor.setSingleStep(0.001)
        self.gain_factor.setToolTip("Measured gain error factor. Example: -100 nA reads -102.6 nA → gain = 1.026.")
        self.gain_factor.valueChanged.connect(self.update_gain_factor)

        self.current_offset_na = QDoubleSpinBox()
        self.current_offset_na.setRange(-1e9, 1e9)
        self.current_offset_na.setDecimals(6)
        self.current_offset_na.setValue(4.0)
        self.current_offset_na.setSingleStep(0.1)
        self.current_offset_na.setSuffix(" nA")
        self.current_offset_na.setToolTip("Additive PC-side current offset correction. Positive value shifts displayed calibrated current upward.")
        self.current_offset_na.valueChanged.connect(self.update_current_offset)


        row.addWidget(QLabel("Port"))
        row.addWidget(self.port_box)
        row.addWidget(refresh_btn)
        row.addWidget(QLabel("Baud"))
        row.addWidget(self.baud_box)
        row.addWidget(self.connect_btn)
        row.addWidget(self.ping_btn)
        row.addWidget(self.stop_btn)
        row.addWidget(self.zero_btn)
        row.addWidget(self.auto_y_btn)
        row.addStretch()
        row.addWidget(self.save_btn)

        info_row.addWidget(QLabel("TIA Rf"))
        info_row.addWidget(self.rf_mohm)
        info_row.addWidget(self.rf_info_label)
        info_row.addSpacing(20)
        info_row.addWidget(self.zero_offset_label)
        info_row.addSpacing(20)
        info_row.addWidget(QLabel("Gain"))
        info_row.addWidget(self.gain_factor)
        info_row.addWidget(QLabel("Offset"))
        info_row.addWidget(self.current_offset_na)
        info_row.addWidget(self.gain_cal_btn)
        info_row.addStretch()

        outer.addLayout(row)
        outer.addLayout(info_row)

        return box

    def build_manual_tab(self):
        w = QWidget()
        layout = QHBoxLayout(w)

        form_box = QGroupBox("Manual Control (DAC controls RE path; TIA bias is fixed near 0.5 V)")
        form = QFormLayout(form_box)

        self.dac_ch = QComboBox()
        self.dac_ch.addItems(["0", "1"])

        self.dac_mv = QDoubleSpinBox()
        self.dac_mv.setRange(0, 2500)
        self.dac_mv.setDecimals(3)
        self.dac_mv.setValue(100)
        self.dac_mv.setSuffix(" mV")

        set_dac_btn = QPushButton("Set DAC")
        set_dac_btn.clicked.connect(self.set_dac)

        read_adc_btn = QPushButton("Read ADC")
        read_adc_btn.clicked.connect(lambda: self.send(CMD_READ_ADC))

        clear_btn = QPushButton("Clear Plot")
        clear_btn.clicked.connect(lambda: self.clear_data("manual"))

        form.addRow("DAC channel", self.dac_ch)
        form.addRow("DAC output / RE-control voltage", self.dac_mv)
        form.addRow(set_dac_btn)
        form.addRow(read_adc_btn)
        form.addRow(clear_btn)
        plot_box = QWidget()
        plot_layout = QVBoxLayout(plot_box)
        self.et_plot = pg.PlotWidget(title="DAC / RE-control Voltage vs Time")
        self.et_plot.setLabel("bottom", "Time", units="s")
        self.et_plot.setLabel("left", "Voltage", units="V")
        self.et_curve = self.et_plot.plot()

        self.it_plot = pg.PlotWidget(title="Current vs Time")
        self.it_plot.setLabel("bottom", "Time", units="s")
        self.it_plot.setLabel("left", "Current", units="A")
        self.it_curve = self.it_plot.plot()
        plot_layout.addWidget(self.et_plot, 1)
        plot_layout.addWidget(self.it_plot, 1)

        layout.addWidget(form_box, 1)
        layout.addWidget(plot_box, 4)
        return w

    def build_ca_tab(self):
        w = QWidget()
        layout = QHBoxLayout(w)

        form_box = QGroupBox("Chronoamperometry")
        form = QFormLayout(form_box)

        self.ca_potential = QDoubleSpinBox()
        self.ca_potential.setRange(0, 2500)
        self.ca_potential.setValue(100)
        self.ca_potential.setSuffix(" mV")

        self.ca_duration = QSpinBox()
        self.ca_duration.setRange(0, 24 * 3600)
        self.ca_duration.setValue(0)
        self.ca_duration.setSuffix(" s")

        self.ca_interval = QSpinBox()
        self.ca_interval.setRange(100, 60000)
        self.ca_interval.setValue(100)
        self.ca_interval.setSuffix(" ms")

        self.ca_avg_points = QSpinBox()
        self.ca_avg_points.setRange(1, 4096)
        self.ca_avg_points.setValue(1)
        self.ca_avg_points.setToolTip("Number of consecutive samples averaged into one CA point. Use 1 for normal CA.")

        self.ca_status_label = QLabel("CA averaging: 1 sample/point")

        start_btn = QPushButton("Start CA")
        start_btn.clicked.connect(self.start_ca)

        stop_btn = QPushButton("Stop CA")
        stop_btn.clicked.connect(lambda: self.send(CMD_STOP))

        clear_btn = QPushButton("Clear CA Plot")
        clear_btn.clicked.connect(lambda: self.clear_data("ca"))

        form.addRow("DAC / RE-control voltage", self.ca_potential)
        form.addRow("Duration, 0=continuous", self.ca_duration)
        form.addRow("Raw sample interval", self.ca_interval)
        form.addRow("Average points", self.ca_avg_points)
        form.addRow(start_btn)
        form.addRow(stop_btn)
        form.addRow(clear_btn)

        plot_box = QWidget()
        plot_layout = QVBoxLayout(plot_box)
        self.et_plot_ca = pg.PlotWidget(title="CA: DAC / RE-control Voltage vs Time")
        self.et_plot_ca.setLabel("bottom", "Time", units="s")
        self.et_plot_ca.setLabel("left", "Voltage", units="V")
        self.et_curve_ca = self.et_plot_ca.plot()

        self.it_plot_ca = pg.PlotWidget(title="CA: Current vs Time")
        self.it_plot_ca.setLabel("bottom", "Time", units="s")
        self.it_plot_ca.setLabel("left", "Current", units="A")
        self.it_curve_ca = self.it_plot_ca.plot()
        plot_layout.addWidget(self.et_plot_ca, 1)
        plot_layout.addWidget(self.it_plot_ca, 1)

        layout.addWidget(form_box, 1)
        layout.addWidget(plot_box, 4)
        return w

    def build_hp_ca_tab(self):
        w = QWidget()
        layout = QHBoxLayout(w)

        form_box = QGroupBox("High Precision Chronoamperometry (PC-side multi-sample average)")
        form = QFormLayout(form_box)

        self.hp_ca_potential = QDoubleSpinBox()
        self.hp_ca_potential.setRange(0, 2500)
        self.hp_ca_potential.setValue(100)
        self.hp_ca_potential.setSuffix(" mV")

        self.hp_ca_duration = QSpinBox()
        self.hp_ca_duration.setRange(0, 24 * 3600)
        self.hp_ca_duration.setValue(0)
        self.hp_ca_duration.setSuffix(" s")

        self.hp_ca_interval = QSpinBox()
        self.hp_ca_interval.setRange(100, 60000)
        self.hp_ca_interval.setValue(100)
        self.hp_ca_interval.setSuffix(" ms")

        self.hp_avg_points = QSpinBox()
        self.hp_avg_points.setRange(1, 4096)
        self.hp_avg_points.setValue(16)
        self.hp_avg_points.setToolTip("Number of consecutive ADC packets averaged into one displayed high-precision point.")

        self.hp_status_label = QLabel("HP CA: stopped")

        start_btn = QPushButton("Start High Precision CA")
        start_btn.clicked.connect(self.start_hp_ca)

        stop_btn = QPushButton("Stop HP CA")
        stop_btn.clicked.connect(self.stop_hp_ca)

        clear_btn = QPushButton("Clear HP Plot")
        clear_btn.clicked.connect(self.clear_hp_ca_data)

        form.addRow("DAC / RE-control voltage", self.hp_ca_potential)
        form.addRow("Duration, 0=continuous", self.hp_ca_duration)
        form.addRow("Raw sample interval", self.hp_ca_interval)
        form.addRow("Average points", self.hp_avg_points)
        form.addRow(start_btn)
        form.addRow(stop_btn)
        form.addRow(clear_btn)
        form.addRow(self.hp_status_label)

        plot_box = QWidget()
        plot_layout = QVBoxLayout(plot_box)
        self.hp_et_plot = pg.PlotWidget(title="High Precision CA: Averaged DAC / RE-control Voltage vs Time")
        self.hp_et_plot.setLabel("bottom", "Time", units="s")
        self.hp_et_plot.setLabel("left", "Voltage", units="V")
        self.hp_et_curve = self.hp_et_plot.plot()

        self.hp_it_plot = pg.PlotWidget(title="High Precision CA: Averaged Current vs Time")
        self.hp_it_plot.setLabel("bottom", "Time", units="s")
        self.hp_it_plot.setLabel("left", "Current", units="A")
        self.hp_it_curve = self.hp_it_plot.plot()
        plot_layout.addWidget(self.hp_et_plot, 1)
        plot_layout.addWidget(self.hp_it_plot, 1)

        layout.addWidget(form_box, 1)
        layout.addWidget(plot_box, 4)
        return w

    def build_cv_tab(self):
        w = QWidget()
        layout = QHBoxLayout(w)

        form_box = QGroupBox("CV / LSV")
        form = QFormLayout(form_box)

        self.cv_start = QDoubleSpinBox()
        self.cv_start.setRange(0, 2500)
        self.cv_start.setValue(100)
        self.cv_start.setSuffix(" mV")

        self.cv_end = QDoubleSpinBox()
        self.cv_end.setRange(0, 2500)
        self.cv_end.setValue(800)
        self.cv_end.setSuffix(" mV")

        self.cv_step = QDoubleSpinBox()
        self.cv_step.setRange(1, 1000)
        self.cv_step.setValue(10)
        self.cv_step.setSuffix(" mV")

        self.cv_interval = QSpinBox()
        self.cv_interval.setRange(100, 60000)
        self.cv_interval.setValue(100)
        self.cv_interval.setSuffix(" ms")

        self.cv_cycles = QSpinBox()
        self.cv_cycles.setRange(1, 20)
        self.cv_cycles.setValue(1)

        start_btn = QPushButton("Start CV / LSV")
        start_btn.clicked.connect(self.start_cv)

        pause_btn = QPushButton("Pause CV / LSV")
        pause_btn.clicked.connect(self.pause_cv_lsv)

        clear_btn = QPushButton("Clear CV / LSV Plot")
        clear_btn.clicked.connect(lambda: self.clear_data("cv"))

        form.addRow("Start DAC / RE-control voltage", self.cv_start)
        form.addRow("End DAC / RE-control voltage", self.cv_end)
        form.addRow("Step", self.cv_step)
        form.addRow("Step interval", self.cv_interval)
        form.addRow("Half-cycles", self.cv_cycles)
        form.addRow(start_btn)
        form.addRow(pause_btn)
        form.addRow(clear_btn)

        plot_box = QWidget()
        plot_layout = QVBoxLayout(plot_box)
        self.et_plot_cv = pg.PlotWidget(title="CV/LSV: DAC / RE-control Voltage vs Time")
        self.et_plot_cv.setLabel("bottom", "Time", units="s")
        self.et_plot_cv.setLabel("left", "Voltage", units="V")
        self.et_curve_cv = self.et_plot_cv.plot()

        self.it_plot_cv = pg.PlotWidget(title="CV/LSV: Current vs Time")
        self.it_plot_cv.setLabel("bottom", "Time", units="s")
        self.it_plot_cv.setLabel("left", "Current", units="A")
        self.it_curve_cv = self.it_plot_cv.plot()

        self.ie_plot = pg.PlotWidget(title="Current vs Measured WE-RE")
        self.ie_plot.setLabel("bottom", "Measured WE-RE", units="V")
        self.ie_plot.setLabel("left", "Current", units="A")
        self.ie_curve = self.ie_plot.plot()
        plot_layout.addWidget(self.et_plot_cv, 1)
        plot_layout.addWidget(self.it_plot_cv, 1)
        plot_layout.addWidget(self.ie_plot, 1)

        layout.addWidget(form_box, 1)
        layout.addWidget(plot_box, 4)
        return w

    def build_pulse_tab(self):
        w = QWidget()
        layout = QHBoxLayout(w)

        form_box = QGroupBox("Pulse Voltammetry")
        form = QFormLayout(form_box)

        self.pulse_start = QDoubleSpinBox()
        self.pulse_start.setRange(0, 2500)
        self.pulse_start.setValue(100)
        self.pulse_start.setSuffix(" mV")

        self.pulse_end = QDoubleSpinBox()
        self.pulse_end.setRange(0, 2500)
        self.pulse_end.setValue(800)
        self.pulse_end.setSuffix(" mV")

        self.pulse_step = QDoubleSpinBox()
        self.pulse_step.setRange(1, 1000)
        self.pulse_step.setValue(10)
        self.pulse_step.setSuffix(" mV")

        self.pulse_amp = QDoubleSpinBox()
        self.pulse_amp.setRange(1, 1000)
        self.pulse_amp.setValue(50)
        self.pulse_amp.setSuffix(" mV")

        self.pulse_interval = QSpinBox()
        self.pulse_interval.setRange(100, 60000)
        self.pulse_interval.setValue(100)
        self.pulse_interval.setSuffix(" ms")

        dpv_btn = QPushButton("Start DPV")
        dpv_btn.clicked.connect(self.start_dpv)
        swv_btn = QPushButton("Start SWV")
        swv_btn.clicked.connect(self.start_swv)

        form.addRow("Start DAC / RE-control voltage", self.pulse_start)
        form.addRow("End DAC / RE-control voltage", self.pulse_end)
        form.addRow("Staircase step", self.pulse_step)
        form.addRow("Pulse / square amplitude", self.pulse_amp)
        form.addRow("Half-period / interval", self.pulse_interval)
        form.addRow(dpv_btn)
        form.addRow(swv_btn)

        stop_btn = QPushButton("Stop DPV / SWV")
        stop_btn.clicked.connect(lambda: self.send(CMD_STOP))
        clear_btn = QPushButton("Clear DPV / SWV Plot")
        clear_btn.clicked.connect(lambda: self.clear_data("pulse"))
        form.addRow(stop_btn)
        form.addRow(clear_btn)

        plot_box = QWidget()
        plot_layout = QVBoxLayout(plot_box)

        self.pulse_et_plot = pg.PlotWidget(title="DPV/SWV: Potential vs Time")
        self.pulse_et_plot.setLabel("bottom", "Time", units="s")
        self.pulse_et_plot.setLabel("left", "Potential", units="V")
        self.pulse_et_curve = self.pulse_et_plot.plot()

        self.pulse_it_plot = pg.PlotWidget(title="DPV/SWV: Current vs Time")
        self.pulse_it_plot.setLabel("bottom", "Time", units="s")
        self.pulse_it_plot.setLabel("left", "Current", units="A")
        self.pulse_it_curve = self.pulse_it_plot.plot()

        self.pulse_plot = pg.PlotWidget(title="DPV/SWV: Current vs Measured WE-RE")
        self.pulse_plot.setLabel("bottom", "Measured WE-RE", units="V")
        self.pulse_plot.setLabel("left", "Current", units="A")
        self.pulse_curve = self.pulse_plot.plot()

        plot_layout.addWidget(self.pulse_et_plot, 1)
        plot_layout.addWidget(self.pulse_it_plot, 1)
        plot_layout.addWidget(self.pulse_plot, 1)

        layout.addWidget(form_box, 1)
        layout.addWidget(plot_box, 4)
        return w

    def build_eis_tab(self):
        w = QWidget()
        layout = QHBoxLayout(w)

        form_box = QGroupBox("EIS Placeholder: square perturbation")
        form = QFormLayout(form_box)

        self.eis_dc = QDoubleSpinBox()
        self.eis_dc.setRange(0, 2500)
        self.eis_dc.setValue(500)
        self.eis_dc.setSuffix(" mV")

        self.eis_amp = QDoubleSpinBox()
        self.eis_amp.setRange(1, 500)
        self.eis_amp.setValue(10)
        self.eis_amp.setSuffix(" mV")

        self.eis_half_period = QSpinBox()
        self.eis_half_period.setRange(100, 60000)
        self.eis_half_period.setValue(500)
        self.eis_half_period.setSuffix(" ms")

        self.eis_duration = QSpinBox()
        self.eis_duration.setRange(1, 24 * 3600)
        self.eis_duration.setValue(10)
        self.eis_duration.setSuffix(" s")

        start_btn = QPushButton("Start EIS Perturbation")
        start_btn.clicked.connect(self.start_eis)

        form.addRow("DC DAC / RE-control voltage", self.eis_dc)
        form.addRow("Amplitude", self.eis_amp)
        form.addRow("Half-period", self.eis_half_period)
        form.addRow("Duration", self.eis_duration)
        form.addRow(start_btn)
        stop_btn = QPushButton("Stop EIS")
        stop_btn.clicked.connect(self.stop_scan)
        clear_btn = QPushButton("Clear EIS Plot")
        clear_btn.clicked.connect(lambda: self.clear_data("eis"))
        form.addRow(stop_btn)
        form.addRow(clear_btn)

        self.eis_plot = pg.PlotWidget(title="EIS placeholder: Current vs Time")
        self.eis_plot.setLabel("bottom", "Time", units="s")
        self.eis_plot.setLabel("left", "Current", units="A")
        self.eis_curve = self.eis_plot.plot()

        layout.addWidget(form_box, 1)
        layout.addWidget(self.eis_plot, 4)
        return w

    def build_terminal_tab(self):
        w = QWidget()
        layout = QVBoxLayout(w)

        self.raw_text = QTextEdit()
        self.raw_text.setReadOnly(True)

        row = QHBoxLayout()
        self.show_raw_check = QCheckBox("Show raw UART text")
        self.show_raw_check.setChecked(True)

        clear_btn = QPushButton("Clear Raw")
        clear_btn.clicked.connect(self.raw_text.clear)

        row.addWidget(self.show_raw_check)
        row.addStretch()
        row.addWidget(clear_btn)

        send_row = QHBoxLayout()
        self.raw_input = QLineEdit()
        self.raw_input.setPlaceholderText("ASCII command or hex bytes, e.g. AA 55 01 00 00 ...")
        send_ascii_btn = QPushButton("Send ASCII")
        send_ascii_btn.clicked.connect(self.send_raw_ascii)
        send_hex_btn = QPushButton("Send HEX")
        send_hex_btn.clicked.connect(self.send_raw_hex)
        send_row.addWidget(self.raw_input, 1)
        send_row.addWidget(send_ascii_btn)
        send_row.addWidget(send_hex_btn)

        layout.addLayout(row)
        layout.addLayout(send_row)
        layout.addWidget(self.raw_text)
        return w

    def refresh_ports(self):
        current = self.port_box.currentText()
        self.port_box.clear()

        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            label = f"{p.device} - {p.description}"
            self.port_box.addItem(label, p.device)

        if current:
            idx = self.port_box.findText(current)
            if idx >= 0:
                self.port_box.setCurrentIndex(idx)

    def selected_port(self):
        return self.port_box.currentData() or self.port_box.currentText().split(" ")[0]

    def toggle_connection(self):
        if self.worker.ser and self.worker.ser.is_open:
            self.worker.close_port()
            self.connect_btn.setText("Connect")
            self.set_status("Disconnected")
            return

        port = self.selected_port()
        if not port:
            QMessageBox.warning(self, "No COM Port", "No COM port selected.")
            return

        try:
            self.worker.open_port(port, int(self.baud_box.currentText()))
            self.connect_btn.setText("Disconnect")
            self.set_status(f"Connected to {port}")

            # Send a ping after opening. This makes firmware enter binary protocol mode
            # and suppress plain CSV logging after it receives a valid frame.
            QTimer.singleShot(300, lambda: self.send(CMD_PING))

        except Exception as e:
            QMessageBox.critical(self, "Serial Error", str(e))

    def send(self, cmd: int, payload: bytes = b""):
        if not (self.worker.ser and self.worker.ser.is_open):
            self.set_status("Not connected")
            return
        self.worker.send_frame(cmd, payload)

    def send_raw_bytes(self, data: bytes):
        if not (self.worker.ser and self.worker.ser.is_open):
            self.set_status("Not connected")
            return
        self.worker.send_raw(data)

    def send_raw_ascii(self):
        text = self.raw_input.text() if hasattr(self, "raw_input") else ""
        if not text:
            return
        data = (text + "\r\n").encode(errors="ignore")
        self.send_raw_bytes(data)
        if hasattr(self, "raw_text"):
            self.raw_text.append(f"> ASCII: {text}")

    def send_raw_hex(self):
        text = self.raw_input.text() if hasattr(self, "raw_input") else ""
        try:
            data = bytes.fromhex(text.replace(",", " "))
        except ValueError as e:
            QMessageBox.warning(self, "Invalid HEX", str(e))
            return
        self.send_raw_bytes(data)
        if hasattr(self, "raw_text"):
            self.raw_text.append(f"> HEX: {data.hex(' ').upper()}")

    def set_dac(self):
        ch = int(self.dac_ch.currentText())
        code = dac_code_from_mv(self.dac_mv.value())
        payload = struct.pack("<BH", ch, code)
        self.send(CMD_SET_DAC, payload)

    def new_data_buffer(self):
        return {"t": [], "i": [], "tia_mv": [], "e": [], "e_set": [], "source": []}

    def append_mode_data(self, mode, d):
        if mode not in self.mode_buffers:
            mode = "manual"
        b = self.mode_buffers[mode]
        b["t"].append(d["t_ms"] / 1000.0)
        b["i"].append(d["i_a"])
        b["tia_mv"].append(d.get("tia_mv", 0.0))
        b["e"].append(d.get("e_meas_v", 0.0))
        b["e_set"].append(d.get("e_set_v", d.get("e_meas_v", 0.0)))
        b["source"].append(d.get("source", "?"))
        max_len = 5000
        for k in b:
            if len(b[k]) > max_len:
                b[k] = b[k][-max_len:]

    def clear_data(self, mode=None):
        """Clear only one mode by default; use mode='all' to clear every plot."""
        if mode is None:
            mode = self.current_mode
        modes = self.mode_buffers.keys() if mode == "all" else [mode]
        for m in modes:
            if m in self.mode_buffers:
                self.mode_buffers[m] = self.new_data_buffer()
        if mode in ("ca", "all"):
            self.ca_avg_buffer = []
            if hasattr(self, "ca_status_label"):
                self.ca_status_label.setText(f"CA averaging: {int(self.ca_avg_points.value())} sample/point")
        self.refresh_plots()

    def stop_scan(self):
        self.send(CMD_STOP)
        self.current_mode = "manual"
        self.ca_avg_buffer = []
        self.set_status("Scan stopped")

    def poll_environment(self):
        if not (self.worker.ser and self.worker.ser.is_open):
            return
        # Avoid injecting manual reads into active scan buffers. Active scans still
        # update the environment label through their normal data packets.
        if self.current_mode != "manual":
            return
        self.env_poll_pending = True
        self.send(CMD_READ_ADC)

    def process_ca_sample(self, d):
        n = int(self.ca_avg_points.value()) if hasattr(self, "ca_avg_points") else 1
        if n <= 1:
            self.append_mode_data("ca", d)
            if hasattr(self, "ca_status_label"):
                self.ca_status_label.setText("CA averaging: 1 sample/point")
            return
        self.ca_avg_buffer.append(dict(d))
        if len(self.ca_avg_buffer) < n:
            if hasattr(self, "ca_status_label"):
                self.ca_status_label.setText(f"CA collecting {len(self.ca_avg_buffer)}/{n} samples")
            return
        buf = self.ca_avg_buffer[:n]
        self.ca_avg_buffer = self.ca_avg_buffer[n:]
        avg = dict(buf[-1])
        avg["t_ms"] = sum(x["t_ms"] for x in buf) / n
        avg["e_set_v"] = sum(x.get("e_set_v", x.get("e_meas_v", 0.0)) for x in buf) / n
        avg["e_meas_v"] = sum(x.get("e_meas_v", 0.0) for x in buf) / n
        avg["tia_mv"] = sum(x.get("tia_mv", 0.0) for x in buf) / n
        raw_current_a = self.current_from_tia_mv(avg["tia_mv"])
        avg["i_a_raw"] = raw_current_a
        avg["i_a_gain_cal"] = self.current_gain_corrected(raw_current_a)
        avg["i_a"] = self.display_current(raw_current_a)
        self.append_mode_data("ca", avg)
        if hasattr(self, "ca_status_label"):
            noise_pa = float(np.std([x["i_a"] for x in buf])) * 1e12 if n > 1 else 0.0
            self.ca_status_label.setText(f"CA averaged {n} samples/point, raw std {noise_pa:.3f} pA")

    def current_gain_corrected(self, current_a):
        gain = max(float(self.current_gain_factor), 1e-12)
        return current_a / gain + self.current_offset_a

    def display_current(self, current_a):
        if self.current_gain_cal_enabled:
            return self.current_gain_corrected(current_a)
        return current_a

    def update_gain_factor(self, *args):
        if hasattr(self, "gain_factor"):
            self.current_gain_factor = float(self.gain_factor.value())
        self.update_gain_cal_label()
        self.rebuild_display_currents()

    def update_current_offset(self, *args):
        if hasattr(self, "current_offset_na"):
            self.current_offset_a = float(self.current_offset_na.value()) * 1e-9
        self.update_gain_cal_label()
        self.rebuild_display_currents()

    def toggle_gain_calibration(self, *args):
        self.current_gain_cal_enabled = self.gain_cal_btn.isChecked()
        self.update_gain_cal_label()
        self.rebuild_display_currents()

    def update_gain_cal_label(self):
        # Status text intentionally removed; the checkable button state indicates ON/OFF.
        pass

    def rebuild_display_currents(self):
        for b in self.mode_buffers.values():
            b["i"] = [self.display_current(self.current_from_tia_mv(v)) for v in b["tia_mv"]]
        self.refresh_plots()

    def get_rf_ohm(self):
        return max(self.rf_mohm.value() * 1e6, 1.0)

    def current_from_tia_mv(self, tia_mv):
        return (tia_mv / 1000.0) / self.get_rf_ohm()

    def recalculate_current_from_rf(self):
        if hasattr(self, "rf_info_label"):
            pa_per_mv = 1e9 / self.rf_mohm.value()
            self.rf_info_label.setText(f"Current scale: 1.000 mV → {pa_per_mv:.3f} pA")

        if hasattr(self, "mode_buffers"):
            for b in self.mode_buffers.values():
                b["i"] = [self.display_current(self.current_from_tia_mv(v)) for v in b["tia_mv"]]
            self.refresh_plots()

        if self.latest_data and "tia_mv" in self.latest_data:
            raw_i = self.current_from_tia_mv(self.latest_data["tia_mv"])
            self.latest_data["i_a_raw"] = raw_i
            self.latest_data["i_a_gain_cal"] = self.current_gain_corrected(raw_i)
            self.latest_data["i_a"] = self.display_current(raw_i)
        if self.last_zero_offset and "tia_mv" in self.last_zero_offset:
            z = dict(self.last_zero_offset)
            z["i_a"] = self.current_from_tia_mv(z["tia_mv"])
            self.last_zero_offset = z
            self.zero_offset_label.setText(self.format_zero_offset_from_data(z))

    def format_zero_offset_from_data(self, d):
        """Estimate the 0-offset from the last received sample.

        The existing firmware returns only ACK for CMD_ZERO_OFFSET, not the
        exact averaged offset. Therefore this label reports the PC-side
        estimate from the latest reading taken while the input is shorted.
        The displayed current uses the current TIA feedback resistor setting.
        """
        if not d:
            return "Zero offset: no pre-cal sample"

        offset_mv = d.get("tia_mv", d.get("i_a", 0.0) * 1e9 * self.rf_mohm.value())
        current_pa = self.current_from_tia_mv(offset_mv) * 1e12
        adc_raw = d.get("adc_raw", None)
        source = d.get("source", "?")

        if adc_raw is None:
            return f"Zero offset est.: {offset_mv:+.6f} mV, {current_pa:+.3f} pA ({source})"
        return f"Zero offset est.: {offset_mv:+.6f} mV, {current_pa:+.3f} pA, raw {adc_raw:+d} ({source})"

    def zero_current(self):
        if not (self.worker.ser and self.worker.ser.is_open):
            QMessageBox.warning(self, "Not connected", "Please connect to the MSP430 first.")
            return

        msg = (
            "SD24B 0-offset calibration will average the selected current/TIA channel "
            "with zero input.\n\n"
            "Please short the channel that needs calibration now. For the current channel, "
            "short the sensor/electrode input so the expected current is 0.\n\n"
            "After the short is connected, click OK to continue."
        )
        if QMessageBox.information(self, "Prepare 0-offset calibration", msg,
                                   QMessageBox.Ok | QMessageBox.Cancel) != QMessageBox.Ok:
            self.set_status("0-offset calibration cancelled")
            return

        confirm = (
            "Confirm that the channel is already shorted.\n\n"
            "The MSP430 will now average 512 SD24B samples and subtract this value "
            "from subsequent current readings. Continue?"
        )
        if QMessageBox.question(self, "Start 0-offset calibration", confirm,
                                QMessageBox.Yes | QMessageBox.No, QMessageBox.No) != QMessageBox.Yes:
            self.set_status("0-offset calibration cancelled")
            return

        # Snapshot the latest shorted reading so the GUI can show the estimated
        # calibration amount. The current firmware replies with ACK only, so the
        # exact internal 512-sample average is not available to the PC.
        self.pending_zero_snapshot = dict(self.latest_data) if self.latest_data else None
        self.zero_offset_label.setText(self.format_zero_offset_from_data(self.pending_zero_snapshot))

        # Firmware command CMD_ZERO_OFFSET currently calibrates the current channel CH1.
        # Payload is uint16 sample_count; firmware limits it to 2048 samples.
        self.set_status("Running SD24B 0-offset calibration...")
        self.zero_btn.setEnabled(False)
        self.send(CMD_ZERO_OFFSET, struct.pack("<H", 512))

    def start_ca(self):
        self.current_mode = "ca"
        self.clear_data("ca")
        n = int(self.ca_avg_points.value())
        raw_interval = int(self.ca_interval.value())
        effective_ms = raw_interval * max(n, 1)
        if hasattr(self, "ca_status_label"):
            self.ca_status_label.setText(f"CA averaging {n} sample/point, effective interval {effective_ms} ms")
        payload = struct.pack(
            "<iIH",
            mv_i32(self.ca_potential.value()),
            int(self.ca_duration.value()) * 1000,
            raw_interval,
        )
        self.send(CMD_START_CA, payload)

    def start_cv(self):
        payload = struct.pack(
            "<iiiHH",
            mv_i32(self.cv_start.value()),
            mv_i32(self.cv_end.value()),
            mv_i32(self.cv_step.value()),
            int(self.cv_interval.value()),
            int(self.cv_cycles.value()),
        )
        self.current_mode = "cv"
        self.clear_data("cv")
        self.send(CMD_START_CV, payload)

    def pause_cv_lsv(self):
        # Firmware currently has no true resume command; pause is implemented by STOP.
        # Restart with Start CV / LSV when ready.
        self.stop_scan()
        self.set_status("CV / LSV paused by STOP; press Start CV / LSV to restart this scan.")

    def start_dpv(self):
        self._start_pulse(CMD_START_DPV)

    def start_swv(self):
        self._start_pulse(CMD_START_SWV)

    def _start_pulse(self, cmd):
        payload = struct.pack(
            "<iiiiH",
            mv_i32(self.pulse_start.value()),
            mv_i32(self.pulse_end.value()),
            mv_i32(self.pulse_step.value()),
            mv_i32(self.pulse_amp.value()),
            int(self.pulse_interval.value()),
        )
        self.current_mode = "pulse"
        self.clear_data("pulse")
        self.send(cmd, payload)

    def start_eis(self):
        payload = struct.pack(
            "<iiHI",
            mv_i32(self.eis_dc.value()),
            mv_i32(self.eis_amp.value()),
            int(self.eis_half_period.value()),
            int(self.eis_duration.value()) * 1000,
        )
        self.current_mode = "eis"
        self.clear_data("eis")
        self.send(CMD_START_EIS, payload)

    def return_auto_y(self):
        self.auto_y = True
        for plot in self.all_plots():
            plot.enableAutoRange(axis="y", enable=True)

    def all_plots(self):
        # Some code paths call all_plots() during startup before every tab
        # has been constructed. Collect only the plot widgets that already exist.
        plot_names = (
            "et_plot", "it_plot",
            "et_plot_ca", "it_plot_ca",
            "et_plot_cv", "it_plot_cv", "ie_plot",
            "pulse_plot", "pulse_et_plot", "pulse_it_plot",
            "eis_plot",
        )
        return [getattr(self, name) for name in plot_names if hasattr(self, name)]

    @Slot(dict)
    def on_data(self, d):
        d = dict(d)
        if "tia_mv" not in d:
            d["tia_mv"] = d.get("i_a", 0.0) * 1e9  # A -> mV equivalent at 1 MOhm
        raw_current_a = self.current_from_tia_mv(d["tia_mv"])
        d["i_a_raw"] = raw_current_a
        d["i_a_gain_cal"] = self.current_gain_corrected(raw_current_a)
        d["i_a"] = self.display_current(raw_current_a)
        d["gain_cal_enabled"] = self.current_gain_cal_enabled
        d["gain_factor"] = self.current_gain_factor
        self.latest_data = dict(d)
        t = d["t_ms"] / 1000.0

        if "temp_c" in d:
            self.env_label.setText(
                f"T: {d['temp_c']:.2f} °C   H: {d['humidity_pct']:.2f} %   P: {d['pressure_hpa']:.2f} hPa"
            )

        # Environment polling is display-only; it should not contaminate scan plots.
        if self.env_poll_pending and self.current_mode == "manual":
            self.env_poll_pending = False
        else:
            if self.current_mode == "ca":
                self.process_ca_sample(d)
            else:
                self.append_mode_data(self.current_mode, d)

        if hasattr(self, "raw_text") and hasattr(self, "show_raw_check") and self.show_raw_check.isChecked():
            self.raw_text.append(
                f"RX DATA mode={self.current_mode} t={t:.3f}s Eset={d.get('e_set_v', 0):+.6f}V "
                f"Emeas={d.get('e_meas_v', 0):+.6f}V I={d.get('i_a', 0)*1e9:+.6f}nA "
                f"raw={d.get('adc_raw', '')}"
            )
            if self.raw_text.document().blockCount() > 1000:
                cursor = self.raw_text.textCursor()
                cursor.movePosition(cursor.Start)
                cursor.select(cursor.BlockUnderCursor)
                cursor.removeSelectedText()
                cursor.deleteChar()

        if self.csv_writer:
            self.csv_writer.writerow([
                t,
                self.current_mode,
                d.get("source", ""),
                d["e_set_v"],
                d["e_meas_v"],
                d["i_a_raw"],
                d["i_a_gain_cal"],
                self.current_gain_factor,
                self.current_offset_a * 1e9,
                int(self.current_gain_cal_enabled),
                d.get("tia_mv", ""),
                self.get_rf_ohm(),
                d["adc_raw"],
                d.get("temp_c", ""),
                d.get("humidity_pct", ""),
                d.get("pressure_hpa", ""),
            ])

    @Slot(str)
    def on_text_line(self, text):
        if self.show_raw_check.isChecked():
            self.raw_text.append(text)
            # Keep raw terminal from becoming huge.
            if self.raw_text.document().blockCount() > 1000:
                cursor = self.raw_text.textCursor()
                cursor.movePosition(cursor.Start)
                cursor.select(cursor.BlockUnderCursor)
                cursor.removeSelectedText()
                cursor.deleteChar()

    @Slot(int)
    def on_ack(self, cmd):
        if cmd == CMD_ZERO_OFFSET:
            self.zero_btn.setEnabled(True)
            self.last_zero_offset = self.pending_zero_snapshot
            self.zero_offset_label.setText(self.format_zero_offset_from_data(self.last_zero_offset))
            self.set_status("SD24B 0-offset calibration finished")
            # Refresh the display with a new sample after calibration.
            QTimer.singleShot(100, lambda: self.send(CMD_READ_ADC))
            QMessageBox.information(
                self,
                "0-offset calibration finished",
                "Calibration is complete. You can remove the short now.\n\n"
                + self.format_zero_offset_from_data(self.last_zero_offset)
            )
        else:
            self.set_status(f"ACK from MSP430 for cmd 0x{cmd:02X}")

    @Slot(int)
    def on_error(self, cmd):
        if cmd == CMD_ZERO_OFFSET:
            self.zero_btn.setEnabled(True)
            self.zero_offset_label.setText("Zero offset: calibration failed")
            QMessageBox.critical(self, "0-offset calibration failed", "MSP430 returned an error for calibration.")
        self.set_status(f"MSP430 returned ERROR for cmd 0x{cmd:02X}")

    def disable_auto_y(self):
        self.auto_y = False

    def refresh_plots(self):
        # During startup Qt may trigger a refresh before every tab has created
        # its curve objects. Guard each plot so startup/order changes cannot crash.
        if not hasattr(self, "mode_buffers"):
            return

        m = self.mode_buffers
        b = m.get("manual", self.new_data_buffer())
        if hasattr(self, "et_curve"):
            self.et_curve.setData(b["t"], b["e_set"] if b["t"] else [])
        if hasattr(self, "it_curve"):
            self.it_curve.setData(b["t"], b["i"] if b["t"] else [])

        b = m.get("ca", self.new_data_buffer())
        if hasattr(self, "et_curve_ca"):
            self.et_curve_ca.setData(b["t"], b["e_set"])
        if hasattr(self, "it_curve_ca"):
            self.it_curve_ca.setData(b["t"], b["i"])

        b = m.get("cv", self.new_data_buffer())
        if hasattr(self, "et_curve_cv"):
            self.et_curve_cv.setData(b["t"], b["e_set"])
        if hasattr(self, "it_curve_cv"):
            self.it_curve_cv.setData(b["t"], b["i"])
        if hasattr(self, "ie_curve"):
            self.ie_curve.setData(b["e"], b["i"])

        b = m.get("pulse", self.new_data_buffer())
        if hasattr(self, "pulse_et_curve"):
            self.pulse_et_curve.setData(b["t"], b["e_set"])
        if hasattr(self, "pulse_it_curve"):
            self.pulse_it_curve.setData(b["t"], b["i"])
        if hasattr(self, "pulse_curve"):
            self.pulse_curve.setData(b["e"], b["i"])

        b = m.get("eis", self.new_data_buffer())
        if hasattr(self, "eis_curve"):
            self.eis_curve.setData(b["t"], b["i"])

        if self.auto_y:
            for plot in self.all_plots():
                plot.enableAutoRange(axis="y", enable=True)

    def toggle_csv(self):
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None
            self.save_btn.setText("Start CSV")
            return

        path, _ = QFileDialog.getSaveFileName(self, "Save CSV", "msp430_data.csv", "CSV Files (*.csv)")
        if not path:
            return

        self.csv_file = open(path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "time_s", "mode", "source", "e_set_v", "e_meas_v",
            "current_a", "current_gain_cal_a", "current_gain_factor", "current_offset_cal_na", "gain_cal_enabled",
            "tia_mv", "rf_ohm", "adc_raw", "temp_c", "humidity_pct", "pressure_hpa"
        ])
        self.save_btn.setText("Stop CSV")

    def set_status(self, text):
        self.status_label.setText(text)

    def closeEvent(self, event):
        self.worker.close_port()
        if self.csv_file:
            self.csv_file.close()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    pg.setConfigOptions(antialias=True)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())
