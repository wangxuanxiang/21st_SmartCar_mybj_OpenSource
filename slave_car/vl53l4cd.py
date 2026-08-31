# vl53l4cd.py
# MicroPython driver for VL53L4CD Time of Flight Distance Sensor
# Ported from Adafruit CircuitPython VL53L4CD driver

import struct
import time
from micropython import const

_VL53L4CD_SOFT_RESET = const(0x0000)
_VL53L4CD_I2C_SLAVE_DEVICE_ADDRESS = const(0x0001)
_VL53L4CD_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND = const(0x0008)
_VL53L4CD_XTALK_PLANE_OFFSET_KCPS = const(0x0016)
_VL53L4CD_XTALK_X_PLANE_GRADIENT_KCPS = const(0x0018)
_VL53L4CD_XTALK_Y_PLANE_GRADIENT_KCPS = const(0x001A)
_VL53L4CD_RANGE_OFFSET_MM = const(0x001E)
_VL53L4CD_INNER_OFFSET_MM = const(0x0020)
_VL53L4CD_OUTER_OFFSET_MM = const(0x0022)
_VL53L4CD_I2C_FAST_MODE_PLUS = const(0x002D)
_VL53L4CD_GPIO_HV_MUX_CTRL = const(0x0030)
_VL53L4CD_GPIO_TIO_HV_STATUS = const(0x0031)
_VL53L4CD_SYSTEM_INTERRUPT = const(0x0046)
_VL53L4CD_RANGE_CONFIG_A = const(0x005E)
_VL53L4CD_RANGE_CONFIG_B = const(0x0061)
_VL53L4CD_RANGE_CONFIG_SIGMA_THRESH = const(0x0064)
_VL53L4CD_MIN_COUNT_RATE_RTN_LIMIT_MCPS = const(0x0066)
_VL53L4CD_INTERMEASUREMENT_MS = const(0x006C)
_VL53L4CD_THRESH_HIGH = const(0x0072)
_VL53L4CD_THRESH_LOW = const(0x0074)
_VL53L4CD_SYSTEM_INTERRUPT_CLEAR = const(0x0086)
_VL53L4CD_SYSTEM_START = const(0x0087)
_VL53L4CD_RESULT_RANGE_STATUS = const(0x0089)
_VL53L4CD_RESULT_SPAD_NB = const(0x008C)
_VL53L4CD_RESULT_SIGNAL_RATE = const(0x008E)
_VL53L4CD_RESULT_AMBIENT_RATE = const(0x0090)
_VL53L4CD_RESULT_SIGMA = const(0x0092)
_VL53L4CD_RESULT_DISTANCE = const(0x0096)
_VL53L4CD_RESULT_OSC_CALIBRATE_VAL = const(0x00DE)
_VL53L4CD_FIRMWARE_SYSTEM_STATUS = const(0x00E5)
_VL53L4CD_IDENTIFICATION_MODEL_ID = const(0x010F)

RANGE_VALID = const(0x00)
RANGE_WARN_SIGMA_ABOVE = const(0x01)
RANGE_WARN_SIGMA_BELOW = const(0x02)
RANGE_ERROR_DISTANCE_BELOW_DETECTION_THRESHOLD = const(0x03)
RANGE_ERROR_INVALID_PHASE = const(0x04)
RANGE_ERROR_HW_FAIL = const(0x05)
RANGE_WARN_NO_WRAP_AROUND_CHECK = const(0x06)
RANGE_ERROR_WRAPPED_TARGET_PHASE_MISMATCH = const(0x07)
RANGE_ERROR_PROCESSING_FAIL = const(0x08)
RANGE_ERROR_CROSSTALK_FAIL = const(0x09)
RANGE_ERROR_INTERRUPT = const(0x0A)
RANGE_ERROR_MERGED_TARGET = const(0x0B)
RANGE_ERROR_SIGNAL_TOO_WEAK = const(0x0C)
RANGE_ERROR_OTHER = const(0xFF)

_RANGE_STATUS_MAP = (
    RANGE_ERROR_OTHER, RANGE_ERROR_OTHER, RANGE_ERROR_OTHER, RANGE_ERROR_HW_FAIL,
    RANGE_WARN_SIGMA_BELOW, RANGE_ERROR_INVALID_PHASE, RANGE_WARN_SIGMA_ABOVE,
    RANGE_ERROR_WRAPPED_TARGET_PHASE_MISMATCH, RANGE_ERROR_DISTANCE_BELOW_DETECTION_THRESHOLD,
    RANGE_VALID, RANGE_ERROR_OTHER, RANGE_ERROR_OTHER, RANGE_ERROR_CROSSTALK_FAIL,
    RANGE_ERROR_OTHER, RANGE_ERROR_OTHER, RANGE_ERROR_OTHER, RANGE_ERROR_OTHER,
    RANGE_ERROR_OTHER, RANGE_ERROR_INTERRUPT, RANGE_WARN_NO_WRAP_AROUND_CHECK,
    RANGE_ERROR_OTHER, RANGE_ERROR_OTHER, RANGE_ERROR_MERGED_TARGET, RANGE_ERROR_SIGNAL_TOO_WEAK,
)

class VL53L4CD:
    """MicroPython driver for the VL53L4CD distance sensor."""
    
    def __init__(self, i2c, address=0x29, i2c_retries=3):
        self._i2c = i2c
        self._address = address
        self._ranging = False
        self._interrupt_polarity_cache = None
        self._inter_measurement_cache = None
        # I2C 重试次数 应对激光脉冲电流导致的总线瞬态故障 (EIO)
        self._i2c_retries = i2c_retries
        
        # Check sensor ID
        model_id, module_type = self.model_info
        if model_id != 0xEB or module_type != 0xAA:
            raise RuntimeError("Wrong sensor ID or type!")
        
        self._sensor_init()
    
    def _sensor_init(self):
        init_seq = (
            b"\x12" b"\x00" b"\x00" b"\x11" b"\x02" b"\x00" b"\x02" b"\x08"
            b"\x00" b"\x08" b"\x10" b"\x01" b"\x01" b"\x00" b"\x00" b"\x00"
            b"\x00" b"\xff" b"\x00" b"\x0f" b"\x00" b"\x00" b"\x00" b"\x00"
            b"\x00" b"\x20" b"\x0b" b"\x00" b"\x00" b"\x02" b"\x14" b"\x21"
            b"\x00" b"\x00" b"\x05" b"\x00" b"\x00" b"\x00" b"\x00" b"\xc8"
            b"\x00" b"\x00" b"\x38" b"\xff" b"\x01" b"\x00" b"\x08" b"\x00"
            b"\x00" b"\x01" b"\xcc" b"\x07" b"\x01" b"\xf1" b"\x05" b"\x00"
            b"\xa0" b"\x00" b"\x80" b"\x08" b"\x38" b"\x00" b"\x00" b"\x00"
            b"\x00" b"\x0f" b"\x89" b"\x00" b"\x00" b"\x00" b"\x00" b"\x00"
            b"\x00" b"\x00" b"\x01" b"\x07" b"\x05" b"\x06" b"\x06" b"\x00"
            b"\x00" b"\x02" b"\xc7" b"\xff" b"\x9b" b"\x00" b"\x00" b"\x00"
            b"\x01" b"\x00" b"\x00"
        )
        
        self._wait_for_boot()
        self._write_register(0x002D, init_seq)
        self._interrupt_polarity_cache = None
        self._start_vhv()
        self.clear_interrupt()
        self.stop_ranging()
        self._write_register(_VL53L4CD_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, b"\x09")
        self._write_register(0x0B, b"\x00")
        self._write_register(0x0024, b"\x05\x00")
        self.inter_measurement = 0
        self.timing_budget = 10
    
    @property
    def model_info(self):
        """Return a tuple of (Model ID, Module Type)."""
        info = self._read_register(_VL53L4CD_IDENTIFICATION_MODEL_ID, 2)
        return info[0], info[1]
    
    @property
    def distance(self):
        """Return distance in centimeters."""
        dist = self._read_register(_VL53L4CD_RESULT_DISTANCE, 2)
        dist = struct.unpack(">H", dist)[0]
        return dist / 10.0
    
    @property
    def range_status(self):
        """Return measurement validity status."""
        status = self._read_register(_VL53L4CD_RESULT_RANGE_STATUS, 1)
        status = status[0] & 0x1F
        if status < 24:
            return _RANGE_STATUS_MAP[status]
        return RANGE_ERROR_OTHER

    def read_measurement(self):
        """Read range status and distance with one I2C transaction."""
        result = self._read_register(_VL53L4CD_RESULT_RANGE_STATUS, 15)
        status_index = result[0] & 0x1F
        status = _RANGE_STATUS_MAP[status_index] if status_index < 24 else RANGE_ERROR_OTHER
        distance_mm = (result[13] << 8) | result[14]
        return distance_mm / 10.0, status
    
    @property
    def sigma(self):
        """Return sigma estimator for noise in centimeters."""
        sigma = self._read_register(_VL53L4CD_RESULT_SIGMA, 2)
        sigma = struct.unpack(">H", sigma)[0]
        return sigma / 40.0
    
    @property
    def timing_budget(self):
        """Get ranging duration in milliseconds."""
        osc_freq_data = self._read_register(0x0006, 2)
        osc_freq = struct.unpack(">H", osc_freq_data)[0]
        
        macro_period_us = 16 * (int(2304 * (1073741824.0 / osc_freq)) >> 6)
        
        macrop_high_data = self._read_register(_VL53L4CD_RANGE_CONFIG_A, 2)
        macrop_high = struct.unpack(">H", macrop_high_data)[0]
        
        ls_byte = (macrop_high & 0x00FF) << 4
        ms_byte = (macrop_high & 0xFF00) >> 8
        ms_byte = 0x04 - (ms_byte - 1) - 1
        
        timing_budget_ms = (((ls_byte + 1) * (macro_period_us >> 6)) - ((macro_period_us >> 6) >> 1)) >> 12
        if ms_byte < 12:
            timing_budget_ms >>= ms_byte
        
        if self.inter_measurement == 0:
            timing_budget_ms += 2500
        else:
            timing_budget_ms *= 2
            timing_budget_ms += 4300
        
        return int(timing_budget_ms / 1000)
    
    @timing_budget.setter
    def timing_budget(self, val):
        if self._ranging:
            raise RuntimeError("Must stop ranging first.")
        
        if not 10 <= val <= 200:
            raise ValueError("Timing budget must be 10ms to 200ms.")
        
        inter_meas = self.inter_measurement
        if inter_meas != 0 and val > inter_meas:
            raise ValueError(f"Timing budget cannot be greater than inter-measurement period ({inter_meas})")
        
        osc_freq_data = self._read_register(0x0006, 2)
        osc_freq = struct.unpack(">H", osc_freq_data)[0]
        if osc_freq == 0:
            raise RuntimeError("Osc frequency is 0.")
        
        timing_budget_us = val * 1000
        macro_period_us = int(2304 * (1073741824.0 / osc_freq)) >> 6
        
        if inter_meas == 0:
            timing_budget_us -= 2500
        else:
            timing_budget_us -= 4300
            timing_budget_us //= 2
        
        # Configure RANGE_CONFIG_A
        ms_byte = 0
        timing_budget_us <<= 12
        tmp = macro_period_us * 16
        ls_byte = int(((timing_budget_us + ((tmp >> 6) >> 1)) / (tmp >> 6)) - 1)
        while (ls_byte >> 8) & 0xFFFFFF > 0:
            ls_byte >>= 1
            ms_byte += 1
        ms_byte = (ms_byte << 8) + (ls_byte & 0xFF)
        self._write_register(_VL53L4CD_RANGE_CONFIG_A, struct.pack(">H", ms_byte))
        
        # Configure RANGE_CONFIG_B
        ms_byte = 0
        tmp = macro_period_us * 12
        ls_byte = int(((timing_budget_us + ((tmp >> 6) >> 1)) / (tmp >> 6)) - 1)
        while (ls_byte >> 8) & 0xFFFFFF > 0:
            ls_byte >>= 1
            ms_byte += 1
        ms_byte = (ms_byte << 8) + (ls_byte & 0xFF)
        self._write_register(_VL53L4CD_RANGE_CONFIG_B, struct.pack(">H", ms_byte))
    
    @property
    def inter_measurement(self):
        """Get inter-measurement period in milliseconds."""
        reg_val_data = self._read_register(_VL53L4CD_INTERMEASUREMENT_MS, 4)
        reg_val = struct.unpack(">I", reg_val_data)[0]
        
        clock_pll_data = self._read_register(_VL53L4CD_RESULT_OSC_CALIBRATE_VAL, 2)
        clock_pll = struct.unpack(">H", clock_pll_data)[0] & 0x3FF
        clock_pll = int(1.065 * clock_pll)
        
        value = int(reg_val / clock_pll) if clock_pll != 0 else 0
        self._inter_measurement_cache = value
        return value
    
    @inter_measurement.setter
    def inter_measurement(self, val):
        if self._ranging:
            raise RuntimeError("Must stop ranging first.")
        
        timing_bud = self.timing_budget
        if val != 0 and val < timing_bud:
            raise ValueError(f"Inter-measurement period cannot be less than timing budget ({timing_bud})")
        
        clock_pll_data = self._read_register(_VL53L4CD_RESULT_OSC_CALIBRATE_VAL, 2)
        clock_pll = struct.unpack(">H", clock_pll_data)[0] & 0x3FF
        int_meas = int(1.055 * val * clock_pll)
        
        self._write_register(_VL53L4CD_INTERMEASUREMENT_MS, struct.pack(">I", int_meas))
        self._inter_measurement_cache = val
        self.timing_budget = timing_bud
    
    def start_ranging(self, wait=True):
        """Start ranging operation."""
        if self._ranging:
            return
        inter_measurement = self._inter_measurement_cache
        if inter_measurement is None:
            inter_measurement = self.inter_measurement
        if inter_measurement == 0:
            self._write_register(_VL53L4CD_SYSTEM_START, b"\x21")  # Continuous mode
        else:
            self._write_register(_VL53L4CD_SYSTEM_START, b"\x40")  # Autonomous mode
        
        self._ranging = True
        if not wait:
            return

        # Initialization may wait; runtime callers should use wait=False.
        for _ in range(1000):
            if self.data_ready:
                break
            time.sleep_ms(1)
        else:
            raise TimeoutError("Timeout waiting for data ready.")

        self.clear_interrupt()
    
    def stop_ranging(self):
        """Stop ranging operation."""
        if not self._ranging:
            return
        self._write_register(_VL53L4CD_SYSTEM_START, b"\x00")
        self._ranging = False
    
    def clear_interrupt(self):
        """Clear new data interrupt."""
        self._write_register(_VL53L4CD_SYSTEM_INTERRUPT_CLEAR, b"\x01")
    
    @property
    def data_ready(self):
        """Check if new data is ready."""
        status_data = self._read_register(_VL53L4CD_GPIO_TIO_HV_STATUS, 1)
        return (status_data[0] & 0x01) == self._interrupt_polarity
    
    @property
    def _interrupt_polarity(self):
        if self._interrupt_polarity_cache is None:
            int_pol_data = self._read_register(_VL53L4CD_GPIO_HV_MUX_CTRL, 1)
            int_pol = (int_pol_data[0] & 0x10) >> 4
            self._interrupt_polarity_cache = 0 if int_pol else 1
        return self._interrupt_polarity_cache
    
    def _wait_for_boot(self):
        for _ in range(1000):
            status_data = self._read_register(_VL53L4CD_FIRMWARE_SYSTEM_STATUS, 1)
            if status_data[0] == 0x03:
                return
            time.sleep_ms(1)
        raise TimeoutError("Timeout waiting for system boot.")
    
    def _start_vhv(self):
        self.start_ranging()
        for _ in range(1000):
            if self.data_ready:
                return
            time.sleep_ms(1)
        raise TimeoutError("Timeout starting VHV.")
    
    def _write_register(self, address, data):
        """Write to sensor register (retry on transient I2C errors)."""
        if isinstance(data, int):
            data = bytes([data])
        elif isinstance(data, (bytes, bytearray)):
            pass
        else:
            data = bytes(data)

        # Write address followed by data
        for attempt in range(self._i2c_retries):
            try:
                self._i2c.writeto_mem(self._address, address, data, addrsize=16)
                return
            except OSError:
                if attempt == self._i2c_retries - 1:
                    raise
                time.sleep_ms(1)

    def _read_register(self, address, length=1):
        """Read from sensor register (retry on transient I2C errors)."""
        for attempt in range(self._i2c_retries):
            try:
                return self._i2c.readfrom_mem(self._address, address, length, addrsize=16)
            except OSError:
                if attempt == self._i2c_retries - 1:
                    raise
                time.sleep_ms(1)
    
    def set_address(self, new_address):
        """Set new I2C address for the sensor."""
        self._write_register(_VL53L4CD_I2C_SLAVE_DEVICE_ADDRESS, struct.pack(">B", new_address))
        self._address = new_address
