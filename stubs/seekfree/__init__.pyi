from typing import Any, List, Optional, Union

__all__ = [
    "MOTOR_CONTROLLER",
    "BLDC_CONTROLLER",
    "KEY_HANDLER",
    "IMU660RX",
    "IMU963RX",
    "DL1X",
    "TSL1401",
    "WIRELESS_UART",
    "WIFI_SPI",
]

class MOTOR_CONTROLLER:
    """直流电机控制器。"""

    PWM_C30_DIR_C31: int
    PWM_C28_DIR_C29: int
    PWM_D4_DIR_D5: int
    PWM_D6_DIR_D7: int
    PWM_C30_PWM_C31: int
    PWM_C28_PWM_C29: int
    PWM_D4_PWM_D5: int
    PWM_D6_PWM_D7: int

    def __init__(
        self, index: int, freq: int, duty: int = 0, invert: bool = False
    ) -> None:
        """
        Args:
            index: PWM 通道/引脚配置索引 (使用常量)
            freq: PWM 频率
            duty: 初始占空比
            invert: 是否反转方向
        """
        ...

    def duty(self, duty: Optional[int] = None) -> int:
        """
        设置或获取占空比。

        Args:
            duty: 占空比 (-10000 到 10000)
        """
        ...

    def info(self) -> None: ...
    @staticmethod
    def help() -> None: ...

class BLDC_CONTROLLER:
    """无刷电机控制器。"""

    PWM_C25: int
    PWM_C27: int

    def __init__(self, index: int, freq: int = 50, highlevel_us: int = 1000) -> None:
        """
        Args:
            index: 引脚索引
            freq: 频率
            highlevel_us: 高电平时间 (us)
        """
        ...

    def highlevel_us(self, us: Optional[int] = None) -> int:
        """设置或获取高电平时间。"""
        ...

    def info(self) -> None: ...
    @staticmethod
    def help() -> None: ...

class KEY_HANDLER:
    """按键处理。"""

    def __init__(self, period: int) -> None:
        """
        Args:
            period: 扫描周期
        """
        ...

    def capture(self) -> None:
        """扫描按键（在定时器中调用）。"""
        ...

    def get(self) -> List[int]:
        """
        获取按键状态。
        Returns: [key1_state, key2_state, ...] 0:无, 1:短按, 2:长按
        """
        ...

    def clear(self, index: int) -> None:
        """
        清除按键状态。
        Args:
            index: 按键索引 (1-based)
        """
        ...

    def info(self) -> None: ...
    @staticmethod
    def help() -> None: ...

class IMU660RX:
    """6轴 IMU 传感器 (LSM6DSO)。"""

    def __init__(self, capture_div: int = 1) -> None:
        """
        Args:
            capture_div: 采集分频
        """
        ...

    def capture(self) -> None:
        """采集数据。"""
        ...

    def get(self) -> List[int]:
        """
        获取数据。
        Returns: [acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z]
        """
        ...

    def read(self) -> List[int]: ...
    def info(self) -> None: ...
    @staticmethod
    def help() -> None: ...

class IMU963RX:
    """9轴 IMU 传感器。"""

    def __init__(self, capture_div: int = 1) -> None: ...
    def capture(self) -> None: ...
    def get(self) -> List[int]:
        """
        Returns: [acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z]
        """
        ...

    def read(self) -> List[int]: ...
    def info(self) -> None: ...
    @staticmethod
    def help() -> None: ...

class DL1X:
    """ToF 测距传感器。"""

    def __init__(self, capture_div: int = 1) -> None: ...
    def capture(self) -> None: ...
    def get(self) -> int:
        """Returns: 距离 (mm)"""
        ...

    def read(self) -> int: ...
    def info(self) -> None: ...
    @staticmethod
    def help() -> None: ...

class TSL1401:
    """线性 CCD 传感器。"""

    RES_8BIT: int
    RES_12BIT: int

    def __init__(self, capture_div: int = 1) -> None: ...
    def set_resolution(self, resolution: int) -> None: ...
    def capture(self) -> None: ...
    def get(self, index: int) -> List[int]:
        """
        获取指定 CCD 通道的数据引用。
        Args:
            index: CCD 通道索引 [0-3]
        Returns: 像素列表
        """
        ...

    def read(self) -> List[int]: ...
    def info(self) -> None: ...
    @staticmethod
    def help() -> None: ...

class WIRELESS_UART:
    """无线串口模块。"""

    CCD1_BUFFER_INDEX: int
    CCD2_BUFFER_INDEX: int
    CCD3_BUFFER_INDEX: int
    CCD4_BUFFER_INDEX: int
    CCD1_2_BUFFER_INDEX: int
    CCD3_4_BUFFER_INDEX: int

    def __init__(self, baudrate: int = 460800) -> None: ...
    def send_str(self, s: str) -> None: ...
    def send_oscilloscope(
        self,
        d1: float,
        d2: float = 0,
        d3: float = 0,
        d4: float = 0,
        d5: float = 0,
        d6: float = 0,
        d7: float = 0,
        d8: float = 0,
    ) -> None:
        """发送虚拟示波器数据。"""
        ...

    def send_ccd_image(self, index: int) -> None: ...
    def data_analysis(self) -> List[int]:
        """解析接收数据。"""
        ...

    def get_data(self, index: int) -> float: ...
    def info(self) -> None: ...
    @staticmethod
    def help() -> None: ...

class WIFI_SPI:
    """SPI 接口 Wi-Fi 模块。"""

    TCP_CONNECT: int
    UDP_CONNECT: int
    CCD1_BUFFER_INDEX: int
    CCD2_BUFFER_INDEX: int
    CCD3_BUFFER_INDEX: int
    CCD4_BUFFER_INDEX: int
    CCD1_2_BUFFER_INDEX: int
    CCD3_4_BUFFER_INDEX: int

    def __init__(
        self, ssid: str, password: str, type: int, ip: str, port: str
    ) -> None: ...
    def send_str(self, s: str) -> None: ...
    def send_oscilloscope(
        self,
        d1: float,
        d2: float = 0,
        d3: float = 0,
        d4: float = 0,
        d5: float = 0,
        d6: float = 0,
        d7: float = 0,
        d8: float = 0,
    ) -> None: ...
    def send_ccd_image(self, index: int) -> None: ...
    def data_analysis(self) -> List[int]: ...
    def get_data(self, index: int) -> float:
        """
        获取调参数据。

        Args:
            index: 数据通道索引 [0-7]
        Returns:
            float: 接收到的浮点数值
        """
        ...

    def info(self) -> None: ...
    @staticmethod
    def help() -> None: ...
