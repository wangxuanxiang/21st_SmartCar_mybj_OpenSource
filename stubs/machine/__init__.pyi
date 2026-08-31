from typing import Any, Callable, Optional, Sequence, Union, List
import os as os

__all__ = [
    "execfile",
    "reset",
    "soft_reset",
    "freq",
    "unique_id",
    "idle",
    "disable_irq",
    "enable_irq",
    "Pin",
    "ADC",
    "PWM",
    "UART",
    "SPI",
    "I2C",
    "SoftI2C",
    "os",
]

"""
Machine 模块包含与特定硬件相关的函数。
"""

# --- Global Functions in this port ---
def execfile(filename: str) -> None:
    """
    执行一个 Python 脚本文件。

    Args:
        filename: 脚本文件路径
    """
    ...

def reset() -> None:
    """复位设备。"""
    ...

def soft_reset() -> None:
    """执行软复位。"""
    ...

def freq() -> int:
    """获取 CPU 频率。"""
    ...

def unique_id() -> bytes:
    """获取设备唯一 ID。"""
    ...

def idle() -> None:
    """使 CPU 进入空闲状态。"""
    ...

def disable_irq() -> int:
    """
    禁用中断。

    Returns:
        int: 先前的中断状态
    """
    ...

def enable_irq(state: int) -> None:
    """
    启用中断。

    Args:
        state: 先前的中断状态
    """
    ...

# --- Pin ---
class Pin:
    """
    GPIO 引脚控制类。
    """

    IN: int
    OUT: int
    OPEN_DRAIN: int
    PULL_UP: int
    PULL_UP_47K: int  # Custom
    PULL_UP_22K: int  # Custom
    PULL_DOWN: int
    PULL_HOLD: int
    DRIVE_OFF: int
    DRIVE_0: int
    DRIVE_1: int
    DRIVE_2: int
    DRIVE_3: int
    DRIVE_4: int
    DRIVE_5: int
    DRIVE_6: int
    IRQ_RISING: int
    IRQ_FALLING: int

    def __init__(
        self,
        id: Union[str, int],
        mode: int,
        pull: int = -1,
        value: Any = None,
        drive: int = 0,
        alt: int = -1,
    ) -> None:
        """
        初始化 Pin 对象。

        Args:
            id: 引脚编号或名称
            mode: 输入/输出模式 (IN, OUT, OPEN_DRAIN)
            pull: 上拉/下拉电阻配置 (PULL_UP, PULL_DOWN, None)
            value: 初始输出值
            drive: 驱动能力
            alt: 复用功能
        """
        ...

    def init(
        self,
        mode: int,
        pull: int = -1,
        value: Any = None,
        drive: int = 0,
        alt: int = -1,
    ) -> None:
        """
        重新初始化引脚。
        """
        ...

    def value(self, x: Optional[Union[int, bool]] = None) -> int:
        """
        获取或设置引脚电平。

        Args:
            x: 如果提供，设置引脚电平；否则返回当前电平。
        """
        ...

    def on(self) -> None:
        """设置引脚为高电平。"""
        ...

    def off(self) -> None:
        """设置引脚为低电平。"""
        ...

    def high(self) -> None:
        """设置引脚为高电平。"""
        ...

    def low(self) -> None:
        """设置引脚为低电平。"""
        ...

    def toggle(self) -> None:
        """翻转引脚电平。"""
        ...

    def irq(
        self,
        handler: Optional[Callable[[Any], Any]] = None,
        trigger: int = 3,
        hard: bool = False,
    ) -> None:
        """
        配置外部中断。

        Args:
            handler: 中断处理函数
            trigger: 触发方式 (IRQ_RISING, IRQ_FALLING)
            hard: 是否使用硬件中断
        """
        ...

# --- ADC ---
class ADC:
    """
    模数转换器 (ADC) 类。
    """

    def __init__(self, pin: Union[int, str, Pin]) -> None:
        """
        初始化 ADC。

        Args:
            pin: Pin 对象或引脚 ID
        """
        ...

    def read_u16(self) -> int:
        """
        读取 16 位无符号整数形式的模拟值 (0-65535)。
        """
        ...

# --- PWM ---
class PWM:
    """
    脉冲宽度调制 (PWM) 类。
    """

    def __init__(
        self, pin: Union[int, str, Pin], freq: int = 0, duty_u16: int = 0
    ) -> None:
        """
        初始化 PWM。

        Args:
            pin: Pin 对象或引脚 ID
            freq: 频率 (Hz)
            duty_u16: 占空比 (0-65535)
        """
        ...

    def init(self, freq: int, duty_u16: int) -> None:
        """重新初始化 PWM 参数。"""
        ...

    def deinit(self) -> None:
        """关闭 PWM。"""
        ...

    def freq(self, f: Optional[int] = None) -> int:
        """设置或获取频率。"""
        ...

    def duty_u16(self, d: Optional[int] = None) -> int:
        """设置或获取 16 位占空比。"""
        ...

# --- UART ---
class UART:
    """
    通用异步收发传输器 (UART) 类。
    """

    def __init__(
        self,
        id: int,
        baudrate: int = 9600,
        bits: int = 8,
        parity: Optional[int] = None,
        stop: int = 1,
        timeout: int = 0,
    ) -> None:
        """
        初始化 UART。

        Args:
            id: UART ID
            baudrate: 波特率
            bits: 数据位
            parity: 校验位 (None, 0=even, 1=odd)
            stop: 停止位
            timeout: 超时时间 (ms)
        """
        ...

    def init(
        self,
        baudrate: int = 9600,
        bits: int = 8,
        parity: Optional[int] = None,
        stop: int = 1,
    ) -> None:
        """重新初始化 UART 参数。"""
        ...

    def read(self, nbytes: Optional[int] = None) -> bytes:
        """
        读取数据。

        Args:
            nbytes: 读取字节数，如果为 None 则读取所有可用数据。
        """
        ...

    def readinto(
        self, buf: Union[bytearray, memoryview], nbytes: Optional[int] = None
    ) -> Optional[int]:
        """
        读取数据到缓冲区。
        """
        ...

    def readline(self) -> Optional[bytes]:
        """读取一行数据。"""
        ...

    def write(self, buf: Union[bytes, bytearray, memoryview, str]) -> Optional[int]:
        """
        写入数据。

        Args:
            buf: 要写入的数据 (bytes 或 str)
        """
        ...

    def any(self) -> int:
        """检查是否有待读取的数据。"""
        ...

# --- SPI ---
class SPI:
    """
    串行外设接口 (SPI) 类。
    """

    def __init__(
        self,
        id: int,
        baudrate: int = 1000000,
        polarity: int = 0,
        phase: int = 0,
        bits: int = 8,
        firstbit: int = 0,
        sck: Optional[Pin] = None,
        mosi: Optional[Pin] = None,
        miso: Optional[Pin] = None,
    ) -> None:
        """
        初始化 SPI。
        """
        ...

    def init(
        self,
        baudrate: int = 1000000,
        polarity: int = 0,
        phase: int = 0,
        bits: int = 8,
        firstbit: int = 0,
    ) -> None:
        """重新初始化 SPI 参数。"""
        ...

    def read(self, nbytes: int, write: int = 0x00) -> bytes:
        """
        读取数据。

        Args:
            nbytes: 读取字节数
            write: 读取时发送的填充字节
        """
        ...

    def readinto(
        self, buf: Union[bytearray, memoryview], write: int = 0x00
    ) -> Optional[int]:
        """读取数据到缓冲区。"""
        ...

    def write(self, buf: Union[bytes, bytearray, memoryview]) -> Optional[int]:
        """写入数据。"""
        ...

    def write_readinto(
        self,
        write_buf: Union[bytes, bytearray, memoryview],
        read_buf: Union[bytearray, memoryview],
    ) -> Optional[int]:
        """同时写入和读取数据。"""
        ...

# --- I2C ---
class I2C:
    """
    I2C 总线类。
    """

    def __init__(
        self,
        id: int,
        scl: Optional[Pin] = None,
        sda: Optional[Pin] = None,
        freq: int = 400000,
    ) -> None:
        """
        初始化 I2C。

        Args:
            id: I2C ID (-1 为软件 I2C)
            scl: SCL 引脚
            sda: SDA 引脚
            freq: 频率
        """
        ...

    def init(
        self, scl: Optional[Pin] = None, sda: Optional[Pin] = None, freq: int = 400000
    ) -> None:
        """重新初始化 I2C 参数。"""
        ...

    def scan(self) -> List[int]:
        """
        扫描总线上的 I2C 设备。

        Returns:
            List[int]: 设备地址列表
        """
        ...

    def readfrom(self, addr: int, nbytes: int, stop: bool = True) -> bytes:
        """
        从指定地址读取数据。

        Args:
            addr: 设备地址
            nbytes: 读取字节数
            stop: 是否发送停止信号
        """
        ...

    def readfrom_into(
        self, addr: int, buf: Union[bytearray, memoryview], stop: bool = True
    ) -> None:
        """从指定地址读取数据到缓冲区。"""
        ...

    def writeto(
        self, addr: int, buf: Union[bytes, bytearray], stop: bool = True
    ) -> int:
        """
        向指定地址写入数据。

        Args:
            addr: 设备地址
            buf: 数据
            stop: 是否发送停止信号
        """
        ...

    def writevto(
        self, addr: int, vector: Sequence[Union[bytes, bytearray]], stop: bool = True
    ) -> int:
        """向指定地址写入向量数据。"""
        ...

# --- SoftI2C ---
class SoftI2C(I2C):
    """
    基于 GPIO 翻转的软件 I2C 实现。
    """

    def __init__(
        self, scl: Pin, sda: Pin, freq: int = 400000, timeout: int = 255
    ) -> None:
        """
        初始化软件 I2C。

        Args:
            scl: SCL 引脚
            sda: SDA 引脚
            freq: 频率
            timeout: 超时时间
        """
        ...
