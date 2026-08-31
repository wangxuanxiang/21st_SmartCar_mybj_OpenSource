from typing import Union, Optional

__all__ = [
    "localtime",
    "mktime",
    "sleep",
    "sleep_ms",
    "sleep_us",
    "ticks_ms",
    "ticks_us",
    "ticks_cpu",
    "ticks_add",
    "ticks_diff",
    "time",
]

def localtime(secs: Optional[int] = None) -> tuple:
    """
    将时间戳转换为本地时间元组。

    Args:
        secs: 时间戳（秒）。如果未提供，则使用当前时间。

    Returns:
        tuple: (year, month, mday, hour, minute, second, weekday, yearday)
    """
    ...

def mktime(t: tuple) -> int:
    """
    将本地时间元组转换为时间戳。

    Args:
        t: 时间元组 (year, month, mday, hour, minute, second, weekday, yearday)

    Returns:
        int: 时间戳（秒）
    """
    ...

def sleep(seconds: Union[int, float]) -> None:
    """
    休眠指定的秒数。

    Args:
        seconds: 休眠时间（秒），可以是浮点数。
    """
    ...

def sleep_ms(ms: int) -> None:
    """
    休眠指定的毫秒数。

    Args:
        ms: 休眠时间（毫秒）。
    """
    ...

def sleep_us(us: int) -> None:
    """
    休眠指定的微秒数。

    Args:
        us: 休眠时间（微秒）。
    """
    ...

def ticks_ms() -> int:
    """
    获取毫秒级计数器值。
    值的范围是未定义的，主要用于计算时间差。

    Returns:
        int: 当前毫秒计数。
    """
    ...

def ticks_us() -> int:
    """
    获取微秒级计数器值。
    值的范围是未定义的，主要用于计算时间差。

    Returns:
        int: 当前微秒计数。
    """
    ...

def ticks_cpu() -> int:
    """
    获取 CPU 周期计数器值。
    具体的频率取决于硬件。

    Returns:
        int: 当前 CPU 周期计数。
    """
    ...

def ticks_add(ticks: int, delta: int) -> int:
    """
    计算 ticks 加上 delta 后的值，处理溢出。

    Args:
        ticks: 基准 tick 值
        delta: 增加的 tick 值（可以是负数）

    Returns:
        int: 新的 tick 值
    """
    ...

def ticks_diff(ticks1: int, ticks2: int) -> int:
    """
    计算两个 tick 值之间的差值 (ticks1 - ticks2)，处理溢出。

    Args:
        ticks1: 结束 tick 值
        ticks2: 开始 tick 值

    Returns:
        int: 时间差
    """
    ...

def time() -> int:
    """
    获取当前时间戳（秒）。

    Returns:
        int: 自 Epoch 以来的秒数。
    """
    ...
