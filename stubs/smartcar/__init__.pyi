from typing import Any, Callable, List, Optional, Union
from machine import Pin

__all__ = ["ticker", "ADC_Group", "encoder"]

class ticker:
    """硬件定时器中断，用于周期任务。"""

    def __init__(self, id: int) -> None:
        """
        Args:
            id:  Ticker ID [0-3]
        """
        ...

    def start(self, period_ms: int) -> None:
        """
        启动 Ticker。

        Args:
            period_ms: 周期 (ms)
        """
        ...

    def stop(self) -> None:
        """停止 Ticker。"""
        ...

    def callback(self, handler: Callable[["ticker"], None]) -> None:
        """
        设置回调函数。

        Args:
            handler: 回调函数，接受 ticker 实例作为参数
        """
        ...

    def capture_list(self, *modules: Any) -> None:
        """
        设置需要周期性捕获数据的模块列表。

        Args:
            *modules: 传感器或控制器实例
        """
        ...

class ADC_Group:
    """ADC 组采样，用于电磁传感器等。"""

    # 周期常量
    PMODE0: int
    PMODE1: int
    PMODE2: int
    PMODE3: int

    # 平均次数常量
    AVG1: int
    AVG4: int
    AVG8: int
    AVG16: int
    AVG32: int

    def __init__(self, id: int) -> None:
        """
        Args:
            id: use ADC 1, 2 etc.
        """
        ...

    def init(self, id: int, period: int = PMODE3, average: int = AVG16) -> None: ...
    def addch(self, pin: str) -> None:
        """
        添加 ADC 通道。

        Args:
            pin: 引脚名称
        """
        ...

    def capture(self) -> None:
        """触发采集。"""
        ...

    def get(self) -> List[int]:
        """获取采集结果。"""
        ...

    def read(self) -> List[int]:
        """采集并获取结果。"""
        ...

class encoder:
    """正交编码器接口。"""

    def __init__(self, phaseA: str, phaseB: str, invert: bool = False) -> None:
        """
        Args:
            phaseA: A 相引脚
            phaseB: B 相引脚
            invert: 是否反转方向
        """
        ...

    def capture(self) -> None:
        """捕获计数（通常在 ticker 中调用）。"""
        ...

    def get(self) -> int:
        """获取脉冲计数。"""
        ...

    def read(self) -> int:
        """获取脉冲计数（直接读取）。"""
        ...
