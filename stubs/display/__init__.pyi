from typing import Any, List, Optional, Sequence
from machine import Pin

__all__ = ["LCD_Drv", "LCD"]

class LCD_Drv:
    """LCD 驱动接口，负责底层 SPI 通信配置。"""

    LCD200_TYPE: int  #: IPS200 屏幕类型

    def __init__(
        self, SPI_INDEX: int, BAUDRATE: int, DC_PIN: Pin, RST_PIN: Pin, LCD_TYPE: int
    ) -> None:
        """
        Args:
            SPI_INDEX: SPI 接口索引
            BAUDRATE: SPI 速率 (e.g. 60000000)
            DC_PIN: 命令/数据选择引脚
            RST_PIN: 复位引脚
            LCD_TYPE: 屏幕类型常量
        """
        ...

class LCD:
    """LCD 显示操作类。"""

    def __init__(self, drv: LCD_Drv) -> None:
        """
        Args:
            drv: LCD_Drv 驱动实例
        """
        ...

    def color(self, pcolor: int, bgcolor: int) -> None:
        """
        设置全局前景色和背景色。

        Args:
            pcolor: 前景色 (RGB565)
            bgcolor: 背景色 (RGB565)
        """
        ...

    def mode(self, dir: int) -> None:
        """
        设置显示方向。

        Args:
            dir: 0:竖屏, 1:横屏, 2:竖屏180度, 3:横屏180度
        """
        ...

    def clear(self, color: Optional[int] = None) -> None:
        """
        清屏。

        Args:
            color: 可选，清屏颜色。如果不填使用背景色。
        """
        ...

    def str12(self, x: int, y: int, str: str, color: Optional[int] = None) -> None:
        """
        显示 12 号字体字符串。
        """
        ...

    def str16(self, x: int, y: int, str: str, color: Optional[int] = None) -> None:
        """
        显示 16 号字体字符串。
        """
        ...

    def str24(self, x: int, y: int, str: str, color: Optional[int] = None) -> None:
        """
        显示 24 号字体字符串。
        """
        ...

    def str32(self, x: int, y: int, str: str, color: Optional[int] = None) -> None:
        """
        显示 32 号字体字符串。
        """
        ...

    def line(
        self, x1: int, y1: int, x2: int, y2: int, color: int, thick: int = 1
    ) -> None:
        """
        画线。
        Args:
            x1, y1: 起点坐标
            x2, y2: 终点坐标
            color: 颜色
            thick: 线宽
        """
        ...

    def wave(
        self, x: int, y: int, width: int, height: int, data: Sequence[int], max: int
    ) -> None:
        """
        显示波形。

        Args:
            x: 起始显示 X 坐标
            y: 起始显示 Y 坐标
            width: 数据显示宽度
            height: 数据显示高度
            data: 数据对象 (List[int])
            max: 数据最大值 (归一化用)
        """
        ...
