from typing import Optional

__all__ = [
    "collect",
    "disable",
    "enable",
    "isenabled",
    "mem_alloc",
    "mem_free",
    "threshold",
]

def collect() -> None:
    """
    立即执行垃圾回收，回收不再被引用的对象。

    MicroPython 中用于释放循环引用等引用计数无法回收的对象，
    并合并相邻空闲块。
    """
    ...

def disable() -> None:
    """
    禁用自动垃圾回收。内存耗尽时仍会触发自动回收。
    """
    ...

def enable() -> None:
    """
    启用自动垃圾回收。
    """
    ...

def isenabled() -> bool:
    """
    查询自动垃圾回收是否启用。

    Returns:
        bool: True 表示已启用。
    """
    ...

def mem_alloc() -> int:
    """
    获取当前已分配的堆内存字节数。

    Returns:
        int: 已分配字节数。
    """
    ...

def mem_free() -> int:
    """
    获取当前空闲的堆内存字节数。

    Returns:
        int: 空闲字节数。
    """
    ...

def threshold(amount: Optional[int] = None) -> int:
    """
    查询或设置自动垃圾回收触发阈值。

    Args:
        amount: 新的阈值（字节）。若为 None 则只查询。

    Returns:
        int: 当前阈值。
    """
    ...
