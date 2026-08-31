from typing import List, Tuple, Union, Any, Iterator

__all__ = [
    "chdir",
    "getcwd",
    "listdir",
    "mkdir",
    "remove",
    "rmdir",
    "rename",
    "stat",
    "statvfs",
    "sync",
    "urandom",
    "dupterm",
    "mount",
    "umount",
    "uname",
]

def chdir(path: str) -> None:
    """
    改变当前工作目录。

    Args:
        path: 目标目录路径
    """
    ...

def getcwd() -> str:
    """
    获取当前工作目录。

    Returns:
        str: 当前工作目录路径
    """
    ...

def listdir(dir: str = ".") -> List[str]:
    """
    列出指定目录下的文件和文件夹。

    Args:
        dir: 目录路径，默认为当前目录

    Returns:
        List[str]: 文件名列表
    """
    ...

def mkdir(path: str) -> None:
    """
    创建一个新目录。

    Args:
        path: 新目录的路径
    """
    ...

def remove(path: str) -> None:
    """
    删除一个文件。

    Args:
        path: 文件路径
    """
    ...

def rmdir(path: str) -> None:
    """
    删除一个目录。目录必须为空。

    Args:
        path: 目录路径
    """
    ...

def rename(old_path: str, new_path: str) -> None:
    """
    重命名文件或目录。

    Args:
        old_path: 原路径
        new_path: 新路径
    """
    ...

def stat(path: str) -> Tuple[int, int, int, int, int, int, int, int, int, int]:
    """
    获取文件或目录的状态信息。

    Args:
        path: 路径

    Returns:
        tuple: (mode, ino, dev, nlink, uid, gid, size, atime, mtime, ctime)
    """
    ...

def statvfs(path: str) -> Tuple[int, int, int, int, int, int, int, int, int, int]:
    """
    获取文件系统的状态信息。

    Args:
        path: 文件系统路径

    Returns:
        tuple: (bsize, frsize, blocks, bfree, bavail, files, ffree, favail, flag, namemax)
    """
    ...

def sync() -> None:
    """
    同步所有文件系统。
    """
    ...

def urandom(n: int) -> bytes:
    """
    生成 n 个字节的随机数据。

    Args:
        n: 字节数

    Returns:
        bytes: 随机字节串
    """
    ...

def dupterm(stream_object: Any, index: int = 0) -> Any:
    """
    复制或切换 UART 到标准输入/输出。
    """
    ...

def mount(fsobj: Any, mount_point: str, *, readonly: bool = False) -> Any:
    """
    挂载文件系统。

    Args:
        fsobj: 文件系统对象
        mount_point: 挂载点路径
        readonly: 是否只读
    """
    ...

def umount(mount_point: str) -> None:
    """
    卸载文件系统。

    Args:
        mount_point: 挂载点路径
    """
    ...

def uname() -> Tuple[str, str, str, str, str]:
    """
    获取系统信息。

    Returns:
        tuple: (sysname, nodename, release, version, machine)
    """
    ...
