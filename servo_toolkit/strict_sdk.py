"""仅为本工具的单播请求约束接收事务；不修改 vendored 上游 SDK。"""
from __future__ import annotations

import threading
import time

from .vendor.scservo_sdk import (
    COMM_NOT_AVAILABLE, COMM_PORT_BUSY, COMM_RX_CORRUPT, COMM_RX_TIMEOUT,
    COMM_SUCCESS, COMM_TX_FAIL, hls,
)

# 所有 StrictHls 实例在同一 PortHandler 上共享锁。其他绕过本包装的访问不受保护。
_PORT_LOCK_SETUP = threading.Lock()


class StrictHls(hls):
    """PING/READ/WRITE/0x0B 单播应答；未验证真实硬件和 USB 回显。"""

    def __init__(self, port, *, timeout_seconds=0.2):
        super().__init__(port)
        if timeout_seconds <= 0:
            raise ValueError("timeout_seconds must be positive")
        self.timeout_seconds = timeout_seconds
        with _PORT_LOCK_SETUP:
            if not hasattr(port, "_microduck_transaction_lock"):
                port._microduck_transaction_lock = threading.Lock()
            self._transaction_lock = port._microduck_transaction_lock

    def txRxPacket(self, txpacket):
        # 严格限定本工具所需的单播帧；错误格式不发送。
        if (len(txpacket) < 6 or any(not isinstance(v, int) or not 0 <= v <= 255 for v in txpacket)
                or txpacket[2] > 253 or txpacket[3] + 4 != len(txpacket)):
            return None, COMM_NOT_AVAILABLE, 0
        inst = txpacket[4]
        # 允许的帧长：PING 6；READ 8；WRITE 1 字节 8 或 2 字节 9；0x0B 8。
        # 2 字节写只放开目标位置寄存器 42：拆成两次 1 字节写会出现中间目标值。
        write_len_ok = len(txpacket) == 8 or (len(txpacket) == 9 and txpacket[5] == 42)
        if (inst == 0x01 and len(txpacket) != 6
                or inst == 0x02 and (len(txpacket) != 8 or txpacket[6] not in (1, 2))
                or inst == 0x03 and not write_len_ok
                or inst == 0x0B and len(txpacket) != 8
                or inst not in (0x01, 0x02, 0x03, 0x0B)):
            return None, COMM_NOT_AVAILABLE, 0
        expected_data = txpacket[6] if inst == 0x02 else 0
        # 仅改主 ID 的写指令允许由旧 ID 或新 ID 返回 ACK；两者仍需新 ID 回读验证。
        valid_ids = {txpacket[2]}
        if inst == 0x03 and txpacket[5] == 5 and txpacket[6] <= 253:
            valid_ids.add(txpacket[6])
        if not self._transaction_lock.acquire(blocking=False):
            return None, COMM_PORT_BUSY, 0
        owns_busy = False
        port = self.portHandler
        try:
            if port.is_using:
                return None, COMM_PORT_BUSY, 0
            # 一次不确定应答污染该连接，不自动清空缓冲再假定后续帧可信。
            if getattr(port, "_microduck_tainted", False) or port.getBytesAvailable():
                port._microduck_tainted = True
                return None, COMM_RX_CORRUPT, 0
            port.is_using = True
            owns_busy = True
            # SDK 格式组包；不调用其会清空输入缓冲的 txPacket。
            txpacket[0:2] = [0xFF, 0xFF]
            txpacket[-1] = (~sum(txpacket[2:-1])) & 0xFF
            if port.writePort(txpacket) != len(txpacket):
                port._microduck_tainted = True
                return None, COMM_TX_FAIL, 0

            # pyserial 端口为非阻塞 read；总截止覆盖噪声、错误 ID 与不完整帧。
            deadline = time.monotonic() + self.timeout_seconds
            received = bytearray()
            total = expected_data + 6
            while time.monotonic() < deadline:
                chunk = port.readPort(total - len(received))
                if not chunk:
                    time.sleep(0.001)
                    continue
                received.extend(chunk)
                if (len(received) > total
                        or (len(received) >= 1 and received[0] != 0xFF)
                        or (len(received) >= 2 and received[1] != 0xFF)
                        or (len(received) >= 3 and received[2] not in valid_ids)
                        or (len(received) >= 4 and received[3] != expected_data + 2)
                        or (len(received) >= 5 and received[4] > 0x7F)):
                    port._microduck_tainted = True
                    return None, COMM_RX_CORRUPT, 0
                if len(received) == total:
                    if received[-1] != ((~sum(received[2:-1])) & 0xFF):
                        port._microduck_tainted = True
                        return None, COMM_RX_CORRUPT, 0
                    # 跟在应答后的残留字节不能被忽略为干净总线。
                    if port.getBytesAvailable():
                        port._microduck_tainted = True
                        return None, COMM_RX_CORRUPT, 0
                    return list(received), COMM_SUCCESS, received[4]
            port._microduck_tainted = True
            return None, COMM_RX_CORRUPT if received else COMM_RX_TIMEOUT, 0
        except BaseException:
            if owns_busy:
                port._microduck_tainted = True
            raise
        finally:
            if owns_busy:
                self.portHandler.is_using = False
            self._transaction_lock.release()

    def ping(self, scs_id):
        # 此处只发 PING；第一个返回值固定为 0，不代表型号。
        # 官方 ping 随后的型号 READ 会覆盖 PING 状态；先显式保留并检查。
        if not 0 <= scs_id <= 253:
            return 0, COMM_NOT_AVAILABLE, 0
        packet = [0, 0, scs_id, 2, 0x01, 0]
        _, result, error = self.txRxPacket(packet)
        if result != COMM_SUCCESS or error:
            return 0, result, error
        # 型号读取由 inspect 在验证 endian 后进行，避免提前解释多字节数据。
        return 0, result, error
