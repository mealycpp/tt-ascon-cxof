"""Full-stack chip test, deterministic-timing receiver.

After locking onto the first byte's start bit, subsequent bytes are
decoded at fixed offsets of 10 bit-periods from the previous start edge.
This avoids the mid-byte-edge confusion entirely.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer

SOF = 0xAA
EOF_BYTE = 0x55

CMD_PING        = 0x01
CMD_GET_VERSION = 0x02
CMD_WRITE_REG   = 0x10
CMD_READ_REG    = 0x11
CMD_START       = 0x30
CMD_GET_STATUS  = 0x40

ST_OK = 0x00

CLOCK_PERIOD_NS = 20
BAUD_DIV = 434
BIT_PERIOD_NS = CLOCK_PERIOD_NS * BAUD_DIV
BYTE_PERIOD_NS = BIT_PERIOD_NS * 10   # 10 = start + 8 data + stop


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_frame(cmd: int, payload: bytes = b"") -> bytes:
    body = bytes([len(payload), cmd]) + payload
    crc = crc16_ccitt(body)
    return bytes([SOF]) + body + bytes([crc & 0xFF, (crc >> 8) & 0xFF, EOF_BYTE])


async def uart_send_byte(dut, byte: int):
    dut.ui_in.value = (int(dut.ui_in.value) & 0xFE)
    await Timer(BIT_PERIOD_NS, units="ns")
    for i in range(8):
        bit = (byte >> i) & 1
        dut.ui_in.value = (int(dut.ui_in.value) & 0xFE) | bit
        await Timer(BIT_PERIOD_NS, units="ns")
    dut.ui_in.value = (int(dut.ui_in.value) & 0xFE) | 1
    await Timer(BIT_PERIOD_NS, units="ns")


async def uart_send_frame(dut, frame: bytes):
    for b in frame:
        await uart_send_byte(dut, b)


def _tx(dut):
    return int(dut.uo_out.value) & 1


class FrameRecv:
    """Tracks the chip's TX line. The first falling edge from idle is the
    start of the response frame; all subsequent bytes follow at fixed
    BYTE_PERIOD intervals.
    """
    HIGH_THRESHOLD_NS = BIT_PERIOD_NS * 2     # 2 bit periods of idle = frame boundary

    def __init__(self, dut):
        self.dut = dut
        self.first_edge_ns = None      # sim time of frame's first start bit
        self.bytes_consumed = 0

    async def watcher(self):
        """Background: find the first frame-start falling edge.

        Frame-start = at least 2 bit-periods of continuous HIGH followed
        by a falling edge. Starting with high_ns=0 forces the watcher
        to genuinely observe idle before accepting any falling edge,
        which prevents locking onto residual mid-byte transitions from
        a previous frame.
        """
        high_ns = 0
        prev = 1
        while self.first_edge_ns is None:
            await RisingEdge(self.dut.clk)
            cur = _tx(self.dut)
            if cur == 1:
                high_ns += CLOCK_PERIOD_NS
                prev = 1
            else:
                if prev == 1 and high_ns >= self.HIGH_THRESHOLD_NS:
                    self.first_edge_ns = cocotb.utils.get_sim_time(units='ns')
                    return
                high_ns = 0
                prev = 0

    async def wait_first_edge(self):
        while self.first_edge_ns is None:
            await RisingEdge(self.dut.clk)

    async def recv_byte(self):
        """Decode the next byte at the expected time offset."""
        await self.wait_first_edge()
        target_start = self.first_edge_ns + self.bytes_consumed * BYTE_PERIOD_NS
        t_now = cocotb.utils.get_sim_time(units='ns')
        # land at mid-bit-0 = target_start + 1.5 * BIT_PERIOD
        target = target_start + (BIT_PERIOD_NS * 3) // 2
        if target > t_now:
            await Timer(int(round(target - t_now)), units="ns")
        byte = 0
        for i in range(8):
            bit = _tx(self.dut)
            byte |= (bit << i)
            if i < 7:
                await Timer(BIT_PERIOD_NS, units="ns")
        self.bytes_consumed += 1
        self.dut._log.info(f"recv b={byte:02x} idx={self.bytes_consumed-1} first_edge={self.first_edge_ns}")
        return byte

    def reset_for_next_frame(self):
        """Call before expecting another frame from the chip."""
        self.first_edge_ns = None
        self.bytes_consumed = 0


async def recv_frame(rcv: FrameRecv):
    sof = await rcv.recv_byte()
    assert sof == SOF, f"expected SOF, got {sof:02x}"
    length = await rcv.recv_byte()
    status = await rcv.recv_byte()
    payload = bytes([await rcv.recv_byte() for _ in range(length - 1)])
    crc_lo = await rcv.recv_byte()
    crc_hi = await rcv.recv_byte()
    eof = await rcv.recv_byte()
    assert eof == EOF_BYTE, f"expected EOF, got {eof:02x}"
    expected_crc = crc16_ccitt(bytes([length, status]) + payload)
    actual_crc = crc_lo | (crc_hi << 8)
    assert expected_crc == actual_crc, f"CRC mismatch: got {actual_crc:04x}, expected {expected_crc:04x}"
    return status, payload


async def _setup(dut):
    cocotb.start_soon(Clock(dut.clk, CLOCK_PERIOD_NS, units="ns").start())
    dut.ena.value = 1
    dut.ui_in.value = 1
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await Timer(200, units="ns")
    dut.rst_n.value = 1
    await Timer(200, units="ns")


async def send_and_recv(dut, frame_bytes):
    """Send a request, return the chip's response (status, payload)."""
    # ensure chip's tx is solidly idle (HIGH) for >=3 bit periods before
    # starting the watcher.  Otherwise watcher can lock onto residual
    # transitions from a previous frame's tail.
    for _ in range(50):
        high_streak = 0
        for _ in range(BIT_PERIOD_NS * 3 // CLOCK_PERIOD_NS):
            await RisingEdge(dut.clk)
            if _tx(dut) == 1:
                high_streak += 1
            else:
                high_streak = 0
                break
        if high_streak >= (BIT_PERIOD_NS * 3 // CLOCK_PERIOD_NS):
            break
    rcv = FrameRecv(dut)
    cocotb.start_soon(rcv.watcher())
    await uart_send_frame(dut, frame_bytes)
    return await recv_frame(rcv)


@cocotb.test()
async def test_ping(dut):
    await _setup(dut)
    status, payload = await send_and_recv(dut, build_frame(CMD_PING))
    assert status == ST_OK
    assert payload == b""
    dut._log.info("PING ok")


@cocotb.test()
async def test_get_version(dut):
    await _setup(dut)
    status, payload = await send_and_recv(dut, build_frame(CMD_GET_VERSION))
    assert status == ST_OK
    assert len(payload) == 2
    assert payload[0] == 0x01
    assert payload[1] == 0xAC


@cocotb.test()
async def test_write_read_register(dut):
    """Real verification: write 0x10 to register 0x02, read it back, expect 0x10."""
    await _setup(dut)
    s, _ = await send_and_recv(dut, build_frame(CMD_WRITE_REG, bytes([0x02, 0x10])))
    assert s == ST_OK, f"write status {s:02x}"
    s, p = await send_and_recv(dut, build_frame(CMD_READ_REG, bytes([0x02])))
    assert s == ST_OK, f"read status {s:02x}"
    assert len(p) == 1, f"read payload len {len(p)}"
    assert p[0] == 0x10, f"expected 0x10, got 0x{p[0]:02x}"
    dut._log.info("write/read register ok")
