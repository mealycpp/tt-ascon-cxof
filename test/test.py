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


# ===== End-to-end CXOF KAT tests through UART =====

KATS = [
    {"name": "Count=1: empty cs, empty msg", "cs": b"", "msg": b"",
     "expected": bytes.fromhex("4F50159EF70BB3DAD8807E034EAEBD44C4FA2CBBC8CF1F05511AB66CDCC52990")},
    {"name": "Count=2: 1B cs",  "cs": bytes([0x10]), "msg": b"",
     "expected": bytes.fromhex("0C93A483E7D574D49FE52CCE03EE646117977D57A8AA57704AB4DAF44B501430")},
    {"name": "Count=3: 2B cs",  "cs": bytes([0x10, 0x11]), "msg": b"",
     "expected": bytes.fromhex("D1106C7622E79FE955BD9D79E03B918E770FE0E0CDDDE28BEB924B02C5FC936B")},
    {"name": "Count=9: 8B cs",  "cs": bytes(range(0x10, 0x18)), "msg": b"",
     "expected": bytes.fromhex("61324766441DD6C11E1736BAD1D2185820885ED76FE2CE537775A6E855EEAFD2")},
    {"name": "Count=100: msg=3B, cs=empty", "cs": b"", "msg": bytes([0x00, 0x01, 0x02]),
     "expected": bytes.fromhex("1093DA88C318F6D9F26E1A222DBC30016D03953EDFD9BA3D75D7D8451B9DF542")},
    {"name": "Count=50: msg=1B, cs=16B", "cs": bytes(range(0x10, 0x20)), "msg": bytes([0x00]),
     "expected": bytes.fromhex("2B024A542F34D07360EE5FC3AC5A5ADE3F144DE1959C7BBCF2664357A47C6F12")},
    {"name": "Count=500: msg=15B, cs=4B", "cs": bytes([0x10, 0x11, 0x12, 0x13]),
     "msg": bytes(range(0x00, 0x0F)),
     "expected": bytes.fromhex("FA0E8B98F0F30CC376879268A72FF602BA483F857FCAE88F7A3E66E6289A116C")},
    {"name": "Count=1000: msg=30B, cs=9B", "cs": bytes(range(0x10, 0x19)),
     "msg": bytes(range(0x00, 0x1E)),
     "expected": bytes.fromhex("D3CB03D419D215D91733CEDBB709CA48BCAD775BD5321698F5F032B2B042D904")},
]

# Register map (matches docs/info.md)
REG_CS_LENGTH    = 0x02
REG_MSG_LENGTH   = 0x03
REG_OUT_LEN_LO   = 0x04
REG_OUT_LEN_HI   = 0x05
REG_CS_BASE      = 0x10
REG_MSG_BASE     = 0x30
REG_OUT_BASE     = 0x50
REG_STATUS       = 0x00


async def write_reg(dut, addr, data):
    """Write one byte to a register."""
    s, _ = await send_and_recv(dut, build_frame(CMD_WRITE_REG, bytes([addr, data])))
    assert s == ST_OK, f"WRITE_REG addr=0x{addr:02x} returned status 0x{s:02x}"


async def read_reg(dut, addr):
    """Read one byte from a register."""
    s, p = await send_and_recv(dut, build_frame(CMD_READ_REG, bytes([addr])))
    assert s == ST_OK, f"READ_REG addr=0x{addr:02x} returned status 0x{s:02x}"
    assert len(p) == 1, f"READ_REG returned {len(p)} bytes"
    return p[0]


async def run_uart_kat(dut, kat):
    """Run one KAT through the full UART path: write inputs, start, poll, read output."""
    dut._log.info(f"=== UART KAT: {kat['name']} ===")

    # Load CS bytes
    for i, b in enumerate(kat["cs"]):
        await write_reg(dut, REG_CS_BASE + i, b)
    # Load MSG bytes
    for i, b in enumerate(kat["msg"]):
        await write_reg(dut, REG_MSG_BASE + i, b)
    # Set lengths
    await write_reg(dut, REG_CS_LENGTH, len(kat["cs"]))
    await write_reg(dut, REG_MSG_LENGTH, len(kat["msg"]))
    await write_reg(dut, REG_OUT_LEN_LO, 32 & 0xFF)
    await write_reg(dut, REG_OUT_LEN_HI, (32 >> 8) & 0xFF)

    # Start
    s, _ = await send_and_recv(dut, build_frame(CMD_START))
    assert s == ST_OK, f"START returned 0x{s:02x}"

    # Poll status until busy=0 (bit 0 = busy, bit 1 = done)
    for poll in range(200):
        status = await read_reg(dut, REG_STATUS)
        if status & 0x08:    # result_present (latched)
            break
    else:
        raise TimeoutError("result_present never asserted")

    # Read 32 output bytes
    got = bytes([await read_reg(dut, REG_OUT_BASE + i) for i in range(32)])

    dut._log.info(f"  got:  {got.hex().upper()}")
    dut._log.info(f"  want: {kat['expected'].hex().upper()}")
    assert got == kat["expected"], f"KAT mismatch for {kat['name']}"
    dut._log.info(f"  PASS")


@cocotb.test()
async def test_uart_kat_count1(dut):
    """First KAT through UART path. If this passes, the full data path works."""
    await _setup(dut)
    await run_uart_kat(dut, KATS[0])


@cocotb.test()
async def test_uart_all_kats(dut):
    """Run all 8 KATs through the UART path, with chip reset between each."""
    failures = []
    for i, kat in enumerate(KATS):
        await _setup(dut)   # full reset between KATs to clear result_present latch
        try:
            await run_uart_kat(dut, kat)
            dut._log.info(f"KAT {i+1}/{len(KATS)} OK")
        except AssertionError as e:
            failures.append(f"{kat['name']}: {e}")
            dut._log.error(f"KAT {i+1}/{len(KATS)} FAILED: {e}")
    if failures:
        for f in failures:
            dut._log.error(f"  - {f}")
        raise AssertionError(f"{len(failures)}/{len(KATS)} KATs failed")
    dut._log.info(f"All {len(KATS)} KATs passed through UART path")


@cocotb.test()
async def test_uart_kat_001(dut):
    await _setup(dut)
    await run_uart_kat(dut, KATS[0])

@cocotb.test()
async def test_uart_kat_002(dut):
    await _setup(dut)
    await run_uart_kat(dut, KATS[1])

@cocotb.test()
async def test_uart_kat_003(dut):
    await _setup(dut)
    await run_uart_kat(dut, KATS[2])

@cocotb.test()
async def test_uart_kat_004(dut):
    await _setup(dut)
    await run_uart_kat(dut, KATS[3])

@cocotb.test()
async def test_uart_kat_005(dut):
    await _setup(dut)
    await run_uart_kat(dut, KATS[4])

@cocotb.test()
async def test_uart_kat_006(dut):
    await _setup(dut)
    await run_uart_kat(dut, KATS[5])

@cocotb.test()
async def test_uart_kat_007(dut):
    await _setup(dut)
    await run_uart_kat(dut, KATS[6])

@cocotb.test()
async def test_uart_kat_008(dut):
    await _setup(dut)
    await run_uart_kat(dut, KATS[7])
