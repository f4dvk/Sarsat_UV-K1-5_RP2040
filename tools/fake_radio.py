#!/usr/bin/env python3
"""
fake_radio.py -- bench test the RP2040 <-> radio serial link without a radio.

Wire a 3.3 V USB-TTL adapter to the RP2040 C-Board UART instead of the handheld:
    adapter GND  -> C-Board GND
    adapter RX   <- RP2040 GP0  (TX, MCU -> radio)
    adapter TX   -> RP2040 GP1  (RX, radio -> MCU)

Then:
    ./fake_radio.py /dev/ttyUSB0                 # decode what the RP2040 sends
    ./fake_radio.py /dev/ttyUSB0 --ack           # + reply an ACK to every 0x06Cx
    ./fake_radio.py /dev/ttyUSB0 --ack --status  # + answer HELLO with a fake
                                                 #   radio status (VFO/freq/mod)

Frame (both directions), egzumer/KD8CEC App/app/uart.c:
    AB CD | size:u16 LE | <inner> | crc16/xmodem:u16 LE | DC BA
    inner = [ID:u16 LE][data_size:u16 LE][data...]  (inner+crc XOR-masked)
Replies use ID 0xCDAB in the outer sense; here we send application replies with
ID = command | 0x8000 so the RP2040 can tell an ACK from a fresh command.
"""
import sys, time, argparse

OBF = bytes([0x16,0x6C,0x14,0xE6,0x2E,0x91,0x0D,0x40,
             0x21,0x35,0xD5,0x40,0x13,0x03,0xE9,0x80])

def crc16(buf):
    c = 0
    for b in buf:
        c ^= b << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if (c & 0x8000) else (c << 1) & 0xFFFF
    return c

def build(cmd_id, data=b""):
    inner = bytes([cmd_id & 0xFF, cmd_id >> 8, len(data) & 0xFF, len(data) >> 8]) + data
    pay = inner + crc16(inner).to_bytes(2, "little")
    masked = bytes(b ^ OBF[i % 16] for i, b in enumerate(pay))
    return bytes([0xAB, 0xCD, len(inner) & 0xFF, len(inner) >> 8]) + masked + bytes([0xDC, 0xBA])

def parse_stream(buf):
    """Yield (cmd_id, data) for every complete frame in buf; return leftover."""
    out = []
    i = 0
    while True:
        j = buf.find(b"\xAB\xCD", i)
        if j < 0:
            return out, buf[max(i, len(buf) - 1):]
        if len(buf) < j + 4:
            return out, buf[j:]
        size = buf[j + 2] | (buf[j + 3] << 8)
        end = j + 4 + size + 2 + 2
        if len(buf) < end:
            return out, buf[j:]
        if buf[end - 2:end] != b"\xDC\xBA":
            i = j + 2
            continue
        masked = buf[j + 4:end - 2]
        pay = bytes(b ^ OBF[k % 16] for k, b in enumerate(masked))
        cmd = pay[0] | (pay[1] << 8)
        dlen = pay[2] | (pay[3] << 8)
        out.append((cmd, pay[4:4 + dlen]))
        i = end

NAMES = {0x06C0: "CLEAR", 0x06C1: "TEXT", 0x06C2: "BEACON", 0x06CF: "HELLO"}

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--baud", type=int, default=38400)
    ap.add_argument("--ack", action="store_true", help="reply ACK to each 0x06Cx")
    ap.add_argument("--status", action="store_true",
                    help="answer HELLO with a fake VFO/freq/mod status")
    a = ap.parse_args()

    import serial  # pyserial
    s = serial.Serial(a.port, a.baud, timeout=0.1)
    print(f"listening on {a.port} @ {a.baud}  ack={a.ack} status={a.status}\n")
    buf = b""
    n_text = 0
    while True:
        chunk = s.read(256)
        if chunk:
            buf += chunk
            frames, buf = parse_stream(buf)
            for cmd, data in frames:
                nm = NAMES.get(cmd, f"0x{cmd:04X}")
                if cmd == 0x06C1 and len(data) >= 2:
                    line, inv = data[0], data[1]
                    txt = data[2:].decode("latin1")
                    print(f"  TEXT  L{line:<2} {'(inv) ' if inv else '      '}\"{txt}\"")
                    n_text += 1
                elif cmd == 0x06C0:
                    print(f"  CLEAR   ({n_text} text lines since last)"); n_text = 0
                elif cmd == 0x06CF:
                    print(f"  HELLO proto_ver={data[0] if data else '?'}")
                else:
                    print(f"  {nm}  {data.hex(' ')}")

                if cmd == 0x06CF and a.status:
                    # HELLO reply: vfo, mod(0=FM), rx_freq 10Hz LE, screen, proto
                    f = 40602500                       # 406.025 MHz
                    st = bytes([0, 0]) + f.to_bytes(4, "little") + bytes([0, 1])
                    s.write(build(0x06CF | 0x8000, st))
                    print("  -> HELLO reply (VFO A, FM, 406.02500 MHz)")
                elif a.ack:
                    s.write(build(cmd | 0x8000, b"\x00"))          # status 0 = OK
        else:
            time.sleep(0.02)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
