"""Cross-language wire compatibility and zero-only tester over a PTY."""
import contextlib
import ctypes as C
import io
import os
from pathlib import Path
import random
import select
import struct
import sys
import threading
import tty

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root/'pc'))
import usb_protocol as usb
import zero_test


class Command(C.Structure):
    _fields_ = [('uptime',C.c_uint32),('session',C.c_uint32),('sequence',C.c_uint16),
                ('ttl',C.c_uint16),('flags',C.c_uint16),('route',C.c_uint8),
                ('vx',C.c_float),('vy',C.c_float),('wz',C.c_float)]


class Feedback(C.Structure):
    _fields_ = [('uptime',C.c_uint32),('session',C.c_uint32),('sequence',C.c_uint16),
                ('status',C.c_uint16),('fault',C.c_uint16),('route',C.c_uint8),
                ('vx',C.c_float),('vy',C.c_float),('wz',C.c_float)]


lib = C.CDLL(str(root/'build/tests/protocol.so'))
lib.rm_encode_command.argtypes = [C.POINTER(Command),C.c_void_p]
lib.rm_decode_command.argtypes = [C.c_void_p,C.c_size_t,C.POINTER(Command)]
lib.rm_encode_feedback.argtypes = [C.POINTER(Feedback),C.c_void_p]
rng = random.Random(20261111)
for n in range(10000):
    enabled = bool(n%2)
    x,y,z = (rng.uniform(-.2,.2),rng.uniform(-.2,.2),rng.uniform(-.5,.5)) if enabled else (0.,0.,0.)
    session = rng.randrange(1,2**32); sequence = rng.randrange(65536); uptime = rng.randrange(2**32)
    ttl = rng.randrange(40,151) if enabled else 0
    c = Command(uptime,session,sequence,ttl,enabled,2,x,y,z); output=C.create_string_buffer(34)
    assert lib.rm_encode_command(C.byref(c),output)
    packet=usb.command(session,sequence,uptime,enabled=enabled,ttl=ttl,vx=x,vy=y,wz=z)
    assert packet == output.raw
    f=Feedback(uptime,session,sequence,73,0,2,x,y,z)
    assert lib.rm_encode_feedback(C.byref(f),output)
    decoded=usb.feedback(output.raw)
    assert decoded['sequence']==sequence and decoded['session']==session

stream=usb.Stream()
sp=b'SP'+b'\x5a\x1c\x21'+bytes(36)
sp+=struct.pack('<H',usb.crc(sp))
assert len(sp)==43 and list(stream.push(sp))==[]
assert list(stream.push(output.raw))==[decoded]
print('PASS PC codec: 10000 command/feedback comparisons against actual MCU C codec; atomic SP/nav stream')


def zero_emulator():
    master,slave=os.openpty();tty.setraw(slave);os.set_blocking(master,False)
    done=threading.Event(); failures=[]; counts=[]
    def emulate():
        data=bytearray();uptime=0
        try:
            while not done.is_set():
                if not select.select([master],[],[],.01)[0]:continue
                data.extend(os.read(master,4096))
                while len(data)>=34:
                    p=bytes(data[:34]);del data[:34];c=Command()
                    assert lib.rm_decode_command(p,34,C.byref(c))
                    assert c.flags==0 and c.ttl==0 and c.vx==c.vy==c.wz==0
                    uptime+=20;out=C.create_string_buffer(34)
                    # Locked board ACK is enough to validate communication.
                    f=Feedback(uptime,c.session,c.sequence,64,9,2,0,0,0)
                    assert lib.rm_encode_feedback(C.byref(f),out)
                    os.write(master,out.raw);counts.append(1)
        except Exception as exc:
            failures.append(exc)
    thread=threading.Thread(target=emulate);thread.start()
    try:
        zero_test.check=lambda path:path # PTY only, physical identity checked by real entry point.
        with contextlib.redirect_stdout(io.StringIO()):
            zero_test.run(os.ttyname(slave),.6)
        assert len(counts)>=10 and not failures
    finally:
        done.set();thread.join();os.close(master);os.close(slave)
    print('PASS zero tester over PTY: disabled STOP only, fresh chassis ACK, default motion-lock faults accepted')


zero_emulator()
