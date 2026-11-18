#!/usr/bin/env python3
"""Query only DAP_Info packet size over HID; never issue connect/reset/flash."""
import os
from pathlib import Path
import select
import sys


def main():
    devices=[]
    for entry in Path('/sys/class/hidraw').glob('*'):
        parent=(entry/'device').resolve()
        for item in parent.parents:
            if not (item/'idVendor').exists():
                continue
            vid=(item/'idVendor').read_text().strip()
            pid=(item/'idProduct').read_text().strip()
            serial=(item/'serial').read_text().strip() if (item/'serial').exists() else ''
            if (vid,pid,serial)==('c251','f001','0001A0000002'):
                devices.append(Path('/dev')/entry.name)
            break
    if len(devices)!=1:
        print(f'FAIL: 需要唯一的 MuseLab c251:f001 / 0001A0000002 HID 设备，当前找到 {len(devices)} 个。')
        return 2
    print('设备:',devices[0],flush=True)
    fd=os.open(devices[0],os.O_RDWR|os.O_NONBLOCK)
    try:
        # Drain only already-arrived HID reports, with a fixed upper bound.
        for _ in range(8):
            if not select.select([fd],[],[],0)[0]:break
            os.read(fd,1024)
        # HID report ID=0, DAP_Info=0, selector=0xFF (packet size).
        request=bytes((0,0,0xff))+bytes(62)
        if os.write(fd,request)!=len(request):
            raise RuntimeError('HID 请求写入不完整')
        if not select.select([fd],[],[],1)[0]:
            print('FAIL: USB 可打开、查询可写入，但 1 秒内未收到 DAP_Info 响应。')
            print('请停止其他调试程序，重新插拔下载器，并检查无线接收端单独供电和配对。')
            return 1
        reply=os.read(fd,1024)
        if len(reply)<4 or reply[0:2]!=b'\x00\x02':
            print('FAIL: 非预期 DAP_Info 响应:',reply[:16].hex())
            return 1
        size=int.from_bytes(reply[2:4],'little')
        if not 4<=size<=1024:
            print('FAIL: DAP packet size 超出预期:',size)
            return 1
        print(f'PASS: DAP_Info 查询成功，packet_size={size}。这仅证明探针控制响应，不代表 C 板已连接或烧录成功。')
        return 0
    finally:
        os.close(fd)


if __name__=='__main__':
    try:
        sys.exit(main())
    except (OSError,RuntimeError) as exc:
        sys.exit('FAIL: '+str(exc))
