import serial
import struct


# 把数据分成高字节和低字节
def data2bytes(data):
    rdata = [0xFF] * 2
    if data == -1:
        rdata[0] = 0xFF
        rdata[1] = 0xFF
    else:
        rdata[0] = data & 0xFF
        rdata[1] = (data >> 8) & (0xFF)
    return rdata


# 把十六进制或十进制的数转成bytes
def num2str(num):
    str = hex(num)
    str = str[2:4]
    if len(str) == 1:
        str = "0" + str
    str = bytes.fromhex(str)
    # print(str)
    return str


# 求校验和
def checknum(data, leng):
    result = 0
    for i in range(2, leng):
        result += data[i]
    result = result & 0xFF
    # print(result)
    return result

def _tx_frame(ser, frame_list, rx_len, name=None):
   
    putdata = b"".join(num2str(x) for x in frame_list)
    ser.write(putdata)

    if name in ('CLEAR_ERROR', 'RESET_PARA'):
        print(f'{name} 写入数据 {putdata}')

    # print("发送的数据：")
    # for x in putdata:
    #     print(hex(x))

    getdata = ser.read(rx_len)  # 读 6 通道固定 rx_len 字节

    if len(getdata) != rx_len:
        print(f'ser = {ser}, rx_len = {rx_len}, getdata_len = {len(getdata)}, \n {getdata}')

    if name in ('CLEAR_ERROR', 'RESET_PARA'):
        print(f'{name} 得到返回数据 {getdata}')

    # print("返回的数据：")
    # for x in getdata:
    #     print(hex(x))
    return getdata


def _write6(ser, hand_id, reg_addr, values, range_min, range_max, name):    #写 6 路 16-bit 参数，带范围检查
    if any(v < range_min or v > range_max for v in values):
        print(f"{name} 超出正确范围：{range_min}–{range_max}")
        return

    b = [0xEB, 0x90, hand_id, 0x0F, 0x12, reg_addr & 0xFF, (reg_addr >> 8) & 0xFF]
    for v in values:
        b += data2bytes(v)
    b += [checknum(b, 19)]

    _tx_frame(ser, b, 9)        # 写命令固定等 9 字节应答


def _read6(ser, hand_id, reg_addr, signed=False):   #读 6 路 16-bit，返回 6 元素 list
    b = [0xEB, 0x90, hand_id, 0x04, 0x11, reg_addr & 0xFF, (reg_addr >> 8) & 0xFF, 0x0C]
    b += [checknum(b, 8)]

    getdata = _tx_frame(ser, b, 20)      # 读 6 通道固定 20 字节

    res = [0] * 6
    for i in range(6):
        lo, hi = getdata[7 + i * 2], getdata[8 + i * 2]
        if lo == 0xFF and hi == 0xFF:
            res[i] = -1
        else:
            res[i] = int.from_bytes(bytes([lo, hi]), 'little', signed=signed)
    return res


def _read3(ser, hand_id, reg_addr):     #读 3 字节帧，返回 6 字节 list
    b = [0xEB, 0x90, hand_id, 0x04, 0x11, reg_addr & 0xFF, (reg_addr >> 8) & 0xFF, 0x06]
    b += [checknum(b, 8)]

    getdata = _tx_frame(ser, b, 14)      # 故障/状态固定 14 字节
    return list(getdata[7:13])


def _write16(ser, hand_id, reg_addr, value, name=""):   #写单个 16-bit 寄存器
    b = [0xEB, 0x90, hand_id, 0x05, 0x12, reg_addr & 0xFF, (reg_addr >> 8) & 0xFF, value & 0xFF, (value >> 8) & 0xFF]
    b += [checknum(b, 9)]

    _tx_frame(ser, b, 9, name)
    

# 设置驱动器位置
def setpos(ser, hand_id, pos1, pos2, pos3, pos4, pos5, pos6):
    # 数据范围：-1–2000
    _write6(ser, hand_id, 0x05C2, (pos1, pos2, pos3, pos4, pos5, pos6), -1, 2000, "位置")


# 设置角度
def setangle(ser, hand_id, angle1, angle2, angle3, angle4, angle5, angle6):
    # 数据范围：-1–1000
    _write6(ser, hand_id, 0x05CE, (angle1, angle2, angle3, angle4, angle5, angle6), -1, 1000, "角度")


# 设置力控阈值
def setpower(ser, hand_id, power1, power2, power3, power4, power5, power6):
    # 数据范围：0–1000
    _write6(ser, hand_id, 0x05DA, (power1, power2, power3, power4, power5, power6), 0, 1000, "力控阈值")


# 设置速度
def setspeed(ser, hand_id, speed1, speed2, speed3, speed4, speed5, speed6):
    # 数据范围：0–1000
    _write6(ser, hand_id, 0x05F2, (speed1, speed2, speed3, speed4, speed5, speed6), 0, 1000, "速度")


# 读取驱动器实际的位置值
def get_setpos(ser, hand_id):
    # 寄存器地址 0x05C2，读 12 字节
    return _read6(ser, hand_id, 0x05C2)


# 读取设置角度
def get_setangle(ser, hand_id):
    # 寄存器地址 0x05CE，读 12 字节
    return _read6(ser, hand_id, 0x05CE)


# 读取驱动器设置的力控阈值
def get_setpower(ser, hand_id):
    # 寄存器地址 0x05DA，读 12 字节
    return _read6(ser, hand_id, 0x05DA)


# 读取驱动器实际的位置值
def get_actpos(ser, hand_id):
    # 寄存器地址 0x05FE，读 12 字节
    return _read6(ser, hand_id, 0x05FE)


# 读取实际的角度值
def get_actangle(ser, hand_id):
    # 寄存器地址 0x060A，读 12 字节
    return _read6(ser, hand_id, 0x060A)


# 读取实际的受力
def get_actforce(ser, hand_id):
    # 寄存器地址 0x062E，读 12 字节，有符号
    return _read6(ser, hand_id, 0x062E, signed=True)


# 读取电流
def get_current(ser, hand_id):
    # 寄存器地址 0x063A，读 12 字节
    return _read6(ser, hand_id, 0x063A)


# 读取故障信息
def get_error(ser, hand_id):
    # 寄存器地址 0x0646，读 6 字节
    return _read3(ser, hand_id, 0x0646)


# 读取状态信息
def get_status(ser, hand_id):
    # 寄存器地址 0x064C，读 6 字节
    return _read3(ser, hand_id, 0x064C)


# 清除错误
def clear_error(ser, hand_id):
    # 清除错误：向寄存器 1004 (0x03EC) 写 1
    _write16(ser, hand_id, 0x03EC, 0x0001, "CLEAR_ERROR")


# 恢复出厂设置
def reset_para(ser, hand_id):
    # 恢复出厂设置：向寄存器 1006 (0x03EE) 写 0x0001
    _write16(ser, hand_id, 0x03EE, 0x0001, "RESET_PARA")


import time


def get_status_loop():
    ser_left = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)
    while True:
        print(get_error(ser_left, 1))
        time.sleep(1)


def main(args=None):
    # 串口设置
    # test_ser=serial.Serial('/dev/ttyUSB1',115200)
    # test_ser.isOpen()

    # hand_id = 1

    # actangle=get_actangle(test_ser, hand_id)
    # actforce=get_actforce(test_ser, hand_id)

    # print("actangle/t",actangle)
    # print("actforce/t",actforce)
    get_status_loop()


if __name__ == "__main__":
    main()
