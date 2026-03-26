import serial
from utils.inspire_hand_new_api import clear_error, reset_para

right_hand_id = 0x01
left_hand_id = 0x01
ser_right = serial.Serial("/dev/ttyUSB1", 115200, timeout=1)
ser_left = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)

def reset_hand():
    clear_error(ser_right, right_hand_id)
    clear_error(ser_left, left_hand_id)


if __name__ == "__main__":
    reset_hand()