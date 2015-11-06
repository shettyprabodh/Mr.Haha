from pymouse import PyMouse
from pykeyboard import PyKeyboard
import serial

keyboard = PyKeyboard()

def press_spl_key(code):
    key_code = {0: None,
                1: keyboard.alt_key,
                2: keyboard.control_key,
                3: keyboard.shift_key,
                4: keyboard.left_key,
                5: keyboard.right_key,
                6: keyboard.up_key,
                7: keyboard.down_key,
                8: keyboard.enter_key,
                9: keyboard.backspace_key}
    keyboard.tap_key(key_code[code])

arduino = serial.Serial('/dev/ttyACM0', 9600)
while True:
    line = arduino.readline()
    if line[0] in ['-', '+']:
        if line[0] == '-':
            press_spl_key(int(line[1:]))
        else:
            keyboard.type_string(line[1])
    print line
