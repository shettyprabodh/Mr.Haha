from pymouse import PyMouse
from pykeyboard import PyKeyboard
import serial

keyboard = PyKeyboard()
mouse = PyMouse()
screen_size = mouse.screen_size()
center_x = screen_size[0]/2
center_y = screen_size[1]/2

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
    result = line.split()
    #t = (float(result[0])*100)
    #print result[0]
    result = map(lambda x: (0, float(x))[(float(x)) != 0.0], result)
    print int(float(result[0])), int(float(result[1]))
    print result
    #if line[0] in ['-', '+']:
    #    if line[0] == '-':
    #        press_spl_key(int(line[1:]))
    #    else:
    #        keyboard.type_string(line[1])
    #print line
    mouse_x = center_x + int(result[0]*screen_size[0]/100.0)
    mouse_y = center_y + int((result[1])*screen_size[1]/100.0)
    print mouse_x/10 , mouse_y/10
    mouse.move(mouse_x/10,mouse_y/10)
