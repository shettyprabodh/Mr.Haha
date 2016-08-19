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

arduino = serial.Serial('/dev/ttyACM2', 9600)
result = [0.0, 0.0, 0.0]
alpha_result = 1.0 / 4.0
mouse_x = 0
mouse_y = 0
while True:
    line = arduino.readline()
    print line
    if line[0] == "$":
        line = line[1:]
        new_result = line.split()
        #t = (float(result[0])*100)
        #print result[0]
        # result = map(lambda x: (0, float(x) % 360)[(float(x)) != 0.0], result)
        def slow_func(x):
            return x*x*x
        try:
            result = map(lambda i: alpha_result * float(new_result[i]) + \
                (1.0 - alpha_result) * result[i], range(len(new_result)))
            print result
            # Using older convention
            # mouse_x = center_x + int(result[0]*screen_size[0]/100.0)
            # mouse_y = center_y + int((result[1])*screen_size[1]/100.0)

            mouse_x += 40 * slow_func(result[1] / 256.0) #int((result[1] + 180.0) * screen_size[0] / 80)
            mouse_y -= 40 * slow_func(result[0] / 256.0) #int((result[0] + 180.0) * screen_size[1] / 80.0)
            mouse_x = max(0, (0, mouse_x)[mouse_x < screen_size[0]])
            mouse_y = max(0, min(mouse_y, screen_size[1]))
        except:
            pass # Sometimes arduino throws poorly formatted text
        print mouse_x/10 , mouse_y/10
        mouse.move(mouse_x,mouse_y)
    elif line[0] in ['-', '+']:
        if line[0] in ['-', '+']:
           if line[0] == '-':
               press_spl_key(int(line[1:]))
           else:
               keyboard.type_string(line[1])
    elif line[0] == "#":
        mouse.click(mouse_x, mouse_y, 1)
