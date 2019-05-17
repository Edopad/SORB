import evdev
#test devices using `python /usr/local/lib/python3.5/dist-packages/evdev/evtest.py`
import RPi.GPIO as GPIO
#from mpu9250 import MPU9250
#from machine import I2C, Pin
#import micropython
import threading
import time

#attempts to find the gamepad you are looking for
def get_gamepad_device():
    for device_path in evdev.list_devices():
        try:
            device = evdev.InputDevice(device_path)
            if device.name == 'Xbox Wireless Controller':
                return device
            else:
                device.close()
        except:
            pass
    return None

BTN_XB = evdev.ecodes.ecodes['KEY_HOMEPAGE']
BTN_A = evdev.ecodes.ecodes['BTN_A']
BTN_B = evdev.ecodes.ecodes['BTN_B']
BTN_X = evdev.ecodes.ecodes['BTN_X']
BTN_Y = evdev.ecodes.ecodes['BTN_Y']
BTN_LB = evdev.ecodes.ecodes['BTN_TL']
BTN_RB = evdev.ecodes.ecodes['BTN_TR']
BTN_BACK = evdev.ecodes.ecodes['KEY_BACK']
BTN_START = evdev.ecodes.ecodes['BTN_START']



#i2c = I2C(scl = Pin(22), sda = Pin (21)) #pin 
#sensor = MPU9250(i2c)

class MotorController:
    #pwm frequency, in hz
    PWM_FREQ = 100
    
    def __init__(self, pin_forward
, pin_reverse):
        self._pin_forward = pin_forward
        self._pin_reverse = pin_reverse
        GPIO.setup(pin_forward, GPIO.OUT)
        GPIO.setup(pin_reverse, GPIO.OUT)
        #1 to -1
        self._power = 0
        #pwm controller
        self._pwm_forward = GPIO.PWM(pin_forward, self.PWM_FREQ)
        self._pwm_reverse = GPIO.PWM(pin_reverse, self.PWM_FREQ)
        #start pwm
        self._pwm_forward.start(0)
        self._pwm_reverse.start(0)
    
    @property
    def power(self):
        return self._power

    @power.setter
    def power(self, value):
        """Sets the power to a value between 1 and -1"""
        assert(value >= -1 and value <= 1, 'Motor power out of bounds!: {}'.format(value))
        self._power = value
        
        self._pwm_forward.ChangeDutyCycle(0)
        self._pwm_reverse.ChangeDutyCycle(0)
        if value > 0:
            self._pwm_forward.ChangeDutyCycle(value * 100)
        elif value < 0:
            self._pwm_reverse.ChangeDutyCycle(-value * 100)
            


def gp_main():
    #initialize GPIO, for fun
    #GPIO.setmode(GPIO.BCM)
    #GPIO.setup(26, GPIO.OUT)
    #pwm frequency (hz)
    #pwm_freq = 50
    #pwm = GPIO.PWM(26, pwm_freq)
    #pwm_min = 1500 #mus
    #pwm_max_b = 1300 #mus
    #pwm_max = 1700 #mus
    #ms_to_duty = lambda x: x / (1000000 / pwm_freq)
    #pwm.start(70.0)
    
    global joy_ly
    global joy_lx
    global joy_rx
    
    for event in gamepad.read_loop():
        #button
        #time.sleep(1)
        if event.type == evdev.ecodes.EV_KEY:
            pr = 'pressed' if event.value == 1 else 'released'
            #key list
            keys = {
                evdev.ecodes.ecodes['KEY_HOMEPAGE']: 'X-BOX',
                BTN_A: 'A',
                BTN_B: 'B',
                BTN_X: 'X',
                BTN_Y: 'Y',
                BTN_LB: 'LB',
                BTN_RB: 'RB',
                BTN_BACK: 'BACK',
                BTN_START: 'START',
            }
            key = keys.get(event.code, '?')
            if key == 'A':
                pass
                #pwm.ChangeDutyCycle(100.0 * event.value)
            
            print('Button [{}] {}.'.format(key, pr))
        #analog
        elif event.type == evdev.ecodes.EV_ABS:
            #normalization functions
            n_stick = lambda x: 2.0 * (x / 65535) - 1.0
            n_trigger = lambda x: x / 1023
            n_default = lambda x: x
            invert = lambda f: lambda x: -f(x)
            #key list
            keys = {
                evdev.ecodes.ecodes['ABS_HAT0X']: ('D-PAD (X)', n_default), #-1,0,1 (right)
                evdev.ecodes.ecodes['ABS_HAT0Y']: ('D-PAD (Y)', invert(n_default)), #-1,0,1 (down)
                evdev.ecodes.ecodes['ABS_X']: ('LS (X)', n_stick), #0-65535 (right)
                evdev.ecodes.ecodes['ABS_Y']: ('LS (Y)', invert(n_stick)), #0-65535 (down)
                evdev.ecodes.ecodes['ABS_Z']: ('RS (X)', n_stick), #0-65535 (right)
                evdev.ecodes.ecodes['ABS_RZ']: ('RS (Y)', invert(n_stick)), #0-65535 (down)
                evdev.ecodes.ecodes['ABS_GAS']: ('RT', n_trigger), #0-1023 (pushed)
                evdev.ecodes.ecodes['ABS_BRAKE']: ('LT', n_trigger) #0-1023 (pushed)
            }
            key = keys.get(event.code, ('?', n_default))
            #print('Analog [{}] {}.'.format(key[0], key[1](event.value)))
            if key[0] == 'LS (Y)':
                """val = key[1](event.value)
                print(val)
                val = val if abs(val) > 0.1 else 0
                print('after', val)
                perc = ms_to_duty( val* (pwm_max - pwm_min) + pwm_min)
                perc = abs(val) # / 100
                #perc = key[1](event.value)
                
                print('perc:',perc)
                print(perc * 1 / pwm_freq) 
                
                pwm.ChangeDutyCycle(perc * 100)"""
                #print('Analog [{}] {}.'.format(key[0], key[1](event.value)))
                joy_ly = key[1](event.value)
                joy_ly = joy_ly if abs(joy_ly) > 0.1 else 0
                #print('LS (Y): {:.2f}%'.format(joy_ly * 100))
                print('joy_ly',joy_ly)
#                motor_a.power = val
#                motor_b.power = val
#                motor_c.power = val
            elif key[0] == 'LS (X)' :
                #print('Analog [{}] {}.'.format(key[0], key[1](event.value)))
                joy_lx = key[1](event.value)
                joy_lx = joy_lx if abs(joy_lx) > 0.1 else 0
                #print('LS (X): {:.2f}%'.format(joy_lx * 100))
                print('joy_lx',joy_lx)
            elif key[0] == 'RS (X)' :
                #print('Analog [{}] {}.'.format(key[0], key[1](event.value)))
                joy_rx = key[1](event.value)
                joy_rx = joy_rx if abs(joy_rx) > 0.1 else 0
                #print('RS (X): {:.2f}%'.format(joy_rx * 100))
                print('joy_rx',joy_rx)
                
                
def vector_drive(ly, lx, rx):
    #pass
    #print("vdrive", ly, lx, rx)
    max_input = max(abs(ly),abs(lx))
    max_input = max(abs(rx),abs(max_input))
    
    wheelf = 0*ly + lx + rx
    wheelbr = -ly - lx/((3)**.5) + rx
    wheelbl =  ly - lx/((3)**.5) + rx
    
    
    max_motor = max(abs(wheelf),abs(wheelbr))
    max_motor = max(abs(max_motor),abs(wheelbl))
    
    max_motor = max_motor if not(max_motor == 0) else 1
    
    wheelf  = wheelf  * (max_input/max_motor)
    wheelbl = wheelbl * (max_input/max_motor)
    wheelbr = wheelbr * (max_input/max_motor)
    
    wheelbr = wheelbr if abs(wheelbr) <= 1 else 0
    wheelbl = wheelbl if abs(wheelbl) <= 1 else 0
    wheelf  = wheelf  if abs(wheelf)  <= 1 else 0
    
    #print('wheelbr: {:.2f}%'.format(wheelbr * 100))
    #print('wheelbl: {:.2f}%'.format(wheelbl * 100))
    #print('wheelf: {:.2f}%'.format(wheelf * 100))
    global motor_a
    global motor_c
    global motor_b
    
    motor_a.power = wheelf
    motor_b.power = wheelbr
    motor_c.power = wheelbl


if __name__ == '__main__':
    #attempt to get the gamepad from system devices
    
    GPIO.setmode(GPIO.BCM)
    
    motor_a = MotorController(19, 26)
    motor_b = MotorController(6, 13)
    motor_c = MotorController(21, 5)
    
    joy_ly = 0
    joy_lx = 0
    joy_rx = 0
    
    gamepad = get_gamepad_device()
    joystick = threading.Thread(target = gp_main)
    joystick.setDaemon = False
    
    #if none found, exit.
    if gamepad:
        print('Gamepad found! Using: ', gamepad)
        #joystick.start()
        #vector_drive(joy_ly, joy_lx, joy_rx)
        try:
            joystick.start()
            
            while(1):
                #time.sleep(1)
                #print(joy_ly, joy_lx, joy_rx)
                vector_drive(joy_ly, joy_lx, joy_rx)
                #print("debuggo")
                #pass
        except OSError as err:
            #it's likely that the device disconnects here, I am examining this issue.
            print('OSError thrown! Did the device disconnect?')
            print(err)
        except KeyboardInterrupt:
            print('Keyboard interrupt received. Closing and Exiting.')
        finally:
            #perform any cleanup
            joystick.join()
            GPIO.cleanup()
            gamepad.close()
    else:
        print('Gamepad not found! Exiting.')
