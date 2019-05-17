import evdev
#test devices using `python /usr/local/lib/python3.5/dist-packages/evdev/evtest.py`
import RPi.GPIO as GPIO
#from mpu9250 import MPU9250
#from machine import I2C, Pin
#import micropython


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
            


def main():
    #initialize GPIO, for fun
    GPIO.setmode(GPIO.BCM)
    #GPIO.setup(26, GPIO.OUT)
    #pwm frequency (hz)
    #pwm_freq = 50
    #pwm = GPIO.PWM(26, pwm_freq)
    #pwm_min = 1500 #mus
    #pwm_max_b = 1300 #mus
    #pwm_max = 1700 #mus
    #ms_to_duty = lambda x: x / (1000000 / pwm_freq)
    #pwm.start(70.0)
    
    motor_a = MotorController(19, 26)
    motor_b = MotorController(6, 13)
    motor_c = MotorController(21, 5)
    
    
    for event in gamepad.read_loop():
        #button
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
            print('Analog [{}] {}.'.format(key[0], key[1](event.value)))
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
                
                pwm.ChangeDutyCycle(perc * 100)
                print('Analog [{}] {}.'.format(key[0], key[1](event.value)))"""
                val = key[1](event.value)
                val = val if abs(val) > 0.1 else 0
                print('Power: {:.2f}%'.format(val * 100))
                motor_a.power = val
                motor_b.power = val
                motor_c.power = val
                

if __name__ == '__main__':
    #attempt to get the gamepad from system devices
    gamepad = get_gamepad_device()
    #if none found, exit.
    if gamepad:
        print('Gamepad found! Using: ', gamepad)
        try:
            main()
        except OSError as err:
            #it's likely that the device disconnects here, I am examining this issue.
            print('OSError thrown! Did the device disconnect?')
            print(err)
        except KeyboardInterrupt:
            print('Keyboard interrupt received. Exiting.')
        finally:
            #perform any cleanup
            GPIO.cleanup()
            gamepad.close()
    else:
        print('Gamepad not found! Exiting.')
