'''
pub_lightring.py
Tufts Create®3 Educational Robot Example
by Maddie Pero 

In this example we will publish random colors to the LED ring on the Create®3.
'''

'''
These statements allow the Node class to be used.
'''
import sys
import rclpy
from rclpy.node import Node

'''
These statements import iRobot Create®3 messages and actions.
'''
from irobot_create_msgs.msg import LedColor
from irobot_create_msgs.msg import LightringLeds

'''
Input your namespace here as a global variable. 
'''
namespace = '[Namespace]'

class ColorPalette():      
    '''
    This class defines a class of frequently used colors for easy access later on in the script.
    It is essentially converting RGB values to common colors.
    '''
    def __init__(self):
        print('Common colors that will be used later in the script are being initialized.')
        self.red = LedColor(red=255, green=0, blue=0)
        self.green = LedColor(red=0, green=255, blue=0)
        self.blue = LedColor(red=0, green=0, blue=255)
        self.yellow = LedColor(red=255, green=255, blue=0)
        self.pink = LedColor(red=255, green=0, blue=255)
        self.cyan = LedColor(red=0, green=255, blue=255)
        self.purple = LedColor(red=127, green=0, blue=255)
        self.white = LedColor(red=255, green=255, blue=255)
        self.grey = LedColor(red=189, green=189, blue=189)
        self.tufts_blue = LedColor(red=98, green=166, blue=10)
        self.tufts_brown = LedColor(red=94, green=75, blue=60)

class LEDPublisher(Node):
    '''
    The LEDPublisher class is created which is a subclass of Node.
    This defines the class' constructor.
    '''
    def __init__(self):    
        '''
        The following line calls the Node class's constructor and gives it the Node name,
        which is 'led_publisher.'
        '''
        super().__init__('led_publisher')

        self.cp = ColorPalette()
        
        '''
        We are declaring how we want the Node to publish message. We've imported LightringLeds
        from irobot_create_msgs.msg over the topic '/cmd_lightring' with a queue size of 10.
        Queue size is a quality of service setting that limiits amount of queued messages.
        Basically, we are determining what type of data we want to publish. 
        '''
        self.lights_publisher = self.create_publisher(LightringLeds, namespace + '/cmd_lightring', 10)
        
        '''
        The timer allows the callback to execute every 2 seconds, with a counter iniitialized.
        '''
        self.lightring = LightringLeds()
        self.lightring.override_system = True

    def change_LED(self):
      
        self.lightring.leds = [self.cp.blue, self.cp.green, self.cp.blue, self.cp.green, self.cp.blue, self.cp.green] 
        self.lights_publisher.publish(self.lightring)

    def reset(self):
        '''
        This function releases contriol of the lights and "gives" it back to the robot. 
        '''
        print('Resetting color to white')
        self.lightring.override_system = False
        white = [self.cp.white, self.cp.white, self.cp.white,
                 self.cp.white, self.cp.white, self.cp.white]
        self.lightring.leds = white

        self.lights_publisher.publish(self.lightring)


def main(args=None):
    '''
    The rclpy library is initialized.
    '''
    rclpy.init(args=args)
    
    '''
    The node is created and can be used in other parts of the script.
    '''
    led_publisher = LEDPublisher()

    '''
    The node is "spun" so the callbacks can be called.
    '''
    led_publsiher.change_LED()
    try:
        rclpy.spin(led_publisher)
    except KeyboardInterrupt:
        print('\nCaught Keyboard Interrupt')
    finally:
        print("Done")  # Destroy the node explicitly
        led_publisher.reset()
        led_publisher.destroy_node()
        print('shutting down')
        rclpy.shutdown()


if __name__ == '__main__':
    main()