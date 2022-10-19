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

    def change_LED(self, r=0, g=0, b=0):
      
        self.lightring.leds = [LedColor(red=r, green=g, blue=b), LedColor(red=r, green=g, blue=b), LedColor(red=r, green=g, blue=b), LedColor(red=r, green=g, blue=b), LedColor(red=r, green=g, blue=b), LedColor(red=r, green=g, blue=b)] 
        self.lights_publisher.publish(self.lightring)

    def reset(self):
        '''
        This function releases contriol of the lights and "gives" it back to the robot. 
        '''
        print('Resetting color to white')
        self.lightring.override_system = False
        white = [LedColor(red=255, green=255, blue=255), LedColor(red=255, green=255, blue=255), LedColor(red=255, green=255, blue=255),
                 LedColor(red=255, green=255, blue=255), LedColor(red=255, green=255, blue=255), LedColor(red=255, green=255, blue=255)]
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
    led_publsiher.change_LED(r=0,g=0,b=255)
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
