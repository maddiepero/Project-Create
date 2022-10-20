'''
pub_lightring.py
Tufts Create®3 Educational Robot Example
by Maddie Pero 


'''

'''
The second script we will show you how to write today is a publisher with rclpy
The format is going to be pretty similar to the subscriber so we will move through 
this one a little quicker

This publisher will change the color of the lights on the Create
The create has 6 LEDs in its lighting and each of those LEDs has a r,g and b value 
that determines its color


First we will import sys, rclpy and rclpy.node as we did before
'''
import sys
import rclpy
from rclpy.node import Node

'''
Then we will import the necessary create messages 

This time they are LedColor and LightringLeds

If you go to irobot create messages you can see the the Lightring Leds requires the
LedColor message because LedColor controls one of the six LEDs as explained before 

'''
from irobot_create_msgs.msg import LedColor
from irobot_create_msgs.msg import LightringLeds

'''
We will define the namespace, again this is just good practice to put it in up top that
 makes it easy to change if you run this code for a different robot and less complicated 
when your code gets really long
'''
namespace = 'JonSnow'

class LEDPublisher(Node):
    '''
    Lets define the class for the publisher & initilaize it

    '''
    def __init__(self):    
        super().__init__('led_publisher')
        
        '''
Then we will create the publisher using a very similar format as with the subscriber
We’ll name our publisher lights_publisher
 tell it that it will send the LightringLeds message, over the topic /cmd_lightring 
(reminder to include your namespace here so that it is directed to your specific robot) 
and finally with a queue size of 10.

Queue size is a quality of service setting that limits the amount of queued messages. 
Similarly to how we told the subscriber what type of data it would be receiving when we 
define a publisher we have to tell it how much data it can hold. 

        '''
        self.lights_publisher = self.create_publisher(LightringLeds, namespace + '/cmd_lightring', 10)
        '''
        Next we’ll define the LightrinLeds() message in the class so that we can use the . operator on it to define different components 
        Then like I just explained we can use the . operator to set the override_system boolean to true
        Again we know we have to do this if we go back to irobot create messages, it tells us that one of the required inputs is whether 
we are overriding the system 
        '''
        
        self.lightring = LightringLeds()
        self.lightring.override_system = True
        '''
       We will create a timer callback that executes every 2 seconds, this way the message will publish over the timer every 2 seconds,
       we'll name it change_LED & this will ensure the robot continously recieves the message

        '''
        timer_period = 2
        self.timer = self.create_timer(timer_period, self.change_LED)

'''Next we will define a function that changes the led color
The arguments will be self and r,g,b values between 0 & 255
'''
    def change_LED(self, r=0, g=0, b=255):
        '''
        The color of each led gets defined with LedColor(red =0, green=0, blue=0) 
We know the LedColor takes information in this way by going to the irobot create messages and seeing it takes three three integer variables for red green and blue between the values of 0 & 255
'''
        self.lightring.leds = [LedColor(red=r, green=g, blue=b), LedColor(red=r, green=g, blue=b), LedColor(red=r, green=g, blue=b), LedColor(red=r, green=g, blue=b), LedColor(red=r, green=g, blue=b), LedColor(red=r, green=g, blue=b)] 
'''There are six Leds and we know from the lighting message that it wants a list of the 6 LedColors''' 
        self.lights_publisher.publish(self.lightring)
    '''then we will publish that color '''

    
'''The final function within this class will reset the color of the ring to white and give the robot back control of the lights
The function will take no arguments'''
    def reset(self):
        '''
        We’ll set the override_system boolean to false
        '''
        self.lightring.override_system = False
        '''Define the color white '''
        white = [LedColor(red=255, green=255, blue=255), LedColor(red=255, green=255, blue=255), LedColor(red=255, green=255, blue=255),
                 LedColor(red=255, green=255, blue=255), LedColor(red=255, green=255, blue=255), LedColor(red=255, green=255, blue=255)]
        self.lightring.leds = white
        ''''And then publish that color in the same way we did the other one '''
        self.lights_publisher.publish(self.lightring)

        '''Lets define the main function again outside of the class'''
def main(args=None):
    '''
    We’ll initialize the rclpy library 
    '''
    rclpy.init(args=args)
    
    '''
    Create the node so it can be used later in the script 
    '''
    led_publisher = LEDPublisher()

    '''
    And then we’ll call the change_led function and set the blue value to 255 which should change the light ring blue 
    '''
    try:
        rclpy.spin(led_publisher)
        '''We’ll spin the node'''
    except KeyboardInterrupt:
        print('\nCaught Keyboard Interrupt')
        ''''And then at a keyboard interrupt we’ll reset the lights, destroy the node and shutdown rclpy'''
    finally:
        print("Done")  # Destroy the node explicitly
        led_publisher.reset()
        led_publisher.destroy_node()
        print('shutting down')
        rclpy.shutdown()

'''Finally we’ll run the main function '''
if __name__ == '__main__':
    main()
