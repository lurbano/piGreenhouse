import RPi.GPIO as GPIO
import asyncio

# Reference: https://ben.akrin.com/?p=9768

class motorU:

    def __init__(self, in1=6, in2=13, in3=19, in4=26,
                       step_sleep=0.001):
        self.step_360 = int(4096)   # number of steps required for 360 degree turn
        self.in1 = in1
        self.in2 = in2
        self.in3 = in3
        self.in4 = in4

        # defining stepper motor sequence (found in documentation http://www.4tronix.co.uk/arduino/Stepper-Motors.php)
        self.step_sequence = [[1,0,0,1],
                 [1,0,0,0],
                 [1,1,0,0],
                 [0,1,0,0],
                 [0,1,1,0],
                 [0,0,1,0],
                 [0,0,1,1],
                 [0,0,0,1]]

        #initialize Pins
        GPIO.setmode( GPIO.BCM )
        GPIO.setup( in1, GPIO.OUT )
        GPIO.setup( in2, GPIO.OUT )
        GPIO.setup( in3, GPIO.OUT )
        GPIO.setup( in4, GPIO.OUT )

        GPIO.output( in1, GPIO.LOW )
        GPIO.output( in2, GPIO.LOW )
        GPIO.output( in3, GPIO.LOW )
        GPIO.output( in4, GPIO.LOW )

        self.motor_pins = [in1,in2,in3,in4]



    def cleanup(self):
        GPIO.output( in1, GPIO.LOW )
        GPIO.output( in2, GPIO.LOW )
        GPIO.output( in3, GPIO.LOW )
        GPIO.output( in4, GPIO.LOW )
        GPIO.cleanup()

    def rotate(self, nRotations=1, direction="clockwise"):
        nsteps = int(self.step_360 * nRotations)
        motor_step_counter = 0
        for i in range(nsteps):
            for pin in range(0, len(self.motor_pins)):
                GPIO.output( motor_pins[pin], step_sequence[motor_step_counter][pin])
            if direction=="clockwise":
                motor_step_counter = (motor_step_counter - 1) % 8
            elif direction == "counterClockwise":
                motor_step_counter = (motor_step_counter + 1) % 8

            time.sleep(self.step_sleep)

    async def aRotate(self, nRotations=1, direction="clockwise"):
        nsteps = int(self.step_360 * nRotations)
        motor_step_counter = 0
        for i in range(nsteps):
            for pin in range(0, len(self.motor_pins)):
                GPIO.output( motor_pins[pin], step_sequence[motor_step_counter][pin])
            if direction=="clockwise":
                motor_step_counter = (motor_step_counter - 1) % 8
            elif direction == "counterClockwise":
                motor_step_counter = (motor_step_counter + 1) % 8

            asyncio.sleep(self.step_sleep)

    def openWindow(self):
        self.rotate(direction="counterClockwise") #just because that's my current physical design

    async def aOpenWindow(self):
        await self.aRotate(direction="counterClockwise") #just because that's my current physical design

    def closeWindow(self, direction=True):
        self.rotate(direction="clockwise")  #just because that's my current physical design

    async def aCloseWindow(self, direction=True):
        await self.rotate(direction="clockwise")  #just because that's my current physical design