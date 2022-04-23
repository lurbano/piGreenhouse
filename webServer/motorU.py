import RPi.GPIO as GPIO
import asyncio
import time

# Reference: https://ben.akrin.com/?p=9768

class motorU:

    def __init__(self, in1=6, in2=13,
        in3=19, in4=26,
        step_sleep=0.001,
        trigT = 32.):

        self.step_360 = int(4096)   # number of steps required for 360 degree turn
        self.in1 = in1
        self.in2 = in2
        self.in3 = in3
        self.in4 = in4

        self.step_sleep = step_sleep

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

        # for greenhouse
        self.trigT = trigT
        self.windowOpen = False


    def cleanup(self):
        GPIO.output( self.in1, GPIO.LOW )
        GPIO.output( self.in2, GPIO.LOW )
        GPIO.output( self.in3, GPIO.LOW )
        GPIO.output( self.in4, GPIO.LOW )
        GPIO.cleanup()

    def rotate(self, nRotations=1, direction="clockwise"):
        nsteps = int(self.step_360 * nRotations)
        motor_step_counter = 0
        for i in range(nsteps):
            for pin in range(0, len(self.motor_pins)):
                GPIO.output( self.motor_pins[pin], self.step_sequence[motor_step_counter][pin])
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
                GPIO.output( self.motor_pins[pin], self.step_sequence[motor_step_counter][pin])
            if direction=="clockwise":
                motor_step_counter = (motor_step_counter - 1) % 8
            elif direction == "counterClockwise":
                motor_step_counter = (motor_step_counter + 1) % 8

            await asyncio.sleep(self.step_sleep)

    def openWindow(self):
        self.rotate(direction="counterClockwise") #just because that's my current physical design
        self.windowOpen = True

    async def aOpenWindow(self):
        await self.aRotate(direction="counterClockwise") #just because that's my current physical design
        self.windowOpen = True

    def closeWindow(self, direction=True):
        self.rotate(direction="clockwise")  #just because that's my current physical design
        self.windowOpen = False

    async def aCloseWindow(self, direction=True):
        await self.aRotate(direction="clockwise")  #just because that's my current physical design
        self.windowOpen = False

    def setTrigT(self, T):
        self.trigT = float(T)

    async def aTControl(self, sensor, dt = 60):
        # sensor is the temperature sensor instance of sensor_T
        print(f"Greenhouse Window Control On ({self.trigT})")
        self.TControlOn = True
        while self.TControlOn:
            T = await sensor.aRead_basic()
            if T > self.trigT and self.windowOpen == False:
                print("OPENING WINDOW")
                await self.aOpenWindow()

            elif T < self.trigT and self.windowOpen:
                print("CLOSING WINDOW")
                await self.aCloseWindow()

            await asyncio.sleep(dt)
