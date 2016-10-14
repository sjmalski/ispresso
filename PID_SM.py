import time, logging

logger = logging.getLogger('ispresso')

class PID:

    # got this from brettbeauregard.com/blog/2011/04/improving-the-beginner's-pid-initialization/
    def __init__(self, rate, kp, ki, kd):
        self.Input = 0
        self.Output = 0
        self.Setpoint = 0
        self.ITerm = 0
        self.lastInput= 0
        self.lastTime = 0
        self.kp = 2
        self.ki = 30
        self.kd = 15
        self.SampleTime = 1
        self.outMin= 0
        self.outMax = 100
        self.inAuto = False
        self.SetOutputLimits(self.outMin,self.outMax)
        self.SetSampleTime(rate)
        self.SetTunings(kp,ki,kd)
        self.SetMode("auto")
        
    def compute(self,Input,Setpoint):
        self.Input = Input
        self.Setpoint = Setpoint
        now = time.time()
        #timeChange = now-self.lastTime
        timeChange = self.SampleTime
        if timeChange >= self.SampleTime:  #compute all the error working variables
            error = self.Setpoint - self.Input
            self.ITerm += (self.ki*error)
            if (self.ITerm > self.outMax):
                self.ITerm = self.outMax
            elif self.ITerm < self.outMin:
                self.ITerm = self.outMin
            dInput = (self.Input - self.lastInput)

            #compute PID Output
            self.Output = self.kp * error + self.ITerm - self.kd * dInput

            if self.Output > self.outMax:
                self.Output = self.outMax
            elif self.Output < self.outMin:
                self.Output = self.outMin

            #remember some variables for next time
            self.lastInput = self.Input
            self.lastTime = now

            #logger.debug( "computing output: " + str(self.Output) + " error: " + str(error)
            #             + " K, i, D: " + str(self.kp)+" "+ str(self.ki) + " " + str(self.kd)
            #              + " Iterm: " + str(self.ITerm))
            return self.Output

    def SetTunings(self, Kp, Ki, Kd):
        SampleTimeInSec = self.SampleTime
        self.kp = Kp
        self.ki = Ki* SampleTimeInSec
        self.kd = Kd/ SampleTimeInSec
        #logger.debug("PID changed to : " +str(self.kp)+" "+str(self.ki)+" "+str(self.kd))

    def SetSampleTime(self,NewSampleTime):
        if NewSampleTime > 0:
            ratio = NewSampleTime/self.SampleTime
            self.ki*= ratio
            self.kd /= ratio
            self.SampleTime = NewSampleTime

    def SetOutputLimits(self,Min, Max):
        self.outMin = Min
        self.outMax = Max
        if self.Output>self.outMax:
            self.Output = self.outMax
        elif self.Output < self.outMin:
            self.Output = self.outMin
        if self.ITerm > self.outMax:
            self.ITerm = self.outMax
        elif self.ITerm < self.outMin:
            self.ITerm = self.outMin
    def SetMode(self, Mode, output = None):
        newAuto = (Mode == "auto")
        #if output is not None:
            #self.Output = output
        if newAuto and not self.inAuto:
            #we just went from manual to Auto
            self.Output = output
            self.Initialize()
        self.inAuto = newAuto
    def Initialize(self):
        self.lastInput = self.Input
        self.ITerm = self.Output
        if self.ITerm > self.outMax:
            self.ITerm = self.outMax
        elif self.ITerm < self.outMin:
            self.ITerm = self.outMin

if __name__=="__main__":
    sampleTime = 1
    pid = PID(sampleTime,1,1,1)
    temp = 80
    setpoint = 100
    print pid.compute(temp, setpoint)
    
