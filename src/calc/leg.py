from cmath import acos, cos, sin, sqrt
from math import atan2, fmod
import math


class Leg:
    def __init__(self, QB, R0, LB, L0, L1, L2, RB, PB, YB, Q0 = 0, Q1 = 0, Q2 = 0):
        self.QB = QB
        self.R0 = R0
        self.LB = LB
        self.L0 = L0
        self.L1 = L1
        self.L2 = L2
        self.RB = RB
        self.PB = PB
        self.YB = YB
        
        self.Q0 = Q0
        self.Q1 = Q1
        self.Q2 = Q2     

    def calcAngles(self, xe, ye, ze):
        xm11 = self.LB * math.cos(self.QB)
        dxm11 = float(xe - xm11)

        ym11 = self.LB * math.sin(self.QB)
        dym11 = float(ye - ym11)
        
        try: 
            # CALCULATION
            self.Q0 = atan2(dym11, dxm11) + self.R0 - self.QB

            # You must calculate _Q0 before C because the dependencies on xm12 and ym12
            # _QB is the rotation on the motor from the base. It must be subtracted from the equation because the motor is already rotated this angle
            # _R0 is the extra rotation on the motor. Sarge 5 doesn't have any extra rotation but Sarge 6 does.

            # CALCULATING C
            xm12 = xm11 + self.L0*math.cos(self.Q0 - self.R0 + self.QB)
            ym12 = ym11 + self.L0*math.sin(self.Q0 - self.R0 + self.QB)
            # For the angle of _Q0 we have to subtract out _R0 and add back in _QB to get our position
            # correct for the math that follows in calculating the rest of the variables.

            dxm12 = xe - xm12
            dym12 = ye - ym12

            C = math.sqrt((pow(dxm12,2) + pow(dym12,2)))

            # CALCULATION
            self.Q2 = - math.acos( (pow(C,2) + pow(ze,2) - pow(self.L1,2) - pow(self.L2, 2))/(2*self.L1*self.L2) )
            self.Q1 = atan2(ze, C) - atan2((self.L2*math.sin(self.Q2)),(self.L1 + self.L2*math.cos(self.Q2))) # CHANGE THE FIRST TO (-) IF YOU WANT THE OTHER POSE POSITION
        except ValueError:
            print("POINTS OUT OF RANGE")

    def getQ0(self):
        return self.normalize(self.Q0)
    
    def getQ1(self):
        return self.normalize(self.Q1)

    def getQ2(self):
        return self.normalize(self.Q2)
    
    def normalize(self, x):
        return fmod(x + math.pi, 2.0 * math.pi) - math.pi

    