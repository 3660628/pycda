


from misc import sign, atmosphere, transformation
from numpy import pi,sqrt,cos,sin, arctan, arcsin, array, exp, zeros, matrix, linspace, arange
from pylab import plot, legend, figure, subplot

degtorad = pi/180
radtodeg = 1/degtorad


class Controller:
    def __init__(self,dt):
        self.lf_p = 0.0
        self.lf_i = 0.0
        self.lf_e = 0.0
        self.lf_d = 0.0
        self.maintain_height = False
        self.maintain_heading = False
        self.maintain_velocity = False
        self.cda = False
        
        self.required_heading = 0.0
        self.required_height = 0.0
        
        self.h_req = 200.0
        self.dt = dt
        self.psi_req = 0.0
    def heightcontrol(self,h):
        self.lf_e = (self.h_req-h)/10000.0
        self.lf_d = (self.lf_e - self.lf_p)/self.dt
        self.lf_i += self.lf_e*self.dt
        self.lf_p = self.lf_e
        #5 100 0.01
        Kp = 5.0
        Kd = 120.0
        Ki = 0.01
        return Kp*self.lf_e + Ki*self.lf_i + max(0,min(.5,Kd*self.lf_d))
    def pitchcontrol(self,theta):
        pass
    def headingcontrol(self, psi):
        error = (self.psi_req-psi)/pi
        Kp = -.1
        return Kp*error

    def control(self,t):
        u = zeros([7])
        if t > 750.0 and t < 750.1:
            u[0] = 0.2
        else:
            u[0] = 0.0

        if self.maintain_height:
            pass
        if self.maintain_velocity:
            pass
        if self.maintain_heading:
            pass
        u[1] = 0.0
        u[2] = 0.0
        u[3] = 1.0
        u[4] = 0.0
        u[5] = 0.0
        u[6] = 0.0
        return u
