


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
        self.vf_p = 0.0
        self.vf_i = 0.0
        self.maintain_height = False
        self.maintain_heading = False
        self.maintain_velocity = False
        self.cda = False
        
        self.required_heading = 0.0
        self.required_height = 0.0
        self.v_req = 100.0
        self.h_req = 200.0
        self.dt = dt
        self.psi_req = 0.0
    def heightcontrol(self,h):
        Ku = 4.4
        Tu = 90.0
        Kp = .7*Ku
        Kd = 1.5*Kp*Tu/8.0
        Ki = 2*Kp/Tu
        self.lf_e = (self.h_req-h)/10000.0
        self.lf_d = (self.lf_e - self.lf_p)/self.dt
        if Ki*(self.lf_i + self.lf_e*self.dt) < 1.0 and Ki*(self.lf_i + self.lf_e*self.dt) > -1.0:  
            self.lf_i += self.lf_e*self.dt
        self.lf_p = self.lf_e
        #5 100 0.01
        #max(-.5,min(.5,Kd*self.lf_d))
        return Kp*self.lf_e + Ki*self.lf_i + Kd*self.lf_d
    def velocitycontrol(self,v):
        self.vf_e = (self.v_req-v)/100.0
        self.vf_d = (self.vf_e - self.vf_p)/self.dt
        if self.vf_i + self.vf_e*self.dt < 1.0 and self.vf_i + self.vf_e*self.dt > -1.0:  
            self.vf_i += self.vf_e*self.dt
        self.vf_p = self.vf_e
        #5 100 0.01
        Kp = .1
        Kd = 100.0
        Ki = 0.01
        return Kp*self.lf_e + Ki*self.lf_i + max(0,min(.5,Kd*self.lf_d))

    def trim(self, _input,):
        v_req = _input[0] 
        w_req = _input[1]
        h_req = _input[2]
        
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

    # def J(self, x):
    #     #return u*u + h*h + w*w
    #     return x[0]*x[0] + x[5]*x[5] + x[2]*x[2]
    
    # def derJ(self):
    #     J = self.J
    #     _J = zeros([4])
    #     de = 0.001
    #     dT = 0.001
    #     dtheta = 0.001
    #     dh = 1.0
    #     x = copy(self.x)
    #     tmpu = copy(self.u)
    #     self.u[0] += de
       
    #     xdot = self._derivative()
    #     x += xdot*.01
    #     _J[0] = (J(x) - J(self.x))/de

    #     self.u = copy(tmpu)
    #     x = copy(self.x)
    #     self.u[3] += dT
    #     xdot = self._derivative()
    #     x += xdot*.01
        
    #     _J[1] = (J(x) - J(self.x))/dT
        
    #     x = copy(self.x)
    #     x[10] += dtheta
    #     xdot = self._derivative()
    #     x += xdot*.01
        
    #     _J[2] = (J(x) - J(self.x))/dtheta
    #     self.u = tmpu
    #     x = copy(self.x)
    #     x[5] -= dh
    #     xdot = self._derivative()
    #     x += xdot*.01
    #     _J[3] = (J(x) - J(self.x))/dh
    #     self.u = tmpu
      
        
    #     print _J
    #     return _J
