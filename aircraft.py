#This file contains aicrafts which can be imported in the flightdynamics model
#Data Structure
#Aicraft
#   function: _inertial, define inertia related parameters
#   function: _geometry, define geometry related parameters 
#   function: _calcmach, calculate mach parameters
#   function: _CL, calculate CL 
#   function: _CD, calculate CD 
#   function: _Cm, calculate Cm 
#   function: _CY, calculate CY 
#   function: _Cn, calculate Cn 
#   function: _Cl, calculate Cl 
#   function: thrust, calculate thrust
#   function: _calc
#      -> call: _calcmach, _CL, _CD, _Cm, _CY, _Cn, _Cl and thrust in order 

#Note: Once initialized only _calc function need to be called from flight dynamics model,
#      at every time step, just before calculating the derivative. After calling the fun-
#      ction, value of aircraft parameters are automatically updated which can be used for
#      calculataions. 

from misc import sign, atmosphere, transformation
from numpy import pi,sqrt,cos,sin, arctan, arcsin, array, exp, zeros, matrix, linspace, arange
from pylab import plot, legend, figure, subplot
from progressbar import ProgressBar, AnimatedProgressBar
degtorad = pi/180
radtodeg = 1/degtorad

class Aircraft:
    """Base class of aircraft, contains aircraft parameters and functions to estimates them.
    
       Parameters
       ----------
       gear        <type: bool>  : True if gear down, else False
       spoilers    <type: bool>  : True if spoilers deployed, else False
       flaps       <type: bool>  : True if flaps deployed, else False
       
       m           <type: float> : Mass
       Ixx         <type: float> : Moment of Inertia about x axis
       Iyy         <type: float> : Moment of Inertia about y axis
       Izz         <type: float> : Moment of Inertia about z axis
       Ixz         <type: float> : Moment of Inertia about xz axis
       
       c           <type: float> : Chord (m)
       b           <type: float> : Span (m)
       S           <type: float> : Reference Area (m^2)
       ARw         <type: float> : Wing Aspect ratio
       taperw      <type: float> : Wing Taper ratio
       sweepw      <type: float> : Wing Sweep in rad
       ARh         <type: float> : H-Tail Aspect ratio
       sweeph      <type: float> : H-Tail Sweep in rad
       ARv         <type: float> : V-Tail Aspect ratio
       sweepv      <type: float> : V-Tail Sweep in rad
       lvt         <type: float> : V-Tail length
       Ts          <type: float> : Dry Thrust

       CL          <type: float> : Coeff of Lift
       CD          <type: float> : Coeff of Drag
       Cm          <type: float> : Coeff of Moment
       CY          <type: float> : Coeff of side force
       Cn          <type: float> : Coeff of Moment
       Cl          <type: float> : Coeff of Moment
       thrust      <type: float> : Thrust (N)
       
    """
    def __init__(self):
        self.gear = True     #True->down
        self.spoilers = True  #True->open
        self.flaps = True    #True->open
        self._inertial()
        self._geometry()
    
    def _inertial(self):
        self.m = 4536.0 #Kg
        self.Ixx = 35926.5 #Kg-m2
        self.Iyy = 33940.7 #Kg-m2
        self.Izz = 67085.5 #Kg-m2
        self.Ixz = 3418.17 #Kg-m2

    def _geometry(self):
        self.c = 2.14 #m
        self.b = 10.4 #m
        self.S = 21.5 #m2
        self.ARw = 5.02 
        self.taperw = 0.507
        self.sweepw = 13*degtorad #rad
        self.ARh = 4.0
        self.sweeph = 25*degtorad #rad
        self.ARv = 0.64
        self.sweepv = 40*degtorad #rad
        self.lvt = 4.72 #m
        self.Ts = 26243.2 #N
        
    def _calcmach(self, Mach):
        self.prfac=1 / (sqrt(1 - Mach**2) * 1.015)
        self.wingmach = 1 / ((1 + sqrt(1 + ((self.ARw/(2 * cos(self.sweepw)))**2) \
                                           * (1 - Mach*Mach * cos(self.sweepw)))) * 0.268249)
        self.htailmach = 1 / ((1 + sqrt(1 + ((self.ARh/(2 * cos(self.sweeph)))**2) \
                                            * (1 - Mach*Mach * cos(self.sweeph)))) * 0.294539);
        self.vtailmach = 1 / ((1 + sqrt(1 + ((self.ARv/(2 * cos(self.sweepv)))**2) \
                                            * (1 - Mach*Mach * cos(self.sweepv)))) * 0.480338);
        
    def _CL(self, V,alpha,q,stab,deltae):
        Clo = 0.1095
        if self.gear:
            Clo -= 0.0192
        if self.flaps:
            Clo += 0.5182
        if self.spoilers:
            Clo -= 0.1897

        CLa = 5.6575
        if self.flaps:
            CLa -= 0.0947
        
        CLq = 4.231*self.c/(2*V)
        
        CLs = 1.08
        if self.flaps:
            CLs -= 0.4802
        
        CLe = .5774
        if self.flaps:
            CLe -= 0.2665
        self.CL = Clo + (CLa*alpha + CLq*q + CLs*stab + CLe*deltae)*self.wingmach
    
    def _CD(self):
        CDo = 0.0255
        if self.gear:
            CDo += 0.0191
        if self.flaps:
            CDo += 0.0836
        if self.spoilers:
            CDo += 0.0258
        
        epsilon = 0.0718
        if self.flaps:
            epsilon = 0.079
        self.CD = CDo*self.prfac + epsilon*self.CL*self.CL
    
    def _Cm(self, V, alpha, p, q, deltae):
        cmo = 0.0
        if self.gear:
            cmo += 0.0255
        if self.flaps:
            cmo -= 0.058
        if self.spoilers:
            cmo -= 0.0154
        cmar = -1.231
        if self.flaps:
            cmar += 0.0138
        cmqr = -18.8*self.c/(2*V)
        cmdsr = -2.291
        if self.flaps:
            cmdsr += 0.121
        cmder = -1.398
        if self.flaps:
            cmder += 0.149
        self.Cm = cmo + (cmar*alpha + cmqr*q + cmdsr*p + cmder*deltae)*self.htailmach
        
    def _CY(self, beta, rudder, aileron, spoiler):
        cybr = -0.7162
        if self.flaps:
            cybr += 0.0826
        cydar = -0.00699
        cydrr = 0.1574
        if self.flaps:
            cydrr -= 0.0093
        cydasr = 0.0264
        if self.flaps:
            cydasr == 0.0766

        self.CY = (cybr*beta + cydrr*rudder)*self.vtailmach + (cydar*aileron + cydasr*spoiler )*self.wingmach     

    def _Cn(self, beta, V, r, p, aileron, rudder, spoiler):
        cnbr = 0.1194
        if self.flaps:
            cnbr -= 0.0092
        cnpr = self.CL*(1+3*self.taperw)/(12*(1+self.taperw))*(self.b/(2*V))
        cnrr = (-2*(self.lvt/self.b)*cnbr*self.vtailmach - 0.1*self.CL*self.CL)*(self.b/(2*V))
        cndar = 0.0
        if self.flaps:
            cndar += 0.0028
        cndrr = -0.0713
        if self.flaps:
            cndrr -= 0.0185
        cndasr = -0.0088
        if self.flaps:
            cndasr -= 0.0106
        self.Cn = (cnbr*beta + cndrr*rudder)*self.vtailmach + cnrr*r + cnpr*p \
            + (cndar*aileron + cndasr*spoiler)*self.wingmach
    
    def _Cl(self, beta,p,r,spoiler,rudder,aileron, V, Mach):
        clbr = -0.0918
        if self.flaps:
            clbr -= 0.0092
        clar = 5.6575
        if self.flaps:
            clar -= 0.0947
        clpr = -clar*(1+3*self.taperw)/(12*(1+self.taperw))*(self.b/(2*V))
        clrr = (self.CL*(1+3*self.taperw)/(12*(1+self.taperw))*((Mach*cos(self.sweepw))**2-2)/((Mach*cos(self.sweepw))**2-1))*(self.b/(2*V))
        
        cldar = 0.1537
        if self.flaps:
            cldar += 0.01178
        cldrr = 0.01208
        if self.flaps:
            cldrr += 0.01115
        cldasr = -0.01496
        if self.flaps:
            cldasr -= 0.02376
        self.Cl = (clbr*beta+cldrr*rudder)*self.vtailmach + clrr*r + clpr*p + \
            (cldar*aileron+cldasr*spoiler)*self.wingmach
        
    def _thrust(self,throttle, rho, h):
        self.thrust = throttle*self.Ts*((rho/1.225)**.7)*(1-exp((h-17000)/2000.0));
    
    def _calc(self, Va, x, u):
        """
        V: velocity
        x: state vector
        u: input vector
        """
        #Calc Atmospheric Parameters
        R = 287.0
        gamma = 1.4
        
        
        #Va = [50.0,0.0,0.0] 
        V = sqrt(Va[0]**2+Va[1]**2+Va[2]**2)
        
        alpha = arctan(Va[2]/Va[0])
        
        beta = arcsin(Va[1]/V)
        #u = array([0.0,0.0,0.0,0.0,0.0,0.0,0.0])
        #x = array([V*cos(alpha)*cos(beta),V*sin(beta),V*sin(alpha)*cos(beta),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
        elevator = u[0]
        aileron = u[1]
        rudder = u[2]
        throttle = u[3]
        spoiler = u[4]
        flap = u[5]
        stab = u[6]
        if flap >= 0.65:
            self.flaps = True
        else:
            self.flaps = False
        self.gear = False
        self.spoilers = False
        height = -x[5]
        
        T,P = atmosphere(height)
        
        rho = P/(R*T)
        a = sqrt(gamma*R*T)
        p = x[6]
        q = x[7]
        r = x[8]

        #Calc Mach
        Mach = V/a
        
        self._calcmach(Mach)
        self._thrust(throttle, rho, height)
        self._CL(V, alpha, q, stab, elevator)
        self._CD()
        self._Cm(V, alpha, p, q, elevator)
        self._CY(beta, rudder, aileron, spoiler)
        self._Cn(beta, V, r, p, aileron, rudder, spoiler)
        self._Cl(beta, p, r, spoiler, rudder, aileron, V, Mach)


class SmallBussinessJet(Aircraft):
    pass 
