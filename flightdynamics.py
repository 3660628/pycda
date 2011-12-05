
from aircraft import Aircraft
from controller import Controller
from misc import sign, atmosphere, transformation
from numpy import pi,sqrt,cos,sin, arctan, arcsin, array, exp, zeros, matrix, linspace, arange, savez, tan
from pylab import plot, legend, figure, subplot, copy

degtorad = pi/180
radtodeg = 1/degtorad



class FlightDynamics:
    """This class is a flight dynamics solver which takes input as aircraft and solver for given initial condition, and control action.
       Control action is taken from the object controller!
    """
    def __init__(self,aircraft = Aircraft(), controller = Controller(0.01), x = zeros([12]), u = zeros([7]), _file = 'data.npz', tf = 1500.0, dt = 0.01):
        self.a = aircraft
        self.c = controller
        self.x = x
        self.u = u
        self._file = _file
        self.tf = tf
        self.dt = dt
    def _derivative(self):
        a = self.a
        x = self.x
        u = self.u
        HEB = transformation(x[9],x[10],x[11])
        gb = HEB*matrix([[0.0],[0.0],[9.80665]])
        Va = [x[0],x[1],x[2]]
        V = sqrt(Va[0]**2+Va[1]**2+Va[2]**2)
        alpha = arctan(Va[2] / Va[0])
        beta = arcsin(Va[1] / V)
        T,P = atmosphere(-x[5])
        R = 287.0
        rho = P/R/T
        qbar = .5*rho*V*V
        qbarS = qbar*a.S
        a._calc(Va, x, u)
        CL = a.CL
        CD = a.CD
        CY = a.CY
        Cl = a.Cl
        Cm = a.Cm
        Cn = a.Cn
        thrust = a.thrust 
        Ixx = a.Ixx
        Iyy = a.Iyy
        Izz = a.Izz
        Ixz = a.Ixz

        CX = -CD*cos(alpha) + CL*sin(alpha)
        CZ = -CD*sin(alpha) - CL*cos(alpha)
       
        Xb = (CX*qbarS + thrust)/a.m
        Yb = CY*qbarS/a.m
        Zb = CZ*qbarS/a.m
        Lb = Cl*qbarS*a.b
        Mb = Cm*qbarS*a.c
        Nb = Cn*qbarS*a.b
        nz = -Zb/9.80665
        
        xd0 = Xb + float(gb[0]) + x[8]*x[1] - x[7]*x[2];

	xd1 = Yb + float(gb[1]) - x[8]*x[0] + x[6]*x[2];
	xd2 = Zb + float(gb[2]) + x[7]*x[0] - x[6]*x[1];
	
	y = HEB.transpose()*matrix([[x[0]],[x[1]],[x[2]]])
	xd3 = float(y[0]);
	xd4 = float(y[1]);
	xd5 = float(y[2]);
	
	xd6 = (Izz*Lb + Ixz*Nb - (Ixz*(Iyy - Ixx - Izz)*x[6]+\
                                      (Ixz**2 + Izz*(Izz - Iyy))*x[8]) * x[7]) / (Ixx*Izz - Ixz**2);
	xd7 = 	(Mb - (Ixx - Izz) * x[6] * x[8] - Ixz * (x[6]**2 - x[8]**2)) / Iyy;
	xd8 =	(Ixz*Lb + Ixx*Nb + (Ixz*(Iyy - Ixx - Izz)*x[8] + \
                                        (Ixz**2 + Ixx*(Ixx - Iyy))*x[6])*x[7])/(Ixx*Izz - Ixz**2);

	cosPitch = cos(x[10]);
	if abs(cosPitch) <= 0.00001:
            cosPitch = 0.00001 * sign(cosPitch)
	
	tanPitch	=	sin(x[10]) / cosPitch;
        
	xd9	=	x[6] + (sin(x[9]) * x[7] + cos(x[9]) * x[8]) * tanPitch;
	xd10	=	cos(x[9]) * x[7] - sin(x[9]) * x[8];
	xd11	=	(sin(x[9]) * x[7] + cos(x[9]) * x[8]) / cosPitch;
	
	return array([xd0,xd1,xd2,xd3,xd4,xd5,xd6,xd7,xd8,xd9,xd10,xd11]);
        
    
    def _calc(self):
        t = arange(0,self.tf,self.dt)
        
        #var for saving data
        self.xdata = zeros([12,len(t)])
        self.udata = zeros([7,len(t)])
        self.xddata = zeros([12,len(t)])


                
        for i in range(len(t)):
            self.u = self.c.control(t[i], self.x) 
            self.xdot =  self._derivative()
            self.x += self.xdot*self.dt
            
            #Store data 
            self.xdata[:,i] = self.x
            self.xddata[:,i] = self.xdot
            self.udata[:,i] = self.u
            
            #End Condition
            if -self.x[5] < 0:
                break
        

        savez(self._file, t = t, x = self.xdata, xd = self.xddata, u = self.udata)
            
