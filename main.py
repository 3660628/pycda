from aircraft import SmallBussinessJet
from controller import Controller
from misc import sign, atmosphere, transformation, stepprofile, cdaprofile
from numpy import pi,sqrt,cos,sin, arctan, arcsin, array, exp, zeros, matrix, linspace, arange, savez, zeros_like
from pylab import plot, legend, figure, subplot, show
from flightdynamics import FlightDynamics
from analyzer import _plot
degtorad = pi/180
radtodeg = 1/degtorad
tf = 3500.0
dt = 0.01

#Changing the controller
class TController(Controller):
    def control(self,t,x):
        u = zeros([7])
        self.h_req = stepprofile(x[3])
        thr = self.heightcontrol(-x[5])
        u[3] = max(0,min(1,thr))
        print u[3]
        return u

    


#Defining aircraft
a = SmallBussinessJet()
#Defining controller
c = TController(dt)
#Output data file location
_file = '/home/anand/btpdata/step.npz'

#State vector
x = zeros([12])

#Initialize state vector
x[0] = 200
x[1] = 0.0
x[2] = 0.0
x[5] = -10.0

#Control Vector
u = zeros([7])

#Elevator angle
u[0] = .01
#Throttle
u[3] = .9

#Setting up flight dynamics
f = FlightDynamics(aircraft = a, controller = c, x = x, u = u, _file = _file, tf = tf, dt = dt)
#run
f._calc()

#plot data
_plot(_file)
