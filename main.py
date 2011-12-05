from aircraft import SmallBussinessJet
from controller import Controller
from misc import sign, atmosphere, transformation, stepprofile, cdaprofile
from numpy import pi,sqrt,cos,sin, arctan, arcsin, array, exp, zeros, matrix, linspace, arange, savez, zeros_like
from pylab import plot, legend, figure, subplot, show
from progressbar import ProgressBar, AnimatedProgressBar
from flightdynamics import FlightDynamics
from analyzer import _plot
degtorad = pi/180
radtodeg = 1/degtorad
tf = 2000.0
dt = 0.01

class TController(Controller):
    def control(self,t,x):
        u = zeros([7])
        self.v_req = 140.0
        self.h_req = cdaprofile(x[3])
        thr = self.heightcontrol(-x[5])
        #ele = -self.velocitycontrol(x[0])
        u[3] = max(0,min(1,thr))
        #u[0] = max(-.05,min(.05,ele))
        #u[5] = 1.0
        print u[3]
        return u

    



a = SmallBussinessJet()
c = TController(dt)


_file = './data/test.npz'
x = zeros([12])

x[0] = 140
x[1] = 0.0
x[2] = 0.0
x[5] = -10.0

u = zeros([7])
u[0] = .01
u[3] = .9
f = FlightDynamics(aircraft = a, controller = c, x = x, u = u, _file = _file, tf = tf, dt = dt)
f._calc()
_plot(_file)
