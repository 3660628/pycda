import cPickle as pickle
from pylab import plot, subplot, figure, legend, grid, show
from numpy import load, arctan, zeros_like
from misc import stepprofile, cdaprofile
def _plot(file = '/home/anand/btpdata/case_7.npz'):
    _file = load(file)
    t = _file['t']
    x = _file['x']
    xd = _file['xd'] 
    u = _file['u'] 
    subplot(311)
    plot(t, x[0])
    plot(t, x[1])
    plot(t, x[2])
    grid()
    legend(['Vx','Vy','Vz'])
    subplot(312)
    plot(t, x[9])
    plot(t, x[10])
    plot(t, x[11])
    plot(t, arctan(x[2]/x[0]))
    grid()
    legend(['Phi','Theta','Psi','alpha'])
    subplot(313)
    plot(x[3],-x[5])
    grid()
    
    y = zeros_like(x[3])
    for i in range(len(x[3])):
        y[i] = stepprofile(x[3][i])
    plot(x[3],y)
    
    show()
    figure()
    plot(u[0])
    plot(u[3])
    show()
def comparison():
    cda = load('/home/anand/btpdata/cda.npz')
    step = load('/home/anand/btpdata/step.npz')
    uc = cda['u']
    us = step['u']
    subplot(211)
    plot(uc[3])
    plot(us[3])
    dt = 0.01
    fc = 0.0
    fs = 0.0
    tmpfc = zeros_like(uc[0])
    tmpfs = zeros_like(uc[0])
    for i in range(len(uc[0])):
        print i
        fc += uc[3][i]*dt
        fs += us[3][i]*dt
        tmpfc[i] = fc
        tmpfs[i] = fs
    subplot(212)
    plot(tmpfc)
    plot(tmpfs)
    show()
    
