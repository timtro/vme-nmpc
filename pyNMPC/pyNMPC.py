import numpy as np
from numpy import array
import pylab as plt
import time

T = 0.012 # Time sample coeff.
n = 2 # state space dimension
m = 2 # control space dimension
N = 50 # Prediction Horizon size
Q = 60 # Weighting param
Q0 = 200 # Weighting param
R = 10 # Weighting param
mu = 0.05
eps = .13
obstacles = np.array([[.08],[.2]])
vref=0.4

def f(x, u, Phi, obst):
	'''
	Based on the velocity values in u (we control velocity), this function uses
	Euler integration to predict x over the horizon of N points.
	'''
	x[:, 1:] = np.cumsum(u, axis=1) * T
	Phi[:,:] = 0
	for obs in obst.T:
		dx = x[:,1:]-obs.reshape(2, 1)
		Phi[:,:] += 2 * dx / (np.sqrt(np.sum(dx**2, axis=0))+eps)**2

def lagrange_mult(ex, Phi):
	'''
	Ugly lagrange thingy
	'''
	# TODO Must I compute Phi[:, -1]?
	lp = np.zeros( (n, N-1) )
	lp[:, -1] = ex[:, -1] * Q0
	lp[:, :-1] = ex[:, :-1] * Q - Phi[:, :-1]
	return lp + np.cumsum(lp[:, ::-1], axis=1)[:, ::-1]

def grad(u, p):
	'''
	Returns the gradient of the function we are minimizing.
	'''
	v = np.sqrt(np.sum(u**2, axis=0))
	#print(v)
	return T*p - R*( (vref/v) - 1) * u

x = np.zeros( (n, N) )
u = np.ones( (n, N-1) )*vref/np.sqrt(2)
xref = np.cumsum(u*T, axis=1)
u[1,:] = -u[1,:]
p = np.zeros( (n, N-1) )
Phi = np.zeros( (2, N-1) )
current_grad = np.ones( (n, N-1) )

# Write a thing to compute the airline path, get the error and then
# construct the while loop to do a convergence test.

plt.ion()
f(x, u, Phi, obstacles)
refline, = plt.plot(xref[0, :], xref[1, :], 'r.')
line, = plt.plot(x[0,:], x[1,:], 'b-', marker='.')
obplot, = plt.plot(obstacles[0,:], obstacles[1,:], 'rx')
#gradline, = plt.plot(current_grad[0, :], current_grad[1, :], 'c.')

maxed_step = False
flipcount = 0

while True:
	f(x, u, Phi, obstacles)
	#print((x[1:]-x[:-1])/T)
	print(u)
	ex = x[:, 1:]-xref
	p = lagrange_mult(ex, Phi)
	last_grad = current_grad.copy()
	current_grad = grad(u,p)

	# if np.sum(current_grad**2) < 0.00001:
	# 	u -= mu*current_grad
	# 	break

	# if (np.sum(current_grad*last_grad)) > 0:
	# 	mu *= 2
	# else:
	# 	mu /= 2

	if np.sum(current_grad**2) < 0.0001:
		u -= mu*current_grad
		break

	if (np.sum(current_grad*last_grad)) > 0:
		if not maxed_step and (flipcount > 5):
			mu *= 2
	else:
		flipcount += 1
		mu /= 2
		maxed_step = True
		u += mu * last_grad
		f(x, u, Phi, obstacles)

	u -= mu*current_grad

	line.set_xdata(x[0, :])
	line.set_ydata(x[1, :])
#	gradline.set_xdata(current_grad[0, :])
#	gradline.set_ydata(current_grad[1, :])
	ax = plt.gca()
	#ax.relim()
	#ax.autoscale_view()
	plt.draw()
	#time.sleep(0.55)

print('fin\n')