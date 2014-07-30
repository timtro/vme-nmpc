import numpy as np
from numpy import array
import pylab as plt
import time

T = 0.012 # Time sample coeff

n = 2 # state space dimension
m = 2 # control space dimension
N = 50 # Prediction Horizon size
Q = 50 # Weighting param
Q0 = 50 # Weighting param
R = 5 # Weighting param
mu = 0.20

def f(x, u):
	'''
	Based on the velocity values in u (we control velocity), this function uses
	Euler integration to predict x over the horizon of N points.
	'''
	x[:, 1:] = 0
	x[:, 1:] = np.cumsum(x[:, :-1], axis=1) + np.cumsum(u[:, :-1], axis=1) * T

def lagrange_mult(ex, Phi):
	'''
	Ugly lagrange thingy
	'''
	# TODO Must I compute Phi[:, -1]?
	lp = np.zeros( (n, N) )
	lp[:, -1] = ex[:, -1] * Q0
	lp[:, :-1] = ex[:, :-1] * Q - Phi[:, :-1]
	return lp + np.cumsum(lp[:, ::-1], axis=1)[:, ::-1]

def grad(u, p):
	'''
	Returns the gradient of the function we are minimizing.
	'''
	return R*u + T*p

x = np.zeros( (n, N) )
xref = np.cumsum(np.ones( (n, N) )*T, axis=1)
u = np.ones( (n, N) )
u[1,:] = -u[1,:]
p = np.zeros( (n, N) )
Phi = np.zeros( (2, N) )
current_grad = np.ones( (n, N) )


# Write a thing to compute the airline path, get the error and then
# construct the while loop to do a convergence test.

plt.ion()
f(x, u)
refline = plt.plot(xref[0, :], xref[1, :], 'r.')
line, = plt.plot(x[0,:], x[1,:], 'b.')
gradline, = plt.plot(current_grad[0, :], current_grad[1, :], 'c.')

maxed_step = False

flipcount = 0

while True:
	f(x, u)
	ex = x-xref
	p = lagrange_mult(ex, Phi)
	last_grad = current_grad.copy()
	current_grad = grad(u,p)

	if np.sum(current_grad**2) < 0.01:
		u -= mu*current_grad
		break


	if (np.sum(current_grad*last_grad)) > 0:
		if not maxed_step and (flipcount > 3):
			mu *= 2
			print('yo')
	else:
		flipcount += 1
		mu /= 2
		maxed_step = True
		u += mu * last_grad
		f(x, u)
		print('mo')

	u -= mu*current_grad

	line.set_xdata(x[0, :])
	line.set_ydata(x[1, :])
	gradline.set_xdata(current_grad[0, :])
	gradline.set_ydata(current_grad[1, :])
	ax = plt.gca()
	#ax.relim()
	#ax.autoscale_view()
	plt.draw()
	time.sleep(0.5)


print(u)

print('fin\n')