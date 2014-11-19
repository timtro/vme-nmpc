import numpy as np
from numpy import array
import pylab as plt
import time

from modules import nmpc_funcs as nmpc

T = 0.03 # Time sample coeff.
n = 5 # state space dimension
m = 1 # control space dimension
N = 80 # Prediction Horizon size
Q = 110 # Weighting param
Q0 = N*1 # Weighting param
R = .5 # Weighting param
mu = 0.02
eps = .02
obstacles = np.array([[5],[5]])
waypts = np.array([[10],[10]])
vref = 0.4 * np.ones(N-1)
H = 4

x = np.zeros( (n, N) )
u = np.random.randn( N-1 ).reshape(1,N-1)
xref = np.zeros( (n, N-1) )
p = np.zeros( (n, N-1) )
Phi = np.zeros( (2, N-1) )
current_grad = np.ones( (m, N-1) )

nmpc.airline_path(x[:,0], vref, xref, T, waypts[:, 0])

xymask = np.array([True, False, True, False, False])

# Write a thing to compute the airline path, get the error and then
# construct the while loop to do a convergence test.

nmpc.f(x, u, vref, Phi, obstacles, T, eps, n)


# plt.ion()
# refline, = plt.plot(xref[0, :], xref[1, :], 'r.')
# line, = plt.plot(x[0,:], x[1,:], 'b-', marker='.')
# obplot, = plt.plot(obstacles[0,:], obstacles[1,:], 'ko', ms = 8)
# gradline, = plt.plot(current_grad[0, :], current_grad[1, :], 'c.')


while np.sqrt(np.sum((x[xymask, 0] - waypts[:, 0])**2)) > .1:

	current_grad = np.ones( (m, N-1) )

	nmpc.f(x, u, vref, Phi, obstacles, T, eps, n)
	nmpc.airline_path(x[:, 0], vref, xref, T, waypts[:, 0])

	# line.set_xdata(x[0, :])
	# line.set_ydata(x[2, :])
	# refline.set_xdata(xref[0, :])
	# refline.set_ydata(xref[2, :])
	# ax = plt.gca()
	# ax.relim()
	# ax.autoscale_view()
	# ax.set_aspect('equal')
	# plt.draw()

	while True:
		nmpc.f(x, u, vref, Phi, obstacles, T, eps, n)
		ex = x[:, 1:] - xref
		p = nmpc.lagrange_mult(x, vref, ex, Phi, Q, Q0, T)
		last_grad = current_grad.copy()
		current_grad = nmpc.grad(u, p, vref, T, R)

		# line.set_xdata(x[0, :])
		# line.set_ydata(x[2, :])
		# refline.set_xdata(xref[0, :])
		# refline.set_ydata(xref[2, :])
		# ax = plt.gca()
		# ax.relim()
		# ax.autoscale_view()
		# ax.set_aspect('equal')
		# plt.draw()

		if np.sum(current_grad**2) < 0.001:
			break

		if (np.sum(current_grad*last_grad)) >= 0:
			mu *= 1.02
			u += mu*current_grad
		else:
			u -= mu*last_grad
			mu /= 3

	# Execute H steps (or simulate by replacing the first element in x)
	x[:, 0] = x[:, H]
	# u[:, :-H] = u[:, H:]
	# time.sleep(5)

print('fin\n')