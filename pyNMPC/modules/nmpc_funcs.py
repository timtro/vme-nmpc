import numpy as np

def airline_path( x0, vref, xref, T, tgt):
	print(x0)
	direction = (tgt-x0)
	direction = direction.reshape(2,1) / np.sqrt(2 * np.sum(direction**2)) # The factor of 1/sqrt(2) belons to vref, but is put here to reduce computation
	xref[:,:] = vref*T*direction
	xref[:,:] = x0.reshape(2,1) + np.cumsum(xref, axis=1)

def f(x, u, Phi, obst, T, eps):
	'''
	Based on the velocity values in u (we control velocity), this function uses
	Euler integration to predict x over the horizon of N points.
	'''
	x[:, 1:] = np.cumsum(u, axis=1) * T
	Phi[:,:] = 0
	for obs in obst.T:
		dx = x[:,1:]-obs.reshape(2, 1)
		Phi[:,:] +=  2* dx / (np.sqrt(np.sum(dx**2, axis=0))+eps)**2

def lagrange_mult(ex, Phi, Q, Q0, T):
	'''
	Ugly lagrange thingy
	'''
	# TODO Must I ocmpute Phi[:, -1]?
	lp = np.zeros( ex.shape)
	lp[:, -1] = ex[:, -1] * Q0
	lp[:, :-1] = ex[:, :-1] * Q  - Phi[:, :-1]
	# lp[:, :-1] = ex[:, :-1] * Q * np.linspace(1, 5, num=N-2).reshape(1, N-2) - Phi[:, :-1]

	return lp + np.cumsum(lp[:, ::-1], axis=1)[:, ::-1]

def grad(u, p, vref, T, R):
	'''
	Returns the gradient of the function we are minimizing.
	'''
	v = np.sqrt(np.sum(u**2, axis=0))
	# print(v)
	return T*p - R*( (vref/v) - 1) * u