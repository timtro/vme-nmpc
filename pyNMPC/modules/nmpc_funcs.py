import numpy as np

xymask = np.array([True, False, True, False, False])
dxymask = np.array([False, True, False, True, False])

def airline_path( x0, vref, xref, T, tgt):
	direction = (tgt - x0[xymask])
	direction = direction.reshape(2,1) / np.sqrt(2 * np.sum(direction**2)) # The factor of 1/sqrt(2) belons to vref, but is put here to reduce computation
	xref[xymask, :] = vref*T*direction * np.sqrt(2)
	xref[xymask, 0] += x0[xymask]
	xref[xymask, :] = np.cumsum(xref[xymask,:], axis=1)

def f(x, u, v, Phi, obst, T, eps, n):
	'''
	Based on the velocity values in u (we control velocity), this function uses
	Euler integration to predict x over the horizon of N points.
	'''
	for k in range(1, x.shape[-1]):
		x[4, k] = x[4, k-1] + u[0, k-1] * T
		x[1, k] = v[k-1] * np.sin(x[4, k])
		x[3, k] = v[k-1] * np.cos(x[4, k])
		x[xymask, k] = x[xymask, k-1] + x[dxymask, k-1] * T

	Phi[:,:] = 0
	for obs in obst.T:
		dx = x[xymask,1:]-obs.reshape(2, 1)
		Phi[:,:] +=  2* dx / (np.sqrt(np.sum(dx**2, axis=0))+eps)**2

def lagrange_mult(x, v, ex, Phi, Q, Q0, T):
	'''
	Ugly lagrange thingy
	'''
	# TODO Must I ocmpute Phi[:, -1]?
	lp = np.zeros( x.shape )

	lp[xymask, -1] = ex[xymask, -1] * Q0

	for k in reversed(range(lp.shape[-1]-1)):
		lp[xymask, k] = lp[xymask, k+1] + ex[xymask, k] * Q  - Phi[:, k]
		lp[dxymask, k] = T * lp[xymask, k+1]
		lp[4, k] = -lp[1, k+1]*np.sin(x[4,k])*v[k] + lp[3, k+1]*np.cos(x[4,k])*v[k] \
			+ lp[4, k+1]
		# lp[4, k] = lp[dxymask, k+1].dot(np.array([-np.sin(x[4,k]), np.cos(x[4,k])]))* v[k] \
		# 	+ lp[4, k+1]
	return lp

def grad(u, p, vref, T, R):
	'''
	Returns the gradient of the function we are minimizing.
	'''
	#v = np.sqrt(np.sum(u**2, axis=0))
	# print(v)
	return T*p[4, 1:] - R * u