import numpy as np

xymask = np.array([True, False, True, False, False])
dxymask = np.array([False, True, False, True, False])

def kPlus1Sum(x):
	return ( np.cumsum(x[::-1])[::-1] )[:-1]

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

	# Works, but is terribly slow:
	#
	# for k in range(1, x.shape[-1]):
	# 	x[4, k] = x[4, k-1] + u[0, k-1] * T
	# 	x[1, k] = v[k-1] * np.sin(x[4, k])
	# 	x[3, k] = v[k-1] * np.cos(x[4, k])
	# 	x[xymask, k] = x[xymask, k-1] + x[dxymask, k-1] * T

	x[4, 1:] = u[:] * T
	x[4, 1:] = np.cumsum( x[4, :] )[1:]

	x[1, 1:] = v[:] * np.sin(x[4, 1:])
	x[3, 1:] = v[:] * np.cos(x[4, 1:])

	x[xymask, 1:] = x[dxymask, :-1] * T
	x[xymask, 1:] = np.cumsum( x[xymask, :], axis=1 )[:,1:]

	Phi[:,:] = 0
	for obs in obst.T:
		dx = x[xymask,1:]-obs.reshape(2, 1)
		Phi[:,:] +=  2* dx / (np.sqrt(np.sum(dx**2, axis=0))+eps)**2

def lagrange_mult(x, v, ex, Phi, Q, Q0, T):
	'''
	Ugly lagrange thingy
	'''
	# TODO Must I ocmpute Phi[:, -1]?

	p = np.zeros( x.shape )
	p[xymask, -1] = ex[xymask, -1] * Q0

	# Works, but is terribly slow:
	#
	# for k in reversed(range(p.shape[-1]-1)):
	# 	p[xymask, k] = p[xymask, k+1] + ex[xymask, k] * Q  - Phi[:, k]
	# 	p[dxymask, k] = T * p[xymask, k+1]
	# 	p[4, k] = -p[1, k+1]*np.sin(x[4,k])*v[k] + p[3, k+1]*np.cos(x[4,k])*v[k] \
	# 		+ p[4, k+1]

	p[0, :-1] = Q * ex[0, :] - Phi[0, :]
	p[0, :-1] = kPlus1Sum( p[0, :] )
	p[2, :-1] = Q * ex[2, :] - Phi[1, :]
	p[2, :-1] = kPlus1Sum( p[2, :] )
	p[1, :-1] = T * p[0, 1:]
	p[3, :-1] = T * p[2, 1:]
	p[4, :-1] = -p[1, 1:] * np.sin( x[4, :-1] ) * v[:] \
		+ p[3, 1:] * np.cos( x[4, :-1] ) * v[:]
	p[4, :-1] = kPlus1Sum( p[4, :] )

	return p

def grad(u, p, vref, T, R):
	'''
	Returns the gradient of the function we are minimizing.
	'''
	#v = np.sqrt(np.sum(u**2, axis=0))
	# print(v)
	return T*p[4, 1:] - R * u