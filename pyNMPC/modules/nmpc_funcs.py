import numpy as np

xymask = np.array([True, False, True, False, False])
dxymask = np.array([False, True, False, True, False])

def airline_path( x0, vref, xref, T, tgt):
	direction = (tgt - x0[xymask])
	direction = direction.reshape(2,1) / np.sqrt(2 * np.sum(direction**2)) # The factor of 1/sqrt(2) belons to vref, but is put here to reduce computation
	xref[xymask,:] = vref*T*direction
	xref[xymask,:] = x0[xymask].reshape(2,1) + np.cumsum(xref[xymask,:], axis=1)

def f(x, u, Phi, obst, T, eps, n):
	'''
	Based on the velocity values in u (we control velocity), this function uses
	Euler integration to predict x over the horizon of N points.
	'''
	x[:, 1:] = x[:, 0].reshape(n,1) + np.cumsum(u, axis=1) * T
	Phi[:,:] = 0
	for obs in obst.T:
		dx = x[xymask,1:]-obs.reshape(2, 1)
		Phi[:,:] +=  2* dx / (np.sqrt(np.sum(dx**2, axis=0))+eps)**2

def lagrange_mult(x, v, ex, Phi, Q, Q0, T):
	'''
	Ugly lagrange thingy
	'''
	# TODO Must I ocmpute Phi[:, -1]?
	lp = np.ones( ex.shape )
	lp[4,:] = np.cumsum(lp[4,:])
	# Old lp from dxy control:
	# lp[:, -1] = ex[:, -1] * Q0
	# lp[:, :-1] = ex[:, :-1] * Q  - Phi[:, :-1]
	# return lp + np.cumsum(lp[:, ::-1], axis=1)[:, ::-1]

	lp[xymask, -1] = ex[xymask, -1] * Q0
	lp[xymask, :-1] = ex[xymask, :-1] * Q  - Phi[:, :-1]
	lp[dxymask, :-1] = (np.cumsum( lp[dxymask, ::-1], axis=1)[:, ::-1] * T)[:,1:]
	lp[4, : ] = -np.cumsum(lp[1, ::-1])[::-1] * np.sin(x[4, 1:]) * v[:] \
		+ np.cumsum(lp[3, ::-1])[::-1] * np.cos(x[4, 1:]) * v[:]\
		+ np.cumsum(lp[4, ::-1])[::-1]
	return lp


def grad(u, p, vref, T, R):
	'''
	Returns the gradient of the function we are minimizing.
	'''
	#v = np.sqrt(np.sum(u**2, axis=0))
	# print(v)
	return T*p - R * u