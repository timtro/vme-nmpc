

def Phi(x, y, obs):
    phi = 0
    for r in obs:
        phi = phi + 1 / ((r[0] - x) ** 2 + (r[1] - y) ** 2 + .08)
    return phi
