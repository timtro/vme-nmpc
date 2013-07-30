from numpy import *

X, Y = meshgrid(arange(1, 4, 1), arange(-1, 2, .5))

X = X + random.randn(X.shape[0], X.shape[1])
Y = Y + random.randn(Y.shape[0], Y.shape[1])

for i in range(0,X.shape[0]):
    for j in range(0,Y.shape[1]):
        print('{:f}'.format(X[i,j])+', '+'{:f}'.format(Y[i,j])+',')
