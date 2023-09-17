import numpy as np

def area(P):
    x = [p[0] for p in P.get_approx_representation()]
    y = [p[1] for p in P.get_approx_representation()]
    return 0.5*np.abs(np.dot(x,np.roll(y,1))-np.dot(y,np.roll(x,1)))

