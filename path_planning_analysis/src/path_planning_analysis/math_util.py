from math import sin, cos, sqrt, pi, atan2
PIx2 = pi * 2

def dist(p1, p2):
    return sqrt( pow(p1.x-p2.x, 2) + pow(p1.y-p2.y, 2) )

def a_dist_helper(t1, t2):
    diff = t1 - t2
    mod_diff = abs(diff % PIx2)
    return min(mod_diff, PIx2-mod_diff)

def a_dist(p1, p2):
    return a_dist_helper(p1.theta, p2.theta)

def dot_product(a1, m1, a2, m2):
    theta = a_dist_helper(a1, a2)
    return cos(theta) * m1 * m2

def average(w):
    if len(w)==0:
        return 0.0
    return sum(w)/len(w)

def inv_average(a):
    return sum([1.0/x for x in a])/len(a)

def inverse_scale(q):
    return 1.0 / ( q + 1.0 )

def derivative(t, x):
    ds = []
        
    for i, (ti, xi) in enumerate(zip(t,x)):
        if i>0:
            ds.append((xi-x[i-1])/(ti-t[i-1]))
        else:   
            ds.append(0)
    return ds
