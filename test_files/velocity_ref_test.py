import numpy as np

from sympy import symbols, lambdify, diff, simplify, sin,cos, Array, atan2

t = symbols('t')
a = 8


# Define x(t) and y(t) as per the Lemniscate of Bernoulli
x_t = a * cos(t) / (1 + sin(t)**2)
y_t = a * cos(t) * sin(t) / (1 + sin(t)**2)

# Define r(t) as the steering angle
r_t = atan2(y_t, x_t)

# Create a lambda function for r(t)
get_r = lambdify(t, r_t, 'numpy')

t_i = np.linspace(0, 20, 1000)
r_i = get_r(t_i)

xs = np.array([])
ys = np.array([])

x = 0  
y = 0


for t in t_i:
    r = get_r(t)
    x = x + r*cos(t)
    y = y + r*sin(t)
    xs = np.append(xs, x)
    ys = np.append(ys, y)
    

# plot the result
import matplotlib.pyplot as plt
plt.plot(xs, ys)

plt.savefig('velocity_ref_test.png')

