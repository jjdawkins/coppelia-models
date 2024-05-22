import numpy as np
import matplotlib.pyplot as plt
import sympy as sp

plt.rcParams['text.usetex'] = True
plt.rcParams['font.size'] = 16

t  = sp.symbols('t')
speed = 0.4
x = 1.2 * sp.sin(speed *2 * t)
y = 4 * sp.sin(speed * t)
dot_x = sp.diff(x, t)
dot_y = sp.diff(y, t)

fx = sp.lambdify(t, x, 'numpy')
fy = sp.lambdify(t, y, 'numpy')
fdot_x = sp.lambdify(t, dot_x, 'numpy')
fdot_y = sp.lambdify(t, dot_y, 'numpy')

# plot the figure eight
t_span = np.linspace(0, 2 * np.pi/speed, 1000)
print(f"End time: {t_span[-1]}")

f = plt.figure()
m = 5
ax = f.subplots(m,1) 
ax[0].plot(fx(t_span), fy(t_span))
ax[0].set_title('Figure Eight')
ax[0].set_xlabel('x')
ax[0].set_ylabel('y')
# make the aspect ratio equal
ax[0].set_aspect('equal')

# set size of the axes
ax[0].set_xlim(-2, 2)
ax[0].set_ylim(-6, 6)

# now plot the  x velocity
ax[1].plot(t_span, fdot_x(t_span))
ax[1].set_title(r'$\dot{x}$')
ax[1].set_xlabel('t')
ax[1].set_ylabel(r'$\dot{x}$')

# plot the y velocity
ax[2].plot(t_span, fdot_y(t_span))
ax[2].set_title(r'$\dot{y}$')
ax[2].set_xlabel('t')
ax[2].set_ylabel(r'$\dot{y}$')



# get theta
f_theta = sp.lambdify(t, sp.atan2(dot_y, dot_x), 'numpy')
theta = f_theta(t_span)
# make theta continuous
theta = np.unwrap(theta)
ax[3].plot(t_span, theta)
ax[3].set_title(r'$\theta$')
ax[3].set_xlabel('t')
ax[3].set_ylabel(r'$\theta$')


# plot dot_theta
f_dot_theta = sp.lambdify(t, sp.diff(sp.atan2(dot_y, dot_x), t), 'numpy')
ax[4].plot(t_span, f_dot_theta(t_span))
ax[4].set_title(r'$\dot{\theta}$')
ax[4].set_xlabel('t')
ax[4].set_ylabel(r'$\dot{\theta}$')

# make figure 4 x 8
f.set_size_inches(10, 15)
# dont overlap
plt.tight_layout()







plt.show()
