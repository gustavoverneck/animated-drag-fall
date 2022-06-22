# create_visual.py
'''
dx/dt = v = fv(t,x,v)    =>    x = x + v*dt    ou    x = x + fv(t,x,v) *dt 

dv/dt = a = fa(t,x,v)    =>    v = v + a*dt    ou    v = v + fa(t,x,v) *dt 



Condições iniciais:
t(0) = 0, v(0) = 0 m/s, x(0) = 10 m, dt = 0.00001, t = [0,40s]
'''

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import RK4_2D_quad as py

m = 4; t0 = 0; x0 = 0; y0 = 100; v0 = 20; tf = 100; dt = 0.01; g = 9.8; theta = 45

#teste sem arrasto
k = 0.0

t_list, x_list, y_list, vx_list, vy_list = py.RK4_2D_quad(t0,tf,x0,y0,v0,dt,k,m,g,theta)

#teste com arrasto
k = 0.2

t1_list, x1_list, y1_list, vx1_list, vy1_list = py.RK4_2D_quad(t0,tf,x0,y0,v0,dt,k,m,g,theta)
x_plot = []
y_plot = []
x1_plot = []
y1_plot = []
# Para a animacao
passo = 10
total_frames = int((tf/dt)/passo)
fim = int(tf/dt)
fig, ax = plt.subplots()
def animate(u):
    global t_list, x_list, y_list, passo, x_plot, y_plot, fim
    x_plot = x_list[u*passo]
    y_plot = y_list[u*passo]
    x1_plot = x1_list[u*passo]
    y1_plot = y1_list[u*passo]
    ax.clear()
    ax.plot(x_plot, y_plot, marker="o", color="red", label="sem arrasto")
    ax.plot(x1_plot, y1_plot, marker="o", color="blue", label="com arrasto")
    ax.legend(loc="upper right")
    ax.set_xlim([0, 100])
    ax.set_ylim([0, 150])

ani = animation.FuncAnimation(fig, animate, frames=total_frames, interval=1, repeat=True)
plt.show()

