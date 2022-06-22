
import numpy as np

#definindo uma função pra realizar o calculo de RK4
def RK4_2D_quad(t0,tf,x0,y0,v0,dt,k,m,g,theta):
	t = t0
	x = x0
	y = y0
	theta = theta*np.pi/180
	v0x = v0*np.cos(theta)
	v0y = v0*np.sin(theta)
	vx = v0x
	vy = v0y
	tn = tf
	
	t_list = []
	x_list = []
	y_list = []
	vx_list = []
	vy_list = []

	def fvx(t,x,y,vx,vy):
		return vx
	def fvy(t,x,y,vx,vy):
		return vy
			
	def fax(t,x,y,vx,vy):
		return -k*(np.sqrt(vx**2 + vy**2))*vx/m
		
	def fay(t,x,y,vx,vy):
		return -g-k*(np.sqrt(vx**2 + vy**2))*vy/m

	#definção dos coeficientes de RK4
	#k1
	def k1_vx(t,x,y,vx,vy):
		return fvx(t,x,y,vx,vy)

	def k1_vy(t,x,y,vx,vy):
		return fvy(t,x,y,vx,vy)

	def k1_ax(t,x,y,vx,vy):
		return fax(t,x,y,vx,vy)

	def k1_ay(t,x,y,vx,vy):
		return fay(t,x,y,vx,vy)

	#k2		
	def k2_vx(t,x,y,vx,vy):
		return fvx(t + dt/2.0, x + dt*k1_vx(t,x,y,vx,vy)/2.0, y + dt*k1_vy(t,x,y,vx,vy)/2.0, vx + dt*k1_ax(t,x,y,vx,vy)/2.0, vy + dt*k1_ay(t,x,y,vx,vy)/2.0)

	def k2_vy(t,x,y,vx,vy):
		return fvy(t + dt/2.0, x + dt*k1_vx(t,x,y,vx,vy)/2.0, y + dt*k1_vy(t,x,y,vx,vy)/2.0, vx + dt*k1_ax(t,x,y,vx,vy)/2.0, vy + dt*k1_ay(t,x,y,vx,vy)/2.0)

	def k2_ax(t,x,y,vx,vy):
		return fax(t + dt/2.0, x + dt*k1_vx(t,x,y,vx,vy)/2.0, y + dt*k1_vy(t,x,y,vx,vy)/2.0, vx + dt*k1_ax(t,x,y,vx,vy)/2.0, vy + dt*k1_ay(t,x,y,vx,vy)/2.0)

	def k2_ay(t,x,y,vx,vy):
		return fay(t + dt/2.0, x + dt*k1_vx(t,x,y,vx,vy)/2.0, y + dt*k1_vy(t,x,y,vx,vy)/2.0, vx + dt*k1_ax(t,x,y,vx,vy)/2.0, vy + dt*k1_ay(t,x,y,vx,vy)/2.0)

	#k3
	def k3_vx(t,x,y,vx,vy):
		return fvx(t + dt/2.0,x + dt*k2_vx(t,x,y,vx,vy)/2.0,y + dt*k2_vy(t,x,y,vx,vy)/2.0, vx + dt*k2_ax(t,x,y,vx,vy)/2.0,vy + dt*k2_ay(t,x,y,vx,vy)/2.0)

	def k3_vy(t,x,y,vx,vy):
		return fvy(t + dt/2.0,x + dt*k2_vx(t,x,y,vx,vy)/2.0,y + dt*k2_vy(t,x,y,vx,vy)/2.0, vx + dt*k2_ax(t,x,y,vx,vy)/2.0,vy + dt*k2_ay(t,x,y,vx,vy)/2.0)
	
	def k3_ax(t,x,y,vx,vy):
		return fax(t + dt/2.0,x + dt*k2_vx(t,x,y,vx,vy)/2.0,y + dt*k2_vy(t,x,y,vx,vy)/2.0, vx + dt*k2_ax(t,x,y,vx,vy)/2.0,vy + dt*k2_ay(t,x,y,vx,vy)/2.0)

	def k3_ay(t,x,y,vx,vy):
		return fay(t + dt/2.0,x + dt*k2_vx(t,x,y,vx,vy)/2.0,y + dt*k2_vy(t,x,y,vx,vy)/2.0, vx + dt*k2_ax(t,x,y,vx,vy)/2.0,vy + dt*k2_ay(t,x,y,vx,vy)/2.0)

	#k4

	def k4_vx(t,x,y,vx,vy):
		return fvx(t + dt, x + dt*k3_vx(t,x,y,vx,vy), y + dt*k3_vy(t,x,y,vx,vy), vx + dt*k3_ax(t,x,y,vx,vy), vy + dt*k3_ay(t,x,y,vx,vy))

	def k4_vy(t,x,y,vx,vy):
		return fvy(t + dt, x + dt*k3_vx(t,x,y,vx,vy), y + dt*k3_vy(t,x,y,vx,vy), vx + dt*k3_ax(t,x,y,vx,vy), vy + dt*k3_ay(t,x,y,vx,vy))

	def k4_ax(t,x,y,vx,vy):
		return fax(t + dt, x + dt*k3_vx(t,x,y,vx,vy), y + dt*k3_vy(t,x,y,vx,vy), vx + dt*k3_ax(t,x,y,vx,vy), vy + dt*k3_ay(t,x,y,vx,vy))

	def k4_ay(t,x,y,vx,vy):
		return fay(t + dt, x + dt*k3_vx(t,x,y,vx,vy), y + dt*k3_vy(t,x,y,vx,vy), vx + dt*k3_ax(t,x,y,vx,vy), vy + dt*k3_ay(t,x,y,vx,vy))
		
	while t <= tn and y>=0:
		t_list.append(t)

		vx_list.append(vx)		
		vx  = vx + (dt/6.0)*(k1_ax(t,x,y,vx,vy) + 2*k2_ax(t,x,y,vx,vy) + 2*k3_ax(t,x,y,vx,vy) + k4_ax(t,x,y,vx,vy))

		vy_list.append(vy)	
		vy  = vy + (dt/6.0)*(k1_ay(t,x,y,vx,vy) + 2*k2_ay(t,x,y,vx,vy) + 2*k3_ay(t,x,y,vx,vy) + k4_ay(t,x,y,vx,vy))
		
		x_list.append(x)		
		x  = x + (dt/6.0)*(k1_vx(t,x,y,vx,vy)  + 2*k2_vx(t,x,y,vx,vy)  + 2*k3_vx(t,x,y,vx,vy)  + k4_vx(t,x,y,vx,vy) )

		y_list.append(y)		
		y  = y + (dt/6.0)*(k1_vy(t,x,y,vx,vy)  + 2*k2_vy(t,x,y,vx,vy)  + 2*k3_vy(t,x,y,vx,vy)  + k4_vy(t,x,y,vx,vy) )
	
		t = t + dt

	return t_list, x_list, y_list, vx_list, vy_list


