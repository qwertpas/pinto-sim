import numpy as np
import matplotlib.pyplot as plt

def RK4(f, x, u, dt):
    # Runge-Kutta 4 integration
    k1 = f(x, u)
    k2 = f(x + (dt/2)*k1, u)
    k3 = f(x + (dt/2)*k2, u)
    k4 = f(x + dt*k3, u)
    return x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)

def T1(x_a):
    return x_a
def dT1dxa(x_a):
    return 1
def T2_inv(x_l):
    return x_l
def dT2invdxl(x_l):
    return 1


def sea_model(x, u):
    tau_a, tau_l = u
    x_a, x_a_dot, x_l, x_l_dot = x

    k = 0.45 #spring constant
    m_a = 0.1 #mass of the actuator before spring
    m_l = 0.5 #mass of the load after spring 

    x_a_ddot = (tau_a + k*(T2_inv(x_l) - T1(x_a))*dT1dxa(x_a)) / m_a
    x_l_ddot = (tau_l - k*(T2_inv(x_l) - T1(x_a))*dT2invdxl(x_l)) / m_l
    # x_a_ddot = (tau_a + k*(x_l-x_a)) / m_a
    # x_l_ddot = (tau_l - k*(x_l-x_a)) / m_l

    x_dot = np.array([x_a_dot, x_a_ddot, x_l_dot, x_l_ddot])
    return x_dot



disp = True

if(disp):
    import pygame
    pygame.init()
    screen = pygame.display.set_mode((1280, 720))
    clock = pygame.time.Clock()
    width = screen.get_width()
    height = screen.get_height()
    xrange = [-20, 20]
    pixels_per_meter = width / (xrange[1] - xrange[0])

    def to_disp_pos(x, y=0):
        return (x*pixels_per_meter+width/2, -y*screen.get_height() + height/2)

running = True
dt = 1/60


t = 0
x = [0, 0, 0, 0]
u = [0.1, 0]

xs = []
times = []

while running:
    
    if(x[2] > 15):
        running = False

    if(t > 0.5):
        u = [10, 0]

    x = RK4(sea_model, x, u, dt)

    t += dt
    times.append(t)
    xs.append(x)

    if(disp):
        #check for exit
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        screen.fill("white")
        pygame.draw.line(screen, 'gray', (0,0), (100,100))
        pygame.draw.circle(screen, "red", to_disp_pos(x[0], 0), 1*pixels_per_meter)
        pygame.draw.circle(screen, "blue", to_disp_pos(x[2]+4, 0), 1*pixels_per_meter)
        pygame.draw.line(screen, "orange", to_disp_pos(x[0], 0), to_disp_pos(x[2]+4, 0), width=round(50/(abs(x[0]-x[2]-4)+1)))
        pygame.display.flip()
        dt = clock.tick(60) / 1000
        
if(disp):
    pygame.quit()

xs = np.array(xs)

plt.plot(times, xs[:,0], label='x_a')
plt.plot(times, xs[:,1], label='x_a_dot')
plt.plot(times, xs[:,2], label='x_l')
plt.plot(times, xs[:,3], label='x_l_dot')
plt.title(f"end load vel: {np.round(xs[-1,3], 2)}")
plt.legend()
plt.show()
