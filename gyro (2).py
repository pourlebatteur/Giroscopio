import numpy as np
from vpython import *
import matplotlib.pyplot as plt


#El eje de rotacion inicial del giroscopio es el eje vertical pero como
#consecuencia de la precesion adquiere velocidad angular con respecto a los ejes x,y
# y su eje de rotacion cambia con el tiempo

# Set up the scene
scene = canvas(title="Gyroscope Simulation", width=800, height=600)
scene.range = 2

# Define constants
m = 1  # mass of the gyroscope
r = 0.5  # radius of the gyroscope
I = (2/5) * m * r**2  # moment of inertia
g = vector(4, -5, 0)  # gravitational acceleration (2,-5,0)

# Define initial conditions
theta = 0.5 * pi  # initial angle of the gyroscope with respect to the vertical axis
phi = 0  # initial angle of the gyroscope with respect to the horizontal axis
omega = vector(-2, 5, 0)  # initial angular velocity of the gyroscope

# Create the gyroscope object
gyro = cylinder(pos=vector(0, 0, 0), axis=vector(0, 0, r), radius=r, color=color.yellow)

# Create the x, y, z axis arrows
arrow(pos=vector(-2,1,0), axis=vector(1,0,0), color=color.red, shaftwidth=0.05, headwidth=0.1, headlength=0.2)
arrow(pos=vector(-2,1,0), axis=vector(0,1,0), color=color.green, shaftwidth=0.05, headwidth=0.1, headlength=0.2)
arrow(pos=vector(-2,1,0), axis=vector(0,0,1), color=color.blue, shaftwidth=0.05, headwidth=0.1, headlength=0.2)


# Create the torque graph
torque_graph = graph(title="Torque vs. Time", width=800, height=400, xtitle="Time (s)", ytitle="Torque (N*m)")
torque_curve = gcurve(color=color.red)

# Create the angular velocity graphs
omega_x_graph = graph(title="Angular Velocity about x-axis vs. Time", width=800, height=400, xtitle="Time (s)", ytitle="Angular Velocity (rad/s)")
omega_x_curve = gcurve(color=color.blue)

omega_y_graph = graph(title="Angular Velocity about y-axis vs. Time", width=800, height=400, xtitle="Time (s)", ytitle="Angular Velocity (rad/s)")
omega_y_curve = gcurve(color=color.green)

# Create the angular momentum and angular velocity vectors and labels
L_arrow = arrow(pos=vector(0,0,0), axis=vector(0,0,0), color=color.magenta, shaftwidth=0.05, headwidth=0.1, headlength=0.2, shaftlength=1)
omega_arrow = arrow(pos=vector(0,0,0), axis=vector(0,0,0), color=color.orange, shaftwidth=0.05, headwidth=0.1, headlength=0.2)


label(pos=vector(-2.2, -1.5, 0), text="Angular Momentum", height=16, color=color.magenta)
label(pos=vector(-2.2, -0.8, 0), text="Angular Velocity", height=16, color=color.orange)

# Create the position graph
pos_graph = graph(title="Gyroscope Position", width=800, height=400, xtitle="x", ytitle="y")
pos_curve = gcurve(color=color.blue)

# Define the initial position of the gyroscope
pos = vector(0, r*sin(theta)*sin(phi), r*cos(theta))


# Define the time step and simulation time
dt = 0.01
t = 0

# Run the simulation
while t < 8:
    rate(50)
    
    # Calculate the torque on the gyroscope due to gravity and precession
    #El torque total esta dado por el torque aplicado en el giroscopio y el peso
    tau_gravity = cross(g, gyro.axis) * r
    omega_p = vector(omega.x*cos(theta), 0, -omega.x*sin(theta))
    tau_p = cross(omega_p, gyro.axis) * r
    tau = tau_gravity + tau_p
    
    # Calculate the angular acceleration of the gyroscope
    alpha = tau / I
    
    # Update the angular velocity and angles of the gyroscope
    omega += alpha * dt
    theta += omega.y * dt
    phi += omega.x * dt
    
    # Update the axis of the gyroscope
    gyro.axis = vector(r*sin(theta)*cos(phi), r*sin(theta)*sin(phi), r*cos(theta))
    
    # Update the position of the gyroscope
    pos = gyro.pos + gyro.axis
    
    # Add position to the position graph
    pos_curve.plot(pos.y, pos.z)
    
    # Update the angular momentum and angular velocity vectors
    L = vector(gyro.axis.x*I*omega.x, gyro.axis.y*I*omega.y, gyro.axis.z*I*omega.z)
    L_arrow.axis = L
    omega_arrow.axis = omega
    
    # Add torque to the torque graph
    torque_curve.plot(t, tau.z)
    
    # Add angular velocity to the angular velocity graphs
    omega_x_curve.plot(t, omega.x)
    omega_y_curve.plot(t, omega.y)
    
    # Update the time
    t += dt

# Show the torque and angular velocity graphs
plt.show()









