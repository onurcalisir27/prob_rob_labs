import numpy as np
import matplotlib.pyplot as plt

''''
v[n+1] = av[n] + (1-a) u[n]

'''

def next_speed(v, u):
    a = 0.99
    return a * v + (1-a) * u

def step_input(time):
    if time < 50:
        return 0.0
    elif time < 400:
        return 0.5
    else:
        return 0.0

dt = 0.1
all_t = [i * dt for i in range(1, 5001)]
speed = 0.0
list_speed = []

for i in range(len(all_t)):

    input = step_input(all_t[i])
    speed = next_speed(speed, input)
    list_speed.append(speed)

# plt.figure(1)
# plt.plot(all_t, list_speed)
# plt.grid()
# plt.show()
#

a = 0.9
b = 0.7
dt = 0.1
# [theta, x, y, v, w]
state = np.zeros((5,1))
print(f"State :\n{state}")
state = np.array([[np.pi/3], [1.0], [1.0], [0.5], [1.0]])
print(f"State :\n{state}")

F = np.array([[1.0, 0.0, 0.0, 0.0, dt],
                   [-dt*state[3,0]*np.sin(state[0,0]), 1.0, 0.0, dt*np.cos(state[0,0]), 0.0],
                   [dt*state[3,0]*np.cos(state[0,0]), 0.0, 1.0, dt*np.sin(state[0,0]), 0.0],
                   [0.0, 0.0, 0.0, a, 0.0],
                   [0.0, 0.0, 0.0, 0.0, b]
                   ])
print(f"F matrix:\n{F}")

wheel_r = 33e-3
print(f"Wheel R is {wheel_r} mm")


covariance = np.identity(5) * 0.1
print(f"Covariance matrix: \n{covariance}")

