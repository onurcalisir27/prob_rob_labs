import numpy
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

plt.figure(1)
plt.plot(all_t, list_speed)
plt.grid()
plt.show()


