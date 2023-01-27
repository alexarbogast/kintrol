import numpy as np
import matplotlib.pyplot as plt 

THEME = "LIGHT"

if THEME == "DARK":
    plt.style.use('dark_background')
#plt.rcParams.update({'font.size': 22})

# Define the length of the path
s = 300

# Define the maximum velocity, acceleration, and jerk values
v_max = 100
a_max = 300
j_max = 2000

# find va, sa, sv
va = a_max**2 / j_max
sa = 2 * a_max**3 / j_max**2
sv = 2 * v_max * np.sqrt(v_max / j_max)

if v_max < va and s > sa:
    print("fig. 5.A.")
    tj = np.sqrt(v_max / j_max)
    ta = tj
    tv = s / v_max
elif v_max > va and s < sa:
    print("fig 5.B")
    tj = (s / (2 * j_max))**(1/3)
    ta = tj
    tv = 2 * tj
elif v_max < va and s < sa:
    if s >= sv:
        print("fig 5.C.1")
        tj = np.sqrt(v_max / j_max)
        ta = tj
        tv = s / v_max
    else:
        print("fig 5.C.2")
        tj = (s / (2 * j_max))**(1/3)
        ta = tj
        tv = 2 * tj
elif v_max > va and s > sa:
    if s >= sv:
        print("fig 5.D.1")
        tj = a_max / j_max
        ta = v_max / a_max
        tv = s / v_max
    else:
        print("fig 5.D.2")
        tj = a_max / j_max
        ta = 0.5 * (np.sqrt((4*s*(j_max**2)+a_max**3)/(a_max*(j_max**2))) - a_max/j_max)
        tv = ta + tj

# Values of time corresponding to movement phases
t1 = tj
t2 = ta
t3 = tj + ta
t4 = tv
t5 = tj + tv
t6 = tv + ta
t7 = tv + ta + tj

# Define kinematic parameters corresponding to movement phases
p1 = j_max * (t1**3)/6
v1 = j_max * (t1**2)/2
a1 = j_max * t1

p2 = p1 + v1*(t2 - t1) + a1*((t2 - t1)**2)/2
v2 = v1 + a1*(t2 - t1)
a2 = a1

p3 = p2 + v2*(t3 - t2) + a2*((t3 - t2)**2)/2 - j_max*((t3 - t2)**3)/6
v3 = v2 + a2*(t3 - t2) - j_max*((t3 - t2)**2)/2
a3 = a2 - j_max*(t3 - t2)

p4 = p3 + v3*(t4 - t3)
v4 = v3
a4 = 0

p5 = p4 + v4*(t5 - t4) - j_max*((t5 - t4)**3)/6
v5 = v4 - j_max*((t5 - t4)**2)/2
a5 = -j_max*(t5 - t4)

p6 = p5 + v5*(t6 - t5) + a5*((t6 - t5)**2)/2
v6 = v5 - a_max*(t6 - t5)
a6 = a5

p7 = s
v7 = 0
a7 = 0

def s_curve(t):
    p, v, a = 0, 0, 0

    if t < t1:                  # j(t) = j_max
        p = j_max * t**3/6
        v = j_max * t**2/2
        a = j_max * t
    elif t < t2:                # j(t) = 0
        dt = t - t1
        p = p1 + v1*dt + a1*(dt**2)/2
        v = v1 + a1*dt
        a = a1
    elif t < t3:                # j(t) = -j_max
        dt = t - t2
        p = p2 + v2*dt + a2*(dt**2)/2 - j_max*(dt**3)/6
        v = v2 + a2*dt - j_max*(dt**2)/2
        a = a2 - j_max*dt
    elif t < t4:                # j(t) = 0
        dt = t - t3
        p = p3 + v3*dt
        v = v3
        a = 0
    elif t < t5:                # j(t) = -j_max
        dt = t - t4
        p = p4 + v4*dt - j_max*(dt**3)/6
        v = v4 - j_max*(dt**2)/2
        a = -j_max*dt
    elif t < t6:               # j(t) = 0
        dt = t - t5
        p = p5 + v5*dt + a5*(dt**2)/2
        v = v5 - a_max*dt
        a = a5
    elif t < t7:               # j(t) = j_max        
        dt = t - t6
        p = p6 + v6*dt + a6*(dt**2)/2 + j_max*(dt**3)/6
        v = v6 + a6*dt + j_max*(dt**2)/2
        a = a6 + j_max*dt
    else:
        p = p7
        v = v7
        a = a7

    return p, v, a


if __name__ == "__main__":
    print(f"t1 = {t1}, t2 = {t2}, t3 = {t3}, t4 = {t4}, t5 = {t5}, t6 = {t6}, t7 = {t7}")
    t = np.linspace(0, t7, 1000)

    traj = [s_curve(tt) for tt in t]
    pos = [tr[0] for tr in traj]
    vel = [tr[1] for tr in traj]
    acc = [tr[2] for tr in traj]

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1)

    ax1.plot(t, pos)
    ax1.set_title("Position")
    ax1.set_xlabel("time (s)")
    ax1.grid(True)

    ax2.plot(t, vel)
    ax2.set_title("Velocity")
    ax2.set_xlabel("time (s)")
    ax2.grid(True)

    ax3.plot(t, acc)
    ax3.set_title("Acceleration")
    ax3.set_xlabel("time (s)")
    ax3.grid(True)

    plt.show()
