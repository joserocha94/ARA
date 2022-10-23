import matplotlib.pyplot as plt
import numpy as np

with open('C:/Users/joserocha/Documents/ist/ARA/project/output/ccdf_times_dijkstra_sw.txt') as f:
    lines = f.read().splitlines()

with open('C:/Users/joserocha/Documents/ist/ARA/project/output/ccdf_times_simulator_sw.txt') as f:
    lines2 = f.read().splitlines()

times_dijkstra_sw   = np.array([float(e.strip()) for e in lines])
times_simulator_sw  = np.array([float(e.strip()) for e in lines2])

time = np.arange(0, times_dijkstra_sw.size)

x = np.sort(times_dijkstra_sw);
k = np.sort(times_simulator_sw);

dij_sw_cdf  = x.cumsum() / x.sum()
dij_sw_ccdf = 1 - dij_sw_cdf
sim_sw_cdf  = k.cumsum() / k.sum()
sim_sw_ccdf = 1 - sim_sw_cdf


fig, (ax1) = plt.subplots(ncols=1, figsize=(8, 8))
ax1.plot(time, dij_sw_ccdf, label='Dijkstra')
ax1.plot(time, sim_sw_ccdf, label='Simulator')
ax1.legend()

plt.xlabel("Time (s)")
plt.ylabel("CCDF (%)")
ax1.set_title('Shortest-Widest Paths times')

plt.tight_layout()
plt.show()


########################################################################################
########################################################################################
########################################################################################


with open('C:/Users/joserocha/Documents/ist/ARA/project/output/ccdf_width_dijkstra_sw.txt') as f:
    lines = f.read().splitlines()

with open('C:/Users/joserocha/Documents/ist/ARA/project/output/ccdf_width_simulator_sw.txt') as f:
    lines2 = f.read().splitlines()


width_dijkstra_sw   = np.array([float(e.strip()) for e in lines])
width_simulator_sw  = np.array([float(e.strip()) for e in lines2])

time = np.arange(0, width_dijkstra_sw.size)

x = np.sort(width_dijkstra_sw);
k = np.sort(width_simulator_sw);

dij_sw_cdf  = x.cumsum() / x.sum()
dij_sw_ccdf = 1 - dij_sw_cdf
sim_sw_cdf  = k.cumsum() / k.sum()
sim_sw_ccdf = 1 - sim_sw_cdf


fig, (ax1) = plt.subplots(ncols=1, figsize=(8, 8))
ax1.plot(time, dij_sw_ccdf, label='Dijkstra')
ax1.plot(time, sim_sw_ccdf, label='Simulator')
ax1.legend()

plt.xlabel("Width")
plt.ylabel("CCDF (%)")
ax1.set_title('Shortest-Widest Paths width')

plt.tight_layout()
plt.show()


########################################################################################
########################################################################################
########################################################################################


with open('C:/Users/joserocha/Documents/ist/ARA/project/output/ccdf_length_dijkstra_sw.txt') as f:
    lines = f.read().splitlines()

with open('C:/Users/joserocha/Documents/ist/ARA/project/output/ccdf_length_simulator_sw.txt') as f:
    lines2 = f.read().splitlines()


length_dijkstra_sw   = np.array([float(e.strip()) for e in lines])
length_simulator_sw  = np.array([float(e.strip()) for e in lines2])

time = np.arange(0, length_dijkstra_sw.size)

x = np.sort(length_dijkstra_sw);
k = np.sort(length_simulator_sw);

dij_sw_cdf  = x.cumsum() / x.sum()
dij_sw_ccdf = 1 - dij_sw_cdf
sim_sw_cdf  = k.cumsum() / k.sum()
sim_sw_ccdf = 1 - sim_sw_cdf


fig, (ax1) = plt.subplots(ncols=1, figsize=(8, 8))
ax1.plot(time, dij_sw_ccdf, label='Dijkstra')
ax1.plot(time, sim_sw_ccdf, label='Simulator')
ax1.legend()

plt.xlabel("Length")
plt.ylabel("CCDF (%)")
ax1.set_title('Shortest-Widest Paths length')

plt.tight_layout()
plt.show()


########################################################################################
########################################################################################
########################################################################################



with open('C:/Users/joserocha/Documents/ist/ARA/project/output/ccdf_times_dijkstra_ws.txt') as f:
    lines = f.read().splitlines()

with open('C:/Users/joserocha/Documents/ist/ARA/project/output/ccdf_times_simulator_ws.txt') as f:
    lines2 = f.read().splitlines()

times_dijkstra_ws   = np.array([float(e.strip()) for e in lines])
times_simulator_ws  = np.array([float(e.strip()) for e in lines2])

time = np.arange(0, times_dijkstra_ws.size)

x = np.sort(times_dijkstra_ws);
k = np.sort(times_simulator_ws);

dij_sw_cdf  = x.cumsum() / x.sum()
dij_sw_ccdf = 1 - dij_sw_cdf
sim_sw_cdf  = k.cumsum() / k.sum()
sim_sw_ccdf = 1 - sim_sw_cdf


fig, (ax1) = plt.subplots(ncols=1, figsize=(8, 8))
ax1.plot(time, dij_sw_ccdf, label='Dijkstra')
ax1.plot(time, sim_sw_ccdf, label='Simulator')
ax1.legend()

plt.xlabel("Time (s)")
plt.ylabel("CCDF (%)")
ax1.set_title('Widest-Shortest Paths times')

plt.tight_layout()
plt.show()



########################################################################################
########################################################################################
########################################################################################



with open('C:/Users/joserocha/Documents/ist/ARA/project/output/ccdf_width_dijkstra_ws.txt') as f:
    lines = f.read().splitlines()

with open('C:/Users/joserocha/Documents/ist/ARA/project/output/ccdf_width_simulator_ws.txt') as f:
    lines2 = f.read().splitlines()

width_dijkstra_ws   = np.array([float(e.strip()) for e in lines])
width_simulator_ws  = np.array([float(e.strip()) for e in lines2])

time = np.arange(0, width_dijkstra_ws.size)

x = np.sort(width_dijkstra_ws);
k = np.sort(width_simulator_ws);

dij_sw_cdf  = x.cumsum() / x.sum()
dij_sw_ccdf = 1 - dij_sw_cdf
sim_sw_cdf  = k.cumsum() / k.sum()
sim_sw_ccdf = 1 - sim_sw_cdf


fig, (ax1) = plt.subplots(ncols=1, figsize=(8, 8))
ax1.plot(time, dij_sw_ccdf, label='Dijkstra')
ax1.plot(time, sim_sw_ccdf, label='Simulator')
ax1.legend()

plt.xlabel("Width")
plt.ylabel("CCDF (%)")
ax1.set_title('Widest-Shortest Paths width')

plt.tight_layout()
plt.show()


########################################################################################
########################################################################################
########################################################################################



with open('C:/Users/joserocha/Documents/ist/ARA/project/output/ccdf_length_dijkstra_ws.txt') as f:
    lines = f.read().splitlines()

with open('C:/Users/joserocha/Documents/ist/ARA/project/output/ccdf_length_simulator_ws.txt') as f:
    lines2 = f.read().splitlines()

length_dijkstra_ws   = np.array([float(e.strip()) for e in lines])
length_simulator_ws  = np.array([float(e.strip()) for e in lines2])

time = np.arange(0, length_dijkstra_ws.size)

x = np.sort(length_dijkstra_ws);
k = np.sort(length_simulator_ws);

dij_sw_cdf  = x.cumsum() / x.sum()
dij_sw_ccdf = 1 - dij_sw_cdf
sim_sw_cdf  = k.cumsum() / k.sum()
sim_sw_ccdf = 1 - sim_sw_cdf


fig, (ax1) = plt.subplots(ncols=1, figsize=(8, 8))
ax1.plot(time, dij_sw_ccdf, label='Dijkstra')
ax1.plot(time, sim_sw_ccdf, label='Simulator')
ax1.legend()

plt.xlabel("Length")
plt.ylabel("CCDF (%)")
ax1.set_title('Widest-Shortest Paths lenght')

plt.tight_layout()
plt.show()