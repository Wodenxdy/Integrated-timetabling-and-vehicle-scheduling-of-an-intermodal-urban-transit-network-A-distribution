import numpy as np

np.random.seed(18)

class PassengerGroup:
    def __init__(self, name, origin, destination, arrival_time, number):
        self.name = name
        self.origin = origin
        self.destination = destination
        self.arrival_time = arrival_time
        self.number = number
        self.waiting_time = 99999
        self.board = -1
        self.board_vehicle = -1
        self.flag = 0

    def __str__(self):
        return str(self.name)


S = list(range(0, 4))
L = list(range(0, 2))
T = list(range(1, 30))
W = list(range(0, 3))
M = list(range(1, 4))

a_original = np.array([1, 7, 13])

headway_origin  = a_original[1] - a_original[0]

num_station = len(S)
num_bus = len(L)
num_time = len(T)
num_scenario = len(W)

r_acc = 1
r_dec = 1
max_shift = 2
min_shift = -2
min_headway = 1
max_holding = 2
min_holding = 1
Cap = 10
labmda = np.ones(num_bus)
delta = 1
PROBILITY = [0.3, 0.3, 0.4]
epsilon = [1, 10]
WEIGHT = [1, 1]
WEIGHT_WAIT = 1.5
WEIGHT_TRAVEL = 1
TIMEVALUE = 0.3
Adjusting_Factor = 10
MAX_VEHICLE_DEPOT = 6

exchangeRate = 1.09

VARIPHY = {1:2, 2:3, 3: 5}
WUCHA = 0.001

M1 = max_holding
M2 = 200
M3 = 200
# M1 = 100
DELTA_T = 60

running_nominal = np.load("running_nominal.npy")

p_bodong = np.array([0.1, 0.15, 0.18])
P1 = np.array([[-0.45, 0.5, -0.5],
               [-0.5, -0.5, -0.5],
               [0.5, 0.5, 0.5]])
ones_vector = np.ones(3)


p_come = np.load("passenger_demand_generate.npy")

P = {}
PU = {}
PV = {}
name = 1
for w in W:
    P[w] = []
    for u in S:
        for v in S[u+1:]:
            for t in T:
                if p_come[w, u, v, t - 1] > 0:
                    P[w].append(PassengerGroup(name, u, v, t, p_come[w, u, v, t - 1]))
                    name += 1
    PU[w] = {}
    PV[w] = {}
    for u in S:
        PU[w][u] = []
        PV[w][u] = []
    for p in P[w]:
        o = p.origin
        d = p.destination
        PU[w][o].append(p)
        PV[w][d].append(p)

Nu = {}
for u in S[1:-1]:
    Nu[u] = 0

r_direct = {}

for u in S:
    for v in S[u+1:]:
        for t in T:
            temp = 0
            for v_prime in range(u, v):
                if v_prime == u:
                    time_arrive_former_stop = t - 1
                else:
                    time_arrive_former_stop = temp - 1
                temp += running_nominal[v_prime, time_arrive_former_stop]
            r_direct[u, v, t] = temp
            r_direct[v, u, t] = temp



