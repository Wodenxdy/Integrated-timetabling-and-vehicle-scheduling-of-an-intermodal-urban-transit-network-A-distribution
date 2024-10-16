from gurobipy import *
from GenerateInstance import *


class GurobiModel:
    def __init__(self):
        self.model = Model("SBTP")
        self.iu_com = tuplelist([(i, u) for i in L for u in S])  # 车辆和站点集合
        self.im_com = tuplelist([(i, m) for i in L for m in M])  # 车辆和站点集合
        self.wi_com = tuplelist([(w, i) for w in W for i in L])  # 车辆和站点集合
        self.iuv_com = tuplelist([(i, u, v) for i in L for u in S for v in S if u < v])  # 车辆和站点集合

        self.wium_com = tuplelist([(w, i, u, m) for w in W for i in L for u in S for m in M])  # 车辆和站点集合
        self.wiu_com = tuplelist([(w, i, u) for w in W for i in L for u in S])  # 场景，车辆和站点集合
        self.wiut_com = tuplelist([(w, i, u, t) for w in W for i in L for u in S for t in T])  # 场景，车辆和站点集合
        self.iut_com = tuplelist([(i, u, t) for i in L for u in S for t in T])  # 场景车辆和站点时间集合
        self.wuvt_com = tuplelist([(w, u, v, t) for w in W for u in S for v in S if u < v for t in T])  # 场景车辆和站点时间集合
        self.uvt_com = tuplelist([(u, v, t) for u in S for v in S if u < v for t in T])  # 车辆和站点集合

        self.wuvt_com_yita = tuplelist([(w, u, v, t) for w in W for u in S for v in S if u != v for t in T])  # 场景车辆和站点时间集合
        self.wuvtm_com = tuplelist([(w, u, v, t, m) for w in W for u in S for v in S if u != v for t in T for m in M])  # 车辆和站点集合
        self.uvtm_lam_com = tuplelist([(u, v, t, m) for u in S for v in S if u > v for t in T for m in M])  # 车辆和站点集合
        self.uvtt_prime_com = tuplelist([(u, v, t, t_prime) for u in S for v in S if u < v for t in T for t_prime in T[:t]])  # 车辆和站点集合
        self.iuvtt_prime_com = tuplelist(
            [(i, u, v, t, t_prime) for i in L for u in S for v in S if u < v for t in T for t_prime in T[:t]])  # 车辆和站点集合

        self.pi_com = tuplelist([(p.name, i) for w in W for p in P[w] for i in L])
        self.puvt_com = tuplelist([(p.name, p.origin, p.destination, t) for w in W for p in P[w] for t in T])
        self.wut_com = tuplelist([(w, u, t) for w in W for u in S[1:-1] for t in T])

    def add_variables(self):
        # 1 Intermediate Variables
        self.arrive = self.model.addVars(self.iu_com, vtype=GRB.INTEGER, name='t^a')
        self.departure = self.model.addVars(self.iu_com, vtype=GRB.INTEGER, name='t^d')
        self.r = self.model.addVars(self.iu_com, vtype=GRB.INTEGER, name='r')
        self.s = self.model.addVars(self.iu_com, vtype=GRB.INTEGER, name='s')
        self.headway = self.model.addVars(self.iu_com, lb=min_headway, vtype=GRB.INTEGER, name='headway')
        self.l_FIX = self.model.addVars(self.wiu_com, vtype=GRB.CONTINUOUS, name='l')
        self.b_FIX = self.model.addVars(self.wiu_com, vtype=GRB.CONTINUOUS, name='b')
        self.b_uv_DRT = self.model.addVars(self.wuvt_com_yita, vtype=GRB.CONTINUOUS, name='b')

        self.o_FIX = self.model.addVars(self.wiu_com, vtype=GRB.CONTINUOUS, name='o')
        self.chi = self.model.addVars(self.iuv_com, vtype=GRB.BINARY, name='chi')
        self.gamma = self.model.addVars(self.iu_com, lb=0, ub=M1, vtype=GRB.INTEGER, name='gamma')

        self.t_alight = self.model.addVars(self.wiu_com, vtype=GRB.CONTINUOUS, name='t_alight')
        self.t_board = self.model.addVars(self.wiu_com, vtype=GRB.CONTINUOUS, name='t_board')
        self.capacity_available = self.model.addVars(self.wiu_com, vtype=GRB.INTEGER, name='capacity_available')

        self.vartheta_FIX = self.model.addVars(self.pi_com, vtype=GRB.BINARY, name="ceta")
        self.vartheta_DRT = self.model.addVars(self.puvt_com, vtype=GRB.BINARY, name="ceta")

        self.varsigma_FIX = self.model.addVars(self.pi_com, vtype=GRB.INTEGER, name="ceta")
        self.varsigma_DRT = self.model.addVars(self.puvt_com, vtype=GRB.INTEGER, name="ceta")

        self.varphi_FIX = self.model.addVars(self.pi_com, vtype=GRB.BINARY, name="ceta")
        self.varphi_DRT = self.model.addVars(self.puvt_com, vtype=GRB.BINARY, name="ceta")

        self.WAIT = self.model.addVars(list(range(1, name)), vtype=GRB.INTEGER, name="wait")
        self.TRAVEL = self.model.addVars(list(range(1, name)), vtype=GRB.INTEGER, name="in_vehicle")

        self.N = self.model.addVars(self.wut_com, ub=MAX_VEHICLE_DEPOT, vtype=GRB.INTEGER, name="NumberMus")
        self.AV = self.model.addVars(self.wut_com, vtype=GRB.INTEGER, name="NumberMus")
        self.DV = self.model.addVars(self.wut_com, vtype=GRB.INTEGER, name="NumberMus")

        # 2 Decision variables about FLS
        self.R = self.model.addVars(self.iu_com, vtype=GRB.INTEGER, name='running')
        self.shift = self.model.addVars(L, lb=min_shift, ub=max_shift, vtype=GRB.INTEGER, name='shift')
        self.x = self.model.addVars(self.wiu_com, vtype=GRB.INTEGER, name='x')
        self.y = self.model.addVars(self.iu_com, vtype=GRB.BINARY, name='y')
        self.z = self.model.addVars(self.iut_com, vtype=GRB.BINARY, name='z')
        self.e = self.model.addVars(self.iut_com, vtype=GRB.BINARY, name='z')
        self.h = self.model.addVars(self.iut_com, vtype=GRB.BINARY, name='h')
        self.h_hat = self.model.addVars(self.iut_com, vtype=GRB.BINARY, name='h_hat')
        self.q = self.model.addVars(self.wium_com, vtype=GRB.BINARY, name='q')

        # 3 Decision variables about DRT
        self.yita = self.model.addVars(self.wuvt_com_yita, vtype=GRB.BINARY, name='yita')
        self.pi = self.model.addVars(self.wuvtm_com, vtype=GRB.BINARY, name='pi')

        # 4 Variables about Obj function
        self.OBJ = self.model.addVars(W, vtype=GRB.CONTINUOUS, name='OBJ')
        self.alpah = self.model.addVars(self.wiut_com, vtype=GRB.CONTINUOUS, name='alpha')
        self.Z_wait = self.model.addVars(W, vtype=GRB.CONTINUOUS, name="Z_wait")
        self.Z_travel = self.model.addVars(W, vtype=GRB.CONTINUOUS, name="Z_travel")
        self.Z_oper = self.model.addVars(W, vtype=GRB.CONTINUOUS, name="Z_oper")

        # 5. Variables about DRO
        self.ceta = self.model.addVars(W, lb=0, vtype=GRB.CONTINUOUS)
        self.vari = self.model.addVar(lb=0, vtype=GRB.CONTINUOUS)
        self.xi = self.model.addVar(lb=-9999, vtype=GRB.CONTINUOUS)

    def add_objective_function(self):
        # 1.Total waiting costs
        self.model.addConstrs(self.Z_wait[w] == WEIGHT_WAIT * quicksum(self.WAIT[p.name] * TIMEVALUE * p.number
                                                      for p in P[w]) for w in W)

        # 2.Total operating costs
        self.model.addConstrs(self.Z_travel[w] == WEIGHT_TRAVEL * quicksum(self.TRAVEL[p.name] * TIMEVALUE * p.number
                                                         for p in P[w]) for w in W)

        # 3.Total costs
        for w in W:
            self.model.addConstr(self.Z_oper[w] == quicksum(
                self.q[w, i, u, m] * VARIPHY[m] for i in L for u in S[:-1] for m in M)
                + quicksum(
                self.pi[w, u, v, t, m] * VARIPHY[m] for u in S for v in S if u != v for m in M for t in T)
                                 )

        self.model.addConstrs(self.OBJ[w] == WEIGHT[0] * (self.Z_wait[w] + self.Z_travel[w]) + WEIGHT[1] * self.Z_oper[w] for w in W)
        self.model.setObjective(quicksum(PROBILITY[w] * self.OBJ[w] for w in W) +
                                quicksum(PROBILITY[w] * self.ceta[w] for w in W) + self.vari)

    def add_constraints_timetable(self):
        # 1.Arrival time of each trip
        for i in L:
            for u in S:
                if u == 0:
                    self.model.addConstr(self.arrive[i, u] == a_original[i] + self.shift[i],
                                         name='cons1_{}_{}'.format(i, u))
                else:
                    self.model.addConstr(
                        self.arrive[i, u] == self.departure[i, u - 1] + self.r[i, u - 1]
                        , name='cons1_{}_{}'.format(i, u))

        # 2.Departure time of each trip
        self.model.addConstrs(self.departure[i, u] == self.arrive[i, u] + self.s[i, u] for i in L for u in S)

        # 3.constraints (7) is guaranteed through the definition of variable "self.shift =
        # self.model.addVars(L, lb=min_shift, ub=max_shift, vtype=GRB.INTEGER, name='shift') "

        # 4.Constraints about dwell time
        self.model.addConstrs(self.s[i, u] <= max_holding * self.y[i, u] for i in L for u in S)
        self.model.addConstrs(self.s[i, u] >= min_holding * self.y[i, u] for i in L for u in S)

        # 5.Inherent characteristic of binary variables
        self.model.addConstrs(self.z[i, u, t + 1] <= self.z[i, u, t] for i in L for u in S for t in T[:-1])
        self.model.addConstrs(self.e[i, u, t + 1] <= self.e[i, u, t] for i in L for u in S for t in T[:-1])

        # 6.Defination of binary variables
        for i in L:
            for u in S:
                for t in T:
                    if t == T[0]:
                        self.model.addConstr(self.h[i, u, t] == 1 - self.z[i, u, t], name="hz")
                        self.model.addConstr(self.h_hat[i, u, t] == 1 - self.e[i, u, t], name="hz")
                    else:
                        self.model.addConstr(self.h[i, u, t] == self.z[i, u, t - 1] - self.z[i, u, t], name="hz")
                        self.model.addConstr(self.h_hat[i, u, t] == self.e[i, u, t - 1] - self.e[i, u, t], name="hz")

        # 7.Bounding constraints of Binary variables
        self.model.addConstrs(
            self.departure[i, u] == delta * (quicksum(self.z[i, u, t] for t in T) + 1) for i in L for u in S)
        self.model.addConstrs(
            self.arrive[i, u] == delta * (quicksum(self.e[i, u, t] for t in T) + 1) for i in L for u in S)

        # 8. Actual Running time of each trip
        self.model.addConstrs(
            self.r[i, u] == self.R[i, u] - r_acc * (1 - self.y[i, u]) - r_dec * (1 - self.y[i, u + 1]) for i in
            L for u in S[:-1])

        # 9.Running time of each trip without stop skip
        self.model.addConstrs(self.R[i, u] == quicksum(running_nominal[u, t - 1] * self.h[i, u, t] for t in T)
                           for i in L for u in S[:-1])

        # 10.Headway constraints
        self.model.addConstrs(
            self.headway[i, u] == self.departure[i + 1, u] - self.departure[i, u] for i in L[:-1] for u in S)

        # 11.Constraints about skip stop
        self.model.addConstrs(self.y[i, 0] == 1 for i in L)
        self.model.addConstrs(self.y[i, S[-1]] == 1 for i in L)

    def add_constraints_passengers_movement(self):
        # 1. linearization constraints
        self.model.addConstrs(self.chi[i, u, v] <= self.y[i, u] for i in L for u in S for v in S if u < v)
        self.model.addConstrs(self.chi[i, u, v] <= self.y[i, v] for i in L for u in S for v in S if u < v)
        self.model.addConstrs(
            self.chi[i, u, v] >= self.y[i, u] + self.y[i, v] - 1 for i in L for u in S for v in S
            if u < v)

        # 2. Constraints about FLS
        for w in W:
            for p in P[w]:
                for i in L:
                    u = p.origin
                    v = p.destination
                    self.model.addConstr(
                        self.departure[i, u] * self.chi[i, u, v] - p.arrival_time >= T[-1] * (self.vartheta_FIX[p.name, i] - 1))
                    self.model.addConstr(
                        self.departure[i, u] * self.chi[i, u, v] - p.arrival_time <= T[-1] * self.vartheta_FIX[p.name, i] - WUCHA)
                    self.model.addConstr(self.varsigma_FIX[p.name, i] ==
                                         (self.departure[i, u] - p.arrival_time) * self.vartheta_FIX[p.name, i] +
                                         T[-1] * (1 - self.vartheta_FIX[p.name, i]))

                    # whether the passenger group board the FLS line
                    self.model.addConstr(
                        self.varsigma_FIX[p.name, i] >= self.WAIT[p.name] - (1 + T[-1]) * self.varphi_FIX[p.name, i] + WUCHA)
                    self.model.addConstr(
                        self.varsigma_FIX[p.name, i] <= self.WAIT[p.name] + (1 + T[-1]) * (1 - self.varphi_FIX[p.name, i]))

        # 3. Constraints about DRT
        for w in W:
            for p in P[w]:
                for t in T:
                    u = p.origin
                    v = p.destination
                    self.model.addConstr(t * self.yita[w, u, v, t] - p.arrival_time >= T[-1] *
                                         (self.vartheta_DRT[p.name, u, v, t] - 1))
                    self.model.addConstr(
                        t * self.yita[w, u, v, t] - p.arrival_time <= T[-1] * self.vartheta_DRT[p.name, u, v, t] - WUCHA)
                    self.model.addConstr(
                        self.varsigma_DRT[p.name, u, v, t] ==
                        (t * self.yita[w, u, v, t] - p.arrival_time) * self.vartheta_DRT[p.name, u, v, t] +
                        T[-1] * (1 - self.vartheta_DRT[p.name, u, v, t]))

                    # whether the passenger group board the DRT line
                    self.model.addConstr(
                        self.varsigma_DRT[p.name, u, v, t] >= self.WAIT[p.name] - 2 * T[-1] * self.varphi_DRT[p.name, u, v, t] + WUCHA)
                    self.model.addConstr(
                        self.varsigma_DRT[p.name, u, v, t] <= self.WAIT[p.name] + 2 * T[-1] * (1 - self.varphi_DRT[p.name, u, v, t]))

        # 4. Constraints about waiting and traveling time of each passenger group
        for w in W:
            for p in P[w]:
                varList = []
                u = p.origin
                v = p.destination
                for i in L:
                    varList.append(self.varsigma_FIX[p.name, i])
                for t in T:
                    varList.append(self.varsigma_DRT[p.name, u, v, t])
                self.model.addConstr(self.WAIT[p.name] == min_(varList))

                temp = 0
                for i in L:
                    temp += self.varphi_FIX[p.name, i] * (self.arrive[i, v] - self.departure[i, u])
                for t in T:
                    temp += self.varphi_DRT[p.name, u, v, t] * r_direct[u, v, t]
                self.model.addConstr(self.TRAVEL[p.name] == temp)

        # 5. Constraints about transit mode
        for w in W:
            for p in P[w]:
                self.model.addConstr(quicksum(self.varphi_FIX[p.name, i] for i in L) +
                                     quicksum(self.varphi_DRT[p.name, p.origin, p.destination, t] for t in T) == 1)

    def add_constraints_passenger_FIX(self):
        # 1. Constraints about boarding and alighting passengers
        for i in L:
            for u in S:
                for w in W:
                    temp_board = 0
                    temp_alight = 0
                    for p in P[w]:
                        if p.origin == u:
                            temp_board += (self.varphi_FIX[p.name, i] * p.number)

                        if p.destination == u:
                            temp_alight += (self.varphi_FIX[p.name, i] * p.number)
                    self.model.addConstr(self.b_FIX[w, i, u] == temp_board)
                    self.model.addConstr(self.l_FIX[w, i, u] == temp_alight)

        # 2. Constraints about in-vehicle passengers
        for w in W:
            for i in L:
                for u in S:
                    if u == 0:
                        self.model.addConstr(self.o_FIX[w, i, u] == self.b_FIX[w, i, u])
                    else:
                        self.model.addConstr(self.o_FIX[w, i, u] == self.o_FIX[w, i, u - 1] - self.l_FIX[w, i, u] + self.b_FIX[w, i, u])

        # 3. Constraints about capacity
        self.model.addConstrs(
            self.o_FIX[w, i, u] <= Cap * self.x[w, i, u] for w in W for i in L for u in S)

        # 4. Constraints about formation
        self.model.addConstrs(quicksum(self.q[w, i, u, m] for m in M) == 1 for w in W for i in L for u in S)
        self.model.addConstrs(self.x[w, i, u] == quicksum(m * self.q[w, i, u, m] for m in M) for w in W for i in L for u in S)

    def add_constraints_DRT(self, disable=False):
        # 1. Constraints about boarding passengers
        for u in S:
            for v in S[u+1:]:
                for t in T:
                    for w in W:
                        temp_board = 0
                        for p in P[w]:
                            judge = (p.origin == u) & (p.destination == v)
                            if judge:
                                temp_board += self.varphi_DRT[p.name, u, v, t] * p.number
                        self.model.addConstr(self.b_uv_DRT[w, u, v, t] == temp_board)
                        self.model.addConstr(self.b_uv_DRT[w, u, v, t] <= Cap * quicksum( m *self.pi[w, u, v, t, m] for m in M))

        # 2. Constraints about yita and pi
        self.model.addConstrs(self.yita[w, u, v, t] == quicksum(self.pi[w, u, v, t, m] for m in M)
                              for w in W for u in S for v in S if u != v for t in T)

    def add_constraints_vehicle_circulation(self):
        # 1. Constraints about vehicle circulation
        for w in W:
            for u in S[1:-1]:
                for t in T:
                    if t == 1:
                        self.model.addConstr(self.N[w, u, t] == Nu[u] + self.AV[w, u, t] - self.DV[w, u, t])
                    else:
                        self.model.addConstr(self.N[w, u, t] == self.N[w, u, t - 1] + self.AV[w, u, t] - self.DV[w, u, t])

                    temp_arrival = 0
                    for i in L:
                        temp_arrival += self.x[w, i, u - 1] * self.h_hat[i, u, t]

                    for v in S:
                        if v != u:
                            for m in M:
                                t_temp = t - r_direct[v, u, t]
                                if t_temp in T:
                                    temp_arrival += m * self.pi[w, v, u, t_temp, m]

                    self.model.addConstr(self.AV[w, u, t] == temp_arrival)

                    temp_departure = 0
                    for i in L:
                        temp_departure += self.x[w, i, u] * self.h[i, u, t]

                    for v in S:
                        if v != u:
                            for m in M:
                                temp_departure += m * self.pi[w, u, v, t, m]
                    self.model.addConstr(self.DV[w, u, t] == temp_departure)

    def add_DRO_constraints(self):
        # Constraints about DRO
        self.model.addConstrs(
            P1[w, w] * self.OBJ[w] + P1[w, w] * self.ceta[w] + P1[w, w] * ones_vector[w] * self.xi <= self.vari for w in
            W)
        self.model.addConstrs(
            P1[w, w] * self.OBJ[w] + P1[w, w] * self.ceta[w] + P1[w, w] * ones_vector[w] * self.xi >= -self.vari for w
            in W)

    def solve_model(self):
        self.model.update()
        self.model.setParam('MIPGap', 0)
        self.model.optimize()
