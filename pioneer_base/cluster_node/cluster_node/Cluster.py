import numpy as np
import math
import sympy as sp 
from enum import Enum

class ClusterConfig(Enum):
    TRICEN = "TriangleatCentroid"
    TRILEAD = "TriangleatLeader"
    ELLIPSE = "Ellipse"
    DIAMOND = "Diamond"
    LINE = "Line"

    def __str__(self):
        return self.value

ROVER_DOF = 3

class Cluster():
    def __init__(self, numRobots=5, clusterType=ClusterConfig.TRILEAD, clusterParams=[3, 3, 3, 3, 0, 0, 0], KPgains=None, KVgains=None):
        self.num_robots = numRobots
        self.cluster_type = clusterType
        self.cluster_params = clusterParams
        try:
            if KPgains is None:
                KPgains = [1.0] * (self.num_robots * ROVER_DOF)
            if KVgains is None:
                KVgains = [1.0] * (self.num_robots * ROVER_DOF)
            assert len(KPgains) == self.num_robots * ROVER_DOF, "KPgains must be a list of length numRobots*3"
            assert len(KVgains) == self.num_robots * ROVER_DOF, "KVgains must be a list of length numRobots*3"
        except AssertionError as e:
            print(f"AssertionError: {e}")
            return

        self.Kp = np.diag(KPgains)
        self.Kv = np.diag(KVgains)
        self.initialize_cluster()

    def initialize_cluster(self):
        self.cdes = np.zeros((self.num_robots * ROVER_DOF, 1))
        # self.cdes[0, 0] = 0.5
        # self.cdes[1, 0] = 1.2
        # self.cdes[2, 0] = -0.3
        # self.cdes[3, 0] = 0.05
        # self.cdes[4, 0] = -0.1

        # # cluster pose
        # self.cdes[5, 0] = 0.0
        # self.cdes[6, 0] = 0.0
        # self.cdes[7, 0] = 0.0
      
        self.cdes[8:12, 0] = self.cluster_params[0:4]  # d2-d5
        self.cdes[12:15, 0] = self.cluster_params[4:7]  # beta3-beta5
        self.c = np.zeros((self.num_robots * ROVER_DOF, 1))
        self.cd = np.zeros((self.num_robots * ROVER_DOF, 1))
        self.FKine_func, self.IKine_func, self.Jacobian_func, self.JacobianInv_func = None, None, None, None
        self.configureCluster(self.num_robots, self.cluster_type)

    def getVelocityCommand(self, r, rd):
        self.updateClusterPosition(r, rd)
        cd_cmd = self.calculateLinearControl()
        rd = np.dot(np.array(self.JacobianInv_func(*self.c.flatten())), cd_cmd)
        for i in range(self.num_robots):
            rd[i * ROVER_DOF + 2, 0] = self.wrap_to_pi(rd[i * ROVER_DOF + 2, 0])
        return cd_cmd, rd, self.c

    def update_cdes_tr(self, v_t, v_r, freq):
        t = self.c[2, 0]
        self.cdes[0, 0] += v_t * math.sin(t) / freq
        self.cdes[1, 0] += v_t * math.cos(t) / freq
        self.cdes[2, 0] += v_r / freq
        self.cdes[2, 0] = self.wrap_to_pi(self.cdes[2, 0])
        return

    def update_cdes_vel(self, v_x, v_y, v_r, freq):
        self.cdes[0, 0] += v_x / freq
        self.cdes[1, 0] += v_y / freq
        self.cdes[2, 0] += v_r / freq
        self.cdes[2, 0] = self.wrap_to_pi(self.cdes[2, 0])
        return

    def update_cdes_pos(self, data):
        for i in range(len(data)):
            self.cdes[i, 0] = data[i]
            if i in [2, 3, 4, 5, 6, 7, 12, 13, 14]:
                self.cdes[i, 0] = self.wrap_to_pi(self.cdes[i, 0])
        return

    def update_cluster_shape(self, params):
        if len(params) != 7:
            raise ValueError("params must be a list of length 7: [d2, d3, d4, d5, beta3, beta4, beta5]")
        self.cdes[8:12, 0] = params[0:4]
        self.cdes[12:15, 0] = [self.wrap_to_pi(p) for p in params[4:7]]

    def wrap_to_pi(self, t):
        return (t + np.pi) % (2 * np.pi) - np.pi

    def calculateLinearControl(self):
        c_diff = self.cdes - self.c
        for idx in [2, 3, 4, 5, 6, 7, 12, 13, 14]:
            c_diff[idx, 0] = self.wrap_to_pi(c_diff[idx, 0])
        cd = np.dot(self.Kp, c_diff)
        return cd

    def updateClusterPosition(self, r, rd):
        self.c = self.FKine_func(*r.flatten())
        for idx in [2, 3, 4, 5, 6, 7, 12, 13, 14]:
            self.c[idx, 0] = self.wrap_to_pi(self.c[idx, 0])
        self.cd = np.dot(np.array(self.Jacobian_func(*r.flatten())).astype(np.float64), rd)

    def configureCluster(self, robots, clusterType):
        if robots == 5 and clusterType == ClusterConfig.TRILEAD:
            self.cluster_config_trilead()

    def cluster_config_trilead(self):
        r_sym = sp.symbols('r0:15')
        c_sym = sp.symbols('c0:15')

        x_c = r_sym[0]
        y_c = r_sym[1]
        theta_c = sp.atan2(r_sym[0] - r_sym[3], r_sym[4] - r_sym[1])

        phi = [r_sym[2] - theta_c, r_sym[5] - theta_c, r_sym[8] - theta_c, r_sym[11] - theta_c, r_sym[14] - theta_c]
        d = [0, sp.sqrt((r_sym[0] - r_sym[3])**2 + (r_sym[1] - r_sym[4])**2),
             sp.sqrt((r_sym[6] - r_sym[0])**2 + (r_sym[7] - r_sym[1])**2),
             sp.sqrt((r_sym[3] - r_sym[9])**2 + (r_sym[4] - r_sym[10])**2),
             sp.sqrt((r_sym[6] - r_sym[12])**2 + (r_sym[7] - r_sym[13])**2)]
        beta = [0, 0,
                sp.atan2(r_sym[0] - r_sym[6], r_sym[7] - r_sym[1]) - theta_c,
                sp.atan2(r_sym[10] - r_sym[4], r_sym[9] - r_sym[3]) - theta_c,
                sp.atan2(r_sym[13] - r_sym[7], r_sym[12] - r_sym[6]) - theta_c]

        x = [c_sym[0]]
        y = [c_sym[1]]
        theta = [c_sym[2] + c_sym[3]]

        x.append(x[0] + c_sym[8] * sp.sin(c_sym[2]))
        y.append(y[0] + c_sym[8] * sp.cos(c_sym[2]))
        theta.append(c_sym[2] + c_sym[4])

        x.append(x[0] + c_sym[9] * (sp.sin(c_sym[12]) * sp.cos(c_sym[2]) - sp.cos(c_sym[12]) * sp.sin(c_sym[2])))
        y.append(y[0] + c_sym[9] * (sp.sin(c_sym[12]) * sp.sin(c_sym[2]) + sp.cos(c_sym[12]) * sp.cos(c_sym[2])))
        theta.append(c_sym[2] + c_sym[5])

        x.append(x[1] + c_sym[10] * (sp.cos(c_sym[13]) * sp.cos(c_sym[2]) - sp.sin(c_sym[13]) * sp.sin(c_sym[2])))
        y.append(y[1] + c_sym[10] * (sp.cos(c_sym[13]) * sp.sin(c_sym[2]) + sp.sin(c_sym[13]) * sp.cos(c_sym[2])))
        theta.append(c_sym[2] + c_sym[6])

        x.append(x[2] + c_sym[11] * (sp.cos(c_sym[14]) * sp.cos(c_sym[2]) - sp.sin(c_sym[14]) * sp.sin(c_sym[2])))
        y.append(y[2] + c_sym[11] * (sp.cos(c_sym[14]) * sp.sin(c_sym[2]) + sp.sin(c_sym[14]) * sp.cos(c_sym[2])))
        theta.append(c_sym[2] + c_sym[7])

        self.FKine = sp.Matrix([x_c, y_c, theta_c] + phi + d[1:] + beta[2:])
        self.IKine = sp.Matrix([x[i] for i in range(5)] + [y[i] for i in range(5)] + [theta[i] for i in range(5)])
        self.Jacob = self.FKine.jacobian(r_sym)
        self.JacobInv = self.IKine.jacobian(c_sym)

        self.FKine_func = sp.lambdify(r_sym, self.FKine, 'numpy')
        self.IKine_func = sp.lambdify(c_sym, self.IKine, 'numpy')
        self.Jacobian_func = sp.lambdify(r_sym, self.Jacob, 'numpy')
        self.JacobianInv_func = sp.lambdify(c_sym, self.JacobInv, 'numpy')

    def getDesiredRobotPosition(self):
        r = np.array(self.IKine_func(*self.cdes.flatten())).astype(np.float64)
        return r

    def testTransforms(self, r):
        c = np.array(self.FKine_func(*r.flatten())).astype(np.float64)
        r = np.array(self.IKine_func(*c.flatten())).astype(np.float64)
        return c, r

if __name__ == "__main__":
    # Example usage
    import os
    os.makedirs("cluster_expressions", exist_ok=True)
    cluster = Cluster(numRobots=5, clusterType=ClusterConfig.TRILEAD, clusterParams=[3, 3, 3, 3, 0, 0, 0])
    with open('cluster_expressions/FKine.txt', 'w') as f:
        f.write(sp.pretty(cluster.FKine, wrap_line=False))
    with open('cluster_expressions/IKine.txt', 'w') as f:
        f.write(sp.pretty(cluster.IKine, wrap_line=False))
    with open('cluster_expressions/Jacob.txt', 'w') as f:
        f.write(sp.pretty(cluster.Jacob, wrap_line=False))
    with open('cluster_expressions/JacobInv.txt', 'w') as f:
        f.write(sp.pretty(cluster.JacobInv, wrap_line=False))
    with open("cluster_expressions/IKine.txt", "r") as f:
        print(f.read())



