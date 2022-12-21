import numpy as np
from copy import deepcopy
from scipy.integrate import solve_ivp
from ctr_kinematics.ctr_kinematics.Segment import Segment
from ctr_kinematics.ctr_kinematics.Tube import Tube

import matplotlib.pyplot as plt


class ThreeTubeCTRKinematics(object):
    def __init__(self, system_parameters):
        # Tube parameters
        self.system_parameters = [Tube(**system_parameters['tube_2']), Tube(**system_parameters['tube_1']),
                                  Tube(**system_parameters['tube_0'])]
        # positions and transformation of backbone
        self.r = []
        self.r_transforms = []
        # positions of tube 1, 2 and 3
        self.r1 = []
        self.r2 = []
        self.r3 = []

    def forward_kinematics(self, joints):
        """
        # initial twist (for ivp solver)
        :param joint: Current joint position of robot [beta_0, ..., beta_2, alpha_0, ..., alpha_2]
        :return: End effector position or achieved goal of selected system.
        """
        beta = joints[:3]
        segment = Segment(self.system_parameters[0], self.system_parameters[1],
                          self.system_parameters[2], beta)
        r_0_ = np.array([0, 0, 0]).reshape(3, 1)
        alpha_1_0 = joints[3]
        R_0_ = np.array(
            [[np.cos(alpha_1_0), -np.sin(alpha_1_0), 0], [np.sin(alpha_1_0), np.cos(alpha_1_0), 0], [0, 0, 1]]) \
            .reshape(9, 1)
        alpha_0_ = joints[3:].reshape(3, 1)

        # initial twist
        uz_0_ = np.array([0, 0, 0])
        self.r, U_z, tip = self.kinematic_model(uz_0_, alpha_0_, r_0_, R_0_, segment, beta)
        self.r1 = self.r[tip[1]:tip[0] + 1]
        self.r2 = self.r[tip[2]:tip[1] + 1]
        self.r3 = self.r[:tip[2] + 1]
        assert not np.any(np.isnan(self.r))
        return self.r[-1]

    def kinematic_model(self, uz_0, alpha_0, r_0, R_0, segmentation, beta):
        """
        :param uz_0: Initial twist of backbone
        :param alpha_0: Initial angle of tubes
        :param r_0: Initial position of backbone
        :param R_0: Initial rotation matrix
        :param segmentation: Transition points where shape and internal moments continuity is enforced
        :param beta: Extension values for each tube.
        :return: r, u_z_end, tip_pos: Complete shape, twist at the tip and end-effector tip position.
        """
        tube1 = self.system_parameters[0]
        tube2 = self.system_parameters[1]
        tube3 = self.system_parameters[2]
        Length = np.empty(0)
        r = np.empty((0, 3))
        u_z = np.empty((0, 3))
        alpha = np.empty((0, 3))
        span = np.append([0], segmentation.S)
        for seg in range(0, len(segmentation.S)):
            # Initial conditions, 3 initial twist + 3 initial angle + 3 initial position + 9 initial rotation matrix
            y_0 = np.vstack((uz_0.reshape(3, 1), alpha_0, r_0, R_0)).ravel()
            s_span = np.linspace(span[seg], span[seg + 1] - 1e-6, num=30)
            #s = odeint(self.ode_eq, y_0, s_span, args=(
            #    segmentation.U_x[:, seg], segmentation.U_y[:, seg], segmentation.EI[:, seg], segmentation.GJ[:, seg]),
            #           tfirst=True)
            if np.all(np.diff(s_span) < 0):
                print("s_span not sorted correctly. Resorting...")
                print("linespace: ", s_span[seg], s_span[seg+1] - 1e-6)
                s_span = np.sort(s_span)
            sol = solve_ivp(fun=lambda s, y: self.ode_eq(s, y, segmentation.U_x[:, seg], segmentation.U_y[:, seg],
                                                         segmentation.EI[:, seg], segmentation.GJ[:, seg]),
                            t_span=(min(s_span), max(s_span)), y0=y_0, t_eval=s_span)
            if sol.status == -1:
                print(sol.message)
            s = np.transpose(sol.y)
            Length = np.append(Length, s_span)
            u_z = np.vstack((u_z, s[:, (0, 1, 2)]))
            alpha = np.vstack((alpha, s[:, (3, 4, 5)]))
            r = np.vstack((r, s[:, (6, 7, 8)]))

            # new boundary conditions for next segment
            r_0 = r[-1, :].reshape(3, 1)
            R_0 = np.array(s[-1, 9:]).reshape(9, 1)
            uz_0 = u_z[-1, :].reshape(3, 1)
            alpha_0 = alpha[-1, :].reshape(3, 1)

        d_tip = np.array([tube1.L, tube2.L, tube3.L]) + beta
        u_z_end = np.array([0.0, 0.0, 0.0])
        tip_pos = np.array([0, 0, 0])
        for k in range(0, 3):
            b = np.argmax(Length >= d_tip[k] - 1e-3)  # Find where tube curve starts
            u_z_end[k] = u_z[b, k]
            tip_pos[k] = b

        return r, u_z_end, tip_pos

    def ode_eq(self, s, y, ux_0, uy_0, ei, gj):
        """
        Definition of ODE equation to solve overall backbone shape of CTR.
        :param s: Arc-length distance along backbone.
        :param y: y is 3 initial twist + 3 initial angle + 3 initial position + 9 initial rotation matrix
        :param ux_0: Initial curvature in x for segment.
        :param uy_0: Initial curvature in y for segment.
        :param ei: Youngs modulus times second moment of inertia
        :param gj: Shear modulus times polar moment of inertia
        :return: dydt set of differential equations
        """
        dydt = np.empty([18, 1])
        ux = np.empty([3, 1])
        uy = np.empty([3, 1])
        for i in range(0, 3):
            ux[i] = (1 / (ei[0] + ei[1] + ei[2])) * \
                    (ei[0] * ux_0[0] * np.cos(y[3 + i] - y[3 + 0]) + ei[0] * uy_0[0] * np.sin(y[3 + i] - y[3 + 0]) +
                     ei[1] * ux_0[1] * np.cos(y[3 + i] - y[3 + 1]) + ei[1] * uy_0[1] * np.sin(y[3 + i] - y[3 + 1]) +
                     ei[2] * ux_0[2] * np.cos(y[3 + i] - y[3 + 2]) + ei[2] * uy_0[2] * np.sin(y[3 + i] - y[3 + 2]))
            uy[i] = (1 / (ei[0] + ei[1] + ei[2])) * \
                    (-ei[0] * ux_0[0] * np.sin(y[3 + i] - y[3 + 0]) + ei[0] * uy_0[0] * np.cos(y[3 + i] - y[3 + 0]) +
                     -ei[1] * ux_0[1] * np.sin(y[3 + i] - y[3 + 1]) + ei[1] * uy_0[1] * np.cos(y[3 + i] - y[3 + 1]) +
                     -ei[2] * ux_0[2] * np.sin(y[3 + i] - y[3 + 2]) + ei[2] * uy_0[2] * np.cos(y[3 + i] - y[3 + 2]))

        for j in range(0, 3):
            if ei[j] == 0:
                dydt[j] = 0  # ui_z
                dydt[3 + j] = 0  # alpha_i
            else:
                dydt[j] = ((ei[j]) / (gj[j])) * (ux[j] * uy_0[j] - uy[j] * ux_0[j])  # ui_z
                dydt[3 + j] = y[j]  # alpha_i

        e3 = np.array([0, 0, 1]).reshape(3, 1)
        uz = y[0:3]
        R = np.array(y[9:]).reshape(3, 3)
        u_hat = np.array([(0, - uz[0], uy[0]), (uz[0], 0, -ux[0]), (-uy[0], ux[0], 0)])
        dr = np.dot(R, e3)
        dR = np.dot(R, u_hat).ravel()

        dydt[6] = dr[0]
        dydt[7] = dr[1]
        dydt[8] = dr[2]

        for k in range(3, 12):
            dydt[6 + k] = dR[k - 3]
        return dydt.ravel()


if __name__ == '__main__':
    # Defining parameters of each tube, numbering starts with the most inner tube
    # length, length_curved, diameter_inner, diameter_outer, stiffness, torsional_stiffness,x_curvature, y_curvature
    tube1 = Tube(431e-3, 103e-3, 2 * 0.35e-3, 2 * 0.55e-3, 6.4359738368e+10, 2.5091302912e+10, 21.3, 0)
    tube2 = Tube(332e-3, 113e-3, 2 * 0.7e-3, 2 * 0.9e-3, 5.2548578304e+10, 2.1467424256e+10, 13.108, 0)
    tube3 = Tube(174e-3, 134e-3, 2e-3, 2 * 1.1e-3, 4.7163091968e+10, 2.9788923392e+10, 3.5, 0)
    system_parameters = [tube1, tube2, tube3]
    ctr_kine = ThreeTubeCTRKinematics(system_parameters)

    # Joint variables
    del_q = np.array([0.01, 0.015, 0.019, np.pi, np.pi * 5 / 2, np.pi / 2])
    # Initial position of joints
    q_0 = np.array([-0.2858, -0.2025, -0.0945, 0, 0, 0])
    q = q_0 + del_q

    ee_pos = ctr_kine.forward_kinematics(q)
    print("computed ee_pos: " + str(ee_pos))

    # Sample joint positions and plot shapes
    alphas = np.random.uniform(low=-np.pi, high=np.pi, size=3)
    beta_1 = np.random.uniform(low=-tube1.L, high=0)
    beta_2 = np.random.uniform(low=beta_1, high=0)
    beta_3 = np.random.uniform(low=beta_2, high=0)
    q = np.array([beta_1, beta_2, beta_3, alphas[0], alphas[1], alphas[2]])
    ctr_kine.forward_kinematics(q)

    fig = plt.figure(figsize=(5, 5), dpi=150)
    ax = plt.axes(projection='3d')
    ax.plot3D(ctr_kine.r1[:,0] * 1000, ctr_kine.r1[:,1] * 1000, ctr_kine.r1[:,2] * 1000, linewidth=2.0, c='#2596BE')
    ax.plot3D(ctr_kine.r2[:,0] * 1000, ctr_kine.r2[:,1] * 1000, ctr_kine.r2[:,2] * 1000, linewidth=3.0, c='#D62728')
    ax.plot3D(ctr_kine.r3[:,0] * 1000, ctr_kine.r3[:,1] * 1000, ctr_kine.r3[:,2] * 1000, linewidth=4.0, c='#2Ca02C')
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_zlabel("Z (mm)")
    ax.set_box_aspect([1,1,1])
    plt.show()


