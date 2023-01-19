import numpy as np
from copy import deepcopy
from scipy.integrate import solve_ivp
from ctr_kinematics.Segment import Segment
from ctr_kinematics.Tube import Tube

import matplotlib.pyplot as plt


class TwoTubeCTRKinematics(object):
    def __init__(self, system_parameters):
        # Tube parameters
        self.system_parameters = [Tube(**system_parameters['inner']),
                                  Tube(**system_parameters['outer'])]
        # positions and transformation of backbone
        self.r = []
        self.r_transforms = []
        # positions of tube 1 and 2
        self.r1 = []
        self.r2 = []

    def forward_kinematics(self, joints):
        beta = joints[0:2]
        segment = Segment(self.system_parameters[0], self.system_parameters[1], None, beta)

        r_0 = np.array([0, 0, 0]).reshape(3, 1)  # initial position of robot
        alpha_1_0 = joints[2]  # initial twist angle for tube 1
        R_0_ = np.array(
            [[np.cos(alpha_1_0), -np.sin(alpha_1_0), 0], [np.sin(alpha_1_0), np.cos(alpha_1_0), 0],
             [0, 0, 1]]) \
            .reshape(9, 1)  # initial rotation matrix
        alpha_0_ = joints[2:].reshape(2, 1)

        uz_0_ = np.array([0.0, 0.0]).reshape(2, 1)
        # kinematic_model function
        self.r, U_z, tip = self.kinematic_model(uz_0_, alpha_0_, r_0, R_0_, segment, beta)
        self.r1 = self.r[tip[1]:tip[0] + 1]
        self.r2 = self.r[:tip[1] + 1]
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
        Length = np.empty(0)
        r = np.empty((0, 3))
        u_z = np.empty((0, 2))
        alpha = np.empty((0, 2))
        span = np.append([0], segmentation.S)
        for seg in range(0, len(segmentation.S)):
            # Initial conditions, 2 initial twist + 2 initial angle + 3 initial position + 9 initial rotation matrix
            y_0 = np.vstack((uz_0.reshape(2, 1), alpha_0, r_0, R_0)).ravel()
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
            u_z = np.vstack((u_z, s[:, (0, 1)]))
            alpha = np.vstack((alpha, s[:, (2, 3)]))
            r = np.vstack((r, s[:, (4, 5, 6)]))

            # new boundary conditions for next segment
            r_0 = r[-1, :].reshape(3, 1)
            R_0 = np.array(s[-1, 7:]).reshape(9, 1)
            uz_0 = u_z[-1, :].reshape(2, 1)
            alpha_0 = alpha[-1, :].reshape(2, 1)

        d_tip = np.array([tube1.L, tube2.L]) + beta
        u_z_end = np.array([0.0, 0.0])
        tip_pos = np.array([0, 0])
        for k in range(0, 2):
            try:
                b = np.argmax(Length >= d_tip[k] - 1e-3)  # Find where tube curve starts
            except ValueError:
                # The tube is fully retracted
                return np.array([[0,0,0]]), np.array([0,0]), np.array([0,0])
            u_z_end[k] = u_z[b, k]
            tip_pos[k] = b

        return r, u_z_end, tip_pos

    def ode_eq(self, s, y, ux_0, uy_0, ei, gj):
        """
        Definition of ODE equation to solve overall backbone shape of CTR.
        :param s: Arc-length distance along backbone.
        :param y: y is 2 initial twist + 2 initial angle + 3 initial position + 9 initial rotation matrix
        :param ux_0: Initial curvature in x for segment.
        :param uy_0: Initial curvature in y for segment.
        :param ei: Youngs modulus times second moment of inertia
        :param gj: Shear modulus times polar moment of inertia
        :return: dydt set of differential equations
        """
        dydt = np.empty([16, 1])
        ux = np.empty([2, 1])
        uy = np.empty([2, 1])
        for i in range(0, 2):
            ux[i] = (1 / (ei[0] + ei[1])) * \
                    (ei[0] * ux_0[0] * np.cos(y[2 + i] - y[2 + 0]) + ei[0] * uy_0[0] * np.sin(y[2 + i] - y[2 + 0]) +
                     ei[1] * ux_0[1] * np.cos(y[2 + i] - y[2 + 1]) + ei[1] * uy_0[1] * np.sin(y[2 + i] - y[2 + 1]))
            uy[i] = (1 / (ei[0] + ei[1])) * \
                    (-ei[0] * ux_0[0] * np.sin(y[2 + i] - y[2 + 0]) + ei[0] * uy_0[0] * np.cos(y[2 + i] - y[2 + 0]) +
                     -ei[1] * ux_0[1] * np.sin(y[2 + i] - y[2 + 1]) + ei[1] * uy_0[1] * np.cos(y[2 + i] - y[2 + 1]))

        for j in range(0, 2):
            if ei[j] == 0:
                dydt[j] = 0  # ui_z
                dydt[2 + j] = 0  # alpha_i
            else:
                dydt[j] = ((ei[j]) / (gj[j])) * (ux[j] * uy_0[j] - uy[j] * ux_0[j])  # ui_z
                dydt[2 + j] = y[j]  # alpha_i

        e3 = np.array([0, 0, 1]).reshape(3, 1)
        uz = y[0:2]
        R = np.array(y[7:]).reshape(3, 3)
        u_hat = np.array([[0, -uz.flatten()[0], uy.flatten()[0]], [uz.flatten()[0], 0, -ux.flatten()[0]], [-uy.flatten()[0], ux.flatten()[0], 0]])
        dr = np.dot(R, e3)
        dR = np.dot(R, u_hat).ravel()

        dydt[4] = dr[0]
        dydt[5] = dr[1]
        dydt[6] = dr[2]

        for k in range(3, 12):
            dydt[4 + k] = dR[k - 3]
        return dydt.ravel()
"""
    def ode_eq(self, s, y, ux_0, uy_0, ei, gj):
        # 1st element of y is curvature along x for first tube,
        # 2nd element of y is curvature along y for second tube
        # next 2 elements of y are curvatures along z, e.g., y= [ u1_z  u2_z]
        # next 2 elements of y are twist angles, alpha_i
        # last 12 elements are r (position) and R (orientations), respectively
        dydt = np.empty([18, 1])
        tet1 = y[4]
        tet2 = y[5]
        R_tet1 = np.array([[np.cos(tet1), -np.sin(tet1), 0], [np.sin(tet1), np.cos(tet1), 0],
                           [0, 0, 1]])
        R_tet2 = np.array([[np.cos(tet2), -np.sin(tet2), 0], [np.sin(tet2), np.cos(tet2), 0],
                           [0, 0, 1]])
        u2 = R_tet2.transpose() @ np.array([[y[0]], [y[1]], [y[2]]]) + dydt[
            4] * np.array([[0], [0], [1]])  # Vector of curvature of tube 2
        u = np.array([y[0], y[1], y[2], u2[0, 0], u2[1, 0], y[3]])
        u1 = np.array([y[0], y[1], y[2]]).reshape(3, 1)

        # estimating twist curvature and twist angles
        for i in np.argwhere(gj != 0):
            dydt[2 + i] = ((ei[i]) / (gj[i])) * (u[i * 3] * uy_0[i] - u[i * 3 + 1] * ux_0[i])  # ui_z
            dydt[4 + i] = y[2 + i] - y[2]  # alpha_i

        # estimating curvature of first tube along x and y
        K_inv = np.diag(np.array([1 / np.sum(ei), 1 / np.sum(ei), 1 / np.sum(gj)]))
        K1 = np.diag(np.array([ei[0], ei[0], gj[0]]))
        K2 = np.diag(np.array([ei[1], ei[1], gj[1]]))
        dR_tet1 = np.array([[-np.sin(tet1), -np.cos(tet1), 0], [np.cos(tet1), -np.sin(tet1), 0],
                            [0, 0, 1]])
        dR_tet2 = np.array([[-np.sin(tet2), -np.cos(tet2), 0], [np.cos(tet2), -np.sin(tet2), 0],
                            [0, 0, 1]])
        u_hat1 = np.array([[0, -u1[2], u1[1]], [u1[2], 0, -u1[0]], [-u1[1], u1[0], 0]], dtype=np.float64)
        u_hat2 = np.array([[0, -u2[2], u2[1]], [u2[2], 0, -u2[0]], [-u2[1], u2[0], 0]], dtype=np.float64)
        u_s1 = np.array([ux_0[0], uy_0[0], 0]).reshape(3, 1)
        u_s2 = np.array([ux_0[1], uy_0[1], 0]).reshape(3, 1)
        du = np.zeros((3, 1), dtype=np.float64)
        du = -K_inv @ (R_tet1 @ (K1 @ (dydt[4] * dR_tet1.transpose() @ u1) + u_hat1 @ K1 @ (u1 - u_s1)) + R_tet2 @ (
                K2 @ (dydt[5] * dR_tet2.transpose() @ u2) + u_hat2 @ K2 @ (u2 - u_s2)))
        dydt[0] = du[0, 0]
        dydt[1] = du[1, 0]
        R = np.array(
            [[y[9], y[10], y[11]], [y[12], y[13], y[14]], [y[15], y[16], y[17]]])  # rotation matrix of 1st tube

        # estimating R and r
        e3 = np.array([[0.0], [0.0], [1.0]])
        dr = R @ e3
        dR = (R @ u_hat1).ravel()

        dydt[6] = dr[0, 0]
        dydt[7] = dr[1, 0]
        dydt[8] = dr[2, 0]

        for k in range(3, 12):
            dydt[6 + k] = dR[k - 3]
        return dydt.ravel()
        
        
    def kinematic_model(self, uz_0, alpha_0, r_0, R_0, segmentation, beta):
        tube1 = self.system_parameters[0]
        tube2 = self.system_parameters[1]
        span = np.append([0], segmentation.S)
        Length = np.empty(0)
        r = np.empty((0, 3))
        u_z = np.empty((0, 2))
        alpha = np.empty((0, 2))
        u1_xy_0 = np.array([[0.0], [0.0]])
        u1_xy_0[0, 0] = (1 / (segmentation.EI[0, 0] + segmentation.EI[1, 0])) * \
                        (segmentation.EI[0, 0] * segmentation.U_x[0, 0] +
                         segmentation.EI[1, 0] * segmentation.U_x[1, 0] * np.cos(- alpha_0[1, 0]) +
                         segmentation.EI[1, 0] *
                         segmentation.U_y[1, 0] * np.sin(- alpha_0[1, 0]))
        u1_xy_0[1, 0] = (1 / (segmentation.EI[0, 0] + segmentation.EI[1, 0])) * \
                        (segmentation.EI[0, 0] * segmentation.U_y[0, 0] +
                         -segmentation.EI[1, 0] * segmentation.U_x[1, 0] * np.sin(-alpha_0[1, 0]) +
                         segmentation.EI[1, 0] *
                         segmentation.U_y[1, 0] * np.cos(-alpha_0[1, 0]))

        # reset initial parameters for ode solver
        for seg in range(0, len(segmentation.S)):
            # Initial conditions: 3 initial curvature of tube 1, 3 initial twist for tube 2 and 3, 3 initial angle,
            # 3 initial position, 9 initial rotation matrix
            y_0 = np.vstack((u1_xy_0, uz_0, alpha_0, r_0, R_0)).ravel()
            s = solve_ivp(lambda s, y: self.ode_eq(s, y, segmentation.U_x[:, seg], segmentation.U_y[:, seg],
                                                   segmentation.EI[:, seg], segmentation.GJ[:, seg]),
                          [span[seg], span[seg + 1]],
                          y_0, method='RK23', max_step=0.01)
            Length = np.append(Length, s.t)
            ans = s.y.transpose()
            u_z = np.vstack((u_z, ans[:, (2, 3)]))
            alpha = np.vstack((alpha, ans[:, (4, 5)]))
            r = np.vstack((r, ans[:, (6, 7, 8)]))
            dtheta2 = ans[-1, 3] - ans[-1, 2]
            # new boundary conditions for next segment
            uz_0 = u_z[-1, :].reshape(2, 1)
            r_0 = r[-1, :].reshape(3, 1)
            R_0 = np.array(ans[-1, 9:]).reshape(9, 1)
            alpha_0 = alpha[-1, :].reshape(2, 1)
            u1 = ans[-1, (0, 1, 2)].reshape(3, 1)
            if seg < len(
                    segmentation.S) - 1:  # enforcing continuity of moment to estimate initial curvature for next
                # segment

                K1 = np.diag(np.array([segmentation.EI[0, seg], segmentation.EI[0, seg], segmentation.GJ[0, seg]]))
                K2 = np.diag(np.array([segmentation.EI[1, seg], segmentation.EI[1, seg], segmentation.GJ[1, seg]]))
                U1 = np.array([segmentation.U_x[0, seg], segmentation.U_y[0, seg], 0]).reshape(3, 1)
                U2 = np.array([segmentation.U_x[1, seg], segmentation.U_y[1, seg], 0]).reshape(3, 1)

                GJ = segmentation.GJ
                GJ[segmentation.EI[:, seg + 1] == 0] = 0
                K1_new = np.diag(
                    np.array([segmentation.EI[0, seg + 1], segmentation.EI[0, seg + 1], segmentation.GJ[0, seg + 1]]))
                K2_new = np.diag(
                    np.array([segmentation.EI[1, seg + 1], segmentation.EI[1, seg + 1], segmentation.GJ[1, seg + 1]]))
                U1_new = np.array([segmentation.U_x[0, seg + 1], segmentation.U_y[0, seg + 1], 0]).reshape(3, 1)
                U2_new = np.array([segmentation.U_x[1, seg + 1], segmentation.U_y[1, seg + 1], 0]).reshape(3, 1)

                R_theta2 = np.array(
                    [[np.cos(alpha_0[1, 0]), -np.sin(alpha_0[1, 0]), 0],
                     [np.sin(alpha_0[1, 0]), np.cos(alpha_0[1, 0]), 0],
                     [0, 0, 1]])
                e3 = np.array([0, 0, 1]).reshape(3, 1)
                u2 = R_theta2.transpose() @ u1 + dtheta2 * e3
                K_inv_new = np.diag(np.array(
                    [1 / (segmentation.EI[0, seg + 1] + segmentation.EI[1, seg + 1]),
                     1 / (segmentation.EI[0, seg + 1] + segmentation.EI[1, seg + 1]),
                     1 / (segmentation.GJ[0, seg + 1] + segmentation.GJ[1, seg + 1])]))
                u1_new = K_inv_new @ (K1 @ (u1 - U1) + R_theta2 @ K2 @ (u2 - U2) + K1_new @ U1_new
                                      + R_theta2 @ K2_new @ U2_new - R_theta2 @ K2_new @ (dtheta2 * e3))
                u1_xy_0 = u1_new[0:2, 0].reshape(2, 1)

        # Code differs starting here.
        d_tip = np.array([tube1.L, tube2.L]) + beta
        u_z_end = np.array([0.0, 0.0, 0.0])
        tip_pos = np.array([0, 0, 0])
        for k in range(0, 2):
            try:
                b = np.argmax(Length >= d_tip[k] - 1e-5)
            except ValueError:
                # The tube is fully retracted
                return np.array([0, 0, 0]), np.array([0, 0, 0]), np.array([0, 0, 0])
            u_z_end[k] = u_z[b, k]
            tip_pos[k] = b
        return r, u_z_end, tip_pos
"""


if __name__ == '__main__':
    # Defining parameters of each tube, numbering starts with the most inner tube
    # length, length_curved, diameter_inner, diameter_outer, stiffness, torsional_stiffness, x_curvature, y_curvature
    ctr_system = {'inner':
                      {'length': 400e-3, 'length_curved': 200e-3, 'diameter_inner': 2 * 0.35e-3,
                       'diameter_outer': 2 * 0.55e-3,
                       'stiffness': 70e+9, 'torsional_stiffness': 10.0e+9, 'x_curvature': 12.0, 'y_curvature': 0},
                  'outer':
                      {'length': 300e-3, 'length_curved': 150e-3, 'diameter_inner': 2 * 0.7e-3,
                       'diameter_outer': 2 * 0.9e-3,
                       'stiffness': 70e+9, 'torsional_stiffness': 10.0e+9, 'x_curvature': 6.0, 'y_curvature': 0}
                  }
    ctr_kine = TwoTubeCTRKinematics(ctr_system)

    # Joint variables
    del_q = np.array([0.0, 0.0, 0.0, 0.0])
    # Initial position of joints
    q_0 = np.array([-300e-3, -200e-3, 0, 0])
    q = q_0 + del_q
    ee_pos = ctr_kine.forward_kinematics(q)
    print("computed ee_pos: " + str(ee_pos))

    fig = plt.figure(figsize=(5, 5), dpi=150)
    for _ in range(10):
        alphas = np.random.uniform(low=-np.pi, high=np.pi, size=2)
        beta_1 = np.random.uniform(low=-ctr_system['inner']['length'], high=0)
        beta_2 = np.random.uniform(low=-beta_1, high=0)
        q = np.array([beta_1, beta_2, alphas[0], alphas[1]])
        ctr_kine.forward_kinematics(q)
        ax = plt.axes(projection='3d')
        ax.plot3D(ctr_kine.r1[:, 0] * 1000, ctr_kine.r1[:, 1] * 1000, ctr_kine.r1[:, 2] * 1000, linewidth=2.0, c='#2596BE')
        ax.plot3D(ctr_kine.r2[:, 0] * 1000, ctr_kine.r2[:, 1] * 1000, ctr_kine.r2[:, 2] * 1000, linewidth=3.0, c='#D62728')
        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Y (mm)")
        ax.set_zlabel("Z (mm)")
        ax.set_box_aspect([1, 1, 1])
        plt.show()
