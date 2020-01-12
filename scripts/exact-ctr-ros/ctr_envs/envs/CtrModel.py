import numpy as np
from math import pi, pow
import time

from scipy.integrate import odeint
import rospy
import tf


class TubeParameters(object):
    def __init__(self, length, length_curved, outer_diameter, inner_diameter, stiffness, torsional_stiffness,
                 x_curvature, y_curvature):
        self.L = length
        self.L_s = length - length_curved
        self.L_c = length_curved
        self.J = (pi * (pow(outer_diameter, 4) - pow(inner_diameter, 4))) / 32
        self.I = (pi * (pow(outer_diameter, 4) - pow(inner_diameter, 4))) / 64
        self.E = stiffness
        self.G = torsional_stiffness
        self.U_x = x_curvature
        self.U_y = y_curvature


# Initialized with objects of class TubeParameters
class SegmentRobot(object):
    def __init__(self, t1, t2, t3, base):
        n = 3

        stiffness = np.array([t1.E, t2.E, t3.E])
        curve_x = np.array([t1.U_x, t2.U_x, t3.U_x])
        curve_y = np.array([t1.U_y, t2.U_y, t3.U_y])

        # position of tip of tubes
        d_tip = np.array([t1.L, t2.L, t3.L]) + base
        # positions where bending starts
        d_c = d_tip - np.array([t1.L_c, t2.L_c, t3.L_c])
        points = np.hstack((0, base, d_c, d_tip))
        index = np.argsort(points)
        # floor is used to ensure absolute zero is used
        segment_length = 1e-5 * np.floor(1e5 * np.diff(np.sort(points)))

        e = np.zeros((3, segment_length.size))
        u_x = np.zeros((3, segment_length.size))
        u_y = np.zeros((3, segment_length.size))

        for i in range(n):
            # +1 because, one point is at 0
            # find where the tube begins
            a = np.where(index == i + 1)[0]
            # find where tube curve starts
            b = np.where(index == i + 4)[0]
            # Find where tube ends
            c = np.where(index == i + 7)[0]
            if segment_length[a] == 0:
                a += 1
            if segment_length[b] == 0:
                b += 2
            if (c.item() <= segment_length.size - 1) and (segment_length[c] == 0):
                c += 1

            # For the segment a to c, set the stiffness for tube i
            e[i, np.arange(a, c)] = stiffness[i]
            # For segment b to c set curvature for tube i in x direction
            u_x[i, np.arange(b, c)] = curve_x[i]
            # For segment b to c set curvature for tube i in y direction
            u_y[i, np.arange(b, c)] = curve_y[i]

        # Removing zero lengths
        length = segment_length[np.nonzero(segment_length)]
        ee = e[:, np.nonzero(segment_length)]
        uu_x = u_x[:, np.nonzero(segment_length)]
        uu_y = u_y[:, np.nonzero(segment_length)]

        # Cumulative sum to determine length from origin
        length_sum = np.cumsum(length)
        # s is the segmented abscissa (y-coordinate) of the tube after template (s=0)
        self.S = length_sum[length_sum + min(base) > 0] + min(base)
        # Truncating matrices, removing elements that correspond to the tube before template
        e_t = ee[length_sum + min(base) > np.zeros_like(ee)].reshape(3, len(self.S))
        # Each (i,j) element of above matrices correspond to jth segment of ith tube where first tube is most inner
        self.EI = (e_t.T * np.array([t1.I, t2.I, t3.I])).T
        self.U_x = uu_x[length_sum + min(base) > 0 * ee].reshape(3, len(self.S))
        self.U_y = uu_y[length_sum + min(base) > 0 * ee].reshape(3, len(self.S))
        self.GJ = np.array([t1.G, t2.G, t3.G]) * np.array([t1.J, t2.J, t3.J])


class CTRModel(object):
    def __init__(self, tube_1, tube_2, tube_3):
        self.tube_1 = tube_1
        self.tube_2 = tube_2
        self.tube_3 = tube_3

        rospy.init_node("exact_ctr_kinematics")
        self.br = tf.TransformBroadcaster()

    # q[0:2] are extension values and q[3:5] rotation values from the base
    # q0 are starting joint values
    # uz_0 is column vector of initial torsion about z axis
    # r_0 is base position
    # R_0 is initial base orientation
    # def fk(self, q, q0, uz_0, alpha_0, r_0, R_0):
    def fk(self, q, **kwargs):
        if 'q_0' not in kwargs:
            kwargs['q_0'] = np.array([0, 0, 0, 0, 0, 0])
        if 'r_0' not in kwargs:
            kwargs['r_0'] = np.array([0, 0, 0]).reshape(3, 1)
        if 'uz_0' not in kwargs:
            # initial twist
            kwargs['uz_0'] = np.array([0, 0, 0])

        uz_0 = kwargs['uz_0']
        r_0 = kwargs['r_0']
        alpha_1_0 = q[3] + kwargs['q_0'][3]
        R_0 = np.array(
            [[np.cos(alpha_1_0), -np.sin(alpha_1_0), 0], [np.sin(alpha_1_0), np.cos(alpha_1_0), 0], [0, 0, 1]]) \
            .reshape(9, 1)
        alpha_0 = q[3:].reshape(3, 1) + kwargs['q_0'][3:].reshape(3, 1)

        # position of tubes' base from template (i.e., s=0)
        beta = q[0:3] + kwargs['q_0'][0:3]
        segments = SegmentRobot(self.tube_1, self.tube_2, self.tube_3, beta)

        Length = np.empty(0)
        r = np.empty((0, 3))
        u_z = np.empty((0, 3))
        alpha = np.empty((0, 3))
        span = np.append([0], segments.S)
        for seg in range(0, len(segments.S)):
            # Initial conditions, 3 initial twist + 3 initial angle + 3 initial position + 9 initial rotation matrix
            y_0 = np.vstack((uz_0.reshape(3, 1), alpha_0, r_0, R_0)).ravel()
            s_span = np.linspace(span[seg], span[seg + 1] - 1e-6, num=30)
            s = odeint(self.ode_eq, y_0, s_span, args=(segments.U_x[:, seg], segments.U_y[:, seg],
                                                       segments.EI[:, seg], segments.GJ))
            Length = np.append(Length, s_span)
            u_z = np.vstack((u_z, s[:, (0, 1, 2)]))
            alpha = np.vstack((alpha, s[:, (3, 4, 5)]))
            r = np.vstack((r, s[:, (6, 7, 8)]))

            # new boundary conditions for next segment
            r_0 = r[-1, :].reshape(3, 1)
            R_0 = np.array(s[-1, 9:]).reshape(9, 1)
            uz_0 = u_z[-1, :].reshape(3, 1)
            alpha_0 = alpha[-1, :].reshape(3, 1)

        d_tip = np.array([self.tube_1.L, self.tube_2.L, self.tube_3.L]) + beta
        u_z_end = np.array([0.0, 0.0, 0.0])
        tip_pos = np.array([0, 0, 0])
        for k in range(0, 3):
            b = np.argmax(Length >= d_tip[k] - 1e-3)  # Find where tube curve starts
            u_z_end[k] = u_z[b, k]
            tip_pos[k] = b

        return r, u_z_end, tip_pos

    def ode_eq(self, y, s, ux_0, uy_0, ei, gj):
        # first num_tubes elements represent curvatures along z
        # second num_tubes elements represent twist angles, alpha_i
        # last 12 elements are r (positions), R (orientations)
        dydt = np.empty([18, 1])
        ux = np.empty([3, 1])
        uy = np.empty([3, 1])

        # calculate, for each tube, tube curvature in x and y direction
        for i in range(3):
            ux[i] = (1 / (ei[0] + ei[1] + ei[2])) * \
                (ei[0] * ux_0[0] * np.cos(y[3 + i] - y[3 + 0]) + ei[0] * uy_0[0] * np.sin(y[3 + i] - y[3 + 0]) +
                 ei[1] * ux_0[1] * np.cos(y[3 + i] - y[3 + 1]) + ei[1] * uy_0[1] * np.sin(y[3 + i] - y[3 + 1]) +
                 ei[2] * ux_0[2] * np.cos(y[3 + i] - y[3 + 2]) + ei[2] * uy_0[2] * np.sin(y[3 + i] - y[3 + 2]))
            uy[i] = (1 / (ei[0] + ei[1] + ei[2])) * \
                    (-ei[0] * ux_0[0] * np.sin(y[3 + i] - y[3 + 0]) + ei[0] * uy_0[0] * np.cos(y[3 + i] - y[3 + 0]) +
                     -ei[1] * ux_0[1] * np.sin(y[3 + i] - y[3 + 1]) + ei[1] * uy_0[1] * np.cos(y[3 + i] - y[3 + 1]) +
                     -ei[2] * ux_0[2] * np.sin(y[3 + i] - y[3 + 2]) + ei[2] * uy_0[2] * np.cos(y[3 + i] - y[3 + 2]))
        # ode for twist
        for j in range(3):
            if ei[j] == 0:
                # ui_z
                dydt[j] = 0
                # alpha_i
                dydt[3+j] = 0
            else:
                # ui_z
                dydt[j] = ((ei[j]) / (gj[j])) * (ux[j] * uy_0[j] - uy[j] * ux_0[j])
                # alpha_i
                dydt[3 + j] = y[j]

        e3 = np.array([0, 0, 1]).reshape(3, 1)
        # y[0:2] are position of point materials
        # y[3:end] rotation matrix elements
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

    # n,3 shape transforms, no orientation information
    def publish_transforms(self, r):
        count = 0
        for point in r:
            self.br.sendTransform((point[0] * 100, point[1] * 100, point[2] * 100),
                                  (tf.transformations.quaternion_from_euler(0, 0, 0)),
                                  rospy.Time.now(),
                                  "point_" + str(count),
                                  "world")
            count = count + 1


if __name__ == '__main__':
    # define tube parameters
    # length, length_curved, inner_diameter, outer_diameter, stiffness, torsional_stiffness, x_curvature, y_curvature
    tube1 = TubeParameters(length=431e-3, length_curved=103e-3, inner_diameter=2*0.35e-3, outer_diameter=2*0.55e-3,
                           stiffness=6.4359738368e+10, torsional_stiffness=2.5091302912e+10, x_curvature=21.3,
                           y_curvature=0)
    tube2 = TubeParameters(length=332e-3, length_curved=113e-3, inner_diameter=2 * 0.7e-3, outer_diameter=2 * 0.9e-3,
                           stiffness=5.2548578304e+10, torsional_stiffness=2.1467424256e+10, x_curvature=13.108,
                           y_curvature=0)
    tube3 = TubeParameters(length=174e-3, length_curved=134e-3, inner_diameter=2e-3, outer_diameter=2 * 1.1e-3,
                           stiffness=4.7163091968e+10, torsional_stiffness=2.9788923392e+10, x_curvature=3.5,
                           y_curvature=0)
    # Joint variables(Beta)
    q = np.array([-tube1.L, 0, 0, 0, 0, 0])
    ctr_model = CTRModel(tube1, tube2, tube3)
    while not rospy.is_shutdown():
        r, u_z_end, tip_pos = ctr_model.fk(q)
        ctr_model.publish_transforms(r)
        rospy.sleep(0.5)

