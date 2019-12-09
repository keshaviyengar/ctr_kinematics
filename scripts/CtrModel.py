import numpy as np
import scipy as sp
import scipy.integrate
import rospy


class CtrModel(object):
    def __init__(self):
        # Number of tubes
        self.n = 3

        # Define length of tubes
        self.tube_length = 1e-3 * np.array([431, 332, 174])
        # Define length of curved part of tubes
        self.curved_length = 1e-3 * np.array([103, 113, 134])

        # Material properties and inertias
        # Young's Modulus
        self.E = np.array([6.4359738368e+10, 5.2548578304e+10, 4.7163091968e+10])
        # Polar moment of inertia
        self.J = 1.0e-11 * np.array([0.0120, 0.0653, 0.1686])
        # Second moment of inertia
        self.I = 1.0e-12 * np.array([0.0601, 0.3267, 0.8432])
        # Shear modulus
        self.G = np.array([2.5091302912e+10, 2.1467424256e+10, 2.9788923392e+10])
        # Curvature vector of x-component
        self.Ux = np.array([21.3, 13.108, 3.5])
        # Curvature vector of y-component
        self.Uy = np.array([0, 0, 0])

    # @params: E, Ux, Uy, I, G, J, l, B, l_k
    # stiffness, curvature vector x, curvature vector y, moment of inertia, torsional constant,
    # second moment of inertia vectors for each tube, vector of tube length, tube extension relative to s=0,
    # vector of curved path length.
    # @output: L, d1, E, Ux, Uy, I, G, J
    # Length of each segment, position of each tube end, segment stiffness, segment curvature in x, segment
    # curvature in y, segment moment of inertia, segment torsion constant, segment second moment of inertia
    def segment_robot(self, E, Ux, Uy, I, G, J, l, B, l_k):
        # Vectors are sorted. Elements are from inner most first.
        k = len(l)
        d1 = l + B  # positions of end of tubes
        d2 = d1 - l_k  # position of where bending starts
        points = np.array([0, B, d2, d1])
        L = np.sort(points)
        index = np.argsort(L)
        L = 1e-5 * np.floor(1e5 * np.diff(L))  # length of each segment

        EE = np.zeros(3, len(L))
        II = EE, GG = EE, JJ = EE, UUx = EE, UUy = EE

        # Iterating through tubes
        for i in range(k):
            a = np.asarray(index == (i + 1))  # Find where tube begins
            b = np.asarray(index == (1 * k + i + 1))  # Find where tube curve starts
            c = np.asarray(index == (2 * k + i + 1))  # Find where tube ends

            if L(a) == 0: a = a + 1
            if L(b) == 0: b = b + 1
            if L(c) <= len(L):
                if L(c) == 0: c = c + 1

            EE[i, a:c - 1] = E[i]
            II[i, a:c - 1] = I[i]
            GG[i, a:c - 1] = G[i]
            JJ[i, a:c - 1] = J[i]
            UUx[i, b:c - 1] = Ux[i]
            UUy[i, b:c - 1] = Uy[i]

        l = L[L != 0]  # Get only non-zero elements
        E = np.zeros(k, len(l))
        I = E, G = E, J = E, UUx = E, UUy = E

        for i in range(k):
            E[i, :] = EE[i, L[L != 0]]
            I[i, :] = II[i, L[L != 0]]
            G[i, :] = GG[i, L[L != 0]]
            J[i, :] = JJ[i, L[L != 0]]
            Ux[i, :] = UUx[i, L[L != 0]]
            Uy[i, :] = UUy[i, L[L != 0]]
        L = L[L != 0]

        return L, d1, E, Ux, Uy, I, G, J

    def compute_forward_kinematics(self, q, uz_0):
        l = self.tube_length
        l_k = self.curved_length

        # q0 to q2 is extension and q3 to q5 is rotation of each tube from base
        B = q[1:3]
        alpha = q[4:6] - B * np.transpose(uz_0)
        alpha_0 = alpha[0]

        # Segment tubes
        [L, d_tip, EE, UUx, UUy] = self.segment_robot(self.E, self.Ux, self.Uy,
                                                      self.I, self.G, self.J, l, B, l_k)
        SS = L
        for i in range(len(L)):
            SS[i] = sum(L[0:i])
        # S is segmented abssica of tube after template
        S = SS[SS + np.min(B) > 0] + np.min(B)
        E = np.zeros([self.n, len(S)])
        Ux = E, Uy = E
        for i in range(self.n):
            E[i, :] = EE[i, SS + min(B) > 0]
            Ux[i, :] = UUx[i, SS + np.min(B) > 0]
            Uy[i, :] = UUx[i, SS + np.min(B) > 0]

        # Each (i, j) in the element above matrices correspond to the jth segment of the
        # ith tube with inner most first in order
        # vector of x-coordinates starting at zero
        span = np.array([0, S])
        # Solve length, curvature and twist angles
        Length = np.array([])
        r = np.array([])
        U_z = np.array([])
        a = np.array([])
        r0 = np.zeros([3, 1])
        R0 = np.ndarray([[np.cos(alpha_0), np.sin(alpha_0), 0],
                         [-np.sin(alpha_0), np.cos(alpha_0), 0],
                         [0, 0, 1]])
        R0 = np.reshape([9, 1])

        for segment in range(len(S)):
            s_span = np.array([span[segment], span[segment + 1] - 1e-6])
            y0_1 = np.array([r0, R0])
            y0_2 = np.zeros([2 * self.n, 1])
            y0_2[self.n:2 * self.n] = alpha
            y0_2[0:self.n] = uz_0
            y_0 = np.array([y0_2, y0_1])

            # ode equation
            [s, y] = scipy.integrate.odeint(self.ode, y_0, s_span, [Ux[:, segment], Uy[:, segment],
                                                                    E[:, segment] * np.transpose(self.I),
                                                                    self.G * self.J, self.n])
            # first n elemetns of y are curvatures along z, eg y = [u1_z, u2_z ..]
            # last n elements of y are twist angles, alpha_i
            shape = np.array([y[:, 2 * self.n], y[:, 2 * self.n + 1], y[:, 2 * self.n + 2]])
            Length = np.append([Length, s])
            r = np.append([r, shape])
            U_z = np.append([U_z, y[0:self.n]])
            a = np.append([a, y[:, self.n:2 * self.n]])
            r0 = np.transpose(shape[-1, :])
            R0 = np.transpose(y[-1, 2 * self.n + 3: 2 * self.n + 11])
            uz_0 = np.transpose(U_z[-1, :])
            alpha = np.transpose(a[-1, :])

        Uz = np.zeros([self.n, 1])
        for i in range(self.n):
            index = np.min(np.abs(Length - d_tip[i] + 1e-6))
            Uz[i] = U_z[index, i]

        r1 = r
        tube2_end = np.min(np.abs(Length - d_tip[2]))
        r2 = np.array([r[0:tube2_end, 0], r[0:tube2_end, 1], r[0:tube2_end, 2]])
        tube3_end = np.min(np.abs(Length - d_tip[3]))
        r3 = np.array([r[0:tube3_end, 0], r[0:tube3_end, 1], r[0:tube3_end, 2]])

        return r1, r2, r3, Uz

    def publish_segment_transforms(self):
        pass

    def ode(self, y, Ux, Uy, EI, GJ, n):
        dyds = np.zeros(2 * n + 12, 1)
        # First n elements of y are curvatures along z, eg. y = [u1_z, u2_z ...]
        # Second n elements of y are twist angles, alpha_i
        # Last 12 elements r (position) and R (orientations)

        # Calculate tube's curvature x and y direction
        ux = np.zeros(n, 1)
        uy = np.zeros(n, 1)

        # Calculate other tube's curvature in x and y direction
        for i in range(n):
            ux[i] = (1 / (EI[1] + EI[2] + EI[3]) *
                     EI[1] * Ux[1] * np.cos(y[n + i] - y[n + 1]) + EI[1] * Uy[1] * np.sin(y[n + i] - y[n + 1]) +
                     EI[2] * Ux[2] * np.cos(y[n + i] - y[n + 2]) + EI[2] * Uy[2] * np.sin(y[n + i] - y[n + 2]) +
                     EI[3] * Ux[3] * np.cos(y[n + i] - y[n + 3]) + EI[3] * Uy[3] * np.sin(y[n + i] - y[n + 3]))
            uy[i] = (1 / (EI[1] + EI[2] + EI[3]) *
                     EI[1] * Ux[1] * np.sin(y[n + i] - y[n + 1]) + EI[1] * Uy[1] * np.cos(y[n + i] - y[n + 1]) +
                     EI[2] * Ux[2] * np.sin(y[n + i] - y[n + 2]) + EI[2] * Uy[2] * np.cos(y[n + i] - y[n + 2]) +
                     EI[3] * Ux[3] * np.sin(y[n + i] - y[n + 3]) + EI[3] * Uy[3] * np.cos(y[n + i] - y[n + 3]))

        # Might need to transpose GJ
        GJ[EI == 0] = 0
        for i in range(n):
            if EI[i] == 0:
                # ui_z
                dyds[i] = 0
                # alpha_i
                dyds[n + i] = 0
            else:
                # ui_z
                dyds[i] = ((EI[i]) / (GJ[i])) * (ux[i] * Uy[i] - uy[i] * Ux[i])
                # alpha_i
                dyds[n + 1] = y[i]

        e3 = np.array([0, 0, 1])
        uz = y[0:n]
        # y[0] to y[2] are positions of point materials
        # y[3] to y[11] are rotation matrix elements
        R1 = np.ndarray([y[2 * n + 3], y[2 * n + 4], y[2 * n + 5]],
                        [y[2 * n + 6], y[2 * n + 7], y[2 * n + 8]],
                        [y[2 * n + 9], y[2 * n + 10], y[2 * n + 11]])

        u_hat = np.ndarray([0, -uz[0], uy[1]],
                           [uz[0], 0, ux[0]],
                           [uy[0], ux[0], 0])
        # odes
        dr1 = R1 * e3
        dR1 = R1 * u_hat

        dyds[2 * n] = dr1[0], dyds[2 * n + 1] = dr1[1], dyds[2 * n + 2] = dr1[2]

        dR = np.transpose(dR1)[:]
        for i in range(4, 12):
            dyds[2 * n + i] = dR(i - 2)
