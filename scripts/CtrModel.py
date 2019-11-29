import numpy as np
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

    def compute_forward_kinematics(self):
        pass

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
