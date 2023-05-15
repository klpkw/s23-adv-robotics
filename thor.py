import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3
from spatialmath import base
import roboticstoolbox as rtb


class ThorRobot(rtb.models.DH.Puma560):
    def __init__(self, symbolic=False):

        if symbolic:
            import spatialmath.base.symbolic as sym

            zero = sym.zero()
            pi = sym.pi()
        else:
            from math import pi

            zero = 0.0

        deg = pi / 180
        inch = 0.0254

        base = 0.202  # from mounting surface to shoulder axis

        L = [
            RevoluteDH(
                d=base,  # link length (Dennavit-Hartenberg notation)
                a=0,  # link offset (Dennavit-Hartenberg notation)
                alpha=pi / 2,  # link twist (Dennavit-Hartenberg notation)
                I=[0, 0.35, 0, 0, 0, 0],
                r=[0, 0, 0],
                m=0,  # mass of link
                Jm=200e-6,  # actuator inertia
                G=-62.6111,  # gear ratio
                B=1.48e-3,  # actuator viscous friction coefficient (measured
                # at the motor)
                Tc=[0.395, -0.435],
                qlim=[-180 * deg, 180 * deg],  # minimum and maximum joint angle
            ),
            RevoluteDH(
                d=0,
                a=0.160,
                alpha=zero,
                I=[0.13, 0.524, 0.539, 0, 0, 0],
                r=[-0.3638, 0.006, 0.2275],
                m=17.4,
                Jm=200e-6,
                G=107.815,
                B=0.817e-3,
                Tc=[0.126, -0.071],
                qlim=[-90 * deg, 90 * deg],  # qlim=[-45*deg, 225*deg]
            ),
            RevoluteDH(
                d=0,
                a=0,
                alpha=-pi / 2,
                I=[0.066, 0.086, 0.0125, 0, 0, 0],
                r=[-0.0203, -0.0141, 0.070],
                m=4.8,
                Jm=200e-6,
                G=-53.7063,
                B=1.38e-3,
                Tc=[0.132, -0.105],
                qlim=[-135 * deg, 135 * deg],  # qlim=[-225*deg, 45*deg]
            ),
            RevoluteDH(
                d=0.195,
                a=0,
                alpha=pi / 2,
                I=[1.8e-3, 1.3e-3, 1.8e-3, 0, 0, 0],
                r=[0, 0.019, 0],
                m=0.82,
                Jm=33e-6,
                G=76.0364,
                B=71.2e-6,
                Tc=[11.2e-3, -16.9e-3],
                qlim=[-180 * deg, 180 * deg],  # qlim=[-110*deg, 170*deg]
            ),
            RevoluteDH(
                d=0,
                a=0,
                alpha=-pi / 2,
                I=[0.3e-3, 0.4e-3, 0.3e-3, 0, 0, 0],
                r=[0, 0, 0],
                m=0.34,
                Jm=33e-6,
                G=71.923,
                B=82.6e-6,
                Tc=[9.26e-3, -14.5e-3],
                qlim=[-90 * deg, 90 * deg],
            ),
            RevoluteDH(
                d=0.06715,
                a=0,
                alpha=zero,
                I=[0.15e-3, 0.15e-3, 0.04e-3, 0, 0, 0],
                r=[0, 0, 0.032],
                m=0.09,
                Jm=33e-6,
                G=76.686,
                B=36.7e-6,
                Tc=[3.96e-3, -10.5e-3],
                qlim=[-180 * deg, 180 * deg],
            ),
        ]

        #self.qr = np.array([0, pi / 2, -pi / 2, 0, 0, 0])
        #self.qz = np.zeros(6)
#
        ## nominal table top picking pose
        #self.qn = np.array([0, pi / 4, pi, 0, pi / 4, 0])
#
        #self.addconfiguration("qr", self.qr)
        #self.addconfiguration("qz", self.qz)
        #self.addconfiguration("qn", self.qn)
#
        ## straight and horizontal
        #self.addconfiguration_attr("qs", np.array([0, 0, -pi / 2, 0, 0, 0]))