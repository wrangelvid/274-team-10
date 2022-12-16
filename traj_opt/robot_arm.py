from pydrake.all import (
    namedview,
    LeafSystem_,
    TemplateSystem,
    Variable,
    Expression,
    AutoDiffXd,
    sin,
    cos,
    pow,
    sqrt,
    PyPlotVisualizer,
    PortDataType,
)
from typing import Union
import numpy as np
from sympy import symbols, simplify
from sympy import sin as sy_sin
from sympy import cos as sy_cos
from sympy import sqrt as sy_sqrt

OptimizationType = Union[Expression, float, np.float64, AutoDiffXd]

ArmState = namedview("ArmState", ["th1", "th2", "th3", "dth1", "dth2", "dth3"])

def simplify_exp(exp:Expression, verbose:bool=False)->Expression:
    """Simplifies an Expression with sympy.

    May take some while.    

    Args:
        exp: expression to simplify.
        verbose: prints out how much percent we saved.
  
    Returns:
        Simplified expression.
    """
    drake_variables = list(exp.GetVariables())
    str_variables = [str(var) for var in drake_variables] 

    sy_pow = lambda a,n: a**n

    exec(",".join(str_variables) + "="+ f'symbols("{" ".join(str_variables)}")')

    exp_str = exp.to_string()

    simplified_str = str(simplify((eval(exp_str.replace('sin', 'sy_sin').replace('cos', 'sy_cos').replace('sqrt','sy_sqrt').replace('pow', 'sy_pow')))))
    for i, var in enumerate(str_variables):
        exec(var + f'= drake_variables[{i}]')
    if verbose:
        print(f'Saved: {round(100-100.*len(simplified_str)/float(len(exp_str)), 2)}%')
    simplified_exp = eval(simplified_str)
    return simplified_exp


def M_del(M, row: int, col: int) -> np.array:
    """Returns a copy of M without a given row/col index.

    If M is of size nxm, then the returned matrix has size
    n-1xm-1.
    Args:
        row: index of row to delete.
        col: index of column to delete.

    Returns:
        M without the given row and column at the given index.
    """
    return np.delete(np.delete(M, row, 0), col, 1)


def det2(M: np.array) -> Expression:
    """Computes the determinant of a 2x2 matrix.

    Args:
        M: A 2x2 matrix.

    Returns:
        The determinant of the matrix M.
    """
    if M.shape != (2, 2):
        raise Exception("Can only do 2x2 matrix.")
    return M[0][0] * M[1][1] - M[0][1] * M[1][0]


def det3(M: np.array) -> Expression:
    """Computes the determinant of a 3x3 matrix.

    Args:
        M: A 3x3 matrix.

    Returns:
        The determinant of the matrix M.
    """
    if M.shape != (3, 3):
        raise Exception("Can only do 3x3 matrix.")

    return (
        M[0][0] * det2(M_del(M, 0, 0))
        - M[0][1] * det2(M_del(M, 0, 1))
        + M[0][2] * det2(M_del(M, 0, 2))
    )


def inv3(M: np.array) -> np.array:
    """Computes the inverse of a 3x3 matrix.

    M^-1 = 1/det(M) * adj(M)

    We divide the adjacency matrix of M by the determinant of M.

    Args:
        M: A 3x3 Matrix of Expressions.

    Returns:
        The inverse of the matrix M.
    """
    if M.shape != (3, 3):
        raise Exception("Can only do 3x3 matrix.")

    M_det = det3(M)

    M_inv = np.array(
        [
            [
                det2(M_del(M.T, 0, 0)) / M_det,
                -det2(M_del(M.T, 0, 1)) / M_det,
                det2(M_del(M.T, 0, 2)) / M_det,
            ],
            [
                -det2(M_del(M.T, 1, 0)) / M_det,
                det2(M_del(M.T, 1, 1)) / M_det,
                -det2(M_del(M.T, 1, 2)) / M_det,
            ],
            [
                det2(M_del(M.T, 2, 0)) / M_det,
                -det2(M_del(M.T, 2, 1)) / M_det,
                det2(M_del(M.T, 2, 2)) / M_det,
            ],
        ]
    )
    return M_inv


def Jacobian(expression_matrix, var):
    if isinstance(expression_matrix, Expression):
        return expression_matrix.Jacobian(var)
    else:
        if len(expression_matrix[0]) > 1:
            raise Exception("Nested vector is unhandled.")
        return np.array([e[0].Jacobian(var) for e in expression_matrix])


class ArmDynamics:
    """The Lagrangian dynamics for the four bar throwing Arm.

    The last joint is underactuated and it's stiffness is determined with k.
    """

    def __init__(self, l1: float, l2: float, k: float = 4 * 24, verbose:bool = False):
        """Builds the dynamics of the Arm.

        Args:
            l1: The joint to joint length of the first link.
            l2: The joint to joint length of the second link.
            k: The spring stiffness on either side of the wrist joint.
            verbose: prints out simplificaiton informations and warnings.
        """
        self.lam_bars = 1200 * 2 * 6e-3 * 30e-3
        self.rho_fb2 = 1200 * 6e-3  # denisity of four bar link 2
        self.d_shaft = 8e-3
        self.r_shaft = self.d_shaft / 2
        self.wfb2 = 16e-3  # width of four bar link 2
        self.k = k  # N/m
        self.N = 18.75
        self.lsp0 = 0.028  # m 28 mm

        # joint-to-joint lengths
        self.l1 = l1
        self.l2 = l2
        self.l3 = 50e-3
        self.lfb0 = 40e-3
        self.lfb1 = 25e-3
        self.lfb2 = self.lfb0 + self.l1
        self.d1 = 31e-3  # used for spring
        #self.d2 = 60e-3  # used for spring
        self.d2 = 25e-3
        self.d3 = 30e-3  # used for spring

        # moment of inertias (Izz)
        self.I1A = 378964e-9
        self.I1B = 24605e-9
        self.I2A = 26528e-9
        self.I2B = 24605e-9
        #self.I3 = 55979e-9
        self.I3 = 65113e-9
        self.Ifb1 = 9115e-9
        self.Ifb2 = self.rho_fb2 * (
            1 / 12 * self.lfb2 * self.wfb2 * (self.lfb2**2 + self.wfb2**2)
            + np.pi
            * (self.wfb2 / 2) ** 2
            * ((self.lfb2 / 2) ** 2 + 1 / 2 * (self.wfb2 / 2) ** 2)
            - 2
            * np.pi
            * self.r_shaft**2
            * ((self.lfb2 / 2) ** 2 + 1 / 2 * np.pi * self.r_shaft**2)
        )
        self.Ir = 0.0035 / self.N**2

        # COM distances
        self.lm1A = -28.54e-3
        self.lm1B = -22.52e-3
        self.lm2A = 27.85e-3
        self.lm2B = -22.52e-3
        self.lm3 = 36.41e-3
        self.lmfb1 = 11.29e-3
        self.lmfb2 = self.lfb2 / 2

        # masses
        self.m1A = 360.27e-3
        self.m1B = 72.70e-3
        self.m2A = 78.68e-3
        self.m2B = 72.70e-3
        #self.m3 = 119.88e-3
        self.m3 = 129.49e-3
        self.mfb1 = 27.37e-3
        self.mfb2 = (
            self.lfb2 * self.wfb2
            + np.pi * (self.wfb2 / 2) ** 2
            - 2 * np.pi * self.r_shaft**2
        ) * self.rho_fb2

        # acrylic bar start/stop distances
        self.lb1A = 45.0e-3
        self.lb1B = 43.5e-3
        self.lb2A = 49.0e-3
        self.lb2B = 43.5e-3

        self.th1 = Variable("th1")
        self.th2 = Variable("th2")
        self.th3 = Variable("th3")
        self.dth1 = Variable("dth1")
        self.dth2 = Variable("dth2")
        self.dth3 = Variable("dth3")
        self.ddth1 = Variable("ddth1")
        self.ddth2 = Variable("ddth2")
        self.ddth3 = Variable("ddth3")

        self.tau1 = Variable("tau1")
        self.tau2 = Variable("tau2")

        ihat = np.array([[1], [0], [0]])
        jhat = np.array([[0], [1], [0]])
        khat = np.array([[0], [0], [1]])

        q = np.array([[self.th1], [self.th2], [self.th3]])
        dq = np.array([[self.dth1], [self.dth2], [self.dth3]])
        ddq = np.array([[self.ddth1], [self.ddth2], [self.ddth3]])

        er1hat = np.sin(self.th1) * ihat + np.cos(self.th1) * jhat
        er2hat = np.sin(self.th1 + self.th2) * ihat + np.cos(self.th1 + self.th2) * jhat
        er3hat = (
            np.sin(self.th1 + self.th2 + self.th3) * ihat
            + np.cos(self.th1 + self.th2 + self.th3) * jhat
        )

        ddt = lambda r: Jacobian(r, np.vstack([q, dq])).dot(np.vstack([dq, ddq]))
        magsq = lambda r: r.T.dot(r)
        rotz = lambda thz: np.array(
            [[np.cos(thz), -np.sin(thz), 0], [np.sin(thz), np.cos(thz), 0], [0, 0, 1]]
        )

        # Joint positions
        self.rA = self.l1 * er1hat
        self.rB = self.rA + self.l2 * er2hat
        self.rC1 = -self.lfb0 * er1hat
        self.rC2 = self.rC1 + self.lfb1 * er2hat
        self.rC3 = self.rA + self.lfb1 * er2hat
        self.rS0 = self.rB - self.d1 * er2hat
        self.rS1 = self.rS0 + self.d3 * rotz(np.pi / 2).dot(er2hat)
        self.rS2 = self.rS0 + self.d3 * rotz(-np.pi / 2).dot(er2hat)

        self.rT0 = self.rB + self.d2 * er3hat
        self.rT1 = self.rT0 + self.d3 * rotz(np.pi / 2).dot(er3hat)
        self.rT2 = self.rT0 + self.d3 * rotz(-np.pi / 2).dot(er3hat)
        self.rE = self.rB + self.l3 * er3hat

        bars1 = self.l1 - self.lb1A - self.lb1B
        mbars1 = bars1 * self.lam_bars
        bars2 = self.l2 - self.lb2A - self.lb2B
        mbars2 = bars2 * self.lam_bars

        # COM posionts
        self.rm1A = self.lm1A * er1hat
        self.rm1B = self.rA + self.lm1B * er1hat
        self.rm1bars = self.rA + (self.lb1A + bars1 / 2) * er1hat
        self.rm1 = (
            self.m1A * self.rm1A + mbars1 * self.rm1bars + self.m1B * self.rm1B
        ) / (self.m1A + mbars1 + self.m1B)

        self.rm2A = self.rA + self.lm2A * er2hat
        self.rm2B = self.rB + self.lm2B * er2hat
        self.rm2bars = self.rB + (self.lb2A + bars2 / 2) * er2hat
        self.rm2 = (
            self.m2A * self.rm2A + mbars2 * self.rm2bars + self.m2B * self.rm2B
        ) / (self.m2A + mbars2 + self.m2B)

        self.rm3 = self.rB + self.lm3 * er3hat
        self.rmfb1 = self.rC1 + self.lmfb1 * er2hat
        self.rmfb2 = self.rC2 + self.lmfb2 * er1hat

        # COM velocites
        self.drm1 = ddt(self.rm1)
        self.drm2 = ddt(self.rm2)
        self.drm3 = ddt(self.rm3)
        self.drmfb1 = ddt(self.rmfb1)
        self.drmfb2 = ddt(self.rmfb2)

        # Endeffector Velocity
        self.drE = ddt(self.rE)
        self.speed = self.drE.T.dot(self.drE)[0][0]
        self.speed = simplify_exp(self.speed, verbose)

        self.neg_speed = -sqrt(self.speed)
        self.neg_speed_string = self.neg_speed.to_string()

        # aggergate inertial properites
        m1 = self.m1A + mbars1 + self.m1B
        m2 = self.m2A + mbars2 + self.m2B

        I1 = (
            self.m1A * magsq(self.rm1 - self.rm1A)
            + mbars1 * magsq(self.rm1 - self.rm1bars)
            + self.m1B * magsq(self.rm1 - self.rm1B)
        )
        I2 = (
            self.m2A * magsq(self.rm2 - self.rm2A)
            + mbars2 * magsq(self.rm2 - self.rm2bars)
            + self.m2B * magsq(self.rm2 - self.rm2B)
        )

        # moments conrtibutions to generlized forces.
        M2Q = lambda M, w: Jacobian(w, dq).T.dot(M)

        # Kinetic Energies
        T1 = 0.5 * m1 * magsq(self.drm1) + 0.5 * I1 * self.dth1**2
        T2 = (
            0.5 * m2 * magsq(self.drm2)
            + 0.5 * I2 * (self.dth1 + self.dth2)**2
        )
        T3 = (
            0.5 * self.m3 * magsq(self.drm3)
            + 0.5 * self.I3 * (self.dth1 + self.dth2 + self.dth3) ** 2
        )
        Tfb1 = (
            0.5 * self.mfb1 * magsq(self.drmfb1)
            + 0.5 * self.Ifb1 * (self.dth1 + self.dth2) ** 2
        )
        Tfb2 = (
            0.5 * self.mfb2 * magsq(self.drmfb2)
            + 0.5 * self.Ifb2 * self.dth1**2
        )
        T1r = 0.5 * self.Ir * (self.N * self.dth1) ** 2
        T2r = 0.5 * self.Ir * (self.dth1 + self.N * self.dth2) ** 2

        # Potential Energies of the Springs.
        # This requires simplification to be turned on 
        Vsp1 = 0.5 * self.k * (sqrt((self.rT1 - self.rS1).T.dot(self.rT1 - self.rS1)[0][0]) - self.lsp0)**2
        Vsp2 = 0.5 * self.k * (sqrt((self.rT2 - self.rS2).T.dot(self.rT2 - self.rS2)[0][0]) - self.lsp0)**2

        self.T = T1 + T2 + T3 + Tfb1 + Tfb2 + T1r + T2r
        self.T[0,0] = simplify_exp(self.T[0,0], verbose)

        self.V = np.array([[Vsp1 + Vsp2]])
        self.V[0,0] = simplify_exp(self.V[0,0], verbose)

        Q_tau1 = M2Q(self.tau1 * khat, self.dth1 * khat)
        Q_tau2 = M2Q(self.tau2 * khat, self.dth2 * khat)
        self.Q = Q_tau1 + Q_tau2

        # derive Energy functions and equations of motions
        self.E = self.T + self.V

        self.L = self.T - self.V
        self.L[0,0] = simplify_exp(self.L[0,0], verbose)

        self.eom = ddt(Jacobian(self.L, dq).T) - Jacobian(self.L, q).T - self.Q
        for i in range(3):
            self.eom[i,0] = simplify_exp(self.eom[i,0], verbose)

        self.A = Jacobian(self.eom, ddq)
        A_inv = inv3(self.A)
        for i in range(3):
            for j in range(3):
                A_inv[i,j] = simplify_exp(A_inv[i,j], verbose)

        self.b = self.A.dot(ddq) - self.eom

        for i in range(3):
            self.b[i,0] = simplify_exp(self.b[i,0], verbose)

        self.acc = A_inv.dot(self.b)

        # check if accelerations are valid.
        for i in range(3):
            if self.ddth1 in self.acc[i][0].GetVariables():
                raise Exception('Invalid dynamics. Contain accelerations.')

        # Get string of expresions for autodiff evaluations
        # need to do string substitutions to reduce number of operations
        self.acc_string = [
            self.acc[0][0].to_string(),
            self.acc[1][0].to_string(),
            self.acc[2][0].to_string(),
        ]
    
    def EvalExpression(
        self,
        exp: Expression,
        qv: np.ndarray,
        tau1: OptimizationType = 0,
        tau2: OptimizationType = 0.0) -> OptimizationType:
        """Evaluates an Expression for the the Arm System.

        Args:
            exp: The expression to evaluate.
            qv: The joint angles and velocities as OptimizationType.
            tau1: The torque applied to the first joint.
            tau2: The torque applied to the second joint.

        Returns:
            The evaluated expressions.
        """
        s = ArmState(qv)

        if type(s.th1) is Expression:
            th1 = list(s.th1.GetVariables())[0]
            th2 = list(s.th2.GetVariables())[0]
            th3 = list(s.th3.GetVariables())[0]
            dth1 = list(s.dth1.GetVariables())[0]
            dth2 = list(s.dth2.GetVariables())[0]

            return exp.Substitute(
                {
                    self.th1: th1,
                    self.th2: th2,
                    self.th3: th3,
                    self.dth1: dth1,
                    self.dth2: dth2,
                    self.dth3: dth3,
                    self.tau1: tau1,
                    self.tau2: tau2,
                }
            )

        elif type(s.th1) in [float, np.float64]:
            return exp.Evaluate(
                {
                    self.th1: s.th1,
                    self.th2: s.th2,
                    self.th3: s.th3,
                    self.dth1: s.dth1,
                    self.dth2: s.dth2,
                    self.dth3: s.dth3,
                    self.tau1: tau1,
                    self.tau2: tau2,
                }
            )

        else:
            # Using eval to evaluate autodiff arithmetics
            th1 = s.th1
            th2 = s.th2
            th3 = s.th3
            dth1 = s.dth1
            dth2 = s.dth2
            dth3 = s.dth3
            return eval(exp.to_string())

    def EvalKeypoint(self, keypoint: np.ndarray, qv: np.ndarray) -> np.ndarray:
        """Evaluates a Keypoint for a given State.

        A keypoint can be a position or velocity vector.

        Args:
            keypoint: A position or velocity vector with shape 3x1.
            qv: The joint angles and velocities as OptimizationType.
        Returns:
            An evaluated vector of shape 3x1.
        """
        keypoint_eval = np.zeros(keypoint.shape)
        for i in range(keypoint.shape[0]):
            keypoint_eval[i][0] = self.EvalExpression(keypoint[i][0], qv)

        return keypoint_eval

    def EvalCost(self, qv: np.ndarray) -> OptimizationType:
        """Evaluates the objective cost given a State.

        Here the objective is to maximize the endeffector speed.
        So if the endeffector speed is V_E, the objective follows as:

            cost = - sqrt(V_E.T.dot(V_E))

        Args:
            qv: The joint angles and velocities as OptimizationType.

        Returns:
            The evaluated cost.
        """
        s = ArmState(qv)
        if type(s.th1) in [Expression, float, np.float64]:
            return self.EvalExpression(self.neg_speed, qv)
        else:
            # It's faster to precompute the string of the expression.
            th1 = s.th1
            th2 = s.th2
            th3 = s.th3
            dth1 = s.dth1
            dth2 = s.dth2
            dth3 = s.dth3
            neg_speed = eval(self.neg_speed_string)
        return neg_speed

    def EvalDerivatives(
        self,
        qv: np.ndarray,
        tau1: OptimizationType,
        tau2: OptimizationType,
        fast_build: bool = True) -> np.ndarray:
        """Evaluates the derivatives of the System.

        Args:
            qv: The joint angles and velocities as OptimizationType.
            tau1: The torque applied to the first joint.
            tau2: The torque applied to the second joint.
            fast_build: If true, sets the derivatives to 0 s.t. that diagram.build() terminates faster.

        Returns:
            The evaluated derivatives.
        """
        s = ArmState(qv)
        dqv = [0]*6

        if type(s.th1) in [Expression, float, np.float64]:
            for i in range(3):
                if fast_build:
                    return dqv
                else:
                    dqv[3 + i] = self.EvalExpression(self.acc[i, 0], tau1, tau2)
            dqv[:3] = s[3:]
        else:
            # It's faster to precompute the string of the expression.
            th1 = s.th1
            th2 = s.th2
            th3 = s.th3
            dth1 = s.dth1
            dth2 = s.dth2
            dth3 = s.dth3

            for i in range(3):
                dqv[3 + i] = eval(self.acc_string[i])

            dqv[:3] = s[3:]
        return dqv[:]


@TemplateSystem.define("ArmPlant_")
def ArmPlant_(T):
    class Impl(LeafSystem_[T]):
        def _construct(self, dynamics, converter=None):
            LeafSystem_[T].__init__(self, converter)
            # two inputs (motor torques)
            self.u_port = self.DeclareVectorInputPort("joint_torques", 2)
            # 6 states
            self.DeclareContinuousState(6)
            # full state output
            self.DeclareVectorOutputPort("state", 6, self.CopyStateOut)

            self.dynamics = dynamics

            self.PositionUpperLimit = [3.14, 1.57, 1.57]
            self.PositionLowerLimit = [-3.14, -1.57, -1.57]

        def _construct_copy(self, other, converter=None):
            Impl._construct(self, other.dynamics, converter=converter)

        def DoCalcTimeDerivatives(self, context, derivatives):
            s = ArmState(context.get_mutable_continuous_state_vector().CopyToVector())
            u = self.u_port.Eval(context)

            dqv = self.dynamics.EvalDerivatives(s[:], u[0], u[1])
            derivatives.get_mutable_vector().SetFromVector(dqv)

        def CopyStateOut(self, context, output):
            x = context.get_continuous_state_vector().CopyToVector()
            output.SetFromVector(x)

    return Impl


class Arm2DVisualizer(PyPlotVisualizer):
    def __init__(self, dynamics, ax=None, show=None):
        PyPlotVisualizer.__init__(self, ax=ax, show=show)
        self.DeclareInputPort("state", PortDataType.kVectorValued, 6)
        self.arm_dynamics = dynamics

        self.max_length = (
            self.arm_dynamics.l1 + self.arm_dynamics.l2 + self.arm_dynamics.l3
        )
        self.ax.set_aspect("equal")
        self.ax.set_xlim(-self.max_length, self.max_length)
        self.ax.set_ylim(-self.max_length, self.max_length)

    def plot(self, v1, v2, color="black"):
        x, y = np.array(list(zip(v1, v2)))
        self.ax.plot(x, y, linewidth=3.0, color=color)

    def draw(self, context):
        q = self.EvalVectorInput(context, 0).CopyToVector()
        origin = np.zeros((2, 1))
        self.ax.clear()

        box = np.vstack(
            (
                self.max_length * np.array([1, -1, -1, 1, 1]),
                self.max_length * np.array([1, 1, -1, -1, 1]),
            )
        )
        self.ax.plot(box[0, :], box[1, :], alpha=0)  # keeps a constant plotting frame.
        rA = self.arm_dynamics.EvalKeypoint(self.arm_dynamics.rA, q)[:2]
        rB = self.arm_dynamics.EvalKeypoint(self.arm_dynamics.rB, q)[:2]
        rC1 = self.arm_dynamics.EvalKeypoint(self.arm_dynamics.rC1, q)[:2]
        rC2 = self.arm_dynamics.EvalKeypoint(self.arm_dynamics.rC2, q)[:2]
        rC3 = self.arm_dynamics.EvalKeypoint(self.arm_dynamics.rC3, q)[:2]
        rS0 = self.arm_dynamics.EvalKeypoint(self.arm_dynamics.rS0, q)[:2]
        rS1 = self.arm_dynamics.EvalKeypoint(self.arm_dynamics.rS1, q)[:2]
        rS2 = self.arm_dynamics.EvalKeypoint(self.arm_dynamics.rS2, q)[:2]
        rT0 = self.arm_dynamics.EvalKeypoint(self.arm_dynamics.rT0, q)[:2]
        rT1 = self.arm_dynamics.EvalKeypoint(self.arm_dynamics.rT1, q)[:2]
        rT2 = self.arm_dynamics.EvalKeypoint(self.arm_dynamics.rT2, q)[:2]
        rE = self.arm_dynamics.EvalKeypoint(self.arm_dynamics.rE, q)[:2]

        # plot 4 bar
        self.plot(origin, rC1, "blue")
        self.plot(rC1, rC2, "blue")
        self.plot(rC2, rC3, "blue")

        # plot springs
        self.plot(rT1, rS1, "orange")
        self.plot(rT2, rS2, "orange")
        # plot spring holder
        self.plot(rT0, rT1, "red")
        self.plot(rT0, rT2, "red")
        self.plot(rS0, rS1, "red")
        self.plot(rS0, rS2, "red")

        # plot main arm
        self.plot(origin, rA)
        self.plot(rA, rB)
        self.plot(rB, rE)
        self.ax.set_title("t = {:.3f}".format(context.get_time()))
        self.ax.set_xticks([], [])
        self.ax.set_yticks([], [])
