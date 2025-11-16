import sympy as sp

# Joint angles
q1, q2, q3, q4, q5, q6 = sp.symbols("q1:7")

# Modified DH parameters
alpha = [0, -sp.pi / 2, 0, 0, sp.pi / 2, sp.pi / 2]
a = [0, 0, 0.185, 0.17, 0, 0]
d = [0.23, -0.054, 0, 0.077, 0.077, 0.0855]
theta = [q1, q2 - sp.pi / 2, q3, q4 + sp.pi / 2, q5 + sp.pi / 2, q6]


# Transform matrices
def get_transform(alpha, a, d, theta):
    return sp.simplify(
        sp.Matrix(
            [
                [sp.cos(theta), -sp.sin(theta), 0, a],
                [
                    sp.sin(theta) * sp.cos(alpha),
                    sp.cos(theta) * sp.cos(alpha),
                    -sp.sin(alpha),
                    -sp.sin(alpha) * d,
                ],
                [
                    sp.sin(theta) * sp.sin(alpha),
                    sp.cos(theta) * sp.sin(alpha),
                    sp.cos(alpha),
                    sp.cos(alpha) * d,
                ],
                [0, 0, 0, 1],
            ]
        )
    )


T01 = get_transform(alpha[0], a[0], d[0], theta[0])
T12 = get_transform(alpha[1], a[1], d[1], theta[1])
T23 = get_transform(alpha[2], a[2], d[2], theta[2])
T34 = get_transform(alpha[3], a[3], d[3], theta[3])
T45 = get_transform(alpha[4], a[4], d[4], theta[4])
T56 = get_transform(alpha[5], a[5], d[5], theta[5])

T02 = sp.simplify(T01 * T12)
T03 = sp.simplify(T02 * T23)
T04 = sp.simplify(T03 * T34)
T05 = sp.simplify(T04 * T45)
T06 = sp.simplify(T05 * T56)

# Jacobian matrix
J = sp.Matrix(
    [
        # Row 1 (末端执行器线速度 Vx)
        [
            -0.185 * sin(q1) * sin(q2)
            - 0.17 * sin(q1) * sin(q2 + q3)
            - 0.077 * sin(q1) * sin(q2 + q3 + q4)
            - 0.0855 * sin(q1) * cos(q5) * cos(q2 + q3 + q4)
            - 0.0855 * sin(q5) * cos(q1)
            - 0.023 * cos(q1),
            (-0.0855 * sin(q2 + q3 + q4) * cos(q5) + 0.185 * cos(q2) + 0.17 * cos(q2 + q3) + 0.077 * cos(q2 + q3 + q4))
            * cos(q1),
            (-0.0855 * sin(q2 + q3 + q4) * cos(q5) + 0.185 * cos(q2) + 0.17 * cos(q2 + q3) + 0.077 * cos(q2 + q3 + q4))
            * cos(q1),
            (-0.0855 * sin(q2 + q3 + q4) * cos(q5) + 0.17 * cos(q2 + q3) + 0.077 * cos(q2 + q3 + q4)) * cos(q1),
            -0.0855 * sin(q1) * cos(q5) - 0.0855 * sin(q5) * cos(q1) * cos(q2 + q3 + q4),
            0,
        ],
        # Row 2 (末端执行器线速度 Vy)
        [
            -0.0855 * sin(q1) * sin(q5)
            - 0.023 * sin(q1)
            + 0.185 * sin(q2) * cos(q1)
            + 0.17 * sin(q2 + q3) * cos(q1)
            + 0.077 * sin(q2 + q3 + q4) * cos(q1)
            + 0.0855 * cos(q1) * cos(q5) * cos(q2 + q3 + q4),
            (-0.0855 * sin(q2 + q3 + q4) * cos(q5) + 0.185 * cos(q2) + 0.17 * cos(q2 + q3) + 0.077 * cos(q2 + q3 + q4))
            * sin(q1),
            (-0.0855 * sin(q2 + q3 + q4) * cos(q5) + 0.185 * cos(q2) + 0.17 * cos(q2 + q3) + 0.077 * cos(q2 + q3 + q4))
            * sin(q1),
            (-0.0855 * sin(q2 + q3 + q4) * cos(q5) + 0.17 * cos(q2 + q3) + 0.077 * cos(q2 + q3 + q4)) * sin(q1),
            -0.0855 * sin(q1) * sin(q5) * cos(q2 + q3 + q4) + 0.0855 * cos(q1) * cos(q5),
            0,
        ],
        # Row 3 (末端执行器线速度 Vz)
        [
            0,
            -0.185 * sin(q2) - 0.17 * sin(q2 + q3) - 0.077 * sin(q2 + q3 + q4) - 0.0855 * cos(q5) * cos(q2 + q3 + q4),
            -0.185 * sin(q2) - 0.17 * sin(q2 + q3) - 0.077 * sin(q2 + q3 + q4) - 0.0855 * cos(q5) * cos(q2 + q3 + q4),
            -0.17 * sin(q2 + q3) - 0.077 * sin(q2 + q3 + q4) - 0.0855 * cos(q5) * cos(q2 + q3 + q4),
            0.0855 * sin(q5) * sin(q2 + q3 + q4),
            0,
        ],
        # Row 4 (末端执行器角速度 ωx)
        [
            0,
            -sin(q1),
            -sin(q1),
            -sin(q1),
            sin(q2 + q3 + q4) * cos(q1),
            -sin(q1) * sin(q5) + cos(q1) * cos(q5) * cos(q2 + q3 + q4),
        ],
        # Row 5 (末端执行器角速度 ωy)
        [
            0,
            cos(q1),
            cos(q1),
            cos(q1),
            sin(q1) * sin(q2 + q3 + q4),
            sin(q1) * cos(q5) * cos(q2 + q3 + q4) + sin(q5) * cos(q1),
        ],
        # Row 6 (末端执行器角速度 ωz)
        [1, 0, 0, 0, cos(q2 + q3 + q4), -sin(q2 + q3 + q4) * cos(q5)],
    ]
)
