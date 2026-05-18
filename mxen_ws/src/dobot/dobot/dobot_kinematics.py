import math

L0 = 56.0
L1 = 82.0
L2 = 135.0
L3 = 147.0
L4 = 60.0
L5 = 70.0


def forward_kinematics(theta1, theta2, theta3, theta4):
    x = (3 * math.cos(theta1)) * ((49 * math.cos(theta2 + theta3)) + (45 * math.sin(theta2)) + 20)
    y = (3 * math.sin(theta1)) * ((49 * math.cos(theta2 + theta3)) + (45 * math.sin(theta2)) + 20)
    z = (135 * math.cos(theta2)) - ((147 * math.sin(theta2 + theta3)) + 68 )
    r = theta1 + theta4
    return x, y, z, r


def inverse_kinematics(x, y, z, r):
    d = math.sqrt(x**2 + y**2) - L4
    h = z + L5 - L0 - L1
    c = math.sqrt(d**2 + h**2)
    alpha = math.atan(d/h)

    theta1 = math.atan(y/x)
    theta2 = alpha - math.acos((L2**2 + c**2 - L3**2) / (2 * L2 * c))
    theta3 = math.asin((L2**2 + L3**2 - c**2) / (2 * L2 * L3))
    theta4 = r - theta1

    return theta1, theta2, theta3, theta4
