
from math import cos,sin,atan2,sqrt

def pose_trans(pose, rotation_vector):
    # 转换欧拉角为四元数
    rotation_quaternion = euler_to_quaternion(rotation_vector[0], rotation_vector[1], rotation_vector[2])
    
    # 将原始姿态的四元数表示为矩阵形式
    pre_rotation_quaternion = euler_to_quaternion(pose[3], pose[4], pose[5])
    pose_matrix = quaternion_to_matrix(pre_rotation_quaternion[0],pre_rotation_quaternion[1],pre_rotation_quaternion[2],pre_rotation_quaternion[3])
    
    # 创建旋转变换的四元数
    rotation_matrix = quaternion_to_matrix(rotation_quaternion[0], rotation_quaternion[1], rotation_quaternion[2], rotation_quaternion[3])
    
    # 将旋转变换应用于原始姿态的矩阵表示
    new_pose_matrix = pose_mult(pose_matrix, rotation_matrix)
    
    # 将新的矩阵表示转换回欧拉角
    new_pose = matrix_to_pose(new_pose_matrix)
    
    return new_pose

# 欧拉角转换为四元数表示
def euler_to_quaternion(rx, ry, rz):
    qw = cos(rx/2) * cos(ry/2) * cos(rz/2) + sin(rx/2) * sin(ry/2) * sin(rz/2)
    qx = sin(rx/2) * cos(ry/2) * cos(rz/2) - cos(rx/2) * sin(ry/2) * sin(rz/2)
    qy = cos(rx/2) * sin(ry/2) * cos(rz/2) + sin(rx/2) * cos(ry/2) * sin(rz/2)
    qz = cos(rx/2) * cos(ry/2) * sin(rz/2) - sin(rx/2) * sin(ry/2) * cos(rz/2)
    return [qw, qx, qy, qz]

# 四元数转换为矩阵表示
def quaternion_to_matrix(qw, qx, qy, qz):
    return [[1-2*qy**2-2*qz**2, 2*qx*qy-2*qz*qw, 2*qx*qz+2*qy*qw],
            [2*qx*qy+2*qz*qw, 1-2*qx**2-2*qz**2, 2*qy*qz-2*qx*qw],
            [2*qx*qz-2*qy*qw, 2*qy*qz+2*qx*qw, 1-2*qx**2-2*qy**2]]

# 矩阵相乘
def pose_mult(matrix1, matrix2):
    result = []
    for i in range(3):
        row = []
        for j in range(3):
            value = 0
            for k in range(3):
                value += matrix1[i][k] * matrix2[k][j]
            row.append(value)
        result.append(row)
    return result

# 矩阵转换回欧拉角
def matrix_to_pose(matrix):
    rz = atan2(matrix[1][0], matrix[0][0])
    ry = atan2(-matrix[2][0], sqrt(matrix[2][1]**2 + matrix[2][2]**2))
    rx = atan2(matrix[2][1], matrix[2][2])
    return [rx, ry, rz]


print(pose_trans((171.10,-351.95,432.39,1.317,0.727,-1.039),(0,0,1)))