import numpy as np
from scipy.spatial.transform import Rotation as R
from Lab1_FK_answers import *
import torch


def CCD(meta_data, joint_positions, joint_orientations, target_pose): # TODO finish CCD
    path, path_name, path1, path2 = meta_data.get_path_from_root_to_end()
    path_positions = []
    for joint in path:
        path_positions.append(joint_positions[joint])
    path_offsets = [np.array([0, 0, 0])]
    for i in range(len(path) - 1):
        path_offsets.append(meta_data.joint_initial_position[path[i + 1]] - meta_data.joint_initial_position[path[i]])
    path_orientations = []
    for i in range(len(path2) - 1):
        path_orientations.append(R.from_quat(joint_orientations[path2[i + 1]]))
    path_orientations.append(R.from_quat(joint_orientations[path2[-1]]))
    for i in range(len(path1) - 1):
        path_orientations.append(R.from_quat(joint_orientations[path1[-i]]))
    path_orientations.append(R.identity())

    cnt = 0
    end_index = path_name.index(meta_data.end_joint)
    while np.linalg.norm(joint_positions[path[end_index]]) >= 1e-2 and cnt <= 10:
        for i in range(end_index):
            current_index = end_index - 1 - i
            current_position = path_positions[current_index]
            end_position = path_positions[end_index]
            current2target = target_pose - current_position
            current2end = end_position - current_position
            # norm
            current2target = current2target / np.linalg.norm(current2target)
            current2end = current2end / np.linalg.norm(current2end)

            # calculate axis-angle form
            rotation_axis = np.cross(current2end, current2target)
            #norm
            rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
            rotation_radius = np.arccos(np.dot(current2end, current2target) / (np.linalg.norm(current2target) * \
                                                                               np.linalg.norm(current2end)))
            rotation_vector = R.from_rotvec(rotation_radius * rotation_axis)

            # calculate positions and orientations
            path_orientations[current_index] = rotation_vector * path_orientations[current_index]
            path_rotations = [path_orientations[0]]

            for j in range(len(path_orientations) - 1):
                path_rotations.append(R.inv(path_orientations[j]) * path_orientations[j + 1])

            for j in range(current_index, end_index):
                path_positions[j + 1] = path_positions[j] + path_orientations[j].apply(path_offsets[j + 1])
                if j + 1 < end_index:
                    path_orientations[j + 1] = path_orientations[j] * path_rotations[j + 1]
        cnt += 1

    return path_positions, path_orientations


def forward_kinematics(meta_data, joint_positions, joint_orientations, current_pos):
    path, path_name, path1, path2 = meta_data.get_path_from_root_to_end()
    rotation_matrix = torch.randn((len(path), 4), requires_grad=True)# in a quat form
    pass


def part1_inverse_kinematics(meta_data, joint_positions, joint_orientations, target_pose):
    """
    ?????????????????????????????????
    ??????:
        meta_data: ??????????????????????????????????????????????????????????????????meta_data???
        joint_positions: ?????????????????????????????????numpy?????????shape???(M, 3)???M????????????
        joint_orientations: ?????????????????????????????????numpy?????????shape???(M, 4)???M????????????
        target_pose: ????????????????????????numpy?????????shape???(3,)
    ??????:
        ??????IK????????????
        joint_positions: ???????????????????????????????????????numpy?????????shape???(M, 3)???M????????????
        joint_orientations: ???????????????????????????????????????numpy?????????shape???(M, 4)???M????????????
    """
    path_positions, path_orientations = CCD(meta_data, joint_positions, joint_orientations, target_pose)
    path, path_name, path1, path2 = meta_data.get_path_from_root_to_end()

    # calculate the rotation of cuurent joints
    joint_rotations = R.identity(len(meta_data.joint_name))
    for i in range(len(meta_data.joint_parent)):
        if meta_data.joint_parent[i] == -1:
            joint_rotations[i] = R.from_quat(joint_orientations[i])
        else:
            joint_rotations[i] = R.inv(R.from_quat(joint_orientations[meta_data.joint_parent[i]])) * R.from_quat(
                joint_orientations[i])

    # the forward kinematics of the path joints
    for i in range(len(path2) - 1):
        joint_orientations[path2[i + 1]] = path_orientations[i].as_quat()
    joint_orientations[path2[-1]] = path_orientations[len(path2) - 1].as_quat()
    for i in range(len(path1) - 1):
        joint_orientations[path1[-i]] = path_orientations[i + len(path2)].as_quat()

    for i in range(len(path)):
        joint_positions[path[i]] = path_positions[i]

    # the forward kinematics of other joints
    for i in range(len(meta_data.joint_parent)):
        if meta_data.joint_parent[i] == -1:
            continue
        if meta_data.joint_name[i] not in path_name:
            joint_positions[i] = joint_positions[meta_data.joint_parent[i]] + \
                R.from_quat(joint_orientations[meta_data.joint_parent[i]]).apply(meta_data.joint_initial_position[i] - \
                meta_data.joint_initial_position[meta_data.joint_parent[i]])
            joint_orientations[i] = (R.from_quat(joint_orientations[meta_data.joint_parent[i]]) * joint_rotations[i]).as_quat()

    return joint_positions, joint_orientations


def part2_inverse_kinematics(meta_data, joint_positions, joint_orientations, relative_x, relative_z, target_height):
    """
    ??????lWrist?????????RootJoint???????????????xz??????????????????????????????IK??????????????????bvh??????
    """
    target_pose = np.array([joint_positions[0][0] + relative_x, target_height, joint_positions[0][2] + relative_z])
    path_positions, path_orientations = CCD(meta_data, joint_positions, joint_orientations, target_pose)
    path, path_name, _, _ = meta_data.get_path_from_root_to_end()

    joint_rotations = R.identity(len(meta_data.joint_parent))
    for i in range(len(meta_data.joint_parent)):
        if meta_data.joint_parent[i] == -1:
            joint_rotations[i] = R.from_quat(joint_orientations[i])
        else:
            joint_rotations[i] = R.inv(R.from_quat(joint_orientations[meta_data.joint_parent[i]])) * R.from_quat(joint_orientations[i])

    # calculate the rotation and position of path joints
    for i in range(len(path)):
        joint_positions[path[i]] = path_positions[i]
        joint_orientations[path[i]] = path_orientations[i].as_quat()

    #joint_orientations[path[-1]] = (R.from_quat(joint_orientations[path[-1]]) * R.from_euler('XYZ', [-90, 0, 0], degrees=True)).as_quat()
    # calculate the rotation and position of other joints
    for i in range(len(meta_data.joint_parent)):
        if meta_data.joint_parent[i] == -1:
            continue
        if meta_data.joint_name[i] not in path_name:
            joint_orientations[i] = (R.from_quat(joint_orientations[meta_data.joint_parent[i]]) * joint_rotations[i]).as_quat()
            joint_positions[i] = joint_positions[meta_data.joint_parent[i]] + \
                R.from_quat(joint_orientations[meta_data.joint_parent[i]]).apply(meta_data.joint_initial_position[i] - \
                                                            meta_data.joint_initial_position[meta_data.joint_parent[i]])


    return joint_positions, joint_orientations


def bonus_inverse_kinematics(meta_data, joint_positions, joint_orientations, left_target_pose, right_target_pose):
    """
    ???????????????????????????????????????????????????????????????????????????????????????
    """

    return joint_positions, joint_orientations
