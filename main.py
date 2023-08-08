import logging

import URBasic

from Model.joint_position_model import JointPositionModel
from Model.move_j_control_model import MoveJControlModel
from Model.move_l_control_model import MoveLControlModel
from Model.tcp_position_model import TcpPositionModel

host = '127.0.0.1'


def MoveCommand():
    mj_1_joint_position_model = JointPositionModel()
    mj_1_joint_position_model.base = -90.0
    mj_1_joint_position_model.shoulder = 0.0
    mj_1_joint_position_model.elbow = 0.0
    mj_1_joint_position_model.wrist1 = 0.0
    mj_1_joint_position_model.wrist2 = 0.0
    mj_1_joint_position_model.wrist3 = 0.0

    mj_2_joint_position_model = JointPositionModel()
    mj_2_joint_position_model.base = -180.0
    mj_2_joint_position_model.shoulder = 0.0
    mj_2_joint_position_model.elbow = 0.0
    mj_2_joint_position_model.wrist1 = 0.0
    mj_2_joint_position_model.wrist2 = 0.0
    mj_2_joint_position_model.wrist3 = 0.0

    move_j_control_model = MoveJControlModel()
    move_j_control_model.acceleration = 0.5
    move_j_control_model.velocity = 0.5
    move_j_control_model.time_s = 0.0
    move_j_control_model.blend_radius = 0.0
    move_j_control_model.joint_position_model_array.append(mj_1_joint_position_model)
    move_j_control_model.joint_position_model_array.append(mj_2_joint_position_model)

    ml_1_tcp_position_model = TcpPositionModel()
    ml_1_tcp_position_model.x = 0.3
    ml_1_tcp_position_model.y = 0.3
    ml_1_tcp_position_model.z = 0.3
    ml_1_tcp_position_model.rx = 0.0
    ml_1_tcp_position_model.ry = 3.14
    ml_1_tcp_position_model.rz = 0.0

    move_l_control_model = MoveLControlModel()
    move_l_control_model.acceleration = 0.5
    move_l_control_model.velocity = 0.5
    move_l_control_model.time_s = 0.0
    move_l_control_model.blend_radius = 0.0
    move_l_control_model.tcp_position_model_array.append(ml_1_tcp_position_model)

    execute_move_j_command(move_j_control_model=move_j_control_model)
    execute_move_l_command(move_l_control_model=move_l_control_model)


def execute_move_j_command(move_j_control_model):
    robotModel = URBasic.robotModel.RobotModel()
    ur_script_ext = URBasic.urScriptExt.UrScriptExt(host=host, robotModel=robotModel)
    ur_script_ext.reset_error()
    for index, joint_position_model in enumerate(move_j_control_model.joint_position_model_array):
        joint_position_array = JointPositionModel.get_position_array_from_joint_position_model(
            joint_position_model=joint_position_model)
        logging.info('execute_move_j_command {index}'.format(index=index))
        ur_script_ext.movej(q=joint_position_array,
                            a=move_j_control_model.acceleration,
                            v=move_j_control_model.velocity,
                            t=move_j_control_model.time_s,
                            r=move_j_control_model.blend_radius)
    ur_script_ext.close()


def execute_move_l_command(move_l_control_model):
    robotModel = URBasic.robotModel.RobotModel()
    ur_script_ext = URBasic.urScriptExt.UrScriptExt(host=host, robotModel=robotModel)
    ur_script_ext.reset_error()
    for index, tcp_position_model in enumerate(move_l_control_model.tcp_position_model_array):
        tcp_position_array = TcpPositionModel.get_position_array_from_tcp_position_model(
            tcp_position_model=tcp_position_model)
        logging.info('execute_move_l_command {index}'.format(index=index))
        ur_script_ext.movej(q=tcp_position_array,
                            a=move_l_control_model.acceleration,
                            v=move_l_control_model.velocity,
                            t=move_l_control_model.time_s,
                            r=move_l_control_model.blend_radius)
    ur_script_ext.close()


if __name__ == '__main__':
    MoveCommand()
