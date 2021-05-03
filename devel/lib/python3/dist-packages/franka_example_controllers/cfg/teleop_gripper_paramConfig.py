## *********************************************************
##
## File autogenerated for the franka_example_controllers package
## by the dynamic_reconfigure package.
## Please do not edit.
##
## ********************************************************/

from dynamic_reconfigure.encoding import extract_params

inf = float('inf')

config_description = {'name': 'Default', 'type': '', 'state': True, 'cstate': 'true', 'id': 0, 'parent': 0, 'parameters': [], 'groups': [{'name': 'GripperParameter', 'type': 'apply', 'state': True, 'cstate': 'true', 'id': 1, 'parent': 0, 'parameters': [{'name': 'grasp_force', 'type': 'double', 'default': 40.0, 'level': 0, 'description': 'Grasping force to be applied on an object. [N]', 'min': 1.0, 'max': 60.0, 'srcline': 10, 'srcfile': '/home/terry/catkin_ws/src/franka_ros/franka_example_controllers/cfg/teleop_gripper_param.cfg', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}, {'name': 'move_speed', 'type': 'double', 'default': 0.3, 'level': 0, 'description': 'Speed of the follower gripper when opening [m/s]', 'min': 0.01, 'max': 0.4, 'srcline': 13, 'srcfile': '/home/terry/catkin_ws/src/franka_ros/franka_example_controllers/cfg/teleop_gripper_param.cfg', 'edit_method': '', 'ctype': 'double', 'cconsttype': 'const double'}], 'groups': [], 'srcline': 124, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'class': 'DEFAULT::GRIPPERPARAMETER', 'parentclass': 'DEFAULT', 'parentname': 'Default', 'field': 'DEFAULT::gripperparameter', 'upper': 'GRIPPERPARAMETER', 'lower': 'gripperparameter'}], 'srcline': 246, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'class': 'DEFAULT', 'parentclass': '', 'parentname': 'Default', 'field': 'default', 'upper': 'DEFAULT', 'lower': 'groups'}

min = {}
max = {}
defaults = {}
level = {}
type = {}
all_level = 0

#def extract_params(config):
#    params = []
#    params.extend(config['parameters'])
#    for group in config['groups']:
#        params.extend(extract_params(group))
#    return params

for param in extract_params(config_description):
    min[param['name']] = param['min']
    max[param['name']] = param['max']
    defaults[param['name']] = param['default']
    level[param['name']] = param['level']
    type[param['name']] = param['type']
    all_level = all_level | param['level']

