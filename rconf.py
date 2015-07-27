#!/usr/bin/python

cuarc_config = {
    'controllers': {
        'cuarc_controller': {
            'sync_read': False,
            'attached_motors': ['left_leg', 'right_leg'],
            'port': '/dev/ttyACM0'
        }
    },
    'motorgroups': {
        'left_leg': ['l_hip', 'l_knee'],
        'right_leg': ['r_hip', 'r_knee'],
        'left_arm': ['l_shoulder', 'l_elbow'],
        'right_arm': ['r_shoulder', 'r_elbow']
    },
    'motors': {
        'l_hip': {
            'orientation': 'direct',
            'type': 'AX-12A',
            'id': 4,
            'angle_limit': [-40.0, 30.0],
            'offset': 0.0
        },
        'l_knee': {
            'orientation': 'direct',
            'type': 'AX-12A',
            'id': 5,
            'angle_limit': [-150.0, -2.0],
            'offset': 0.0
        },
        'r_hip': {
            'orientation': 'direct',
            'type': 'AX-12A',
            'id': 3,
            'angle_limit': [-150.0, -60.0],
            'offset': 0.0
        },
        'r_knee': {
            'orientation': 'direct',
            'type': 'AX-12A',
            'id': 2,
            'angle_limit': [18.0, 118.0],
            'offset': 0.0
        },
        'l_shoulder': {
            'orientation': 'direct',
            'type': 'AX-12A',
            'id': 6,
            'angle_limit': [-150.0, 150.0],
            'offset': 0.0
        },
        'l_elbow': {
            'orientation': 'direct',
            'type': 'AX-12A',
            'id': 7,
            'angle_limit': [-150.0, 150.0],
            'offset': 0.0
        },
        'r_shoulder': {
            'orientation': 'direct',
            'type': 'AX-12A',
            'id': 8,
            'angle_limit': [-150.0, 150.0],
            'offset': 0.0
        },
        'r_elbow': {
            'orientation': 'direct',
            'type': 'AX-12A',
            'id': 9,
            'angle_limit': [-150.0, 150.0],
            'offset': 0.0
        }
    }
}

arm_config = {}
arm_config['controllers'] = {}

arm_config['controllers']['arm_controller'] = {
    'port': '/dev/ttyACM0',
    'sync_read': False,
    'attached_motors': ['m1', 'm2', 'm3'],
    'protocol': 1,
}

arm_config['motorgroups'] = {
    'arm': ['m1', 'm2', 'm3']
}

arm_config['motors'] = {}
arm_config['motors']['m3'] = {
    'id': 2,
    'type': 'AX-12A',
    'orientation': 'direct',
    'offset': 0.0,
    'angle_limit': (-99, 99)
}

arm_config['motors']['m2'] = {
    'id': 3,
    'type': 'AX-12A',
    'orientation': 'direct',
    'offset': 0.0,
    'angle_limit': (-99, 99)
}

arm_config['motors']['m1'] = {
    'id': 5,
    'type': 'AX-12A',
    'orientation': 'direct',
    'offset': 0.0,
    'angle_limit': (-90, 90)
}
