#!/usr/bin/python

openhand_config = {
    'controllers': {
        'hand_controller': {
            'sync_read': False,
            'attached_motors': ['fingers', 'base'],
            'port': '/dev/ttyACM0'
        }
    },
    'motorgroups': {
        'fingers': ['f1', 'f2', 'thumb'],
        'base': ['base_motor']
    },
    'motors': {
        'f1': {
            'orientation': 'direct',
            'type': 'AX-12A',
            'id': 10,
            'angle_limit': [15.0, 79.0],
            'offset': 0.0
        },
        'f2': {
            'orientation': 'direct',
            'type': 'AX-12A',
            'id': 11,
            'angle_limit': [-40.0, 30.0],
            'offset': 0.0
        },
        'base_motor': {
            'orientation': 'direct',
            'type': 'AX-12A',
            'id': 12,
            'angle_limit': [-59.0, 64.0],
            'offset': 0.0
        },
        'thumb': {
            'orientation': 'direct',
            'type': 'AX-12A',
            'id': 13,
            'angle_limit': [-23., 40.0],
            'offset': 0.0
        }
    }
}
