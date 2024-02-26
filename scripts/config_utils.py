import yaml
from math import pi

def get_package_config (filename):
    '''
    Gets the task requirement information

    Args:
        filename (str): configuration filename. Usually `config.yaml`, provided by eYRC
    Returns:
        rack info dictionary. Refer rack_pose_info assignment under __name__ == "__main__" in task2b.py for details
    '''

    config_f = open (filename)
    config = yaml.safe_load(config_f)
    config_f.close ()

    # config['position'] is a list of dicts, each of which has a single key
    # A single dict with all the keys is more understandable
    config ['positions'] = {'arm': [1.6, -2.45]}
    for p in config ['position']:
        config ['positions'].update (p)

    arm_pos = config ['positions']['arm']
    box_ids = config ['package_id']
    rack_info = []
    for _id in box_ids:
        rack = f'rack{_id}'
        pos = config ['positions'][rack]
        rack_info += [{
            'pickup': { 'trans': pos [0:2], 'rot': pos[2] },
            'rack': rack,
            'box': _id,
        }]
    rack_info.sort (key = lambda rack:
        (rack['pickup']['trans'][0] - arm_pos[0])**2 + (rack['pickup']['trans'][1] - arm_pos[1])**2
    )

    approach_angles = {1: [0.0, 0.8], 2: [-0.8, 0.0], 3: [0.0, -0.8]} # n: [x, y] => approach at n*pi/2, reach offset (x,y) from arm
    for rack in rack_info:
        x, y = rack['pickup']['trans']
        x -= arm_pos[0]; y -= arm_pos[1]

        # ap is angle at which rack approaches arm, represented as a multiple of pi/2
        ap = abs(y) > abs(x)  # points along +y if y component is greater
        if x + y < 0: ap += 2 # reversed if the larger component is less than 0

        if ap not in approach_angles:
            drop = None
        else:
            drop = (ap, approach_angles.pop (ap)) # Approach angle, final position
        rack['drop'] = drop


    for rack in rack_info:
        ap, pos = rack['drop'] or approach_angles.popitem ()
        pos[0] += arm_pos[0]; pos[1] += arm_pos[1]
        rack['drop'] = {'trans': pos, 'rot': ap * pi / 2}

    return rack_info


if __name__ == "__main__":
    for i in get_package_config ("example_config.yaml"):
        print (i)
