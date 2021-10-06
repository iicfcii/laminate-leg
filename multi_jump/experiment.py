import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),'../')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),'../nat_net_sdk')))

import time
from NatNetClient import NatNetClient

def receive_mocap_data(mocap_data):
    print('Frame number: {}, Timestamp {}'.format(
        mocap_data.prefix_data.frame_number,
        mocap_data.suffix_data.timestamp
    ))

    for rb in mocap_data.rigid_body_data.rigid_body_list:
        print('Rigid body ID: {}, Position: {}, Rotation: {}'.format(
            rb.id_num, rb.pos, rb.rot
        ))

    for lm in mocap_data.labeled_marker_data.labeled_marker_list:
        print('Marker ID: {} Position: {}'.format(
            lm.id_num,
            lm.pos
        ))
    pass


CLIENT_ADDR = "192.168.0.119"
SERVER_ADDR = "192.168.0.161"

streaming_client = NatNetClient()
streaming_client.set_client_address(CLIENT_ADDR)
streaming_client.set_server_address(SERVER_ADDR)
streaming_client.set_use_multicast(True)
streaming_client.set_print_level(0)
streaming_client.mocap_data_listener = receive_mocap_data
assert streaming_client.run(), 'Cannot start client.'
time.sleep(1)
assert streaming_client.connected(), 'Cannot connect.'
