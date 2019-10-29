import os, sys
lib_path = os.path.abspath(os.path.join('..'))
sys.path.append(lib_path)

from common.input import *

# Unit: J
E_Tx = 50*1e-9
E_Rx = 50*1e-9
e_fs = 10*1e-12
e_da = 10*1e-12
e_mp = 0.0

# Num of bits
k_bit = 4000

def get_relay_list(individual):
    relay_list = []
    ll = len(individual)
    for i in range(ll):
        if individual[i] == 1:
            relay_list.append(i)
    return relay_list

def generate_connect_matrix(inp: WusnInput, individual):
    "Check if there is a sensor can't connect to any relay and whether sensor can connect to relay or not at the same time"
    connect = {}
    for sn in inp.sensors:
        count = 0
        for i in range (0, len(individual)):
            if individual[i] == 1:
                if distance(sn, inp.relays[i]) <= 2*inp.radius:
                    connect[(sn, inp.relays[i])] = 1
                    count += 1
        if count == 0:
            return False
    return connect

def heuristic(inp: WusnInput, individual) -> WusnOutput:
    connect = generate_connect_matrix(inp, individual)

    if connect == False:
        return WusnOutput(inp, {})

    relay_list = get_relay_list(individual)
    num_sensors_to_relay = [0] * len(individual)
    static_relay_loss = inp.static_relay_loss
    dynamic_relay_loss = inp.dynamic_relay_loss
    sensor_loss = inp.sensor_loss
    # selected_relay_list = []

    num_sensors = inp.num_of_sensors

    config = {}

# Xet tung sensor 
# Voi moi sensor, tim ra gia tri nho nhat giua 2 gia tri: tieu hao cua sensor va tieu hao cua relay (local_max)
# Tim ra gia tri local_max nho nhat (min_max)
# Sensor ket noi RN sao nho min_max giam (hoac khong tang len)

    for i in range(inp.num_of_sensors):
        min_max = float("inf")
        selected_id = 0
        for rn_id in relay_list:
            if (inp.sensors[i], inp.relays[rn_id]) in connect:
                loss1 = sensor_loss[(inp.sensors[i], inp.relays[rn_id])]
                loss2 = (num_sensors_to_relay[rn_id] + 1) * dynamic_relay_loss[inp.relays[rn_id]] + static_relay_loss[inp.relays[rn_id]]
                local_max = max(loss1, loss2)
                if local_max < min_max:
                    min_max = local_max
                    selected_id = rn_id
        config[inp.sensors[i]] = inp.relays[selected_id]
        num_sensors_to_relay[selected_id] += 1
        # if selected_id not in selected_relay_list:
        #     selected_relay_list.append(selected_id)

    # return config, value, len(selected_relay_list)
    return WusnOutput(inp, config)