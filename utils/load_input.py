import json
from DS.position import distance
from DS.vertex import Vertex
from utils.init_log import init_log
from collections import defaultdict

class WsnInput:
    def __init__(self, _max_hop=20, _num_of_relay_positions=40, _num_of_relays=20, _num_of_sensors=40,
                 _radius=20., _relays=None, _sensors=None, _all_vertex=None, _bs=None):
        self.max_hop = _max_hop
        self.relays = _relays
        self.sensors = _sensors
        self.num_of_relay_positions = _num_of_relay_positions
        self.num_of_relays = _num_of_relays
        self.num_of_sensors = _num_of_sensors
        self.radius = _radius
        self.BS = _bs
        self.all_vertex = _all_vertex

        # for layered graph
        self.static_relay_loss = None
        self.dynamic_relay_loss = None
        self.sensor_loss = None

    @classmethod
    def from_file(cls, path):
        f = open(path)
        d = json.load(f)
        return cls.from_dict(d)

    @classmethod
    def from_dict(cls, d):
        max_hop = d['H']
        num_of_relays = d['num_of_relays']
        num_of_relay_positions = d['num_of_relay_positions']
        num_of_sensors = d['num_of_sensors']
        radius = d['radius']
        relay_positions = []
        sensors = []
        name = 0
        BS = Vertex.from_dict(d['center'], "bs", name)
        name += 1

        for i in range(num_of_relay_positions):
            relay_positions.append(Vertex.from_dict(d['relay_positions'][i], "relay", name))
            name += 1

        for i in range(num_of_sensors):
            sensors.append(Vertex.from_dict(d['sensors'][i], "sensor", name))
            name += 1

        all_vertex = [BS]
        all_vertex.extend(relay_positions)
        all_vertex.extend(sensors)
        for i in all_vertex:
            for j in all_vertex:
                if distance(i, j) <= radius and distance(i, j) != 0:
                    i.add_adjacent_vertex(j)
                    j.add_adjacent_vertex(i)

        return cls(max_hop, num_of_relay_positions, num_of_relays, num_of_sensors, radius, relay_positions, sensors, all_vertex, BS)

    def freeze(self):
        self.sensors = tuple(self.sensors)
        self.relays = tuple(self.relays)

    def to_dict(self):
        return {
            'max_hop': self.max_hop,
            'num_of_relay_positions': self.num_of_relay_positions,
            'num_of_relays': self.num_of_relays,
            'num_of_sensors': self.num_of_sensors,
            'relays': list(map(lambda x: x.to_dict(), self.relays)),
            'sensors': list(map(lambda x: x.to_dict(), self.sensors)),
            'all_vertex': list(map(lambda x: x.to_dict(), self.all_vertex)),
            'center': self.BS.to_dict(),
            'radius': self.radius
        }

    def reset_all_hop(self):
        for v in self.all_vertex:
            v.reset_hop()

    def reset_all_child(self):
        for v in self.all_vertex:
            v.reset_child()

    def to_file(self, file_path):
        d = self.to_dict()
        with open(file_path, "wt") as f:
            fstr = json.dumps(d, indent=4)
            f.write(fstr)
    
    def calculate_loss(self):
        sensor_loss = defaultdict(lambda: float('inf'))
        static_relay_loss = {}
        dynamic_relay_loss = {}
        R = self.radius
        BS = self.BS
        for sn in self.sensors:
            for rn in self.relays:
                if distance(sn, rn) <= 2 * R:
                    sensor_loss[(sn, rn)] = WusnConstants.k_bit * (
                            WusnConstants.e_tx + WusnConstants.e_fs * math.pow(distance(sn, rn), 2))

        for rn in self.relays:
            dynamic_relay_loss[rn] = WusnConstants.k_bit * (WusnConstants.e_rx + WusnConstants.e_da)
            static_relay_loss[rn] = WusnConstants.k_bit * WusnConstants.e_mp * math.pow(distance(rn, BS), 4)

        self.static_relay_loss = static_relay_loss
        self.dynamic_relay_loss = dynamic_relay_loss
        self.sensor_loss = sensor_loss

    def __hash__(self):
        return hash((self.max_hop, self.num_of_relay_positions, self.num_of_relays, self.num_of_sensors, self.radius,
                     tuple(self.relays), tuple(self.sensors)))

    def __eq__(self, other):
        return hash(self) == hash(other)


if __name__ == "__main__":
    inp = WsnInput.from_file('./data/ga-dem1_r25_1.in')
    print(inp.relays[0])