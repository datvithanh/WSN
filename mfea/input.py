class Input:
    def __init__(self, sensors=[], positions=[], num_relays=0):
        self.sensors = sensors
        self.positions = positions
        self.num_relays = num_relays

        assert len(self.sensors) == len(set(self.sensors))
        assert len(self.positions) == len(set(self.positions))
