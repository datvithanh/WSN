from DS.edge import Edge
from DS.graph import Graph, check_condition
from DS.vertex import Vertex
from mfea.fitness import get_fitness, get_hop
from copy import deepcopy


class HopConstructor:
    def __init__(self, num_sensors, num_relays, num_positions, list_vertices: Vertex = None):
        if list_vertices is None:
            self._list_vertex = []
        else:
            self._list_vertex = list_vertices
        self.num_sensors = num_sensors
        self.num_relays = num_relays
        self.num_positions = num_positions

    def gen_graph(self, genes: list):
        list_vertices = deepcopy(self._list_vertex)
        if len(genes) != 2 * (self.num_positions + self.num_sensors + 1):
            raise Exception("Error Gen's length is not divisible by 2")
        graph = Graph()
        dict_genes_order = {i: genes[i] for i in range(len(genes) // 2)}
        dict_genes_value = {i - len(genes) // 2: genes[i] for i in range(len(genes) // 2, len(genes))}
        # dict_genes_local_search = {i - 2 * len(genes) // 3: genes[i] for i in range(2 * len(genes) // 3,
        # 3 * len(genes) // 3)}

        position_relay = list(
            dict(sorted({i: genes[i] for i in range(1, self.num_positions + 1)}.items(), key=lambda x: x[1])).keys())[
                         :self.num_relays]
        for i in position_relay:
            edge = Edge(list_vertices[0], list_vertices[i])
            graph.add_edge(edge)

        ignored_position = [i for i in range(1, self.num_positions + 1) if i not in position_relay]

        order = list(dict(sorted(list(dict_genes_order.items())[self.num_positions+1:], key=lambda x: x[1])).keys())

        for i in range(len(order)):
            dict_order_adjacent = dict()
            index_vertex = order[i]
            vertex = list_vertices[index_vertex]
            # print(vertex)
            for j in vertex.adjacent_vertices:
                if j.name not in ignored_position:
                    dict_order_adjacent[j.name] = dict_genes_value[j.name]
            order_adjacent = list(dict(sorted(dict_order_adjacent.items(), key=lambda x: x[1])).keys())

            for j in order_adjacent:
                edge = Edge(list_vertices[j], vertex)
                if check_condition(graph, edge):
                    graph.add_edge(edge)
                    # print(edge)
                    # print(self._list_vertex[j], ' ==> ', vertex)
                    break
        return graph

class LayerContructor:
    def __init__(self):
        pass



if __name__ == '__main__':
    # gen = [0.55, 0.75, 0.35, 0.65, 0.45, 0.99, 0.9, 0.3, 0.4, 0.6, 0.5, 0.7, 0.77, 0.22, 0.57, 0.45, 0.66, 0.14]
    # vertices = [None for _ in range(6)]
    # for i in range(6):
    #     vertices[i] = Vertex(i)

    gen = [0.55, 0.75, 0.35, 0.65, 0.45, 0.99, 0.5, 0.4, 0.9, 0.3, 0.4, 0.6, 0.5, 0.7, 0.2, 0.75, 0.22, 0.57, 0.45, 0.66, 0.14, 0.5, 0.2, 0.6]
    vertices = [None for _ in range(8)]
    for i in range(8):
        vertices[i] = Vertex(name=i)

    vertices[0].adjacent_vertices = [vertices[1], vertices[3], vertices[5]]
    vertices[1].adjacent_vertices = [vertices[0], vertices[2], vertices[4], vertices[5]]
    vertices[2].adjacent_vertices = [vertices[0], vertices[1], vertices[3], vertices[6]]
    vertices[3].adjacent_vertices = [vertices[2], vertices[4], vertices[5]]
    vertices[4].adjacent_vertices = [vertices[1], vertices[3], vertices[3], vertices[7]]
    vertices[5].adjacent_vertices = [vertices[0], vertices[1], vertices[4]]
    vertices[6].adjacent_vertices = [vertices[0], vertices[2], vertices[3]]
    vertices[7].adjacent_vertices = [vertices[1], vertices[3], vertices[5]]

    constructor = Constructor(3, 2, 4, vertices)
    a = constructor.gen_graph(gen)
    print(a)
    print(get_hop(a, vertices[5]))
    print(vertices[5].hop)
