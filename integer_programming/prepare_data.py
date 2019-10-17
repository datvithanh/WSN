from utils.load_input import WsnInput
from DS.position import distance


def prepare(path):
    inp = WsnInput.from_file(path)
    is_adj_matrix = [[0 for _ in range(len(inp.all_vertex))] for _ in range(len(inp.all_vertex))]
    distance_matrix = {}
    for i in inp.all_vertex:
        for j in inp.all_vertex:
            if j.name == 0:
                distance_matrix[i.name, j.name] = 9999
                continue
            if i.name > 0 and j.name <= inp.num_of_relay_positions:
                distance_matrix[i.name, j.name] = 9999
                continue
            if distance(i, j) <= inp.radius and distance(i, j) != 0:
                distance_matrix[i.name, j.name] = distance(i, j)
                is_adj_matrix[i.name][j.name] = 1
            else:
                distance_matrix[i.name, j.name] = 9999
                is_adj_matrix[i.name][j.name] = 0

    return inp, is_adj_matrix, distance_matrix


if __name__ == '__main__':
    a = prepare('./data/ga-dem1_r25_1.in')
    print(a)
