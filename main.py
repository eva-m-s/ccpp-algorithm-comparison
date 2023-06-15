from instances import NetworkInstances
from model import sdn_opt
from model_v2 import sdn_opt_v2
from model_tester import test_opt
from algorithm import algorithm_ccpp

num_switches = NetworkInstances.S
num_controllers = NetworkInstances.K
max_load = NetworkInstances.capacities_model
max_load_algorithm = NetworkInstances.capacities_algorith
switch_loads = NetworkInstances.loads
distances = NetworkInstances.d
k = NetworkInstances.K


if __name__ == '__main__':

    # sdn_opt(num_switches, num_controllers, max_load, switch_loads, distances)

    #test_opt(num_switches, num_controllers, distances, max_load, switch_loads)

    algorithm_ccpp(max_load_algorithm, switch_loads, num_switches, distances, k)

    #sdn_opt_v2(num_switches, num_controllers, max_load_algorithm, switch_loads, distances, k)

