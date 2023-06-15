import numpy as np
from sp import sp
from sp_lr import sp_lr


def algorithm_ccpp(max_load_algorithm, switch_loads, num_switches, distances, k):
    # Step 1: Sort distances in ascending order
    # Flatten the distance matrix into a 1D array
    dis_array = distances.flatten()
    # Sort the 1D array in ascending order
    dis_array = np.sort(dis_array)

    # Step 2: Binary search for the minimum radius r
    min_radius = None
    min_controllers = None
    lower = 0
    upper = len(dis_array) - 1

    while lower < upper:
        mid = (lower + upper) // 2
        r = dis_array[mid]
        num_controllers = sp_lr(max_load_algorithm, switch_loads, num_switches, distances, r, k)

        if num_controllers is not None:
            if num_controllers > k:
                lower = mid + 1
                print("Solution not found")
            else:
                min_radius = r
                min_controllers = num_controllers
                upper = mid
        else:
            print("Solution not found")
            # lower = mid + 1
            break

    print('Step 1:')
    print('Minimum radius: ', min_radius)
    print('Number of controllers: ', min_controllers)
    print('Lower', lower)

    # Step 3: Find placement for minimum radius
    min_radius2 = None
    min_controllers2 = k+1

    index = lower
    num_controllers = sp(max_load_algorithm, switch_loads, num_switches, distances, dis_array[index], k)
    print(num_controllers)

    if num_controllers is not None:
        while num_controllers > k:
            index += 1
            print(f"Index {index}, {len(dis_array)}")
            num_controllers = sp(max_load_algorithm, switch_loads, num_switches, distances, dis_array[index], k)

        print('Step 1:')
        print('Minimum radius: ', min_radius)
        print('Number of controllers: ', min_controllers)
        print('Lower', lower, '\n')
        print('Step 2:')
        print('Minimum radius: ', dis_array[index])
        print('Number of controllers: ', num_controllers)
        print('Lower', index)
    else:
        print("Solution not found")


