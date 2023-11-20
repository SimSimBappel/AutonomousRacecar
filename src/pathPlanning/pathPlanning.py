from fsd_path_planning import PathPlanner, MissionTypes, ConeTypes
from fsd_path_planning.utils.utils import Timer
from pathlib import Path
from typing import Optional
import numpy as np
import json
import matplotlib.pyplot as plt

jsonFile = True

try:
    from tqdm import tqdm
except ImportError:
    print("You can get a progress bar by installing tqdm: pip install tqdm")
    tqdm = lambda x, total=None: x


def load_data():   
    data_path = Path(__file__).parent / "test.json"
        # data_path = Path(__file__).parent / "fss_19_4_laps.json"
        #format yellow,blue,unknown
        # Must give all global cones x and y

        
    # extract data
    data = json.loads(data_path.read_text())[:]

    positions = np.array([d["car_position"] for d in data])
    directions = np.array([d["car_direction"] for d in data])
    cone_observations = [
        [np.array(c).reshape(-1, 2) for c in d["slam_cones"]] for d in data
    ]
    # print(cone_observations)

    # print(directions)

    return cone_observations, positions, directions


path_planner = PathPlanner(MissionTypes.trackdrive)
# you have to load/get the data, this is just an example
if jsonFile:
    cone_observations_json, car_position, car_direction = load_data() 

# global_cones is a sequence that contains 5 numpy arrays with shape (N, 2),
# where N is the number of cones of that type

# ConeTypes is an enum that contains the following values:
# ConeTypes.UNKNOWN which maps to index 0
# ConeTypes.RIGHT/ConeTypes.YELLOW which maps to index 1
# ConeTypes.LEFT/ConeTypes.BLUE which maps to index 2
# ConeTypes.START_FINISH_AREA/ConeTypes.ORANGE_SMALL which maps to index 3
# ConeTypes.START_FINISH_LINE/ConeTypes.ORANGE_BIG which maps to index 4

# car_position is a 2D numpy array with shape (2,)
# car_direction is a 2D numpy array with shape (2,) representing the car's direction vector
# car_direction can also be a float representing the car's direction in radians

if not jsonFile:
    cone_observations = [[
    np.array([], dtype=np.float64).reshape(0, 2),
    np.array([[2.06137085, 12.06208801],
              [2.24887085, 15.12458801],
              [1.62387085, 3.81208801],
              [2.81137085, 18.49958801],
              [1.99887085, 9.62458801]]),
    np.array([[-0.68862915, 12.49958801],
              [-0.62612915, 14.99958801],
              [-0.31362915, 19.12458801],
              [-1.50112915, 4.18708801],
              [-0.93862915, 9.37458801]]),
    np.array([], dtype=np.float64).reshape(0, 2),
    np.array([[1.87387085, 6.18708801],
              [-1.18862915, 6.37458801],
              [1.74887085, 5.62458801],
              [-1.25112915, 5.93708801]])
    ]]

    car_position = np.array([[
                -0.0006561279296875,
                -4.57763671875e-05
            ]])

    car_direction = np.array([[
                0.023904704323199574,
                0.9997142417267149
            ]])

    position = car_position[0]
    direction = car_direction[0]
    cones = cone_observations[0]

else:
    position = car_position[0]
    direction = car_direction[0]
    cones = cone_observations_json[0]


timer = Timer(noprint=True)
try:
    with timer:
        out = path_planner.calculate_path_in_global_frame(
            cones,
            position,
            direction,
            return_intermediate_results=False,
        )
except Exception as e:
    print(f"Error at frame {e}")
    raise
    # results.append(out)

if timer.intervals[-1] > 0.1:
            print(f"Frame took {timer.intervals[-1]:.4f} seconds")



x = out[:, 1]  
y = out[:, 0] 

bluex_cones = cones[1][:, 0]
bluey_cones = cones[1][:, 1] 

yellowx_cones = cones[2][:, 0]  
yellowy_cones = cones[2][:, 1] 

fig, ax = plt.subplots(figsize=(10, 8))

plt.scatter(bluex_cones, bluey_cones, c='b', marker='o', label='Cones')
plt.scatter(yellowx_cones, yellowy_cones, c='y', marker='o', label='Cones')
plt.plot(x, y, label='Path')
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.title('Path in Global Frame')
plt.legend()
ax.set_aspect('equal')
plt.grid(True)
plt.show()

# path = path_planner.calculate_path_in_global_frame(cone_observations, car_position, car_direction)

# path is a Mx4 numpy array, where M is the number of points in the path
# the columns represent the spline parameter (distance along path), x, y and path curvature


