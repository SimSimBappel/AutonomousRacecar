from fsd_path_planning import PathPlanner, MissionTypes, ConeTypes
from fsd_path_planning.utils.utils import Timer
from pathlib import Path
from typing import Optional
import numpy as np
import json
import matplotlib.pyplot as plt

jsonFile = False

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
        np.array([
            # [2.06137085, 12.06208801], #coords of blue cones
            # [2.24887085, 15.12458801],
            # [1.62387085, 3.81208801],
            # [2.81137085, 18.49958801],
            # [1.99887085, 9.62458801]
        [  2.06137085,  12.06208801],
        [  2.24887085,  15.12458801],
        [  1.62387085,   3.81208801],
        [  2.81137085,  18.49958801],
        [  1.99887085,   9.62458801],
        [  3.12387085,  21.93708801],
        [  3.56137085,  25.37458801],
        [  4.81137085,  29.24958801],
        [  7.37387085,  31.87458801],
        [ 10.91782737,  32.70692137],
        [ 21.24887085,  17.49958801],
        [ 21.88205023,  14.49969913],
        [ 20.13978407,  20.73465422],
        [ 20.87387085,  11.56208801],
        [ 13.27945776,  -1.18799202],
        [ 13.49887085,   2.87458801],
        [ 13.68637085,  -5.18791199],
        [ 15.62387085,   6.24958801],
        [ 14.93637085,  -9.06291199],
        [  1.56137085,   1.24958801],
        [  1.93637085,  -3.87541199],
        [  1.62387085,  -1.18791199],
        [  2.06137085,  -6.37541199],
        [  2.31137085,  -8.56291199],
        [  2.49887085, -11.06291199],
        [  5.74887085, -15.93791199],
        [  3.87387085, -13.75041199],
        [  7.79602003, -18.25412618],
        [ 18.12240577, -11.93988324],
        [ 13.24887085, -21.00041199],
        [ 10.99887085, -20.12541199],
        [ 15.80460385, -21.97817225],
        [ 18.32498379, -22.93943094],
        [ 21.31137085, -23.68791199],
        [ 22.56137085, -11.62541199],
        [ 23.99887085, -23.68791199],
        [ 31.01833882, -14.50683651],
        [ 25.50024074, -11.30408382],
        [ 26.20324624, -23.5651567 ],
        [ 28.48064949, -12.72104842],
        [ 35.43637085, -14.06291199],
        [ 29.56137085, -23.12541199],
        [ 32.58648595, -22.91148373],
        [ 39.12387085, -12.62541199],
        [ 35.24887085, -21.93791199],
        [ 37.68637085, -21.06291199],
        [ 42.18637085, -10.06291199],
        [ 41.49887085, -20.68791199],
        [ 44.49887085,  -6.06291199],
        [ 44.87387085, -19.43791199],
        [ 47.99887085, -17.06291199],
        [ 49.93637085, -15.37541199],
        [ 52.31794926, -14.45441502],
        [ 55.18637085, -12.81291199],
        [ 45.37387085,  -2.62541199],
        [ 56.93637085, -11.06291199],
        [ 58.12387085,  -8.12541199],
        [ 45.24887085,   0.93708801],
        [ 44.93637085,   4.49958801],
        [ 45.99887085,   7.31208801],
        [ 48.56137085,   8.93708801],
        [ 51.18637085,   9.74958801],
        [ 54.99887085,   9.49958801],
        [ 60.87387085,  -0.50041199],
        [ 59.56137085,  -4.25041199],
        [ 60.06137085,   5.87458801],
        [ 60.91748476,   3.09275499],
        ]),
    np.array([
        [ -0.68862915,  12.49958801],
        [ -0.62612915,  14.99958801],
        [ -0.31362915,  19.12458801],
        [ -1.50112915,   4.18708801],
        [ -0.93862915,   9.37458801],
        [  0.24651105,  22.38377813],
        [  0.74887085,  26.12458801],
        [  1.12387085,  30.24958801],
        [  2.81137085,  33.74958801],
        [  4.97900211,  35.14119067],
        [  8.56137085,  35.99958801],
        [ 12.68637085,  36.24958801],
        [ 14.88264605,  35.19033669],
        [ 17.12387085,  33.74958801],
        [ 19.62387085,  31.87458801],
        [ 21.24887085,  28.99958801],
        [ 22.81137085,  25.81208801],
        [ 23.81137085,  22.99958801],
        [ 24.43637085,  19.74958801],
        [ 24.87387085,  16.93708801],
        [ 24.95485029,  13.33911207],
        [ 24.18637085,  10.06208801],
        [ 21.81137085,   7.31208801],
        [ 19.64358084,   5.92485435],
        [ 17.74887085,   3.62458801],
        [ 16.49887085,  -0.00041199],
        [ 17.06210846,  -3.70204451],
        [ 24.24887085,  -7.50041199],
        [ 21.27963642,  -7.71970364],
        [ 18.39512734,  -6.94163639],
        [ -1.37612915,  -3.81291199],
        [ -1.43862915,  -6.18791199],
        [ 15.31137085, -25.87541199],
        [ 18.65749936, -26.77208051],
        [ 32.31137085, -11.25041199],
        [ 35.43637085, -10.75041199],
        [ 29.28548786,  -8.92551512],
        [ 37.40075466,  -9.66625447],
        [ 39.37387085,  -7.68791199],
        [ 26.99887085,  -7.87541199],
        [ 40.62387085,  -5.81291199],
        [ 38.87387085, -24.00041199],
        [ 42.99887085, -22.93791199],
        [ 46.18637085, -22.25041199],
        [ 49.18637085, -20.62541199],
        [ 52.18637085, -18.18791199],
        [ 54.99887085, -17.18791199],
        [ 43.68637085,  11.18708801],
        [ 41.87387085,   8.18708801],
        [ 41.56137085,   1.06208801],
        [ 41.18637085,   4.31208801],
        [ 46.06137085,  13.12458801],
        [ 48.68637085,  13.93708801],
        [ 52.18637085,  14.12458801],
        [ 55.68637085,  13.56208801],
        [ 59.12387085,  11.93708801],
        [ 61.56137085,  10.31208801],
        [ 64.06137085,   6.31208801],
        [ 65.43637085,   2.62458801],
        [ 64.12387085,  -1.06291199],
        [ 62.74887085,  -4.75041199],
        [ 61.93637085,  -7.18791199],
        [ 60.93637085,  -9.00041199],
        [ 59.99887085, -11.81291199],
        [ 57.87387085, -15.25041199],
        [ 35.87387085, -25.12541199],
        [ 32.74887085, -25.93791199],
        [ 28.93637085, -26.75041199],
        [ 25.68637085, -26.93791199],
        [ 22.24887085, -27.12541199],
        [ 11.93637085, -24.87541199],
        [  7.99887085, -23.06291199],
        [  5.24887085, -21.06291199],
        [  3.06137085, -18.93791199],
        [  1.24887085, -16.06291199],
        [ -0.12612915, -12.93791199],
        [ -1.37612915,  -9.56291199],
        [ -1.87612915,  -0.81291199],
        [ -1.56362915,   1.24958801],
        [ 41.43637085,  -2.68791199],
        ]),
    np.array([], dtype=np.float64).reshape(0, 2),
    np.array([
            # [1.87387085, 6.18708801],
            # [-1.18862915, 6.37458801],
            # [1.74887085, 5.62458801],
            # [-1.25112915, 5.93708801],
              ]),
    ]]

    
    # car_position = np.array([[
    #             -0.0006561279296875,
    #             -4.57763671875e-05
    #         ]])

    # car_direction = np.array([[
    #             0.023904704323199574,
    #             0.9997142417267149
    #         ]])

    car_position= np.array([[
            -0.029052734375,
            -0.9403076171875
        ]])
    car_direction= np.array([[
        0.01659663747416308,
        0.9998622663269933
    ]]),

    position = car_position[0]
    direction = car_direction[0]
    cones = cone_observations[0]

else:
    position = car_position[0]
    direction = car_direction[0]
    cones = cone_observations_json[0]


# print(cones[2])

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

# print(cone_observations[0][2][0])

x = out[:, 1]  
y = out[:, 0] 

print(x)


# bluex_cones = cones[1][:, 0]
# bluey_cones = cones[1][:, 1] 

# yellowx_cones = cones[2][:, 0]  
# yellowy_cones = cones[2][:, 1] 

fig, ax = plt.subplots(figsize=(10, 8))

# plt.scatter(bluex_cones, bluey_cones, c='b', marker='o', label='Cones')
# plt.scatter(yellowx_cones, yellowy_cones, c='y', marker='o', label='Cones')
plt.scatter(x, y, c='g', label='Path')
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


