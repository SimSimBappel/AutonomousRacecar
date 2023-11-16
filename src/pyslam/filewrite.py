# x = [1, 2, 3, 4]
# y = [1.0, 2.0, 3.0, 4.0]

# with open('/home/simon/autonomousRacecar/src/pyslam/webotskeypoints.txt', 'w') as file:
#         for i in range(len(x)):
#             file.write(f"{x[i]},{y[i]}\n")

filepath = "/home/simon/autonomousRacecar/src/pyslam/webotskeypointsobserved.txt"

with open(filepath, 'r') as file:
    lines = file.readlines()

with open(filepath, 'w') as file:
    for i, line in enumerate(lines):
        file.write(f"{i}: {line}")