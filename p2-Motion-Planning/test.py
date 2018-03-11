import numpy as np
with open("colliders.csv") as myfile:
    head = [next(myfile) for x in range(2)]
latlon = np.fromstring(head[1], dtype='Float64', sep=',')
lat0 = latlon[0]
lon0 = latlon[1]
