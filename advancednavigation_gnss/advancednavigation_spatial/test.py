import time

import advancednavigation_spatial
import numpy as np
import sys

if __name__ == "__main__":

    gnss = advancednavigation_spatial.HAL()
    try:
        if not gnss.init("/dev/ttyUSB0"):
            print("No GPS Found!")
            sys.exit(1)
    except:
        if not gnss.init("/dev/ttyUSB1"):
            print("No GPS Found!")
            sys.exit(1)

    print(gnss.get_gnss_name())

    res = np.array([0, 0, 0, 0, 0, 0], dtype=np.float64)
    while True:
        gnss.get_gnss_info(res)
        t = int(time.time() * 1e6)
        print(res, t)

