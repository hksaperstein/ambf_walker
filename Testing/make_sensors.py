
import numpy as np

raduis = 1

theta = np.linspace(0, 2*np.pi, 20)

for angle in theta:
    x = np.round(raduis * np.cos(angle),3)
    y = np.round(raduis * np.sin(angle),3)

    print("- offset: {x: 0.0, y: 0.4, z: 0.0}")
    print("  direction: {x: " +  str(x) + ", y: 0.0" + ",z: " + str(y) + "}" )
    print("")
