
import numpy as np

raduis = 1

theta = np.linspace(-np.pi, -0.70*np.pi, 75)

for angle in theta:
    x = np.round(raduis * np.cos(angle),3)
    y = np.round(raduis * np.sin(angle),3)

    print("- offset: {x: 0.0, y: 0.0, z: 0.0}")
    print("  direction: {x: 0.0"  + ", y: " + str(x) + ",z: " + str(y) + "}" )
    print("")
