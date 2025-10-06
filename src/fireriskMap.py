import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap


world_size = 50  
resolution = 0.5 
grid_size = int(world_size / resolution)
radius = 10
treeInt = 0

veg_map = np.zeros((grid_size, grid_size), dtype=int)  
fire_map = np.zeros((grid_size, grid_size), dtype=int)  


shortTrees = [
    (10, 15),  
    (25, 30),
    (40, 5),
]
mediumTrees = [
    (2, 17),  
    (14, 22),
    (38, 32),
]
tallTrees = [
    (29, 2), 
    (9, 29),
    (11, 1),
]



def expandingPixel(gx, gy, radius, treeInt):
    for dx in range(-radius, radius + 1):
        for dy in range(-radius, radius + 1):
            nx = gx + dx
            ny = gy + dy
            if 0 <= nx < grid_size and 0 <= ny < grid_size:
                veg_map[ny, nx] = treeInt 


for (x, y) in shortTrees:
    gx = int(x / resolution)
    gy = int(y / resolution)
    expandingPixel(gx, gy, radius, 1)

for (x, y) in mediumTrees:
    gx = int(x / resolution)
    gy = int(y / resolution)

    expandingPixel(gx, gy, radius, 2)

for (x, y) in tallTrees:
    gx = int(x / resolution)
    gy = int(y / resolution)

    expandingPixel(gx, gy, radius, 3)



colors = ["white", "green", "blue", "purple"]
veg_cmap = ListedColormap(colors)

plt.figure(figsize=(6,6))
plt.title("Visible Vegetation FART")
im = plt.imshow(veg_map, cmap=veg_cmap, origin="lower")
cbar = plt.colorbar(im, ticks=[0,1,2,3])
cbar.ax.set_yticklabels(['None','Short','Medium','Tall'])
plt.show()