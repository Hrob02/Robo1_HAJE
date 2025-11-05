import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import math

class FireRiskMap:
    def __init__(self, world_size, resolution):
        self.world_size = world_size
        self.resolution = resolution 

        self.origin_x = 0.0   # default; will be updated later if needed
        self.origin_y = 0.0

        self.grid_size = int(self.world_size / self.resolution)


        self.veg_map = np.zeros((self.grid_size, self.grid_size), dtype=int)
        self.fire_map = np.zeros((self.grid_size, self.grid_size), dtype=float)

        self.weights = {"short": 0.3, "medium": 0.6, "tall": 0.9}
        self.radius = {"short": 4, "medium": 6, "tall": 8}
        self.treeCategory = {"short": 1, "medium": 2, "tall": 3}

        #side-by-side subplots
        plt.ion()
        self.fig, (self.ax_veg, self.ax_fire) = plt.subplots(1, 2, figsize=(12, 6))
        self.fig.suptitle("Fire Risk Detection Maps")

        # Initialize colorbars (will be set on first update)
        self.veg_cbar = None
        self.fire_cbar = None

    def world_to_grid(self, x, y):
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        return gx, gy


    def expandingPixel(self, gx, gy, radius, treeInt, weight):
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                nx = gx + dx
                ny = gy + dy
                if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                    self.veg_map[ny, nx] = treeInt 
                    self.fire_map[ny, nx] = min(self.fire_map[ny, nx] + weight, 1.5)


    def visibleVegetationMap(self, shortTrees, mediumTrees, tallTrees, expand=True):
        # Clear maps for each new scan
        self.veg_map.fill(0)
        self.fire_map.fill(0)

        # Draw vegetation points
        for (x, y) in shortTrees:
            gx, gy = self.world_to_grid(int(x / self.resolution), int(y / self.resolution))
            if expand:
                self.expandingPixel(gx, gy, self.radius["short"], self.treeCategory["short"], self.weights["short"])
            else:
                self.veg_map[gy, gx] = self.treeCategory["short"]

        for (x, y) in mediumTrees:
            gx, gy = self.world_to_grid(int(x / self.resolution), int(y / self.resolution))
            if expand:
                self.expandingPixel(gx, gy, self.radius["medium"], self.treeCategory["medium"], self.weights["medium"])
            else:
                self.veg_map[gy, gx] = self.treeCategory["medium"]

        for (x, y) in tallTrees:
            gx, gy = self.world_to_grid(int(x / self.resolution), int(y / self.resolution))
            if expand:
                self.expandingPixel(gx, gy, self.radius["tall"], self.treeCategory["tall"], self.weights["tall"])
            else:
                self.veg_map[gy, gx] = self.treeCategory["tall"]

        self.updatePlots()


    def updatePlots(self):
        """Update both vegetation and fire risk plots side-by-side."""
        # --- Vegetation map ---
        colors_veg = ["white", "green", "blue", "purple"]
        veg_cmap = ListedColormap(colors_veg)

        self.ax_veg.clear()
        im_veg = self.ax_veg.imshow(
            self.veg_map,
            cmap=veg_cmap,
            origin="lower",
            extent=[0, self.world_size, 0, self.world_size]
        )
        self.ax_veg.set_title("Vegetation Density")
        self.ax_veg.set_xlabel("X (m)")
        self.ax_veg.set_ylabel("Y (m)")


        if self.veg_cbar is None:
            self.veg_cbar = self.fig.colorbar(im_veg, ax=self.ax_veg, ticks=[0,1,2,3])
            self.veg_cbar.ax.set_yticklabels(['None','Short','Medium','Tall'])
        else:
            self.veg_cbar.update_normal(im_veg)

        # Fire risk map
        colors_fire = ["white", "yellow", "orange", "red", "black"]
        fire_cmap = ListedColormap(colors_fire)

        self.ax_fire.clear()
        im_fire = self.ax_fire.imshow(
            self.fire_map,
            cmap=fire_cmap,
            origin="lower",
            vmin=0,
            vmax=1.4,
            extent=[0, self.world_size, 0, self.world_size]
        )
        self.ax_fire.set_title("Fire Fuel / Risk Level")
        self.ax_fire.set_xlabel("X (m)")
        self.ax_fire.set_ylabel("Y (m)")

        if self.fire_cbar is None:
            self.fire_cbar = self.fig.colorbar(im_fire, ax=self.ax_fire)
            self.fire_cbar.set_label("Fire Risk Level")
        else:
            self.fire_cbar.update_normal(im_fire)


        plt.tight_layout()
        plt.pause(0.001)


    def ffdiCalculator(self, temp, humidity, windSpeed, droughtFactor):
        ffdi = 2.0 * math.exp(
            -0.45 + 0.987 * math.log(droughtFactor + 0.001)
            - 0.0345 * humidity + 0.0338 * temp + 0.0234 * windSpeed
        )
        return ffdi


    def ffdiRating(self, ffdi):
        if ffdi < 11:
            return ["Low / Moderate", "green"]
        elif ffdi < 24:
            return ["High", "blue"]
        elif ffdi < 49:
            return ["Very High", "yellow"]
        elif ffdi < 74:
            return ["Severe", "orange"]
        elif ffdi < 99:
            return ["Extreme", "red"]
        else:
            return ["Catastrophic", "dark red"]
