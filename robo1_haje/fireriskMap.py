import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import math

class FireRiskMap:
    def __init__(self, world_size, resolution):
        self.world_size = world_size
        self.resolution = resolution 
        self.grid_size = int(self.world_size / self.resolution)
        self.veg_map = np.zeros((self.grid_size, self.grid_size), dtype=int)  # x, y coords of the graph
        self.fire_map = np.zeros((self.grid_size, self.grid_size), dtype=float) 
        self.weights = {
            "short": 0.3,
            "medium": 0.6,
            "tall": 0.9
        } 
        self.radius = {
            "short": 4,
            "medium": 6,
            "tall": 8
        }
        self.treeCategory = {
            "short": 1,
            "medium": 2,
            "tall": 3
        }
        
        plt.ion()
        self.fig_veg, self.ax_veg = plt.subplots(figsize=(6, 6))
        self.fig_fire, self.ax_fire = plt.subplots(figsize=(6, 6))
            


    def expandingPixel(self, gx, gy, radius, treeInt, weight):
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                nx = gx + dx
                ny = gy + dy
                if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                    self.veg_map[ny, nx] = treeInt 
                    self.fire_map[ny, nx] = min(self.fire_map[ny, nx] + weight, 1.5)

    def visibleVegetionMap(self, shortTrees, mediumTrees, tallTrees, expand=True):
            # Clear maps for each new scan
            self.veg_map.fill(0)
            self.fire_map.fill(0)

            for (x, y) in shortTrees:
                gx, gy = int(x / self.resolution), int(y / self.resolution)
                if expand:
                    self.expandingPixel(gx, gy, self.radius["short"], self.treeCategory["short"], self.weights["short"])
                else:
                    self.veg_map[gy, gx] = self.treeCategory["short"]

            for (x, y) in mediumTrees:
                gx, gy = int(x / self.resolution), int(y / self.resolution)
                if expand:
                    self.expandingPixel(gx, gy, self.radius["medium"], self.treeCategory["medium"], self.weights["medium"])
                else:
                    self.veg_map[gy, gx] = self.treeCategory["medium"]

            for (x, y) in tallTrees:
                gx, gy = int(x / self.resolution), int(y / self.resolution)
                if expand:
                    self.expandingPixel(gx, gy, self.radius["tall"], self.treeCategory["tall"], self.weights["tall"])
                else:
                    self.veg_map[gy, gx] = self.treeCategory["tall"]

            colors = ["white", "green", "blue", "purple"]
            veg_cmap = ListedColormap(colors)

            self.ax_veg.clear()
            im = self.ax_veg.imshow(
                self.veg_map,
                cmap=veg_cmap,
                origin="lower",
                extent=[0, self.world_size, 0, self.world_size]
            )
            self.ax_veg.set_title("Visible Vegetation Map")

            if not hasattr(self, "veg_cbar"):
                self.veg_cbar = self.fig_veg.colorbar(im, ax=self.ax_veg, ticks=[0,1,2,3])
                self.veg_cbar.ax.set_yticklabels(['None','Short','Medium','Tall'])
            else:
                self.veg_cbar.update_normal(im)

            plt.pause(0.001)

    def fireRiskMap(self):
        colors = ["white", "yellow", "orange", "red", "black"]
        cmap = ListedColormap(colors)

        self.ax_fire.clear()
        im = self.ax_fire.imshow(
            self.fire_map,
            cmap=cmap,
            origin="lower",
            vmin=0,
            vmax=1.4,
            extent=[0, self.world_size, 0, self.world_size]
        )
        self.ax_fire.set_title("Fire Fuel Map")


        if not hasattr(self, "fire_cbar"):
            self.fire_cbar = self.fig_fire.colorbar(im, ax=self.ax_fire)
            self.fire_cbar.set_label("Fire Risk Level")
        else:
            self.fire_cbar.update_normal(im)

        plt.pause(0.001)

    def ffdiCalculator(self, temp, humidity, windSpeed, droughtFactor):
        ffdi = 2.0*math.exp(-0.45+0.987 * math.log(droughtFactor + 0.001) -0.0345 * humidity +0.0338 * temp + 0.0234 * windSpeed)

        return ffdi
    
    def ffdiRating(self, ffdi):
        if (ffdi < 11):
            rating = "Low / Moderate"
            colour = "green"
        elif (12 < ffdi < 24):
            rating = "High"
            colour = "blue"
        elif (25 < ffdi < 49):
            rating = "Very High"
            colour = "yellow"
        elif (50 < ffdi < 74):
            rating = "Severe"
            colour = "orange"
        elif ( 75 < ffdi < 99):
            rating = "Extreme"
            colour = "red"
        elif (100 < ffdi):
            rating = "Catastrophic"
            colour = "dark red"
        else:
            rating = "Unable to calculate rating"
        ratingAndColour = [rating, colour]
        return ratingAndColour