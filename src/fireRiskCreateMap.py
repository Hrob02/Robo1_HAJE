import fireriskMap as fire


shortTrees = [(10, 15), (25, 30), (40, 5)]
mediumTrees = [(2, 17), (14, 22), (38, 32)]
tallTrees = [(29, 2), (9, 29), (11, 1)]


class RunningMap:
    def __init__(self):
        pass


    def run(self):
        shortTrees = [(10, 15), (25, 30), (40, 5)]
        mediumTrees = [(2, 17), (14, 22), (38, 32)]
        tallTrees = [(29, 2), (9, 29), (11, 1)]
        
        map = fire.FireRiskMap(world_size=50, resolution=0.8)
        map.visibleVegetionMap(shortTrees, mediumTrees, tallTrees, True)
        map.fireRiskMap()
        ffdiValue = map.ffdiCalculator(35, 20, 30, 8)
        print(ffdiValue)
        ffdiRating = map.ffdiRating(ffdiValue)
        print(ffdiRating)

if __name__ == "__main__":
    runner = RunningMap()
    runner.run()