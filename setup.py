from setuptools import setup
import os

package_name = "robo1_haje"

setup(
    name=package_name,
    version="0.3.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         [os.path.join("resource", package_name)]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/sim.launch.py"]),
        ("share/" + package_name + "/config", ["config/movement_params.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jorian Mitchell",
    maintainer_email="jorian.p.mitchell@outlook.com",
    description="Waypoint-based movement controller for the drone sim.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "movement_node = robo1_haje.movement_node:main",
            "fire_map_node = robo1_haje.fireRiskCreateMap:main",
            "gui = gui.gui:main"
        ],
    },
)