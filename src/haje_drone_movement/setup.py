#from setuptools import find_packages, setup

#package_name = 'haje_drone_movement'

#setup(
#    name=package_name,
#    version='0.0.0',
#    packages=find_packages(exclude=['test']),
#    data_files=[
#        ('share/ament_index/resource_index/packages',
#            ['resource/' + package_name]),
#        ('share/' + package_name, ['package.xml']),
#    ],
#    install_requires=['setuptools'],
#    zip_safe=True,
#    maintainer='parallels',
#    maintainer_email='jorian.p.mitchell@outlook.com',
#    description='TODO: Package description',
#    license='TODO: License declaration',
#    tests_require=['pytest'],
#    entry_points={
#        'console_scripts': [
#        ],
#    },
#)



# this was used for pass and creditish passing
#from setuptools import setup

#package_name = 'haje_drone_movement'

#setup(
#    name=package_name,
#    version='0.0.0',
#    packages=[package_name],
#    data_files=[
#        ('share/ament_index/resource_index/packages',
#         ['resource/' + package_name]),
#        ('share/' + package_name, ['package.xml']),
#        ('share/' + package_name + '/launch', ['launch/movement.launch.py']),
#    ],
#    install_requires=['setuptools'],
#    zip_safe=True,
#    maintainer='parallels',
#    maintainer_email='jorian.p.mitchell@outlook.com',
#    description='Drone movement node',
#    license='MIT',
#    entry_points={
#        'console_scripts': [
#            'movement_node = haje_drone_movement.movement_node:main',
#        ],
#    },
#)


from setuptools import setup
import os

package_name = "haje_drone_movement"

setup(
    name=package_name,
    version="0.3.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         [os.path.join("resource", package_name)]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/movement.launch.py"]),
        ("share/" + package_name + "/config", ["config/movement_params.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="parallels",
    maintainer_email="jorian.p.mitchell@outlook.com",
    description="Waypoint-based movement controller for the Parrot sim.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "movement_node = haje_drone_movement.movement_node:main",
        ],
    },
)

