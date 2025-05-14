from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'swarm_connect'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Required by ament to find the package
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        
        # Install the package.xml file
        ('share/' + package_name, ['package.xml']),
        
        # Install the launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        
        # Install the config files if you have any in the 'cfg' folder
        (os.path.join('share', package_name, 'cfg'), glob('cfg/*.cfg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anton-superior',
    maintainer_email='75181406+KhAlamdar11@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "connectivity_control = swarm_connect.connectivity_control:main",
            "stubborn_agents = swarm_connect.stubborn_agents:main",
            "beacons = swarm_connect.beacons:main",
            "test_octomap_slicer = swarm_connect.test_octomap_slicer:main",
            "inspection_v1 = swarm_connect.inspection_v1:main",
            "beacons_set_waypoints = swarm_connect.beacons_set_waypoints:main",
        ],
    },
)
