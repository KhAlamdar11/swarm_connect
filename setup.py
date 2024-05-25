from setuptools import find_packages, setup

package_name = 'swarm_connect'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            "hardware1 = swarm_connect.hardware1:main",
            "transform_listener = swarm_connect.transform_listener:main"
        ],
    },
)
