from setuptools import find_packages, setup

package_name = 'maze_solver'

setup(
    name=package_name,
    version='0.0.2',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/maze_solver.launch.py']),
        ('share/' + package_name + '/worlds', ['worlds/maze.world']),
        ('share/' + package_name + '/urdf', ['urdf/robot.urdf']),
        ('share/' + package_name + '/config', [
            'config/diff_drive_controller.yaml',
            'config/ball_detector.pt',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Maze solver for ROS2 Jazzy + Gazebo Sim',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'main_node = maze_solver.main_node:main',
        ],
    },
)