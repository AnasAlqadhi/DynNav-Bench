from setuptools import setup
import os

package_name = 'dynnav_bench'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            ['launch/dynnav_bench_launch.py']),
        (os.path.join('share', package_name, 'config'),
            ['config/dynnav_bench.yaml']),
        (os.path.join('share', package_name, 'worlds'),
            ['worlds/dynnav_arena.world']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anas',
    maintainer_email='anas@todo.todo',
    description='DynNav-Bench: A dynamic-obstacle navigation benchmark for deep reinforcement learning',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'environment      = dynnav_bench.environment:main',
            'goal_manager     = dynnav_bench.goal_manager:main',
            'obstacle_controller = dynnav_bench.obstacle_controller:main',
        ],
    },
)
