import os
from setuptools import find_packages, setup

package_name = 'scoutfetch_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         ['launch/scoutfetch_mission.launch.py', 'launch/sim_bottle_search.launch.py',
          'launch/sim_bookstore_search.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ros@todo.todo',
    description='ScoutFetch task orchestration: nav_goal_executor dispatches /navigation_goal to Nav2',
    license='Apache-2.0',
    extras_require={},
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav_goal_executor = scoutfetch_mission.nav_goal_executor:main',
            'target_pose_resolver = scoutfetch_mission.target_pose_resolver:main',
            'task_fsm = scoutfetch_mission.task_fsm:main',
            'start_sim_task = scoutfetch_mission.start_sim_task:main',
            'robot_control = scoutfetch_mission.robot_control:main',
        ],
    },
)
