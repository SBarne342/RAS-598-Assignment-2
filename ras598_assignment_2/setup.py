from setuptools import find_packages, setup
# from glob import glob  
# ^ would have collected up all the files of a type, like the .py files 
# so that I didn't have to list them all out. May be useful to remember 
# for future use)

package_name = 'ras598_assignment_2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['map.yaml', 'cave_filled.png', 'planning.rviz']),
        # ('share/' + package_name, ['grading_scout.py']),
        ('share/' + package_name + '/launch', ['launch/planner_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Moss',
    maintainer_email='sbarne34@asu.edu',
    description='path planning package that uses A* with line-of-sight pruning for autonomous navigation in a cave environment.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'planning_core = ras598_assignment_2.planning_core:main',
            'grading_scout = ras598_assignment_2.grading_scout:main',
        ],
    },
)
