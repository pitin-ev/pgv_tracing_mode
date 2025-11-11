from setuptools import find_packages, setup

package_name = 'pgv_tracing_mode'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/line_drive.launch.py',
            'launch/test_line_drive_sim.launch.py',
            ]),
        ('share/' + package_name + '/config', [
            'config/line_drive.params.yaml',
            'config/sim.params.yaml'
            ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jaerak',
    maintainer_email='jr@pitin-ev.com',
    description='Drive along a line using PGV(R4) PoseStamped with holonomic/non-holonomic control.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'line_drive = pgv_tracing_mode.line_tracing_node:main',
            'dummy_pgv_sim = pgv_tracing_mode.sim_dummy_pgv:main',
            'one_shot_goal = pgv_tracing_mode.one_shot_goal:main',
        ],
    },
)
