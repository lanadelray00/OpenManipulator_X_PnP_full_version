from setuptools import setup

package_name = 'openmanipulator_task_executor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/openmanipulator_task_executor/config', ['src/config/calib_data.npz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='choi',
    maintainer_email='choi@todo.todo',
    description='Pick and place with ArUco',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'open_manipulator_x_pickandplace = openmanipulator_task_executor.open_manipulator_x_pickandplace:main',
            'keyboard_trigger_node = openmanipulator_task_executor.keyboard_trigger_node:main',
        ],
    },
)
