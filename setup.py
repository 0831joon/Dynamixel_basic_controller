from setuptools import setup

package_name = 'dynamixel_test_ctrl'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/main.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kyj',
    maintainer_email='kyj@example.com',
    description='Dynamixel ROS2 control and GUI integration',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'master_node = dynamixel_test_ctrl.dynamixel_master_node:main',
            'gui_node = dynamixel_test_ctrl.dynamixel_gui_node:main',
        ],
    },
)
