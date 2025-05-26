from setuptools import find_packages, setup

package_name = 'odom_tools'

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
    maintainer='dol08222k',
    maintainer_email='dol08222k@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_listener = odom_tools.odom_listener:main',
            'mqtt_goal_sender = odom_tools.mqtt_goal_sender:main',
            'kiosk = odom_tools.kiosk:main',
            'move_turtlebot = odom_tools.move_turtlebot:main',
            'mqtt_receiver = odom_tools.mqtt_receiver:main',
            'object_detection = odom_tools.object_detection:main',
            'video_publisher = odom_tools.video_publisher:main',
            'latency_measurement = odom_tools.latency_measurement:main',
            'temp_kiosk = odom_tools.temp_kiosk:main',

        ],
    },
)
