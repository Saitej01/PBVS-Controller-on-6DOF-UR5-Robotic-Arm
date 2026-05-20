from glob import glob
from setuptools import find_packages, setup

package_name = 'pbvs_camera'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.world')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pavan-kumar-pavada',
    maintainer_email='user@example.com',
    description='UR5e PBVS camera simulation package with Gazebo camera bridge and RGB-D target detection.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pbvs_controller = pbvs_camera.pbvs_controller:main',
            'mock_target_publisher = pbvs_camera.mock_target_publisher:main',
            'gazebo_object_detector = pbvs_camera.gazebo_object_detector:main',
            'rviz_world_markers = pbvs_camera.rviz_world_markers:main',
            'camera_visualizer = pbvs_camera.camera_visualizer:main',
            'camera_color_depth_target_node = pbvs_camera.camera_color_depth_target_node:main',
            'static_gazebo_camera_tf = pbvs_camera.static_gazebo_camera_tf:main',
            'static_eih_camera_tf = pbvs_camera.static_eih_camera_tf:main',
        ],
    },
)
