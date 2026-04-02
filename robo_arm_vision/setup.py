from setuptools import find_packages, setup

package_name = 'robo_arm_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robo_arm_vision.launch.py', 'launch/realsense_glove.launch.py', 'launch/yolo_detection_3d.launch.py']),
    ],
    install_requires=['setuptools', 'numpy<2', 'opencv-python<4.13', 'ultralytics'],
    zip_safe=True,
    maintainer='pitorch',
    maintainer_email='pitorch@todo.todo',
    description='RealSense + Hailo YOLOv8 depth-fusion pipeline with hosted WebSocket streaming.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'robo_arm_vision_node = robo_arm_vision.yolo_detection_node:main',
            'yolo_detection_3d_node = robo_arm_vision.yolo_detection_3d_node:main',
        ],
    },
)
