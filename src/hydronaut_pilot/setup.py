import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'hydronaut_pilot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'scripts'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # 1. Grab any loose TFLite models
        (os.path.join('share', package_name, 'models'), glob('models/*.tflite')),
        
        # 2. Grab the NCNN folder and EVERYTHING inside it!
        (os.path.join('share', package_name, 'models', 'yolo11n_ncnn_model'), glob('models/yolo11n_ncnn_model/*')),

        # 3. Grab the DAE file
        (os.path.join('share', package_name, 'models', 'Swimming'), glob('models/Swimming/*.dae')),

        # 4. Grab the textures
        (os.path.join('share', package_name, 'models', 'Swimming', 'textures'), glob('models/Swimming/textures/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='habib',
    maintainer_email='haibullahkhan2002@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'manager = hydronaut_pilot.manager:main',
            'base_controller = hydronaut_pilot.base_controller:main',
            'system_test = scripts.system_test:main',
            'mission_runner = scripts.mission_runner:main',
            'vision_tracker = scripts.vision_tracker:main',
            'line_tracker = scripts.line_tracker:main',
            'yolo_tracker = scripts.yolo_tracker:main',
            'yolo_tracker2 = scripts.yolo_tracker2:main',
            'aruco_master = scripts.aruco_master_control:main',
        ],
    },
)