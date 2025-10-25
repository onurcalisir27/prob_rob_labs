from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'prob_rob_labs'

launch_files = glob('launch/*_launch.py') + \
    glob('launch/*_launch.xml') + \
    glob('launch/*.yaml') + \
    glob('launch/*.yml')

data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    (os.path.join('share', package_name, 'launch'), launch_files),
]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(where='src', exclude=['test']),
    package_dir={'': 'src'},
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ilija Hadzic',
    maintainer_email='ih2435@columbia.edu',
    description='Prob Rob Labs',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lab4_odom_error = lab4_odom_error.lab4_odom_error:main',
            'vision_processor = vision_processor.vision_processor:main',
            'lab4_ekf = lab4_ekf.lab4_ekf:main',
            'lab4 = lab4.lab4:main',
            'lab3 = lab3.lab3:main',
            'lab2 = lab2.lab2:main',
            'image_mean_feature_x = image_mean_feature_x.image_mean_feature_x:main',
            'flaky_door_opener = flaky_door_opener.flaky_door_opener:main',
        ],
    }
)
