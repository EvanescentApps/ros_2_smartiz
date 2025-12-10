from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mmi_hamster'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ecocain',
    maintainer_email='evan.cocain@telecom-sudparis.eu',
    description='Smartiz+ nodes for MMI',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cam_reader = mmi_hamster.cam_reader:main',
            'microcontroller_communicator = mmi_hamster.microcontroller_communicator:main',
            
            'face_smile_processor = mmi_hamster.face_smile_processor:main',
            'math_quiz_node = mmi_hamster.math_quiz_node:main',
            'game_master = mmi_hamster.game_master:main',
        ],
    },
)
