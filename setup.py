from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'fukuro_remotekey'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament resource index entry
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # launch files
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.py'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shluf',
    maintainer_email='shluf@fukuro.local',
    description='Keyboard teleoperation for the Fukuro omniwheel robot.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ros2 run fukuro_remotekey remotekey
            'remotekey = fukuro_remotekey.remotekey_node:main',
        ],
    },
)
