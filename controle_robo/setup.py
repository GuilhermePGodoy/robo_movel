from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'controle_robo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py')),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml')),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Guilherme Pascoale Godoy',
    maintainer_email='guilhermepascoale1911@gmail.com',
    description='Pacote de controle autônomo do robô móvel.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controle_robo = controle_robo.controle_robo:main',
        ],
    },
)
