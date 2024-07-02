from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'pkg_website_beerpong'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #('share/' + package_name + '/pkg_website_beerpong/launch/', ['launch/website.launch.py']),
        #('share/pkg_website_beerpong/pkg_website_beerpong/launch/', ['website.launch.py']),
        #(os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
        #(os.path.join('src', package_name,package_name, 'launch')),
    ],  
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'website_beerpong = pkg_website_beerpong.app:main'
            #'website_beerpong = pkg_website_beerpong.website_beerpong:main'
        ],
    },
)
