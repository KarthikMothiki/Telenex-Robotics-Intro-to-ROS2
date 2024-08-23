from setuptools import find_packages, setup
import os
import glob

package_name = 'my_pkg'

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
    maintainer='km',
    maintainer_email='km@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = my_pkg.publisher:main',
            'subscriber = my_pkg.subscriber:main',
            'server = my_pkg.server:main',
            'client = my_pkg.client:main',
            'custom_pub = my_pkg.custom_msg_pub:main',
            'custom_sub = my_pkg.custom_msg_sub:main',      
        ],
    },
)
