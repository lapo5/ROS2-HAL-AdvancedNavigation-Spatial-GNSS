import os
from glob import glob
from setuptools import setup


package_name = 'advancednavigation_gnss'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('lib', package_name, 'resources'), glob(package_name+'/advancednavigation_spatial/*.so')),
        (os.path.join('lib', package_name, 'resources'), glob(package_name+'/geo_to_cart_cxx/*.so')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marco Lapolla',
    maintainer_email='marco.lapolla5@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        "gnss_node = advancednavigation_gnss.gnss:main"
        ],
    },
)
