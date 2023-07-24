from setuptools import setup

package_name = 'liftmodule_wrapper'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ChenYing Kuo',
    maintainer_email='chenying.kuo@adlinktech.com',
    description='Liftmodule Wrapper',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'liftmodule_node = liftmodule_wrapper.liftmodule_node:main'
        ],
    },
)
