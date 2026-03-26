from setuptools import setup

package_name = 'remote_operate_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[  
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/remote_operate.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Integrated remote operation package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'neck_control = remote_operate_pkg.neck_control:main',
            'joint_control = remote_operate_pkg.joint_control:main',
            'hand_control = remote_operate_pkg.hand_control:main',
            'body_control = remote_operate_pkg.body_control:main',
        ],
    },
)