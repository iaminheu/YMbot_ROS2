from setuptools import find_packages, setup

package_name = 'vr_receiver_tcp'

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
    maintainer='ymzz-robot',
    maintainer_email='feilong_wang0707@163.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vr_receiver_tcp = vr_receiver_tcp.vr_receiver_tcp:main',
        ],
    },
)
