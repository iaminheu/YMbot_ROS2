
from setuptools import find_packages, setup, find_namespace_packages
import os
from glob import glob
package_name = 'ik_solver_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_namespace_packages(exclude=['test']),
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ymzz-robot',
    maintainer_email='feilong_wang0707@163.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
        'ik_servo_node_coll= ik_solver_pkg.ik_servo_node_collision:main',
        ],
    },
    # 添加脚本解释器路径 使用conda环境的python解释器，而非系统自己的
    # scripts=['ik_solver_pkg/ik_servo_node_collision.py'],
    options={
        'build_scripts': {
            'executable': '/home/ymzz/anaconda3/envs/pin/bin/python',
        },
    },
)
