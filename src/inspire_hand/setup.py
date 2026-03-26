from setuptools import setup

package_name = 'inspire_hand'
utils = "inspire_hand/utils"      ## 加组件需要声明到这

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, utils],     ## 加组件需要声明到这
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Description of your package',
    license='License Type',
    entry_points={
        'console_scripts': [
            # 'inspire_hand = inspire_hand.inspire_hand_node:main',
            'inspire_hand_sub = inspire_hand.inspire_hand_sub:main',
            'single_hand_command = inspire_hand.single_hand_command:main',
            'hand_grasp_state_pub = inspire_hand.hand_grasp_state_pub:main',
        ],
    },
)
