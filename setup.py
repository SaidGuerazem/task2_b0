from setuptools import setup

package_name = 'task2_b0'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='said_guerazem',
    maintainer_email='said.guerazem@city.ac.uk',
    description='Pose estimator for drone targets, works with a given detector msg type string',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'search_pose_estimator_odom = task2_b0.search_pose_estimator_odom:main'
        ],
    },
)
