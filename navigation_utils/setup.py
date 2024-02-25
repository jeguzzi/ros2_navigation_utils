from setuptools import find_packages, setup
import glob

package_name = 'navigation_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/launch', glob.glob('launch/*')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jerome.guzzi',
    maintainer_email='jerome@idsia.ch',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'follow_path_server = navigation_utils.follow_path_no_obstacles:main',
            'test_client = navigation_utils.test_client:main',
            'tracker2pose = navigation_utils.relative_pose_from_tracker:main',
            'detection2pose = navigation_utils.relative_pose_from_detector:main',
        ],
    },
)
