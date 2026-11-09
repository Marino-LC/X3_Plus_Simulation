from setuptools import find_packages, setup

package_name = 'x3_plus_api'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'flask'],
    zip_safe=True,
    maintainer='arrgusr',
    maintainer_email='rivascf@gmail.com',
    description='API server for X3 Plus robot, providing Rest endpoints to access and manipulate the robot state and teleoperation.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'api_server = x3_plus_api.api_server:main',
        ],
    },
)
