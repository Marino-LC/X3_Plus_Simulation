from setuptools import find_packages, setup

package_name = 'owlv2_detector'

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
    maintainer='arrusr',
    maintainer_email='arrg.unam@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'owlv2_node = owlv2_detector.owlv2_node:main',
            'view_saver = owlv2_detector.view_saver:main',
        ],
    },
)
