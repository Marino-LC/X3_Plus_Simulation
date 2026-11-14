from glob import glob

from setuptools import find_packages, setup

package_name = 'vla_pose_eval'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'numpy', 'PyYAML'],
    zip_safe=True,
    maintainer='robousr',
    maintainer_email='tareas2inge@gmail.com',
    description='Medición del error de estimación de pose de objetos con VLA, sin Nav2.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vla_pose_eval = vla_pose_eval.cli:main',
            'recon_pass = vla_pose_eval.nodes.recon_pass:main',
            'ground_truth_publisher = vla_pose_eval.nodes.ground_truth_publisher:main',
            'estimator_node = vla_pose_eval.nodes.estimator_node:main',
        ],
    },
)
