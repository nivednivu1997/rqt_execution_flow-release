from setuptools import setup

package_name = 'rqt_execution_flow'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'plugin.xml']),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='Nived Krishnan',
    maintainer_email='nivednivu1997@gmail.com',
    url='https://github.com/nivednivu1997/rqt_execution_flow',
    description='Live rqt visualizer for ROS 2 action goals and service calls: call tree, '
                'animated flow, statistics, slow-call flags, record and replay.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'rqt_execution_flow = rqt_execution_flow.main:main',
            'execution_flow_demo = rqt_execution_flow.demo:main',
        ],
    },
)
