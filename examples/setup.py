from setuptools import setup
import os
from glob import glob

package_name = 'examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name), glob('launch/*_launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='george',
    maintainer_email='george@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "publisher = examples.publisher:main",
            "subscriber = examples.subscriber:main",
            "service = examples.service:main",
            "client = examples.client:main",
            "param = examples.parameter:main",
            "action_server = examples.action_server:main",
            "action_client = examples.action_client:main",
            "test = examples.test:main"
        ],
    },
)
