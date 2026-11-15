from setuptools import find_packages, setup

package_name = 'python_pkg'

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
    maintainer='shayo',
    maintainer_email='shayansetayandevolleyking5@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'template = python_pkg.template:main',
            'python_publisher_node = python_pkg.python_publisher:main',
            'python_subscriber_node = python_pkg.python_subscriber:main',
            'python_server_node = python_pkg.python_server:main',
            'python_client_node = python_pkg.python_client:main',
            'python_client_empty_node = python_pkg.python_client_empty:main',
            'python_action_server_node = python_pkg.python_action_server:main',
            'python_action_clinet_node = python_pkg.python_action_clinet:main',
            'tf_listener_node = python_pkg.tf_listener_node:main',
            'arm_controller_node = python_pkg.arm_controller:main',
        ],
    },
)
