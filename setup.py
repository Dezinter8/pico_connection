from setuptools import setup

package_name = 'pico_connection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','rclpy','pyserial'],
    zip_safe=True,
    maintainer='newans',
    maintainer_email='josh.newans@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_publisher = pico_connection.serial_publisher:main',
        ],
    },
)
