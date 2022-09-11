from setuptools import setup

package_name = 'ipico_drvs'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RasiewF',
    maintainer_email='filiprasiewicz@gmail.com',
    description='PICO Drivers interface package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ipico_node = ipico_drvs.ipico_drvs:main',
        ],
    },
)
