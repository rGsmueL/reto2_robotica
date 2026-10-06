from setuptools import find_packages, setup

package_name = 'arm_broker'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='equipo',
    maintainer_email='equipo@esan.edu.pe',
    description='Broker de acceso exclusivo al JetCobot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'broker = arm_broker.broker:main',
            'cliente = arm_broker.cliente:main',
        ],
    },
)
