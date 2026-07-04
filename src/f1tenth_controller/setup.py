from setuptools import find_packages, setup

package_name = 'f1tenth_controller'

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
    maintainer='steevens',
    maintainer_email='mscueva@espol.edu.ec',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'follow_the_gap = f1tenth_controller.follow_the_gap:main',
            'lap_timer = f1tenth_controller.lap_timer:main',
        ],
    },
)
