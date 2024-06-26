from setuptools import setup

package_name = 'camera_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iker',
    maintainer_email='A01749675@tec.mx',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['camera=camera_manager.camera_node:main',
                            'single_cam=camera_manager.single_camera:main',
                            'science_camera=camera_manager.science_cameras:main'
        ],
    },
)
