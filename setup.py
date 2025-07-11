from setuptools import setup

package_name = 'create3_control'

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
    maintainer='juanjg',
    maintainer_email='juanjg@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'create3_control = create3_control.create3_gui_nav:main',
            'xbox_teleop = create3_control.xbox_teleop:main',
            'page = create3_control.page:main', 
        ],
    },
)
