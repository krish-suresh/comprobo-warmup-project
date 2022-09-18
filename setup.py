from setuptools import setup

package_name = 'comprobo_warmup_project'

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
    maintainer='ksuresh',
    maintainer_email='krishna4132@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop = comprobo_warmup_project.teleop:main',
            'drive_square = comprobo_warmup_project.drive_square:main',
            'marker_test = comprobo_warmup_project.marker_test_node:main',
            'wall_follower = comprobo_warmup_project.wall_follower:main',
            'person_follower = comprobo_warmup_project.person_follower:main',
            'obstacle_avoider = comprobo_warmup_project.obstacle_avoider:main',
        ],
    },
)
