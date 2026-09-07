from setuptools import find_packages, setup
package_name = 'otolith_viz'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/viz_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Christopher Onyiuke',
    maintainer_email='chrisonyiuke@gmail.com',
    description='Viz helpers',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['viz_node = otolith_viz.viz_node:main']},
)
