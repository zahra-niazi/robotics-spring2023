from setuptools import setup

package_name = 'alicesim'

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
    maintainer='zahra',
    maintainer_email='zahra@todo.todo',
    description='Alice Package, a listener.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'listener = alicesim.alicenode:main',
        ],
    },
)
