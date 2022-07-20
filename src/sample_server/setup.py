from setuptools import setup

package_name = 'sample_server'

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
    maintainer='matthewrhdu',
    maintainer_email='matthewrhdu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'server = sample_server.server:main',
            'client = sample_server.client:main',
            'first = sample_server.first:main',
            'second = sample_server.second:main',
        ],
    },
)
