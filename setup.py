from setuptools import setup

package_name = 'mmikulic_2'

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
    maintainer='Mateo Mikulic',
    maintainer_email='mmikulic@riteh.hr',
    description='2. Domaca zadaca',
    license='GPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scan_subscriber = mmikulic_2.scan_subscriber:main'
        ],
    },
)
