from setuptools import find_packages, setup

package_name = 'mavsim_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Brandon Sutherland',
    maintainer_email='bsuther2@byu.edu',
    description='Bridge for using mavsim with ROSflight',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'mavsim_bridge = mavsim_bridge.mavsim_bridge:main'
        ],
    },
)
