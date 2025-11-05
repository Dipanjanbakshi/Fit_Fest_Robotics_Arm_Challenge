from setuptools import find_packages, setup

package_name = 'fit_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # packages=find_packages(where='src'),
    package_dir={'': '.'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='utk',
    maintainer_email='bakshidipanjan998@gmail.com',
    description='Color detection package for fit robot vision (RGB object detection)',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'color_detector = fit_vision.color_detector:main',
        ],
    },
)