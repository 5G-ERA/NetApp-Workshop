from setuptools import setup

package_name = 'ros2_5g_era_basic_example'

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
    maintainer='ispanhel',
    maintainer_email='ispanhel@fit.vutbr.cz',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_publisher = ros2_5g_era_basic_example.basic_image_publisher:main',
            'result_listener = ros2_5g_era_basic_example.basic_result_listener:main',
            'ml_service = ros2_5g_era_basic_example.ml_service:main'
        ],
    },
)
