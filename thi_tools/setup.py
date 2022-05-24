from setuptools import setup

package_name = 'thi_tools'

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
    maintainer='m1ch1',
    maintainer_email='m4ffle@googlemail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'time_diff_now_node = thi_tools.time_diff_now:main',
          "map_repub_node = thi_tools.map_repub:main",
          "img_repub_diff_qos_node = thi_tools.img_repub_diff_qos:main"
        ],
    },
)
