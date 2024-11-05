from setuptools import setup

package_name = 'ffmpeg_to_foxglove_video'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jan Vojnar',
    maintainer_email='Jan.Vojnar@cvut.cz',
    description='Tool to convert rosbag with ffmpeg camera topic to foxglove compressed video topic.',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'convert = ffmpeg_to_foxglove_video.convert:main',
        ],
    },
)