from setuptools import setup

package_name = 'ei_fetcher'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/camera_and_detectors.launch.py']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mugambi Timothy Mwenda, Roland Sándor Nagy',
    maintainer_email='yc8eu4@inf.elte.hu, newageson@inf.elte.hu',
    description='CreativeCamera: tennis ball (HSV) + people (YOLO ONNX) detectors for ROS 2 Humble.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ball_tracker_rgb = ei_fetcher.ball_tracker_rgb:main',
            'people_detector = ei_fetcher.people_detector:main',
            'video_publisher = ei_fetcher.video_publisher:main',
            'detection_overlay = ei_fetcher.detection_overlay:main',
        ],
    },
)
