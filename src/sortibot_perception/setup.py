from setuptools import setup

package_name = 'sortibot_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # install models directory
        ('share/' + package_name + '/models', [
            'models/model.tflite',
            'models/labels.txt'
        ]),
        # we will create a launch file later
        ('share/' + package_name + '/launch', [
            'launch/perception.launch.py'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mansi',
    maintainer_email='mansi@example.com',
    description='Perception node for SortiBot using Edge Impulse TFLite model',
    license='TODO: License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception_node = sortibot_perception.perception_node:main'
        ],
    },
)
