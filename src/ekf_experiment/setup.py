from setuptools import setup

package_name = 'ekf_experiment'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/run_experiment.launch.py', 'launch/experiment.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Umut Can Gulmez',
    maintainer_email='umut@example.com',
    description='ES-EKF Experiment Framework',
    license='MIT',
    entry_points={
        'console_scripts': [
            'experiment_executor = ekf_experiment.experiment_executor:main',
            'gz_tf_bridge = ekf_experiment.gz_tf_bridge:main',
        ],
    },
)