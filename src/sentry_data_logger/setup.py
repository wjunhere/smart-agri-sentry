from setuptools import find_packages, setup

package_name = 'sentry_data_logger'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/data_logger_params.yaml']),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='team',
    maintainer_email='team@example.com',
    description='Data logger node for Smart Agri Sentry v2.0',
    license='MIT',
    entry_points={
        'console_scripts': [
            'data_logger_node = sentry_data_logger.data_logger_node:main',
        ],
    },
)
