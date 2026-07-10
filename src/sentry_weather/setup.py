from setuptools import find_packages, setup

package_name = 'sentry_weather'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/weather_params.yaml']),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='team',
    maintainer_email='team@example.com',
    description='Weather data ingestion for Smart Agri Sentry',
    license='MIT',
    entry_points={
        'console_scripts': [
            'weather_node = sentry_weather.weather_node:main',
        ],
    },
)
