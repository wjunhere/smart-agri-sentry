from setuptools import find_packages, setup

package_name = 'sentry_advisory'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/advisory_rules.yaml']),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='team',
    maintainer_email='team@example.com',
    description='Advisory node for Smart Agri Sentry v2.0',
    license='MIT',
    entry_points={
        'console_scripts': [
            'advisory_node = sentry_advisory.advisory_node:main',
        ],
    },
)
