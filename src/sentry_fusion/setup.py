from setuptools import find_packages, setup

package_name = 'sentry_fusion'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team',
    maintainer_email='team@example.com',
    description='Fusion node for Smart Agri Sentry v2.0',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fusion_node = sentry_fusion.fusion_node:main',
        ],
    },
)
