from setuptools import setup

package_name = 'sentry_miniprogram'

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
    maintainer='wjun',
    maintainer_email='wjun@example.com',
    description='Mini-program bridge node for WeChat mini-program',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'miniprogram_bridge_node = sentry_miniprogram.miniprogram_bridge_node:main',
        ],
    },
)
