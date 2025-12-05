from setuptools import find_packages, setup

package_name = 'dsr_recovery_watcher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='taesla',
    maintainer_email='taesla@todo.todo',
    description='DSR Driver Watcher - 드라이버 충돌/죽음 감시 노드',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'watcher_node = dsr_recovery_watcher.watcher_node:main',
        ],
    },
)
