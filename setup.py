from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dsr_integrated'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch 파일 포함
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Config 파일 포함
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='taesla',
    maintainer_email='taesla@todo.todo',
    description='Doosan Robot Integrated System - Web Server + Sort Node',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # 리팩토링된 노드
            'sort_node = dsr_integrated.sort_node:main',
            'server_node = dsr_integrated.server_node:main',
            # 테스트 유틸리티
            'test_recovery = dsr_integrated.test_recovery:main',
            # 호환성을 위한 별칭
            'dlar_sort_node = dsr_integrated.sort_node:main',
            'web_server_node = dsr_integrated.server_node:main',
        ],
    },
)
