from setuptools import setup

package_name = 'all_in_one_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@example.com',
    description='The all_in_one_package package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 여기에 현재 패키지의 실행 파일들만 추가
        ],
    },
)
