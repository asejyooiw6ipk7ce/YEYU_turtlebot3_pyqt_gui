from setuptools import find_packages, setup

package_name = 'YEYU_turtlebot3_pyqt_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # .ui 파일도 함께 설치되도록 추가했습니다.
        # (전에는 이 줄이 없어서, colcon build로 설치하면 화면(.ui)을 못 찾는 문제가 있었습니다)
        ('share/' + package_name, ['package.xml', 'resource/turtlebot3_pyqt_gui2.ui']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ktel',
    maintainer_email='asejyooiw6ipk7ce@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        # 지금은 python3 main.py로 직접 실행하고 있어서 이 항목은 당장 쓰지 않습니다.
        # 나중에 "ros2 run YEYU_turtlebot3_pyqt_gui gui"로 실행하고 싶어지면,
        # main.py/gui.py/ros_node.py의 import를 다시 "from .모듈이름 import ..." 형태로
        # 바꿔야 정상 동작합니다 (지금은 점 없는 import라서 이 항목은 동작하지 않습니다).
        'console_scripts': [
            'gui = YEYU_turtlebot3_pyqt_gui.main:main',
        ],
    },
)
