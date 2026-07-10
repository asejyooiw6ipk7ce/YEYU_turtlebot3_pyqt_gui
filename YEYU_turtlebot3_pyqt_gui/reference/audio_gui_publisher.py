#!/usr/bin/env python3

import sys

import rclpy
from rclpy.node import Node

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QTextEdit,
    QPushButton,
    QLabel,
    QVBoxLayout,
    QHBoxLayout,
    QComboBox,
    QDoubleSpinBox,
    QSpinBox,
    QMessageBox,
)
from robot_audio_interfaces.msg import AudioCommand               # 음성명령데이터 AudioCommand msg타입


class AudioCommandPublisher(Node):
    def __init__(self):
        super().__init__('audio_gui_publisher_node')

        # topic_name 파라미터 선언
        self.declare_parameter('topic_name', '/audio/command')    # 명령어 -topic_name:=값 데려와서 topic_name.value에 저장되도록 함
        self.topic_name = self.get_parameter('topic_name').value  # topic_name.value값 -> self.topic_name

        # /audio/command 발행자 생성 ; 변형) 명령어 -topic_name:= 뒤에 원하는 토픽명 정할 수 잇음
        self.publisher = self.create_publisher(
            AudioCommand,
            self.topic_name,
            10
        )

        # 노드 정상 시작, self.topic_name토픽명 로그 출력
        self.get_logger().info(f'Audio GUI publisher started')
        self.get_logger().info(f'Publish topic: {self.topic_name}')

    # 발행할 msg만들기
    def publish_command(self, command_type, text='', sound_id='', volume=1.0, repeat=1 ):
        msg = AudioCommand()        # 주문서 양식: AudioCommand()
        msg.type = command_type     # 매개변수로 받은 값들 msg에 채워넣기
        msg.text = text
        msg.sound_id = sound_id
        msg.volume = float(volume)
        msg.repeat = int(repeat)

        # msg 발행
        self.publisher.publish(msg)

        self.get_logger().info(
            f'Published AudioCommand '
            f'type={msg.type}, text="{msg.text}", '
            f'sound_id="{msg.sound_id}", volume={msg.volume}, repeat={msg.repeat}'
        )


class AudioGuiWindow(QWidget):
    def __init__(self, ros_node: AudioCommandPublisher):
        super().__init__()

        self.ros_node = ros_node

        '''
        # GUI창 만들기
        self.setWindowTitle('TurtleBot3 Audio Command GUI')
        self.resize(560, 420)

        self.title_label = QLabel('TurtleBot3 원격 음성 출력 GUI')

        self.text_edit = QTextEdit()
        self.text_edit.setPlaceholderText(
            'TurtleBot3 USB 스피커로 출력할 문장을 입력하세요.'
        )

        self.sound_combo = QComboBox()
        self.sound_combo.addItem('효과음 없음', '')
        self.sound_combo.addItem('start', 'start')
        self.sound_combo.addItem('waypoint', 'waypoint')
        self.sound_combo.addItem('goal', 'goal')
        self.sound_combo.addItem('warning', 'warning')
        self.sound_combo.addItem('error', 'error')

        self.volume_spin = QDoubleSpinBox()
        self.volume_spin.setRange(0.0, 1.0)
        self.volume_spin.setSingleStep(0.1)
        self.volume_spin.setValue(1.0)

        self.repeat_spin = QSpinBox()
        self.repeat_spin.setRange(1, 10)
        self.repeat_spin.setValue(1)

        self.tts_button = QPushButton('TTS 출력')
        self.effect_button = QPushButton('효과음 출력')
        self.tts_effect_button = QPushButton('효과음 + TTS 출력')
        self.stop_button = QPushButton('재생 정지')
        self.clear_button = QPushButton('내용 지우기')

        self.status_label = QLabel('대기 중')

        sound_layout = QHBoxLayout()
        sound_layout.addWidget(QLabel('효과음:'))
        sound_layout.addWidget(self.sound_combo)

        option_layout = QHBoxLayout()
        option_layout.addWidget(QLabel('볼륨:'))
        option_layout.addWidget(self.volume_spin)
        option_layout.addWidget(QLabel('반복:'))
        option_layout.addWidget(self.repeat_spin)

        button_layout_1 = QHBoxLayout()
        button_layout_1.addWidget(self.tts_button)
        button_layout_1.addWidget(self.effect_button)

        button_layout_2 = QHBoxLayout()
        button_layout_2.addWidget(self.tts_effect_button)
        button_layout_2.addWidget(self.stop_button)
        button_layout_2.addWidget(self.clear_button)

        main_layout = QVBoxLayout()
        main_layout.addWidget(self.title_label)
        main_layout.addWidget(self.text_edit)
        main_layout.addLayout(sound_layout)
        main_layout.addLayout(option_layout)
        main_layout.addLayout(button_layout_1)
        main_layout.addLayout(button_layout_2)
        main_layout.addWidget(self.status_label)

        self.setLayout(main_layout)
        '''

        # 시그널 -> 슬롯 함수 연결
        self.tts_button.clicked.connect(self.publish_tts)                    # tts_button 클릭 -> 2. publish_tts 실행
        self.effect_button.clicked.connect(self.publish_effect)              # effect_button 클릭 -> 3. publish_effect 실행
        self.tts_effect_button.clicked.connect(self.publish_tts_and_effect)  # tts_effext_button 클릭 -> 4. publish_tts_and_effect 실행
        self.stop_button.clicked.connect(self.publish_stop)                  # stop_button 클릭 -> 5. publish_stop 실행
        self.clear_button.clicked.connect(self.clear_text)                   # clear_button 클릭 -> 6. clear_text 실행

        # spin_timer 0.02초마다 울리면 -> 1. spin_ros_once() 실행
        self.spin_timer = QTimer(self)
        self.spin_timer.timeout.connect(self.spin_ros_once)
        self.spin_timer.start(20)

    # 1. spin_timer 울릴 때마다 콜백
    def spin_ros_once(self):
        rclpy.spin_once(self.ros_node, timeout_sec=0.0)                  

    # 2-1. publish_tts() 내부에 호출
    def get_text(self):
        return self.text_edit.toPlainText().strip()         # self.text_edit에 적힌 글자들을 가져와서 공백 제거 후 반환

    # 3-1. publish_effect() 내부에 호출
    def get_sound_id(self):
        return self.sound_combo.currentData()               # self.sound_combo(사운드ID 콤보박스)에 선택된 ID값 반환

    # 4-1. publish_tts_and_effect() 내부에 호출
    def get_volume(self):
        return self.volume_spin.value()                     # self.volume_spin(볼륨 스핀박스)에 있는 숫자 반환

    # 4-2. publish_tts_and_effect() 내부에 호출
    def get_repeat(self):
        return self.repeat_spin.value()                     # self.repeat_spin(반복횟수 스핀박스)에 있는 숫자 반환

    # 2. tts_button 클릭 시그널의 슬롯 함수
    def publish_tts(self):
        text = self.get_text()                             # 입력받은 텍스트(self.get_text) -> text

        # 입력받은 게 없으면
        if not text:
            QMessageBox.warning(
                self,
                '입력 오류',
                'TTS로 출력할 문장을 입력하세요.'
            )
            return

        # ros_node에서 publish_command(선택한 값) 호출
        self.ros_node.publish_command(
            command_type=AudioCommand.TYPE_TTS,
            text=text,
            sound_id='',
            volume=self.get_volume(),
            repeat=self.get_repeat()
        )
        self.status_label.setText('TTS 명령 발행 완료')

    # 3. effect_button 클릭 시그널의 슬롯 함수
    def publish_effect(self):
        sound_id = self.get_sound_id()                      # 입력받은 ID값(self.get_sound_id) -> sound_id 

        # 입력받은 게 없으면
        if not sound_id:
            QMessageBox.warning(
                self,
                '선택 오류',
                '출력할 효과음을 선택하세요.'
            )
            return

        # ros_node에서 publish_command(선택한 값) 호출
        self.ros_node.publish_command(
            command_type=AudioCommand.TYPE_EFFECT,
            text='',
            sound_id=sound_id,
            volume=self.get_volume(),
            repeat=self.get_repeat()
        )
        self.status_label.setText(f'효과음 명령 발행 완료: {sound_id}')

    # 4. tts_effext_button 클릭 시그널의 슬롯 함수
    def publish_tts_and_effect(self):
        text = self.get_text()                               # 입력받은 텍스트(self.get_text) -> text
        sound_id = self.get_sound_id()                       # 입력받은 ID값(self.get_sound_id) -> sound_id 

        # 입력 받은 게 없으면
        if not text:
            QMessageBox.warning(
                self,
                '입력 오류',
                'TTS로 출력할 문장을 입력하세요.'
            )
            return
        if not sound_id:
            QMessageBox.warning(
                self,
                '선택 오류',
                '먼저 출력할 효과음을 선택하세요.'
            )
            return

        # ros_node에서 publish_command(선택한 값) 호출
        self.ros_node.publish_command(
            command_type=AudioCommand.TYPE_TTS_AND_EFFECT,
            text=text,
            sound_id=sound_id,
            volume=self.get_volume(),
            repeat=self.get_repeat()
        )
        self.status_label.setText('효과음 + TTS 명령 발행 완료')

    # 5. stop_button 클릭 시그널의 슬롯 함수
    def publish_stop(self):
        # ros_node에서 publish_command(선택한 값) 호출
        self.ros_node.publish_command(
            command_type=AudioCommand.TYPE_STOP,
            text='',
            sound_id='',
            volume=1.0,
            repeat=1
        )
        self.status_label.setText('재생 정지 명령 발행 완료')

    # 6. clear_button 클릭 시그널의 슬롯 함수
    def clear_text(self):
        self.text_edit.clear()       # self.text_edit 내용 초기화
        self.status_label.setText('내용 삭제 완료')

    
    def closeEvent(self, event):
        self.spin_timer.stop()
        event.accept()


def main(args=None):
    rclpy.init(args=args)

    ros_node = AudioCommandPublisher()

    app = QApplication(sys.argv)
    window = AudioGuiWindow(ros_node)
    window.show()

    exit_code = app.exec_()

    ros_node.destroy_node()
    rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()