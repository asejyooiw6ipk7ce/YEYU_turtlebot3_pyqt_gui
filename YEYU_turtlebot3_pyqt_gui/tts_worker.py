#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tts_worker.py
gTTS 기반 음성 출력을 백그라운드 스레드에서 처리합니다.
GUI가 멈추지 않도록 gTTS 변환 + 오디오 재생을 QThread에서 수행합니다.

필요 패키지: gTTS
    pip install gTTS
재생은 시스템에 설치된 mpg123 / ffplay / afplay(mac) 중 하나를 사용합니다.
    sudo apt install mpg123
"""

import os
import shutil
import subprocess
import tempfile

from PyQt5.QtCore import QThread, pyqtSignal

try:
    from gtts import gTTS
    GTTS_AVAILABLE = True
except ImportError:
    GTTS_AVAILABLE = False


class TTSWorker(QThread):
    finished_ok = pyqtSignal(str)     # 성공 시 출력한 문장 전달
    finished_error = pyqtSignal(str)  # 실패 시 에러 메시지 전달

    def __init__(self, text, lang='ko'):
        super().__init__()
        self.text = text
        self.lang = lang

    def run(self):
        if not self.text.strip():
            self.finished_error.emit('출력할 텍스트가 비어 있습니다.')
            return
        if not GTTS_AVAILABLE:
            self.finished_error.emit('gTTS 패키지가 설치되어 있지 않습니다. (pip install gTTS)')
            return

        tmp_path = None
        try:
            tmp_fd, tmp_path = tempfile.mkstemp(suffix='.mp3')
            os.close(tmp_fd)

            tts = gTTS(text=self.text, lang=self.lang)
            tts.save(tmp_path)

            player = self._find_player()
            if player is None:
                self.finished_error.emit(
                    '오디오 재생 프로그램을 찾을 수 없습니다. (mpg123, ffplay, afplay 중 하나 설치 필요)')
                return

            self._play(player, tmp_path)
            self.finished_ok.emit(self.text)
        except Exception as e:  # noqa: BLE001
            self.finished_error.emit(f'TTS 처리 중 오류: {e}')
        finally:
            if tmp_path and os.path.exists(tmp_path):
                try:
                    os.remove(tmp_path)
                except OSError:
                    pass

    @staticmethod
    def _find_player():
        for candidate in ('mpg123', 'ffplay', 'afplay', 'mpv'):
            if shutil.which(candidate):
                return candidate
        return None

    @staticmethod
    def _play(player, path):
        if player == 'ffplay':
            cmd = ['ffplay', '-nodisp', '-autoexit', '-loglevel', 'quiet', path]
        elif player == 'mpg123':
            cmd = ['mpg123', '-q', path]
        elif player == 'afplay':
            cmd = ['afplay', path]
        else:  # mpv
            cmd = ['mpv', '--no-video', '--really-quiet', path]
        subprocess.run(cmd, check=False)
