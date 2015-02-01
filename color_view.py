try:
    from PySide.QtCore import *
    from PySide.QtGui import *
except:
    from PyQt4.QtGui import *
    from PyQt4.QtCore import *

import re
import signal
signal.signal(signal.SIGINT, signal.SIG_DFL)


class ReadSerialThread(QThread):
    def run(self):
        while True:
            f = open('/dev/ttyACM0')

            for line in f:
                m = re.match(r'r = (\d+)  g = (\d+)  b = (\d+)', line)

                if m:
                    r, g, b = [int(_) // 4 for _ in m.groups()]
                    w.setPalette(QPalette(QColor((r << 16) | (g << 8) | b)))

app = QApplication([])
w = QWidget()
w.setFixedSize(QSize(500, 500))
w.setGeometry(
    QStyle.alignedRect(Qt.LeftToRight, Qt.AlignCenter, w.size(),
                       app.desktop().availableGeometry()))
w.show()

th = ReadSerialThread()
th.start()

app.exec_()
