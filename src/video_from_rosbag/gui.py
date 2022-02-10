from PyQt4 import QtGui, QtCore
import sys
import os


class GUI(object):
    def __init__(self):
        self.app = QtGui.QApplication(sys.argv)

    def select_bag(self):
        files = QtGui.QFileDialog.getOpenFileNames(caption="Select bag file", directory=os.path.expanduser("~"), filter="*bag")
        while len(files) > 1:
            files = QtGui.QFileDialog.getOpenFileNames(caption="Select bag file", directory=os.path.expanduser("~"), filter="*bag")

        if len(files) == 0:
            sys.exit(0)

        bag_path = str(files[0])

        return bag_path

    def select_topics(self, image_topic_list):
        selected_image_topics = {}

        win = QtGui.QWidget()
        scrollArea = QtGui.QScrollArea()
        scrollArea.setWidgetResizable(True)
        scrollAreaWidgetContents = QtGui.QWidget(scrollArea)
        scrollAreaWidgetContents.setGeometry(QtCore.QRect(0, 0, 380, 247))
        scrollArea.setWidget(scrollAreaWidgetContents)
        layout = QtGui.QGridLayout()
        verticalLayoutScroll = QtGui.QVBoxLayout(scrollAreaWidgetContents)
        layoutIndex = 0

        checkboxes = []
        for select in image_topic_list:
            checkbox = QtGui.QCheckBox(select)
            verticalLayoutScroll.addWidget(checkbox)
            layoutIndex += 1
            checkboxes.append(checkbox)

        layout.addWidget(scrollArea)
        btn = QtGui.QPushButton("OK")
        btn.clicked.connect(self.app.quit)
        layout.addWidget(btn, layoutIndex, 0)
        layoutIndex += 1

        win.setLayout(layout)
        win.setWindowTitle("Select image topic")
        win.show()
        self.app.exec_()

        result = {}
        for (checkbox, select) in zip(checkboxes, image_topic_list):
            result[select] = checkbox.isChecked()

        for k, v in result.items():
            if v:
                selected_image_topics[k] = image_topic_list[k]

        return selected_image_topics
