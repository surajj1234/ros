# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'gui_form.ui'
#
# Created: Sun Jan 25 13:43:13 2015
#      by: PyQt4 UI code generator 4.10.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(1059, 581)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.vrLabel = QtGui.QLabel(self.centralwidget)
        self.vrLabel.setGeometry(QtCore.QRect(817, 400, 111, 20))
        self.vrLabel.setObjectName(_fromUtf8("vrLabel"))
        self.mamaRooLabel = QtGui.QLabel(self.centralwidget)
        self.mamaRooLabel.setGeometry(QtCore.QRect(817, 440, 111, 20))
        self.mamaRooLabel.setObjectName(_fromUtf8("mamaRooLabel"))
        self.origamiLabel = QtGui.QLabel(self.centralwidget)
        self.origamiLabel.setGeometry(QtCore.QRect(817, 480, 111, 20))
        self.origamiLabel.setObjectName(_fromUtf8("origamiLabel"))
        self.demoComboBox = QtGui.QComboBox(self.centralwidget)
        self.demoComboBox.setGeometry(QtCore.QRect(940, 220, 81, 27))
        self.demoComboBox.setObjectName(_fromUtf8("demoComboBox"))
        self.demoComboBox.addItem(_fromUtf8(""))
        self.demoComboBox.addItem(_fromUtf8(""))
        self.demoComboBox.addItem(_fromUtf8(""))
        self.label = QtGui.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(850, 230, 91, 20))
        self.label.setObjectName(_fromUtf8("label"))
        self.babyImageLabel = QtGui.QLabel(self.centralwidget)
        self.babyImageLabel.setGeometry(QtCore.QRect(80, 20, 640, 480))
        self.babyImageLabel.setObjectName(_fromUtf8("babyImageLabel"))
        self.babyStateLabel = QtGui.QLabel(self.centralwidget)
        self.babyStateLabel.setGeometry(QtCore.QRect(830, 50, 181, 61))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.babyStateLabel.setFont(font)
        self.babyStateLabel.setObjectName(_fromUtf8("babyStateLabel"))
        self.label_4 = QtGui.QLabel(self.centralwidget)
        self.label_4.setGeometry(QtCore.QRect(840, 120, 58, 17))
        self.label_4.setObjectName(_fromUtf8("label_4"))
        self.timeoutLabel = QtGui.QLabel(self.centralwidget)
        self.timeoutLabel.setGeometry(QtCore.QRect(920, 120, 58, 17))
        self.timeoutLabel.setObjectName(_fromUtf8("timeoutLabel"))
        self.debugImageButton = QtGui.QPushButton(self.centralwidget)
        self.debugImageButton.setGeometry(QtCore.QRect(810, 300, 87, 27))
        self.debugImageButton.setObjectName(_fromUtf8("debugImageButton"))
        self.thresholdSlider = QtGui.QSlider(self.centralwidget)
        self.thresholdSlider.setGeometry(QtCore.QRect(810, 340, 160, 21))
        self.thresholdSlider.setProperty("value", 50)
        self.thresholdSlider.setOrientation(QtCore.Qt.Horizontal)
        self.thresholdSlider.setObjectName(_fromUtf8("thresholdSlider"))
        self.thresholdLabel = QtGui.QLabel(self.centralwidget)
        self.thresholdLabel.setGeometry(QtCore.QRect(990, 340, 58, 17))
        self.thresholdLabel.setObjectName(_fromUtf8("thresholdLabel"))
        self.vrStatusLabel = QtGui.QLabel(self.centralwidget)
        self.vrStatusLabel.setGeometry(QtCore.QRect(960, 400, 91, 17))
        self.vrStatusLabel.setObjectName(_fromUtf8("vrStatusLabel"))
        self.mamaRooStatusLabel = QtGui.QLabel(self.centralwidget)
        self.mamaRooStatusLabel.setGeometry(QtCore.QRect(960, 440, 91, 17))
        self.mamaRooStatusLabel.setObjectName(_fromUtf8("mamaRooStatusLabel"))
        self.origamiStatusLabel = QtGui.QLabel(self.centralwidget)
        self.origamiStatusLabel.setGeometry(QtCore.QRect(960, 480, 91, 17))
        self.origamiStatusLabel.setObjectName(_fromUtf8("origamiStatusLabel"))
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1059, 27))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(MainWindow)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow", None))
        self.vrLabel.setText(_translate("MainWindow", "VR Status", None))
        self.mamaRooLabel.setText(_translate("MainWindow", "mamaRoo Status", None))
        self.origamiLabel.setText(_translate("MainWindow", "Origami Status", None))
        self.demoComboBox.setItemText(0, _translate("MainWindow", "Monitor", None))
        self.demoComboBox.setItemText(1, _translate("MainWindow", "ML", None))
        self.demoComboBox.setItemText(2, _translate("MainWindow", "VR", None))
        self.label.setText(_translate("MainWindow", "Demo", None))
        self.babyImageLabel.setText(_translate("MainWindow", "TextLabel", None))
        self.babyStateLabel.setText(_translate("MainWindow", "Baby Sleeping", None))
        self.label_4.setText(_translate("MainWindow", "Stop in", None))
        self.timeoutLabel.setText(_translate("MainWindow", "0", None))
        self.debugImageButton.setText(_translate("MainWindow", "Debug Image", None))
        self.thresholdLabel.setText(_translate("MainWindow", "TextLabel", None))
        self.vrStatusLabel.setText(_translate("MainWindow", "TextLabel", None))
        self.mamaRooStatusLabel.setText(_translate("MainWindow", "TextLabel", None))
        self.origamiStatusLabel.setText(_translate("MainWindow", "TextLabel", None))

