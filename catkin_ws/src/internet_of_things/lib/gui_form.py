# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'gui_form.ui'
#
# Created: Sun Jan 25 02:09:42 2015
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
        self.vrStatusLabel = QtGui.QLabel(self.centralwidget)
        self.vrStatusLabel.setGeometry(QtCore.QRect(817, 400, 111, 20))
        self.vrStatusLabel.setObjectName(_fromUtf8("vrStatusLabel"))
        self.label_2 = QtGui.QLabel(self.centralwidget)
        self.label_2.setGeometry(QtCore.QRect(817, 440, 111, 20))
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.label_3 = QtGui.QLabel(self.centralwidget)
        self.label_3.setGeometry(QtCore.QRect(817, 480, 111, 20))
        self.label_3.setObjectName(_fromUtf8("label_3"))
        self.vrStatusLabel_2 = QtGui.QLabel(self.centralwidget)
        self.vrStatusLabel_2.setGeometry(QtCore.QRect(950, 400, 91, 17))
        self.vrStatusLabel_2.setObjectName(_fromUtf8("vrStatusLabel_2"))
        self.mamaRooStatusLabel = QtGui.QLabel(self.centralwidget)
        self.mamaRooStatusLabel.setGeometry(QtCore.QRect(950, 440, 91, 17))
        self.mamaRooStatusLabel.setObjectName(_fromUtf8("mamaRooStatusLabel"))
        self.origamiStatusLabel = QtGui.QLabel(self.centralwidget)
        self.origamiStatusLabel.setGeometry(QtCore.QRect(950, 480, 91, 17))
        self.origamiStatusLabel.setObjectName(_fromUtf8("origamiStatusLabel"))
        self.demoComboBox = QtGui.QComboBox(self.centralwidget)
        self.demoComboBox.setGeometry(QtCore.QRect(920, 240, 81, 27))
        self.demoComboBox.setObjectName(_fromUtf8("demoComboBox"))
        self.demoComboBox.addItem(_fromUtf8(""))
        self.demoComboBox.addItem(_fromUtf8(""))
        self.label = QtGui.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(830, 250, 91, 20))
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
        self.debugImageButton.setGeometry(QtCore.QRect(840, 150, 87, 27))
        self.debugImageButton.setObjectName(_fromUtf8("debugImageButton"))
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
        self.vrStatusLabel.setText(_translate("MainWindow", "VR Status", None))
        self.label_2.setText(_translate("MainWindow", "mamaRoo Status", None))
        self.label_3.setText(_translate("MainWindow", "Origami Status", None))
        self.vrStatusLabel_2.setText(_translate("MainWindow", "Disconnected", None))
        self.mamaRooStatusLabel.setText(_translate("MainWindow", "Disconnected", None))
        self.origamiStatusLabel.setText(_translate("MainWindow", "Disconnected", None))
        self.demoComboBox.setItemText(0, _translate("MainWindow", "ML", None))
        self.demoComboBox.setItemText(1, _translate("MainWindow", "VR", None))
        self.label.setText(_translate("MainWindow", "Demo", None))
        self.babyImageLabel.setText(_translate("MainWindow", "TextLabel", None))
        self.babyStateLabel.setText(_translate("MainWindow", "Baby Sleeping", None))
        self.label_4.setText(_translate("MainWindow", "Stop in", None))
        self.timeoutLabel.setText(_translate("MainWindow", "0", None))
        self.debugImageButton.setText(_translate("MainWindow", "Debug Image", None))

