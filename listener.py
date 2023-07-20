#!/usr/bin/env python
import sys
import cv2
import rospy
import pygame
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from serial import Serial
from std_msgs.msg import String

# port setting
PORT = '/dev/ttyUSB1’

# RS232 connection
ser = Serial(PORT, baudrate=9600, timeout=0)

# Sound Setting
pygame.init()
mySound = pygame.mixer.Sound('AI Sound.wav')
mySound.set_volume(1.0)

# Variable Setting
global time_count, drive_stop, emer_brake, com_data, check_count
global arrive_close, lidar_reset_mask, target_location
lidar_reset_mask= 0xE0
emer_brake = 0x80
arrive_close = 0x40
drive_stop = 0x20
com_data = drive_stop
target_location = 0.0
time_count = 0
check_count = 1

# GUI CLASS function
class Video(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # Set the position and size of the GUI window
        self.setGeometry(10, 10, 1366, 768)
        self.setStyleSheet("background-color:#000000;")
        self.setWindowTitle('배달용 User Interface')
        
        self.vF = QLabel()
        
        self.setCentralWidget(self.vF)
        
        self.button = QPushButton('출발지 : 119호', self)
        self.button.setFont(QFont("Arial", 20, QFont.Bold))
        self.button.setStyleSheet("border-style:outset;"
                                  "background-color:#A6A6A6;"
                                  "border-width:4px;"
                                  "border-color:#221F26;")
        self.button.setGeometry(45, 650, 300, 50)
        self.butSave = QPushButton('도착지', self)
        self.butSave.setFont(QFont("Arial", 20, QFont.Bold))
        self.butSave.setStyleSheet("border-style:outset;"
                                  "background-color:#A6A6A6;"
                                  "border-width:4px;"
                                  "border-color:#221F26;")
        self.butSave.clicked.connect(self.dialog_open)
        self.butSave.setGeometry(345, 650, 300, 50)
        
        self.label_image = QLabel(self)
        self.label_image.setStyleSheet("background-image: url('./CUBIC.png');")
        self.label_image.move(305, 212)
        self.label_image.resize(716, 252)
        
        self.label = QLabel("도착지 : ", self)
        self.label.setStyleSheet("background-color:#A6A198;")
        self.label.setFont(QFont("Arial", 20, QFont.Bold))
        self.label.move(1000, 5)
        self.label.resize(150, 35)
        
        self.now_date = QDate.currentDate()
        self.label_date = QLabel("  ", self)
        self.label_date.setStyleSheet("background-color:#000000;"
                                      "color: white;")
        self.label_date.setFont(QFont("Arial", 20, QFont.Bold))
        self.label_date.setAlignment(Qt.AlignCenter)
        self.label_date.setText(self.now_date.toString(Qt.ISODate))
        self.label_date.move(30, 5)
        self.label_date.resize(250, 35)
        
        self.label_time = QLabel("  ", self)
        self.label_time.setStyleSheet("background-color:#000000;"
                                      "color: white;")
        self.label_time.setFont(QFont("Arial", 20, QFont.Bold))
        self.label_time.setAlignment(Qt.AlignCenter)
        self.label_time.move(30, 45)
        self.label_time.resize(250, 35)
        self.lineEdit = QLineEdit("  ", self)
        self.lineEdit.setStyleSheet("background-color:#A6A198;"
                                    "border:0px;")
        self.lineEdit.setFont(QFont("Arial", 20, QFont.Bold))
        self.lineEdit.setAlignment(Qt.AlignCenter)
        self.lineEdit.move(1100, 5)
        self.lineEdit.resize(160, 35)
        
        self.label = QLabel("현위치 : ", self)
        self.label.setStyleSheet("background-color:#A6A198;")
        self.label.setFont(QFont("Arial", 20, QFont.Bold))
        self.label.move(1000, 45)
        self.label.resize(150, 35)
        self.lineEdit_distance = QLineEdit("  ", self)
        self.lineEdit_distance.setStyleSheet("background-color:#A6A198;"
                                    "border:0px;")
        self.lineEdit_distance.setFont(QFont("Arial", 20, QFont.Bold))
        self.lineEdit_distance.setAlignment(Qt.AlignCenter)
        self.lineEdit_distance.move(1100, 45)
        self.lineEdit_distance.resize(160, 35)
        
        self.label = QLabel("현방향 : ", self)
        self.label.setStyleSheet("background-color:#A6A198;")
        self.label.setFont(QFont("Arial", 20, QFont.Bold))
        self.label.move(1000, 85)
        self.label.resize(150, 35)
        self.lineEdit_angle = QLineEdit("  ", self)
        self.lineEdit_angle.setStyleSheet("background-color:#A6A198;"
                                    "border:0px;")
        self.lineEdit_angle.setFont(QFont("Arial", 20, QFont.Bold))
        self.lineEdit_angle.setAlignment(Qt.AlignCenter)
        self.lineEdit_angle.move(1100, 85)
        self.lineEdit_angle.resize(160, 35)
        self.button = QPushButton('출발', self)
        self.button.setFont(QFont("Arial", 20, QFont.Bold))
        self.button.setStyleSheet("border-style:outset;"
                                  "background-color:#A6A6A6;"
                                  "border-width:4px;"
                                  "border-color:#221F26;")
        self.button.clicked.connect(self.start_dialog)
        self.button.setGeometry(645, 650, 300, 50)
        self.button = QPushButton('수령', self)
        self.button.setFont(QFont("Arial", 20, QFont.Bold))
        self.button.setStyleSheet("border-style:outset;"
                                  "background-color:#A6A6A6;"
                                  "border-width:4px;"
                                  "border-color:#221F26;")
        self.button.clicked.connect(self.finish_dialog)
        self.button.setGeometry(945, 650, 300, 50)
        self.listener()
        self.dialog = QDialog()
        self.msg = QMessageBox()
        
    def ImageUpdateSlot(self, Image):
        self.FeedLabel.setPixmap(QPixmap.fromImage(Image))
    def CancelFeed(self, Image):
        self.FeedLabel.setPixmap(QPixmap.fromImage(Image))
        
    def dialog_open(self): # 버튼 추가
        set_size_font = 20
        btnDialog = QPushButton("118호", self.dialog)
        btnDialog.setFont(QFont("Arial", set_size_font, QFont.Bold))
        btnDialog.move(10, 490)
        btnDialog.clicked.connect(self.dialog_room_118)
        btnDialog = QPushButton("117호", self.dialog)
        btnDialog.setFont(QFont("Arial", set_size_font, QFont.Bold))
        btnDialog.move(10, 430)
        btnDialog.clicked.connect(self.dialog_room_117)
        btnDialog = QPushButton("116호", self.dialog)
        btnDialog.setFont(QFont("Arial", set_size_font, QFont.Bold))
        btnDialog.move(10, 370)
        btnDialog.clicked.connect(self.dialog_room_116)
        btnDialog = QPushButton("115호", self.dialog)
        btnDialog.setFont(QFont("Arial", set_size_font, QFont.Bold))
        btnDialog.move(10, 310)
        btnDialog.clicked.connect(self.dialog_room_115)
        btnDialog = QPushButton("114호", self.dialog)
        btnDialog.setFont(QFont("Arial", set_size_font, QFont.Bold))
        btnDialog.move(10, 250)
        btnDialog.clicked.connect(self.dialog_room_114)
        btnDialog = QPushButton("113호", self.dialog)
        btnDialog.setFont(QFont("Arial", set_size_font, QFont.Bold))
        btnDialog.move(10, 190)
        btnDialog.clicked.connect(self.dialog_room_113)
        btnDialog = QPushButton("112호", self.dialog)
        btnDialog.setFont(QFont("Arial", set_size_font, QFont.Bold))
        btnDialog.move(10, 130)
        btnDialog.clicked.connect(self.dialog_room_112)
        btnDialog = QPushButton("111호", self.dialog)
        btnDialog.setFont(QFont("Arial", set_size_font, QFont.Bold))
        btnDialog.move(10, 70)
        btnDialog.clicked.connect(self.dialog_room_111)
        btnDialog = QPushButton("110호", self.dialog)
        btnDialog.setFont(QFont("Arial", set_size_font, QFont.Bold))
        btnDialog.move(10, 10)
        btnDialog.clicked.connect(self.dialog_room_110)
        btnDialog = QPushButton("103-1호", self.dialog)
        btnDialog.setFont(QFont("Arial", set_size_font, QFont.Bold))
        btnDialog.move(290, 70)
        btnDialog.clicked.connect(self.dialog_room_103_1)
        btnDialog = QPushButton("103호", self.dialog)
        btnDialog.setFont(QFont("Arial", set_size_font, QFont.Bold))
        btnDialog.move(312, 370)
        btnDialog.clicked.connect(self.dialog_room_103)
        btnDialog = QPushButton("cancel", self.dialog)
        btnDialog.setFont(QFont("Arial", set_size_font, QFont.Bold))
        btnDialog.move(150, 550)
        btnDialog.clicked.connect(self.dialog_close)

        # QDialog Setting
        self.center()
        self.dialog.setWindowFlag(Qt.FramelessWindowHint)
        self.dialog.setWindowModality(Qt.ApplicationModal)
        self.dialog.setStyleSheet("background-color:#BDBDBD;")
        self.dialog.resize(400, 600)
        self.dialog.move(520, 80)
        self.dialog.show()
    
    def callback(self, data):
        global time_count, emer_brake, com_data, target_location
        global arrive_close, lidar_reset_mask, dialog_open, check_count
        
        self.now_time = QTime.currentTime()
        self.now_time = self.now_time.toString(Qt.DefaultLocaleLongDate)
        self.label_time.setText(self.now_time[:-4])
        
        # Emergency Brake Option
        if (data.data[5] == 'O'):
            com_data = com_data | emer_brake
        elif (data.data[5] == 'X'):
            com_data = com_data & (~emer_brake)
            
        # Communication Delay Count
        if (time_count == 1):
            time_count = 0
            
            # Communication Data Setting
            com_data = com_data & lidar_reset_mask # reset
            if (int(data.data[1]) <= 3):
                print_data = (4 - int(data.data[1])) * 5
                com_data = com_data | int(data.data[1])
                self.lineEdit_angle.setText("-%s도" % print_data)
            elif (int(data.data[1]) == 4):
                com_data = com_data | int(data.data[1])
                self.lineEdit_angle.setText("0도")
            else:
                print_data = (int(data.data[1]) * 5) - 20
                com_data = com_data | int(data.data[1])
                self.lineEdit_angle.setText("+%s도" % print_data)
            # Camera Data Setting
            camera_data = float(data.data[8 : (len(data.data) - 1)])
            self.lineEdit_distance.setText("%sm" % round(camera_data, 3))
            
            # Arrive Checking
            if ((camera_data >= target_location)):
                com_data = com_data | arrive_close
                if (check_count == 0):
                    # mySound.play(loops=2)
                    check_count += 1
            else:
                com_data = com_data & (~arrive_close)
               
            # data communication
            ser.write(bytes(bytearray([com_data])))
            current_data = ser.read()
                
            print('-' * 60)
            print(' ' * 27, 'transmit data: 0x%x' % com_data)            
            print(' ' * 27, 'PIC current data : %s' % hex(ord(current_data)))
            
            if (int(data.data[1]) <= 3):
                rospy.loginfo('I heard "Angle : -%s도"' % print_data)
            elif (int(data.data[1]) == 4):
                rospy.loginfo('I heard "Angle : 0도"')
            else:
                rospy.loginfo('I heard "Angle : +%s도"' % print_data)
                
            rospy.loginfo('brake point : %s' % data.data[5])
            rospy.loginfo('camera data : %sm' % round(camera_data, 3))
        else:
            time_count += 1
        
    def listener(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("chatter", String, self.callback)
        
    def start_dialog(self):
        global com_data, drive_stop, check_count
        
        self.msg.setIcon(QMessageBox.Information)
        self.msg.setWindowTitle('출발')
        self.msg.setText('출발하시겠습니까?')
        self.msg.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
        retval = self.msg.exec_()
        
        if retval == QMessageBox.Ok:
            com_data = com_data & (~drive_stop)
            check_count = 0
            
    def finish_dialog(self):
        global com_data, drive_stop
        
        self.msg.setIcon(QMessageBox.Information)
        self.msg.setWindowTitle('수령')
        self.msg.setText('수령하셨습니까?')
        self.msg.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
        retval = self.msg.exec_()
        if retval == QMessageBox.Ok:
            self.lineEdit.clear()
            com_data = com_data | drive_stop
    # Dialog Close Event        
    def dialog_room_118(self):
        global target_location, dialog_open
        self.lineEdit.setText("118호")
        dialog_open = 1
        target_location = float(0.0)
        self.lineEdit.setAlignment(Qt.AlignCenter)
        self.dialog.close()
    def dialog_room_117(self):
        global target_location, dialog_open
        self.lineEdit.setText("117호")
        dialog_open = 1
        target_location = float(4.0)
        self.lineEdit.setAlignment(Qt.AlignCenter)
        self.dialog.close()
    def dialog_room_116(self):
        global target_location, dialog_open
        self.lineEdit.setText("116호")
        dialog_open = 1
        target_location = float(7.0)
        self.lineEdit.setAlignment(Qt.AlignCenter)
        self.dialog.close()
    def dialog_room_115(self):
        global target_location, dialog_open
        self.lineEdit.setText("115호")
        dialog_open = 1
        target_location = float(10.0)
        self.lineEdit.setAlignment(Qt.AlignCenter)
        self.dialog.close()
    def dialog_room_114(self):
        global target_location, dialog_open
        self.lineEdit.setText("114호")
        dialog_open = 1
        target_location = float(14.0)
        self.lineEdit.setAlignment(Qt.AlignCenter)
        self.dialog.close()
    def dialog_room_113(self):
        global target_location, dialog_open
        self.lineEdit.setText("113호")
        dialog_open = 1
        target_location = float(17.0)
        self.lineEdit.setAlignment(Qt.AlignCenter)
        self.dialog.close()
    def dialog_room_112(self):
        global target_location, dialog_open
        self.lineEdit.setText("112호")
        dialog_open = 1
        target_location = float(20.0)
        self.lineEdit.setAlignment(Qt.AlignCenter)
        self.dialog.close()
    def dialog_room_111(self):
        global target_location, dialog_open
        self.lineEdit.setText("111호")
        dialog_open = 1
        target_location = float(24.0)
        self.lineEdit.setAlignment(Qt.AlignCenter)
        self.dialog.close()
    def dialog_room_110(self):
        global target_location, dialog_open
        self.lineEdit.setText("110호")
        dialog_open = 1
        target_location = float(27.0)
        self.lineEdit.setAlignment(Qt.AlignCenter)
        self.dialog.close()
    def dialog_room_103_1(self):
        global target_location, dialog_open
        self.lineEdit.setText("103-1호")
        dialog_open = 1
        target_location = float(28.5)
        self.lineEdit.setAlignment(Qt.AlignCenter)
        self.dialog.close()
    def dialog_room_103(self):
        global target_location, dialog_open
        self.lineEdit.setText("103호")
        dialog_open = 1
        target_location = float(7.5)
        self.lineEdit.setAlignment(Qt.AlignCenter)
        self.dialog.close()
    def dialog_close(self):
        self.dialog.close()
        # 창 센터 맞추기
    def center(self):
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())

if __name__ == '__main__':
    print('port name :', ser.portstr)
    app = QApplication(sys.argv)
    win = Video()
    win.center()
    win.showMaximized()
    sys.exit(app.exec_())