import os
import rospy
import rospkg
import math 
  
from python_qt_binding import loadUi
#from python_qt_binding.QtCore import QAbstractListModel, QFile, QIODevice, Qt, Signal
#from python_qt_binding.QtGui import QCompleter, QFileDialog, QGraphicsScene, QIcon, QImage, QPainter, QWidget
#from python_qt_binding.QtSvg import QSvgGenerator

#from python_qt_binding.QtGui import QGraphicsScene 
from qt_gui.plugin import Plugin
#from python_qt_binding import loadUi

from python_qt_binding.QtWidgets import QWidget, QGraphicsScene
from python_qt_binding.QtGui import QPainter, QPixmap, QColor, QPen
from python_qt_binding.QtCore import Qt
#from python_qt_binding.QtCore import *
from std_srvs.srv import SetBool, Empty
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range, BatteryState, Temperature

from rqt_gauge import rqt_gauge

from PyQt5.QtCore import pyqtSignal

from time import sleep


class Ccsr2Dashboard(Plugin):

    def __init__(self, context):
        super(Ccsr2Dashboard, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('Ccsr2Dashboard')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_ccsr2_dashboard'), 'resource', 'Ccsr2Dashboard.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('Ccsr2DashboardUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
        
        # self._widget.motorsToggleEnable.clicked.connect(self.motorEnable)
        self._widget.leftMotorCheckbox.clicked.connect(self.lmotorEnable)
        self._widget.rightMotorCheckbox.clicked.connect(self.rmotorEnable)
        self._widget.tempSensorCheckbox.clicked.connect(self.tempSensorEnable)
        self._widget.pmonCheckbox.clicked.connect(self.powerMonitorEnable)
        self._widget.rpLidarCheckbox.clicked.connect(self.rpLidarEnable)
        self._widget.motorsOn.clicked.connect(self.motorEnable)
        self._widget.temperatureDisp.setNumDigits(2)
       # self._widget.rightMotorCheckBox.clicked.connect(self.rmotorEnable)
        
        self._widget.rpLidarCheckbox.setChecked(True)


 #       rospy.wait_for_service('start_motor')
 #       rospy.wait_for_service('stop_motor')
 #       rospy.wait_for_service('enablelmotor')
 #       rospy.wait_for_service('enablermotor')
 #      rospy.wait_for_service('enable_temp_sensor')
        rospy.Subscriber('enablelmotor_event', Bool, self.lmotorEnableNotificationCallback)
        rospy.Subscriber('enablermotor_event', Bool, self.rmotorEnableNotificationCallback)
        rospy.Subscriber('temperature', Temperature, self.temperatureCallback)
        rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.poseCallback)
        rospy.Subscriber('odom', Odometry, self.odomCallback)
        rospy.Subscriber('pmon', BatteryState, self.pmonCallback)
    #    rospy.Subscriber('range0', Range, self.rangeCallback)

        self.speedometer = rqt_gauge(1.5, 100, 'm/s')
        self.thermometer = rqt_gauge(80, 100, 'C')
        self.current = rqt_gauge(1.0, 100, 'A')
        self._widget.tempGraphicsView.setScene(self.thermometer.scene)
        self._widget.currentGraphicsView.setScene(self.current.scene)
        self._widget.speedometerGraphicsView.setScene(self.speedometer.scene)

        self._speed_lin = 0;
        self._speed_ang = 0;
        self._temperature = 0;
        self._current = 0;
#        self.speedometer.draw_gauge(0.1)


        #self.connect(self, SIGNAL("changeUI(PyQt_PyObject)"), self.monitoringSlot)        
        

        self.odom_trigger.connect(self.handle_odom_trigger)
        self.temp_trigger.connect(self.handle_temp_trigger)
        self.pmon_trigger.connect(self.handle_pmon_trigger)

        # Emit the signal.
        
        
    odom_trigger = pyqtSignal()
    temp_trigger = pyqtSignal()
    pmon_trigger = pyqtSignal()

    def handle_odom_trigger(self):
        self.speedometer.draw_gauge(self._speed_lin)
        pass

    def handle_pmon_trigger(self):
        self.current.draw_gauge(self._current)
        pass

    def handle_temp_trigger(self):
        self.thermometer.draw_gauge(self._temperature)
        pass

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass
     
    def temperatureCallback(self, msg):
        self._widget.temperatureDisp.display(msg.temperature)
        self._temperature = msg.temperature
        self.temp_trigger.emit()
        pass

    def poseCallback(self, msg):
        self._widget.poseX.display(msg.pose.pose.position.x)
        self._widget.poseY.display(msg.pose.pose.position.y)
        pass

    def odomCallback(self, msg):
        self._widget.speedLin.display(msg.twist.twist.linear.x)
        self._widget.speedAng.display(msg.twist.twist.angular.z)
     #   self.emit(SIGNAL("changeUI(PyQt_PyObject)"), msg)
        self.odom_trigger.emit()
        self._speed_lin = msg.twist.twist.linear.x;
#        self._speed_ang = msg.twist.twist.angular.z;
        pass

    def pmonCallback(self, msg):
        self._widget.currentDisp.display(msg.current)
        self.pmon_trigger.emit()
        self._current = msg.current;
#        self._speed_ang = msg.twist.twist.angular.z;
        pass

    def tempSensorEnable(self):
        enableTempSensor = rospy.ServiceProxy('enable_temp_sensor', SetBool)
        if(self._widget.tempSensorCheckbox.isChecked()):
            enableTempSensor(True)
        else:
            enableTempSensor(False)
            self.thermometer.draw_gauge(1)
            self._widget.temperatureDisp.display(0)
        pass

    def powerMonitorEnable(self):
        enablePowerMonitor = rospy.ServiceProxy('enable_pmon', SetBool)
        if(self._widget.pmonCheckbox.isChecked()):
            enablePowerMonitor(True)
        else:
            enablePowerMonitor(False)
            self.current.draw_gauge(0)
            self._widget.currentDisp.display(0)
        pass

    def rpLidarEnable(self):
        enablerpLidar = rospy.ServiceProxy('start_motor', Empty)
        disablerpLidar = rospy.ServiceProxy('stop_motor', Empty)
        if(self._widget.rpLidarCheckbox.isChecked()):
            enablerpLidar()
        else:
            disablerpLidar()
        pass

    def lmotorEnable(self):
        enablelmotor = rospy.ServiceProxy('enablelmotor', SetBool)
        if(self._widget.leftMotorCheckbox.isChecked()):
            enablelmotor(True)
        else:
            enablelmotor(False)
            self.speedometer.draw_gauge(0)
            self._widget.speedLin.display(0)
            self._widget.speedAng.display(0)
        pass

    def lmotorEnableNotificationCallback(self, msg):
        self._widget.leftMotorCheckbox.setChecked(msg.data)

    def rmotorEnable(self):
        enablermotor = rospy.ServiceProxy('enablermotor', SetBool)
        if(self._widget.rightMotorCheckbox.isChecked()):
            enablermotor(True)
        else:
            enablermotor(False)
            self.speedometer.draw_gauge(0)
            self._widget.speedLin.display(0)
            self._widget.speedAng.display(0)
        pass

    def rmotorEnableNotificationCallback(self, msg):
        self._widget.rightMotorCheckbox.setChecked(msg.data)

    def motorEnable(self):
        enablelmotor = rospy.ServiceProxy('enablelmotor', SetBool)
        enablermotor = rospy.ServiceProxy('enablermotor', SetBool)
        enablermotor(True)
        enablelmotor(True)
        pass
    
    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
