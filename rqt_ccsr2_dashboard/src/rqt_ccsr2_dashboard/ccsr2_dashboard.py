import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from std_srvs.srv import SetBool
from std_msgs.msg import Bool
from sensor_msgs.msg import Temperature

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
        self._widget.motorsOn.clicked.connect(self.motorEnable)
        self._widget.temperatureDisp.setNumDigits(2)
       # self._widget.rightMotorCheckBox.clicked.connect(self.rmotorEnable)
        

        rospy.wait_for_service('enablelmotor')
        rospy.wait_for_service('enablermotor')
        rospy.wait_for_service('enable_temp_sensor')
        rospy.Subscriber('enablelmotor_event', Bool, self.lmotorEnableNotificationCallback)
        rospy.Subscriber('enablermotor_event', Bool, self.rmotorEnableNotificationCallback)
        rospy.Subscriber('temperature', Temperature, self.temperatureCallback)

    
        

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass
    
    def temperatureCallback(self, msg):
        self._widget.temperatureDisp.display(msg.temperature)
        pass

    def tempSensorEnable(self):
        enableTempSensor = rospy.ServiceProxy('enable_temp_sensor', SetBool)
        if(self._widget.tempSensorCheckbox.isChecked()):
            enableTempSensor(True)
        else:
            enableTempSensor(False)
        pass

    def lmotorEnable(self):
        enablelmotor = rospy.ServiceProxy('enablelmotor', SetBool)
        if(self._widget.leftMotorCheckbox.isChecked()):
            enablelmotor(True)
        else:
            enablelmotor(False)
        pass

    def lmotorEnableNotificationCallback(self, msg):
        self._widget.leftMotorCheckbox.setChecked(msg.data)

    def rmotorEnable(self):
        enablermotor = rospy.ServiceProxy('enablermotor', SetBool)
        if(self._widget.rightMotorCheckbox.isChecked()):
            enablermotor(True)
        else:
            enablermotor(False)
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
