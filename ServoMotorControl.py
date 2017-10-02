#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

"""
 @file ServoMotorControl.py
 @brief ServoMotorController
 @date $Date$


"""
import sys
import time
sys.path.append(".")

# Import RTM module
import RTC
import OpenRTM_aist

#I2C
import RPi.GPIO as GPIO
from time import sleep
import smbus  # I2C module
import math

# Import Service implementation class
# <rtc-template block="service_impl">

# </rtc-template>

# Import Service stub modules
# <rtc-template block="consumer_import">
# </rtc-template>


# This module's spesification
# <rtc-template block="module_spec">
servomotorcontrol_spec = ["implementation_id", "ServoMotorControl", 
		 "type_name",         "ServoMotorControl", 
		 "description",       "ServoMotorController", 
		 "version",           "1.0.0", 
		 "vendor",            "UoA", 
		 "category",          "Aizu", 
		 "activity_type",     "STATIC", 
		 "max_instance",      "1", 
		 "language",          "Python", 
		 "lang_type",         "SCRIPT",
		 ""]
# </rtc-template>

##
# @class ServoMotorControl
# @brief ServoMotorController
# 
# 
class ServoMotorControl(OpenRTM_aist.DataFlowComponentBase):
	
	##
	# @brief constructor
	# @param manager Maneger Object
	# 
	def __init__(self, manager):
		OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)

		ServoMotor1_arg = [None] * ((len(RTC._d_TimedFloat) - 4) / 2)
		self._d_ServoMotor1 = RTC.TimedFloat(*ServoMotor1_arg)
		"""
		"""
		self._ServoMotor1In = OpenRTM_aist.InPort("ServoMotor1", self._d_ServoMotor1)
		ServoMotor2_arg = [None] * ((len(RTC._d_TimedFloat) - 4) / 2)
		self._d_ServoMotor2 = RTC.TimedFloat(*ServoMotor2_arg)
		"""
		"""
		self._ServoMotor2In = OpenRTM_aist.InPort("ServoMotor2", self._d_ServoMotor2)
		ServoMotor3_arg = [None] * ((len(RTC._d_TimedFloat) - 4) / 2)
		self._d_ServoMotor3 = RTC.TimedFloat(*ServoMotor3_arg)
		"""
		"""
		self._ServoMotor3In = OpenRTM_aist.InPort("ServoMotor3", self._d_ServoMotor3)


		


		# initialize of configuration-data.
		# <rtc-template block="init_conf_param">
		
		# </rtc-template>


		 
	##
	#
	# The initialize action (on CREATED->ALIVE transition)
	# formaer rtc_init_entry() 
	# 
	# @return RTC::ReturnCode_t
	# 
	#
	def onInitialize(self):
		# Bind variables and configuration variable
		
		# Set InPort buffers
		self.addInPort("ServoMotor1",self._ServoMotor1In)
		self.addInPort("ServoMotor2",self._ServoMotor2In)
		self.addInPort("ServoMotor3",self._ServoMotor3In)
		
		# Set OutPort buffers
		
		# Set service provider to Ports
		
		# Set service consumers to Ports
		
		# Set CORBA Service Ports
		
		self.cnt=0
		self.err=0
		## smbus
		self.bus = smbus.SMBus(1)
		self.PCA9685_ADDRESS = 0x40
		
		self._d_ServoMotor1.data = 0
		self._d_ServoMotor2.data = 0
		self._d_ServoMotor3.data = 0


		return RTC.RTC_OK
	
	#	##
	#	# 
	#	# The finalize action (on ALIVE->END transition)
	#	# formaer rtc_exiting_entry()
	#	# 
	#	# @return RTC::ReturnCode_t
	#
	#	# 
	#def onFinalize(self):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The startup action when ExecutionContext startup
	#	# former rtc_starting_entry()
	#	# 
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onStartup(self, ec_id):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The shutdown action when ExecutionContext stop
	#	# former rtc_stopping_entry()
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onShutdown(self, ec_id):
	#
	#	return RTC.RTC_OK
	
		##
		#
		# The activated action (Active state entry action)
		# former rtc_active_entry()
		#
		# @param ec_id target ExecutionContext Id
		# 
		# @return RTC::ReturnCode_t
		#
		#
	def onActivated(self, ec_id):
			print "onActivated"
		self.err =1
		self.bus.write_byte_data(self.PCA9685_ADDRESS, 0x00, 0x00)
		freq = 0.9*50
		prescaleval = 25000000.0    # 25MHz
		prescaleval /= 4096.0       # 12-bit
		prescaleval /= float(freq)
		prescaleval -= 1.0
		prescale = int(math.floor(prescaleval + 0.5))
		oldmode = self.bus.read_byte_data(self.PCA9685_ADDRESS, 0x00)
		newmode = (oldmode & 0x7F) | 0x10
		self.bus.write_byte_data(self.PCA9685_ADDRESS, 0x00, newmode)
		self.bus.write_byte_data(self.PCA9685_ADDRESS, 0xFE, prescale)
		self.bus.write_byte_data(self.PCA9685_ADDRESS, 0x00, oldmode)
		self.bus.write_byte_data(self.PCA9685_ADDRESS, 0x00, oldmode | 0xa1)
		self.cnt=0
		self.err =2

		return RTC.RTC_OK
	
		##
		#
		# The deactivated action (Active state exit action)
		# former rtc_active_exit()
		#
		# @param ec_id target ExecutionContext Id
		#
		# @return RTC::ReturnCode_t
		#
		#
	def onDeactivated(self, ec_id):
	
		return RTC.RTC_OK
	
		##
		#
		# The execution action that is invoked periodically
		# former rtc_active_do()
		#
		# @param ec_id target ExecutionContext Id
		#
		# @return RTC::ReturnCode_t
		#
		#
	def onExecute(self, ec_id):
			
		self.err=3
		servo=0
		if self._ServoMotor1In.isNew():
			servo =self._d_ServoMotor1.data
			#サーボモータ1値取得
			self._d_ServoMotor1 = self._ServoMotor1In.read()
			if self._d_ServoMotor1.data >= 0 and self._d_ServoMotor1.data <= 180 and servo !=self._d_ServoMotor1.data :
				self.err=4
				duty_int= int(self.calc_duty(self._d_ServoMotor1.data))
				self.bus.write_i2c_block_data(self.PCA9685_ADDRESS,14,[0, 0,  duty_int & 0xFF , duty_int >>8])
			else :
				print "servmotor1 value limite "

		if self._ServoMotor2In.isNew():
			servo =self._d_ServoMotor2.data
			#サーボモータ2値取得
			self._d_ServoMotor2 = self._ServoMotor2In.read()
			if self._d_ServoMotor2.data >= 0 and self._d_ServoMotor2.data <= 180 and servo !=self._d_ServoMotor2.data:
				self.err=5
				duty_int= int(self.calc_duty(self._d_ServoMotor2.data))
				self.bus.write_i2c_block_data(self.PCA9685_ADDRESS,10,[0, 0, duty_int & 0xFF , duty_int >>8])
			else :
				print "servmotor2 value limite "
		
		if self._ServoMotor3In.isNew():
			servo =self._d_ServoMotor3.data
			#サーボモータ1値取得
			self._d_ServoMotor3 = self._ServoMotor3In.read()
			if self._d_ServoMotor3.data >= 0 and self._d_ServoMotor3.data <= 180 and servo !=self._d_ServoMotor3.data:
				self.err=6
				duty_int= int(self.calc_duty(self._d_ServoMotor3.data))
				self.bus.write_i2c_block_data(self.PCA9685_ADDRESS,6,[0, 0, duty_int & 0xFF , duty_int >>8])
			else :
				print "servmotor3 value limite "
		

		return RTC.RTC_OK
	
	def calc_duty(self,angle):
    		duty = 0.0
    		duty = 102 + (492 - 102)/180 * angle

    		return duty	
	
	#	##
	#	#
	#	# The aborting action when main logic error occurred.
	#	# former rtc_aborting_entry()
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onAborting(self, ec_id):
	#
	#	return RTC.RTC_OK
	
		##
		#
		# The error action in ERROR state
		# former rtc_error_do()
		#
		# @param ec_id target ExecutionContext Id
		#
		# @return RTC::ReturnCode_t
		#
		#
	def onError(self, ec_id):
	
		return RTC.RTC_OK
	
	#	##
	#	#
	#	# The reset action that is invoked resetting
	#	# This is same but different the former rtc_init_entry()
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onReset(self, ec_id):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The state update action that is invoked after onExecute() action
	#	# no corresponding operation exists in OpenRTm-aist-0.2.0
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#

	#	#
	#def onStateUpdate(self, ec_id):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The action that is invoked when execution context's rate is changed
	#	# no corresponding operation exists in OpenRTm-aist-0.2.0
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onRateChanged(self, ec_id):
	#
	#	return RTC.RTC_OK
	



def ServoMotorControlInit(manager):
    profile = OpenRTM_aist.Properties(defaults_str=servomotorcontrol_spec)
    manager.registerFactory(profile,
                            ServoMotorControl,
                            OpenRTM_aist.Delete)

def MyModuleInit(manager):
    ServoMotorControlInit(manager)

    # Create a component
    comp = manager.createComponent("ServoMotorControl")

def main():
	mgr = OpenRTM_aist.Manager.init(sys.argv)
	mgr.setModuleInitProc(MyModuleInit)
	mgr.activateManager()
	mgr.runManager()

if __name__ == "__main__":
	main()

