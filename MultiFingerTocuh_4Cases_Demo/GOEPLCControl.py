'''
Multi Finger Touch API
Version 1.0.1 2020/10/29 update
author mudu_liu@zhbojay.com
interpreter python3 || python2
'''

'''
Multi Finger Touch API
Version 1.0.2 2020/11/21 update
author mudu_liu@zhbojay.com
interpreter python3 || python2
add comments to other function
'''

import time
import serial
import serial.tools.list_ports
import io
import struct
import sys,os
import binascii
import datetime
import math
import platform

Interpreter = platform.python_version()[0]
print(Interpreter)


class BojayPLCCommandClass:

    ReadPLCVersion = '%01#RDD0030000302'

    # Step move
    MoveStep_xAxis = '%01#WCSR00201'
    MoveStep_yAxis = '%01#WCSR00241'
    MoveStep_zAxis = '%01#WCSR00281'
    MoveStep_rAxis = '%01#WCSR00391'

    # Step set
    SetStep_xAxis = '%01#WDD0100001001'
    SetStep_yAxis = '%01#WDD0100801009'
    SetStep_zAxis = '%01#WDD0101601017'
    SetStep_rAxis = '%01#WDD0099600997'

    # Step get
    GetStep_xAxis = '%01#RDD006000060054'
    GetStep_yAxis = '%01#RDD006020060254'
    GetStep_zAxis = '%01#RDD006040060454'

    # Set speed
    SetSpeed_xAxis = '%01#WDD0020000201'
    SetSpeed_yAxis = '%01#WDD0021000211'
    SetSpeed_zAxis = '%01#WDD0022000221'
    SetSpeed_rAxis = '%01#WDD0023000231'

    # Get speed
    GetSpeed_xAxis = '%01#RDD0020000201'
    GetSpeed_yAxis = '%01#RDD0021000211'
    GetSpeed_zAxis = '%01#RDD0022000221'
    GetSpeed_rAxis = '%01#RDD0023000231'

    # Set move distance
    SetDistance_xAxis = '%01#WDD0020200203'
    SetDistance_yAxis = '%01#WDD0021200213'
    SetDistance_zAxis = '%01#WDD0022200223'
    SetDistance_rAxis = '%01#WDD0023200233'

    # Move x&y&z
    XYMove = '%01#WCSR002F1'
    # XYZMove = "%01#WCSR002B1"
    XMove = '%01#WCSR002C1'
    YMove = '%01#WCSR002D1'
    ZMove = '%01#WCSR002E1'
    RMove = '%01#WCSR002A1'

    # Get x&y&z
    GetCoordiante_xAxis = '%01#RDD0014600147'
    GetCoordiante_yAxis = '%01#RDD0015000151'
    GetCoordiante_zAxis = '%01#RDD0015400155'
    GetCoordiante_rAxis = '%01#RDD0015800159'


    # Single of moving axis
    SingleMoveFinish_xAxis = '%01#RCSR0052'
    SingleMoveFinish_yAxis = '%01#RCSR0054'
    SingleMoveFinish_zAxis = '%01#RCSR0056'
    SingleMoveFinish_rAxis = '%01#RCSR0057'
    SingleMoveFinish_xyAxis = '%01#RCSR005C'
    # SingleMoveFinish_xyzAxis = '%01#RCSR0059'
    SingleHomeFinish_xAxis = '%01#RCSR0100'
    SingleHomeFinish_yAxis = '%01#RCSR0101'
    SingleHomeFinish_zAxis = '%01#RCSR0102'
    SingleHomeFinish_rAxis = '%01#RCSR0119'
    SingleHomeFinish_xyzAxis = '%01#RCSR0104'

    # Get Limit
    GetMaxLimit_xAxis = '%01#RDD0062000621'
    GetMaxLimit_yAxis = '%01#RDD0062200623'
    GetMaxLimit_zAxis = '%01#RDD0062400625'
    GetMaxLimit_rAxis = '%01#RDD0062600627'
    GetMinLimit_xAxis = '%01#RDD0063000631'
    GetMinLimit_yAxis = '%01#RDD0063200633'
    GetMinLimit_zAxis = '%01#RDD0063400635'
    GetMinLimit_rAxis = '%01#RDD0063600637'

    # Set Limit
    SetMaxLimit_xAxis = '%01#WDD0210002101'
    SetMinLimit_xAxis = '%01#WDD0210402105'
    SetMaxLimit_yAxis = '%01#WDD0210802109'
    SetMinLimit_yAxis = '%01#WDD0211202113'
    SetMaxLimit_zAxis = '%01#WDD0211602117'
    SetMinLimit_zAxis = '%01#WDD0212002121'
    SetMaxLimit_rAxis = '%01#WDD0212802129'
    SetMinLimit_rAxis = '%01#WDD0213202133'

    # reset
    ResetCommand_ON = '%01#WCSR00841'
    ResetCommand_OFF = '%01#WCSR00840'

    #SingleAxisGoHome
    SingleGoHome_XAxis = '%01#WCSR03001'
    SingleGoHome_YAxis = '%01#WCSR03011'
    SingleGoHome_ZAxis = '%01#WCSR03021'
    SingleGoHome_RAxis = '%01#WCSR03091'

    # Sensor axis
    XAisHomeSensor = '%01#RCSX0018'
    XAxisLLSensor = '%01#RCSX0019'
    XAxisHLSensor = '%01#RCSX001A'

    YAisHomeSensor = '%01#RCSX0015'
    YAxisLLSensor = '%01#RCSX0016'
    YAxisHLSensor = '%01#RCSX0017'

    ZAisHomeSensor = '%01#RCSX001B'
    ZAxisLLSensor = '%01#RCSX001C'
    ZAxisHLSensor = '%01#RCSX001D'

    CylinderInSensor = '%01#RCSX0011'
    CylinderOutSensor = '%01#RCSX0012'

    CurtainSensor = '%01#RCSX0010'
    CalibrationSensor = '%01#RCSX001F'

    # E71
    EnableUSB1 = '%01#WCSR01221'
    EnableUSB2 = "%01#WCSR01421"
    EnableUSB_all = '%01#WCSR01291'
    DisableUSB1 = '%01#WCSR01220'
    DisableUSB2 = "%01#WCSR01420"
    DisableUSB_all = '%01#WCSR01290'
    USB1Sensor = "%01#RCSX0302"
    USB2Sensor = "%01#RCSX0303"

    EnablePower1 = '%01#WCSR01201'
    EnablePower2 = '%01#WCSR01401'
    EnablePower_all = '%01#WCSR01271'
    DisablePower1 = '%01#WCSR01200'
    DisablePower2 = '%01#WCSR01400'
    DisablePower_all = '%01#WCSR01270'
    Power1Sensor = "%01#RCSX0300"
    Power2Sensor = "%01#RCSX0301"




    LockDUT='%01#WCSR00441'
    UnLockDUT='%01#WCSR00440'

    DUTLockSensor = '%01#RCSX0304'
    DUTSensor = '%01#RCSX0014'

    PenASensor = '%01#RCSX0300'
    PenBSensor = '%01#RCSX0301'

    HolderE71 = "%01#RCSX0305"
    HolderLUCY = "%01#RCSX0306"
    HolderEthel = "%01#RCSX0307"

myBojayPLCCommandClass = BojayPLCCommandClass()


class GOEControlClass:
    # E71

    DUTLock = 4
    DUTUnlock = 5


    DUT1 = 12
    DUT2 = 13

    # E45
    X_axis = 100
    Y_axis = 200
    Z_axis = 300
    R_axis = 350
    XY_axis = 400
    XYZ_axis = 401

    # DUT1 = 500
    # DUT2 = 501
    # DUT3 = 502
    # DUT4 = 503

    # Set_DM_5V_ON = 504
    # Set_DM_5V_OFF = 505
    # DUT_LOCK = 506
    # DUT_OPEN= 507

    Cylinder_IN = 508
    Cylinder_OUT = 509

    EStopOn = 510
    EStopOff = 511

    # Cylinder_UP = 512
    # Cylinder_DOWN = 513
    # Cylinder_LOCK = 514
    # Cylinder_OPEN = 515

    Alarm_On = 516
    Alarm_Off = 517
    # DUTALL = 518

    PenA_Up = 518
    PenA_Down = 519
    PenB_Up = 520
    PenB_Down = 521

    Max_limit = 1100
    Min_limit = 1200

    DUT_In = 2100
    DUT_Out = 2200
    DUTRight_In = 2110
    DUTRight_Out = 2210

    LED_Left = 3100
    LED_Right = 3200

    Red_OFF = 4101
    Red_ON = 4100
    Yellow_OFF = 4201
    Yellow_ON = 4200
    Green_ON = 4301
    Green_OFF = 4300

    # USBLeft_ON = 5101
    # USBLeft_OFF = 5100
    # USBRight_ON = 5201
    # USBRight_OFF = 5200

    Sensor_X_Max = 6100
    Sensor_X_Min = 6110
    Sensor_X_Origin = 6120

    Sensor_Y_Max = 6200
    Sensor_Y_Min = 6210
    Sensor_Y_Origin = 6220

    Sensor_Z_Max = 6300
    Sensor_Z_Min = 6310
    Sensor_Z_Origin = 6320

    Sensor_LeftHolder_In = 6400
    Sensor_LeftHolder_Out = 6410
    # Sensor_RightHolder_In = 6500
    # Sensor_RightHolder_Out = 6510

    Sensor_LeftDUT = 6600
    Sensor_RightDUT = 6700

    Sensor_Curtain = 6800

    Sensor_Calibrate = 6900 # %01#RCSX001F
    Sensor_TouchFinger = 7000 # %01#RCSX001E

    SerialPortOpen = False

    myTollerance = 0.0000001
    myWaitTime = 0.1
    strErrorMessage ="ok"

    #add new sensor
    CylinderINSensor = 7001
    CylinderOUTSensor = 7002
    # FingerprintWorkSensor = 7004
    # DUTLockSensor = 7005
    # USB1Sensor = 7006
    # USB2Sensor = 7007
    # USB3Sensor = 7008
    # USB4Sensor = 7009
    # CheckDUT1Sensor = 7010
    # CheckDUT2Sensor = 7011
    # CheckDUT3Sensor = 7012
    # CheckDUT4Sensor = 7013
    # OSS1CheckSensor = 7014
    # OSS2CheckSensor = 7015
    # OSS3CheckSensor = 7016
    # OSS4CheckSensor = 7017
    # USBALLSensor = 7018

    XAxisCalibration = 0
    YAxisCalibration = 0
    ZAxisCalibration = 0

    bFirstRunFunction = True
    bDrawCircle = False
    bDisableUSBSensor = False

    ZAxisMaxLimit = 0
    ZAxisMinLimit = 0
    XAxisMaxLimit = 0
    XAxisMinLimit = 0
    YAxisMaxLimit = 0
    YAxisMinLimit = 0
    RAxisMaxLimit = 0
    RAxisMinLimit = 0

    SensorOn = 1
    SensorOFF = 2

    LeftUp = 3
    LeftDown = 4
    RightUp = 5
    RightDown = 6
    BackUp = 7
    BackDown = 8
    FrontUp = 9
    FrontDown = 10
    CheckDUTSensor = 11

    # for MultiFingerTouch of E71/Ethel/Lucy

    # ********************************************************************************#
    # Open the PLC port
    '''
    Index    :  1
    Function :  open serial port
    Param    :  no use
    Return   :  0:succuess -1: fail 
    Error    :  call strErrorMessage
    '''
    def OpenSerial(self):
        try:
            port_list = list(serial.tools.list_ports.comports())
            if (len(port_list) < 1):
                self.strErrorMessage = "fail:There is no serial port"
                return -1
            bFindSerialPort = False;
            for i in range(0, len(port_list)):
                err = self.ChooseCOM(port_list[i].device)
                if (err == 0):
                    bFindSerialPort = True
                    return 0
                else:
                    self.ser.close()
                    continue
            if (bFindSerialPort == False):
                self.strErrorMessage = "There is no suitable port"
                return -1
        except:
            self.strErrorMessage = "Open serial port fail"
            return -1
        return 0


    '''
    Index    :  2
    Function :  close serial port
    Param    :  Nose
    Return   :  0:succuess
    Error    :  call strErrorMessage
    '''
    def CloseSerial(self):
        try:
            if (self.ser.is_open == True):
                self.ser.close()
            return 0
        except:
            self.strErrorMessage = "CloseSerial fail"
            return -1

    '''
       Index    :  3
       Function :  move to specified coordinate
       Param    :  ofWhatAxis: specified axis, can be set X_axis/Y_axis/Z_axis/R_axis
                    Value    : sepcified coordinate
       Return   :  0:succuess -1:fail
       Error    :  call strErrorMessage
    '''
    def MoveToCoordinates(self,ofWhatAxis,Value,timeout=10):
        if(self.ser.isOpen() == False):
            self.strErrorMessage =  "The serial port is not opened"
            return -1
        # Move X-Axis
        if (ofWhatAxis == self.X_axis):
            if (Value < self.XAxisMinLimit):
                Value = self.XAxisMinLimit
            if(Value > self.XAxisMaxLimit):
                Value = self.XAxisMaxLimit

            command = myBojayPLCCommandClass.SetDistance_xAxis #'%01#WDD0020200203'
            finalByte = self.__flipByte(Value)
            command = command + finalByte
            ret = self.__writeRead(command)
            if (ret == 0):
                moveCommand = myBojayPLCCommandClass.XMove #'%01#WCSR002C1'
                ret = self.__writeRead(moveCommand)
                if(ret == 0):
                    mytimeCount = 0
                    while (self.GetmoveSignal(self.X_axis) == 1):
                        if (mytimeCount > timeout):
                            self.strErrorMessage = "MoveToCoordinates read command timeout"
                            return -1
                        time.sleep(0.005)
                        mytimeCount += 0.005

                    moveCommand = '%01#WCSR002C0'
                    ret = self.__writeRead(moveCommand)
                    print(self.GetmoveSignal(self.X_axis))
                    return 0
                else:
                    self.strErrorMessage = "MoveToCoordinates write command fail"
                    return -1
            else:
                self.strErrorMessage = "MoveToCoordinates write command fail"
                return -1

        # Move Y-Axis
        elif (ofWhatAxis == self.Y_axis):
            if (Value < self.YAxisMinLimit):
                Value = self.YAxisMinLimit
            if(Value > self.YAxisMaxLimit):
                Value = self.YAxisMaxLimit
            command = myBojayPLCCommandClass.SetDistance_yAxis #'%01#WDD0021200213'
            finalByte = self.__flipByte(Value)
            command = command + finalByte
            ret = self.__writeRead(command)
            if (ret == 0):
                moveCommand = myBojayPLCCommandClass.YMove #'%01#WCSR002D1'
                ret = self.__writeRead(moveCommand)
                if(ret == 0):
                    mytimeCount = 0
                    while (self.GetmoveSignal(self.Y_axis) == 1):
                        if (mytimeCount > timeout):
                            self.strErrorMessage = "MoveToCoordinates read command timeout"
                            return -1
                        time.sleep(0.005)
                        mytimeCount += 0.005

                    moveCommand = '%01#WCSR002D0'
                    ret = self.__writeRead(moveCommand)
                    return 0
                else:
                    self.strErrorMessage = "MoveToCoordinates write command fail"
                    return -1
            else:
                self.strErrorMessage = "MoveToCoordinates write command fail"
                return -1

        # Move Z-Axis
        elif (ofWhatAxis == self.Z_axis):
            if (Value < self.ZAxisMinLimit):
                Value = self.ZAxisMinLimit
            if(Value > self.ZAxisMaxLimit):
                Value = self.ZAxisMaxLimit
            command = myBojayPLCCommandClass.SetDistance_zAxis #'%01#WDD0022200223'
            finalByte = self.__flipByte(Value)
            command = command + finalByte
            ret = self.__writeRead(command)
            if (ret == 0):
                moveCommand = myBojayPLCCommandClass.ZMove #'%01#WCSR002E1'
                ret = self.__writeRead(moveCommand)
                if(ret == 0):
                    mytimeCount = 0
                    while (self.GetmoveSignal(self.Z_axis) == 1):
                        if (mytimeCount > timeout):
                            self.strErrorMessage = "MoveToCoordinates read command timeout"
                            return -1
                        time.sleep(0.005)
                        mytimeCount += 0.005

                    moveCommand = '%01#WCSR002E0'
                    ret = self.__writeRead(moveCommand)
                    return 0
                else:
                    self.strErrorMessage = "MoveToCoordinates write command fail"
                    return -1
            else:
                self.strErrorMessage = "MoveToCoordinates write command fail"
                return -1
                # Move Z-Axis
        elif (ofWhatAxis == self.R_axis):
            if (Value < self.RAxisMinLimit):
                Value = self.RAxisMinLimit
            if (Value > self.RAxisMaxLimit):
                Value = self.RAxisMaxLimit
            command = myBojayPLCCommandClass.SetDistance_rAxis  # '%01#WDD0023200233'
            finalByte = self.__flipByte(Value)
            command = command + finalByte
            ret = self.__writeRead(command)
            if (ret == 0):
                moveCommand = myBojayPLCCommandClass.RMove  # '%01#WCSR002A1'
                ret = self.__writeRead(moveCommand)
                if (ret == 0):
                    mytimeCount = 0
                    while (self.GetmoveSignal(self.R_axis) == 1):
                        if (mytimeCount > timeout):
                            self.strErrorMessage = "MoveToCoordinates read command timeout"
                            return -1
                        time.sleep(0.005)
                        mytimeCount += 0.005

                    moveCommand = '%01#WCSR002A0'
                    ret = self.__writeRead(moveCommand)
                    return 0
                else:
                    self.strErrorMessage = "MoveToCoordinates write command fail"
                    return -1
            else:
                self.strErrorMessage = "MoveToCoordinates input parameter error"
                return -1

    '''
    Index    :  4
    Function :  sync move x/y axis move
    Param    :  xValue/yValue    : sepcified coordinate
                timeout          : max time to finish the movement
    Return   :  0:succuess -1:fail
    Error    :  call strErrorMessage
    '''
    def SynchronousXY(self, xValue, yValue, timeout=100):
        try:

            if (xValue < self.XAxisMinLimit):
                xValue = self.XAxisMinLimit
            if (xValue > self.XAxisMaxLimit):
                xValue = self.XAxisMaxLimit
            if (yValue < self.YAxisMinLimit):
                yValue = self.YAxisMinLimit
            if (yValue > self.YAxisMaxLimit):
                yValue = self.YAxisMaxLimit

            command = myBojayPLCCommandClass.SetDistance_xAxis  # '%01#WDD0020200203'
            finalByte = self.__flipByte(xValue)
            command = command + finalByte
            ret = self.__writeRead(command)
            if (ret != 0):
                self.strErrorMessage = "SynchronousXY %01#WDD0020200203 fail"
                return -1
            command = myBojayPLCCommandClass.SetDistance_yAxis  # '%01#WDD0021200213'
            finalByte = self.__flipByte(yValue)
            command = command + finalByte
            ret = self.__writeRead(command)
            if (ret != 0):
                self.strErrorMessage = "SynchronousXY %01#WDD0021200213 fail"
                return -1

            if (self.bDrawCircle == True):
                command = '%01#WCSR00321'
            else:
                command = myBojayPLCCommandClass.XYMove  # '%01#WCSR002F1'
            # bcc = self.__bccValue(command)
            # command = command + bcc + '\r'
            # command = command.upper()
            # command = command.encode('utf-8')
            ret = self.__writeRead(command)
            if (ret != 0):
                self.strErrorMessage = "SynchronousXY %01#WCSR002F1 fail"
                return -1
            else:
                mytimeCount = 0
                while (self.GetmoveSignal(self.X_axis) == 1):
                    if (mytimeCount > timeout):
                        self.strErrorMessage = "SynchronousXY time out"
                        return -1
                    time.sleep(self.myWaitTime)
                    mytimeCount += self.myWaitTime

                mytimeCount = 0
                while (self.GetmoveSignal(self.Y_axis) == 1):
                    if (mytimeCount > timeout):
                        self.strErrorMessage = "SynchronousXY time out"
                        return -1
                    time.sleep(self.myWaitTime)
                    mytimeCount += self.myWaitTime
                return 0
        except Exception as e:
            self.strErrorMessage = "SynchronousXY fail %s" % e
            return -1


    '''
     Index    :  5
     Function :  set a specified speed 
     Param    :  ofWhatAxis: specified axis, can be set X_axis/Y_axis/Z_axis/R_axis
                  Value    : sepcified speed
     Return   :  0:succuess -1:fail
     Error    :  call strErrorMessage
     '''
    def SetSpeed(self, ofWhatAxis, Value):
        if (self.ser.isOpen() == False):
            self.strErrorMessage = "The serial port is not open"
            return -1
        try:
            if (ofWhatAxis == self.X_axis):
                command = myBojayPLCCommandClass.SetSpeed_xAxis  # '%01#WDD0020000201'
            elif (ofWhatAxis == self.Y_axis):
                command = myBojayPLCCommandClass.SetSpeed_yAxis  # '%01#WDD0021000211'
            elif (ofWhatAxis == self.Z_axis):
                command = myBojayPLCCommandClass.SetSpeed_zAxis  # '%01#WDD0022000221'
            elif (ofWhatAxis == self.R_axis):
                command = myBojayPLCCommandClass.SetSpeed_rAxis  # '%01#WDD0023000231'
            finalByte = self.__flipByte(Value)
            command = command + finalByte
            ret = self.__writeRead(command)
            if (ret == -1):
                self.strErrorMessage = "SetSpeed read command fail"
                return -1
            return 0
        except Exception as e:
            self.strErrorMessage = "SetSpeed except %s" % e
            return -1


    '''
        Index    :  6
        Function :  get a specified speed 
        Param    :  ofWhatAxis: specified axis, can be set X_axis/Y_axis/Z_axis/R_axis
        Return   :  speed value:succuess -1:fail
        Error    :  call strErrorMessage
    '''
    def GetSpeed(self,ofWhatAxis):
        if(self.ser.isOpen() == False):
            self.strErrorMessage = "The serial port is not open"
            return -1
        try:
            if (ofWhatAxis == self.X_axis):
                command = myBojayPLCCommandClass.GetSpeed_xAxis #'%01#RDD0020000201'
            elif(ofWhatAxis == self.Y_axis):
                command = myBojayPLCCommandClass.GetSpeed_yAxis #'%01#RDD0021000211'
            elif(ofWhatAxis == self.Z_axis):
                command = myBojayPLCCommandClass.GetSpeed_zAxis #'%01#RDD0022000221'
            elif (ofWhatAxis == self.R_axis):
                command = myBojayPLCCommandClass.GetSpeed_rAxis #'%01#RDD0023000231'
            bcc = self.__bccValue(command)
            command = command + bcc + '\r'
            command = command.upper()
            if Interpreter == '3':
                command = command.encode('utf-8')
            self.ser.write(command)

            #read data
            readString = self.ReadData(0.1)
            if("fail" in readString):
                self.strErrorMessage = "read data timeout"
                return -1
            value = self.__getValueOfByte(readString)
            if (ofWhatAxis == self.R_axis):
                return value*10
            return value
        except:
            self.strErrorMessage = "GetSpeed error"
            return  -1


    '''
      Index    :  7
      Function :  get the current coordinate 
      Param    :  ofWhatAxis: specified axis, can be set X_axis/Y_axis/Z_axis/R_axis
      Return   :  coordinate value:succuess -9999:fail
      Error    :  call strErrorMessage
    '''
    def GetCurrentCoordinates(self,ofWhatAxis):
        if(self.ser.isOpen() == False):
            self.strErrorMessage = "the serial port is not opened"
            return -9999
        try:
            if (ofWhatAxis == self.X_axis):
                command = myBojayPLCCommandClass.GetCoordiante_xAxis #'%01#RDD0014600147'
            elif(ofWhatAxis == self.Y_axis):
                command = myBojayPLCCommandClass.GetCoordiante_yAxis #'%01#RDD0015000151'
            elif(ofWhatAxis == self.Z_axis):
                command = myBojayPLCCommandClass.GetCoordiante_zAxis #'%01#RDD0015400155'
            elif(ofWhatAxis==self.R_axis):
                command = myBojayPLCCommandClass.GetCoordiante_rAxis #'%01#RDD0015800159'
            bcc = self.__bccValue(command)
            command = command + bcc + '\r'
            command = command.upper()
            if Interpreter == '3':
                command = command.encode('utf-8')
            self.ser.write(command)

            # read data
            bGetDataFromPLC = False
            readString =  self.ReadData(0.1)
            if ("fail" in readString):
                self.strErrorMessage = "GetCurrentCoordinates timeout"
                return -9999
            else:
                value = self.__getValueOfByte(readString)
                return round((value*10),2)
        except:
            self.strErrorMessage = "GetCurrentCoordinates error"
            return -9999

    '''
       Index    :  8
       Function :  All axis moves back to the origin 
       Param    :  timeout: the default is 15
       Return   :  0:succuess -1:fail
       Error    :  call strErrorMessage
    '''
    def SignalReset(self, timeout=15):
        try:
            mytimeCount = 0
            command = myBojayPLCCommandClass.ResetCommand_ON  # '%01#WCSR00841'
            bcc = self.__bccValue(command)
            command = command + bcc + '\r'
            ret = self.__writeRead(command)
            if (ret == 0):
                # wait for x-axis is ready
                time.sleep(self.myWaitTime)
                mytimeCount = 0
                while (self.GetHomeFinishState(self.XYZ_axis) == 1):
                    if (mytimeCount > timeout):
                        break
                    time.sleep(0.5)
                    mytimeCount += self.myWaitTime
                command = myBojayPLCCommandClass.ResetCommand_OFF  # '%01#WCSR00840'
                bcc = self.__bccValue(command)
                command = command + bcc + '\r'
                ret = self.__writeRead(command)
                if (ret == -1):
                    self.strErrorMessage = "SignalReset read error"
                    return -1
                if (mytimeCount > timeout):
                    self.strErrorMessage = "SignalReset Reset time out"
                    return -1
                return 0
            else:
                self.strErrorMessage = "SignalReset write command fail"
                return -1
        except Exception as e:
            self.strErrorMessage = "SignalReset except %s" % e
            return -1

    '''
    Index    :  9
    Function :  set a specified limit 
    Param    :  ofWhatAxis: specified axis, can be set X_axis/Y_axis/Z_axis/R_axis
                ofWhatLimit: max/min limit, can be set MaxLimit/MinLimit
                value: sepcified coordinate
    Return   :  0:succuess -1:fail
    Error    :  call strErrorMessage
    '''
    def SetLimit(self, ofWhatAxis, ofWhatLimit, value):
        try:
            # Set X-axis max / min limit
            if (ofWhatAxis == self.X_axis):
                if (ofWhatLimit == self.Max_limit):
                    command = myBojayPLCCommandClass.SetMaxLimit_xAxis  # '%01#WDD0210002101'
                elif (ofWhatLimit == self.Min_limit):
                    command = myBojayPLCCommandClass.SetMinLimit_xAxis  # '%01#WDD0210402105'
                else:
                    self.strErrorMessage = "SetPLCLimit input parameter is not correct"
                    return -1
            # Set Y-axis max / min limit
            elif (ofWhatAxis == self.Y_axis):
                if (ofWhatLimit == self.Max_limit):
                    command = myBojayPLCCommandClass.SetMaxLimit_yAxis  # '%01#WDD0210802109'
                elif (ofWhatLimit == self.Min_limit):
                    command = myBojayPLCCommandClass.SetMinLimit_yAxis  # '%01#WDD0211202113'
                else:
                    self.strErrorMessage = "SetPLCLimit input parameter is not correct"
                    return -1
            # Set Z-axis max / min limit
            elif (ofWhatAxis == self.Z_axis):
                if (ofWhatLimit == self.Max_limit):
                    command = myBojayPLCCommandClass.SetMaxLimit_zAxis  # '%01#WDD0211602117'
                elif (ofWhatLimit == self.Min_limit):
                    command = myBojayPLCCommandClass.SetMinLimit_zAxis  # '%01#WDD0212002121'
                else:
                    self.strErrorMessage = "SetPLCLimit input parameter is not correct"
                    return -1
            # Set R-axis max / min limit
            elif (ofWhatAxis == self.R_axis):
                if (ofWhatLimit == self.Max_limit):
                    command = myBojayPLCCommandClass.SetMaxLimit_rAxis  # '%01#WDD0212802129'
                elif (ofWhatLimit == self.Min_limit):
                    command = myBojayPLCCommandClass.SetMinLimit_rAxis  # '%01#WDD0213202133'
                else:
                    self.strErrorMessage = "SetPLCLimit input parameter is not correct"
                    return -1

            finalByte = self.__flipByte(value)
            command = command + finalByte
            ret = self.__writeRead(command)
            if ret != 0:
                self.strErrorMessage = 'SetLimit fail'
                return -1
            return 0
        except:
            self.strErrorMessage = "SetPLCLimit except"
            return -1

    '''
        Index    :  10
        Function :  Get a specified limit 
        Param    :  ofWhatAxis: specified axis, can be set X_axis/Y_axis/Z_axis/R_axis
                    ofWhatLimit: max/min limit, can be set MaxLimit/MinLimit
        Return   :  coordinate value:succuess -9999:fail
        Error    :  call strErrorMessage
    '''
    def GetLimit(self, ofWhatAxis, ofWhatLimit):
        if (self.ser.isOpen() == False):
            self.strErrorMessage = "the serial port is not open"
            return -9999
        try:
            # Get X-axis max / min limit
            if (ofWhatAxis == self.X_axis):
                if (ofWhatLimit == self.Max_limit):
                    command = myBojayPLCCommandClass.GetMaxLimit_xAxis  # '%01#RDD0062000621'
                elif (ofWhatLimit == self.Min_limit):
                    command = myBojayPLCCommandClass.GetMinLimit_xAxis  # '%01#RDD0063000631'
                else:
                    self.strErrorMessage = "GetLimit input parameter is not correct"
                    return -9999
            # Get Y-axis max / min limit
            elif (ofWhatAxis == self.Y_axis):
                if (ofWhatLimit == self.Max_limit):
                    command = myBojayPLCCommandClass.GetMaxLimit_yAxis  # '%01#RDD0062200623'
                elif (ofWhatLimit == self.Min_limit):
                    command = myBojayPLCCommandClass.GetMinLimit_yAxis  # '%01#RDD0063200633'
                else:
                    self.strErrorMessage = "GetLimit input parameter is not correct"
                    return -9999
            elif (ofWhatAxis == self.Z_axis):
                if (ofWhatLimit == self.Max_limit):
                    command = myBojayPLCCommandClass.GetMaxLimit_zAxis  # '%01#RDD0062400625'
                elif (ofWhatLimit == self.Min_limit):
                    command = myBojayPLCCommandClass.GetMinLimit_zAxis  # '%01#RDD0063400635'
                else:
                    self.strErrorMessage = "GetLimit input parameter is not correct"
                    return -9999
            elif (ofWhatAxis == self.R_axis):
                if (ofWhatLimit == self.Max_limit):
                    command = myBojayPLCCommandClass.GetMaxLimit_rAxis  # '%01#RDD0062400625'
                elif (ofWhatLimit == self.Min_limit):
                    command = myBojayPLCCommandClass.GetMinLimit_rAxis  # '%01#RDD0063400635'
                else:
                    self.strErrorMessage = "GetLimit input parameter is not correct"
                    return -9999
            else:
                self.strErrorMessage = "GetLimit input parameter is not correct"
                return -9999

            # write data
            bcc = self.__bccValue(command)
            command = command + bcc + '\r'
            command = command.upper()
            if Interpreter == '3':
                command = command.encode('utf-8')
            self.ser.write(command)

            # read data
            readString = self.ReadData(0.1)
            if ("fail" in readString):
                self.strErrorMessage = "read data timeout"
                return -9999
            else:
                value = self.__getValueOfByte(readString)
                return (value * 10)
        except:
            self.strErrorMessage = "GetLimit error"
            return -9999

    '''
    Index    :  11
    Function :  lock/unlock the dut
    Param    :  state: specified action, can be set LockDUT/UnlockDUT
    Return   :  0:succuess -1:fail
    Error    :  call strErrorMessage
    '''
    def DUTLockOrUnlock(self,state,OfWhatDUT=14):
        try:
            if state == self.DUTLock:
                command=myBojayPLCCommandClass.LockDUT
                exceptValue = 1

            elif state == self.DUTUnlock:
                command = myBojayPLCCommandClass.UnLockDUT
                exceptValue = 0
            ret = self.__writeRead(command)
            if ret != 0:
                return -1
            Timeout = 3
            mytimeout = 0
            while mytimeout < Timeout:
                ret1 = self.__readONorOFF(myBojayPLCCommandClass.DUTLockSensor)
                if (ret1 == exceptValue):
                    return 0
                time.sleep(0.1)
                mytimeout = mytimeout + 0.1
            if mytimeout >= Timeout:
                self.strErrorMessage = "DUTLockOrUnlock timeout"
                return -1
            return 0
        except Exception as e:
            self.strErrorMessage = 'DUTLockOrUnlock except %s' % e
            return -1

    '''
        Index    :  12
        Function :  get holder state
        Param    :  No
        Return   :  E71/Ethel/LUCY:succuess -1:fail
        Error    :  call strErrorMessage
    '''
    def GetHolderState(self):
        try:
            ListHolderCommand = [myBojayPLCCommandClass.HolderE71,myBojayPLCCommandClass.HolderEthel,myBojayPLCCommandClass.HolderLUCY]
            for command in ListHolderCommand:
                ret = self.__readONorOFF(command)
                if (ret == 1):
                    if command == ListHolderCommand[0]:
                        return "E71"
                    elif command == ListHolderCommand[1]:
                        return  "Ethel"
                    elif command == ListHolderCommand[2]:
                        return "LUCY"
                    else:
                        self.strErrorMessage = 'GetHolderState error'
                        return  -1
            self.strErrorMessage = 'GetHolderState error'
            return -1
        except Exception as e:
            self.strErrorMessage = 'GetHolderState except %s' % e
            return -1

    '''
        Index    :  13
        Function :  Get Led Light Color
        Param    :  ofWhichLED: DUT1/DUT2
                    ofWhatColor: Red_OFF/Red_ON/Yellow_OFF/Yellow_ON/Green_OFF/Green_ON
        Return   :  0:succuess -1:fail
        Error    :  call strErrorMessage
    '''
    def SetLedLightColor(self, ofWhichLED, ofWhatColor):
        try:
            if (self.ser.isOpen() == False):
                self.strErrorMessage = "The serial port is not open"
                return -1
            # Set DUT1 Color
            if (ofWhichLED == self.DUT1):
                # LEFT LED RED Color
                if (ofWhatColor == self.Red_OFF):
                    command = '%01#WCSR00850'
                elif (ofWhatColor == self.Red_ON):
                    command = '%01#WCSR00851'
                # LEFT LED Yellow Color
                elif (ofWhatColor == self.Yellow_OFF):
                    command = '%01#WCSR00860'
                elif (ofWhatColor == self.Yellow_ON):
                    command = '%01#WCSR00861'
                # LEFT LED Green Color
                elif (ofWhatColor == self.Green_OFF):
                    command = '%01#WCSR00870'
                elif (ofWhatColor == self.Green_ON):
                    command = '%01#WCSR00871'
                else:
                    self.strErrorMessage = "Input parameter error"
                    return -1  # Color Parameter Error
            # Set DUT2 Color
            elif (ofWhichLED == self.DUT2):
                # LEFT LED RED Color
                if (ofWhatColor == self.Red_OFF):
                    command = '%01#WCSR008A0'
                elif (ofWhatColor == self.Red_ON):
                    command = '%01#WCSR008A1'
                # LEFT LED Yellow Color
                elif (ofWhatColor == self.Yellow_OFF):
                    command = '%01#WCSR008B0'
                elif (ofWhatColor == self.Yellow_ON):
                    command = '%01#WCSR008B1'
                # LEFT LED Green Color
                elif (ofWhatColor == self.Green_OFF):
                    command = '%01#WCSR008C0'
                elif (ofWhatColor == self.Green_ON):
                    command = '%01#WCSR008C1'
                else:
                    self.strErrorMessage = "Input parameter error"
                    return -1  # Color Parameter Error

            ret = self.__writeRead(command)
            if (ret != 0):
                self.strErrorMessage = "SetLedLightColor write command fail"
                return -1
            return 0
        except Exception as e:
            self.strErrorMessage = "SetLedLightColor except %s" % e
            return -1  # Color Parameter Error

    '''
          Index    :  14
          Function :  check if the DUT is placed on the fixture
          Param    :  ofWhatSensor: CheckDUTSensor
          Return   :  1:Yes 0:No -1:fail
          Error    :  call strErrorMessage
    '''
    def GetSensorStatus(self, ofWhatSensor):
        if (self.ser.isOpen() == False):
            self.strErrorMessage = "The serial port is not open"
            return -1
        # X-axis Sensor
        if (ofWhatSensor == self.Sensor_X_Origin):
            command = myBojayPLCCommandClass.XAisHomeSensor  # '%01#RCSX0018'
        elif (ofWhatSensor == self.Sensor_X_Max):
            command = myBojayPLCCommandClass.XAxisHLSensor  # '%01#RCSX001A'
        elif (ofWhatSensor == self.Sensor_X_Min):
            command = myBojayPLCCommandClass.XAxisLLSensor  # '%01#RCSX0019'
        # Y-axis Sensor
        elif (ofWhatSensor == self.Sensor_Y_Origin):
            command = myBojayPLCCommandClass.YAisHomeSensor  # '%01#RCSX0015'
        elif (ofWhatSensor == self.Sensor_Y_Max):
            command = myBojayPLCCommandClass.YAxisHLSensor  # '%01#RCSX0017'
        elif (ofWhatSensor == self.Sensor_Y_Min):
            command = myBojayPLCCommandClass.YAxisLLSensor  # '%01#RCSX0016'
        # Z-axis Sensor
        elif (ofWhatSensor == self.Sensor_Z_Origin):
            command = myBojayPLCCommandClass.ZAisHomeSensor  # '%01#RCSX001B'
        elif (ofWhatSensor == self.Sensor_Z_Max):
            command = myBojayPLCCommandClass.ZAxisHLSensor  # '%01#RCSX001D'
        elif (ofWhatSensor == self.Sensor_Z_Min):
            command = myBojayPLCCommandClass.ZAxisLLSensor  # '%01#RCSX001C'
        # Z-axis Sensor
        elif (ofWhatSensor == self.Sensor_Z_Origin):
            command = myBojayPLCCommandClass.ZAisHomeSensor  # '%01#RCSX001B'
        elif (ofWhatSensor == self.Sensor_Z_Max):
            command = myBojayPLCCommandClass.ZAxisHLSensor  # '%01#RCSX001D'
        elif (ofWhatSensor == self.Sensor_Z_Min):
            command = myBojayPLCCommandClass.ZAxisLLSensor  # '%01#RCSX001C'
        # Other
        elif (ofWhatSensor == self.CylinderINSensor):
            command = myBojayPLCCommandClass.CylinderInSensor  # '%01#RCSX0011'
        elif (ofWhatSensor == self.CylinderOUTSensor):
            command = myBojayPLCCommandClass.CylinderOutSensor  # '%01#RCSX0012'
        elif (ofWhatSensor == self.Sensor_Curtain):
            command = myBojayPLCCommandClass.CurtainSensor  # '%01#RCSX0010'
        elif (ofWhatSensor == self.Sensor_Calibrate):
            command = myBojayPLCCommandClass.CalibrationSensor  # '%01#RCSX001F'
        elif (ofWhatSensor == self.CheckDUTSensor):
            command = '%01#RCSX0014'
        elif (ofWhatSensor == self.Sensor_TouchFinger):
            command = '%01#RCSX001E'
        else:
            self.strErrorMessage = "GetSensorStatus Input parameter error"
            return -1
        ret = self.__readONorOFF(command)
        ret = int(ret)
        if (ret == -1):
            self.strErrorMessage = "GetSensorStatus Read command fail"
            return -1
        return ret

    '''
         Index    :  15
         Function : Singl Axis moves back to the origin 
         Param    :  ofWhatAxis: specified axis, can be set X_axis/Y_axis/Z_axis/R_axis
                     timeout: the default is 15
         Return   :  0:succuess -1:fail
         Error    :  call strErrorMessage
    '''
    def SinglAxisGoHome(self, ofWhatAxis, timeout=15):
        try:
            if ofWhatAxis == self.X_axis:
                command = myBojayPLCCommandClass.SingleGoHome_XAxis  # '%01#WCSR03001'
            elif ofWhatAxis == self.Y_axis:
                command = myBojayPLCCommandClass.SingleGoHome_YAxis
            elif ofWhatAxis == self.Z_axis:
                command = myBojayPLCCommandClass.SingleGoHome_ZAxis
            elif ofWhatAxis == self.R_axis:
                command = myBojayPLCCommandClass.SingleGoHome_RAxis

            mytimeCount = 0
            bcc = self.__bccValue(command)
            command = command + bcc + '\r'
            ret = self.__writeRead(command)
            if (ret == 0):
                # wait for x-axis is ready
                time.sleep(self.myWaitTime)
                mytimeCount = 0
                while (self.GetHomeFinishState(ofWhatAxis) == 1):
                    if (mytimeCount > timeout):
                        break
                    time.sleep(0.5)
                    mytimeCount += self.myWaitTime
                command = myBojayPLCCommandClass.ResetCommand_OFF  # '%01#WCSR00840'
                bcc = self.__bccValue(command)
                command = command + bcc + '\r'
                ret = self.__writeRead(command)
                if (ret == -1):
                    self.strErrorMessage = "SignalReset read error"
                    return -1
                if (mytimeCount > timeout):
                    self.strErrorMessage = "SignalReset Reset time out"
                    return -1
                return 0
            else:
                self.strErrorMessage = "SignalReset write command fail"
                return -1
        except Exception as e:
            self.strErrorMessage = "SignalReset except %s" % e
            return -1

    '''
         Index    :  16
         Function : Set cylinder in or out
         Param    :  action: Cylinder_IN or Cylinder_OUT
         Return   :  0:succuess -1:fail
         Error    :  call strErrorMessage
    '''
    def Set_CylindeFunction(self,action):
        if(self.ser.isOpen() == False):
            self.strErrorMessage =  "The serial port is not opened"
            return  -1
        try:
            if (action == self.Cylinder_IN):
                command = "%01#WCSR00771"
            elif (action == self.Cylinder_OUT):
                command = "%01#WCSR00791"

            ret = self.__writeRead(command)
            if (ret != 0):
                self.strErrorMessage = "Set_CylindeFunction Read command fail"
                return -1

            #get
            if(action == self.Cylinder_IN):
                command = "%01#RCSX0011"
                exceptRet = 1
            elif(action == self.Cylinder_OUT):
                command = "%01#RCSX0012"
                exceptRet = 1


            timeOut = 5
            myTimeCount = 0
            while(myTimeCount < timeOut):
                ret = self.__readONorOFF(command)
                ret = int(ret)
                if(ret == -1):
                    self.strErrorMessage = "Set_CylindeFunction error"
                    return -1
                elif(ret != exceptRet):
                    time.sleep(0.2)
                    myTimeCount += 0.2
                elif(ret == exceptRet):
                    break
            if (action == self.Cylinder_IN or action == self.Cylinder_OUT):
                if (action == self.Cylinder_IN):
                    command = "%01#WCSR00770"
                elif (action == self.Cylinder_OUT):
                    command = "%01#WCSR00790"
                ret = self.__writeRead(command)
                if (ret != 0):
                    self.strErrorMessage = "Set_CylindeFunction Read command fail"
                    return -1
            if(myTimeCount >= timeOut):
                self.strErrorMessage = "Set_CylindeFunction time out"
                return -1
            return 0
        except:
            self.strErrorMessage = "Set_CylindeFunction error"
            return -1

    '''
        Index    :  17
        Function : Action touch pen A/B dowm or up 
        Param    :  state: PenA_Up\PenA_Down\PenB_Up\PenB_Down
        Return   :  0:succuess -1:fail
        Error    :  call strErrorMessage
    '''
    def ActionTouchPen(self, state):
        try:
            if (state == self.PenA_Up):
                command = "%01#WCSR00400"
                SensorCommand = myBojayPLCCommandClass.PenASensor
                exceptValue = 1
            elif (state == self.PenA_Down):
                command = "%01#WCSR00401"
                SensorCommand = myBojayPLCCommandClass.PenASensor
                exceptValue = 0
            elif (state == self.PenB_Up):
                command = "%01#WCSR00410"
                SensorCommand = myBojayPLCCommandClass.PenBSensor
                exceptValue = 1
            elif (state == self.PenB_Down):
                command = "%01#WCSR00411"
                SensorCommand = myBojayPLCCommandClass.PenBSensor
                exceptValue = 0
            ret = self.__writeRead(command)
            if (ret != 0):
                self.strErrorMessage = "ActionTouchPen error"
                return -1
            Timeout = 3
            mytimeout = 0
            while mytimeout < Timeout:
                ret1 = self.__readONorOFF(SensorCommand)
                if (ret1 == exceptValue):
                    return 0
                time.sleep(0.1)
                mytimeout = mytimeout + 0.1
            if mytimeout >= Timeout:
                self.strErrorMessage = "ActionTouchPen timeout"
                return -1
            return 0

            return 0
        except:
            self.strErrorMessage = "ActionTouchPen error"
            return -1

    '''
        Index    :  18
        Function : Set the value of the relative movement 
        Param    :  ofWhatAxis: specified axis, can be set X_axis/Y_axis/Z_axis/R_axis
                   Value: sepcified coordinate
        Return   :  0:succuess -1:fail
        Error    :  call strErrorMessage
    '''


    def SetStepValue(self, ofWhatAxis, Value):
        if (self.ser.isOpen() == False):
            self.strErrorMessage = "the serial port is not opened"
            return -1
        try:
            if (ofWhatAxis == self.X_axis):
                command = myBojayPLCCommandClass.SetStep_xAxis  # '%01#WDD0100001001'
            elif (ofWhatAxis == self.Y_axis):
                command = myBojayPLCCommandClass.SetStep_yAxis  # '%01#WDD0100801009'
            elif (ofWhatAxis == self.Z_axis):
                command = myBojayPLCCommandClass.SetStep_zAxis  # '%01#WDD0101601017'
            elif (ofWhatAxis == self.R_axis):
                command = myBojayPLCCommandClass.SetStep_rAxis  # '%01#WDD0099600997'
            finalByte = self.__flipByte(Value)
            command = command + finalByte
            ret = self.__writeRead(command)
            if (ret == -1):
                self.strErrorMessage = "SetStepValue write command fail"
                return -1
            return ret
        except Exception as e:
            self.strErrorMessage = "SetStepValue except %s" % e
            return -1


    # def DrawCicleFlag(self,state):
    #     self.bDrawCircle = state
    #
    # #DOE dot
    # def DotFunction(self,dotXPosition,dotYPosition,dotZPosition,times):
    #     try:
    #         err = self.MoveToCoordinates(self.Z_axis,0,5)
    #         if (err != 0):
    #             return -1
    #         for i in range(0,times,1):
    #             err = self.MoveToCoordinates(self.X_axis,dotXPosition,5)
    #             if (err != 0):
    #                 return -1
    #             err = self.MoveToCoordinates(self.Y_axis,dotYPosition,5)
    #             if (err != 0):
    #                 return -1
    #             err = self.MoveToCoordinates(self.Z_axis,dotZPosition,5)
    #             if (err != 0):
    #                 return -1
    #             err = self.MoveToCoordinates(self.Z_axis, 0,5)
    #             if (err != 0):
    #                 return -1
    #             err = self.MoveToCoordinates(self.X_axis, 0,5)
    #             if (err != 0):
    #                 return -1
    #             err = self.MoveToCoordinates(self.Y_axis, 0,5)
    #             if (err != 0):
    #                 return -1
    #         return 0
    #     except:
    #         self.strErrorMessage = 'DotFunction except'
    #         return -1

    #read data

    '''
           Index    :  19
           Function : read data for plc serial port
           Param    :  timeDelay: the delay in reading data
           Return   :  readString:succuess fail:fail
           
       '''

    def ReadData(self,timeDelay):
        bReadData = False
        for i in range(0, 5, 1):
            time.sleep(timeDelay)
            readString = self.ser.readline()
            if(len(readString) > 1):
                if Interpreter == '3':
                    return readString.decode()
                return readString
            else:
                continue
        if(bReadData == False):
            return "fail"

    # Get BCC Value

    '''
           Index    :  20
           Function : get bcc value
           Param    : code: data to be converted into BCC
           Return   :  bcc:succuess 
          
       '''
    def __bccValue(self, code):
        code1 = ord(code[0])
        code2 = ord(code[1])
        bcc = code1 ^ code2
        for i in range(code.__len__() - 2):
            codetem = ord(code[i + 2])
            bcc = bcc ^ codetem
        bcc = binascii.hexlify(struct.pack('>i', bcc))
        bcc = bcc[6:8]
        if Interpreter == '3':
            return bcc.decode()
        return bcc

    # Write and Read Command
    '''
              Index    :  21
              Function : Write and Read Command,check to see if the command was sent successsfully
              Param    : command : the command to send 
              Return   :  0:succuess -1:fail

          '''
    def __writeRead(self, command):
        try:
            bcc = self.__bccValue(command)
            command = command + bcc + '\r'
            command = command.upper()
            if Interpreter == '3':
                command = command.encode('utf-8')
            self.ser.write(command)
            readString = self.ReadData(0.1)
            if (readString[3] == '$'):
                return 0
            else:
                return -1
        except Exception as e:
            return -1

    # flip Byte Function
    '''
                  Index    :  22
                  Function : flip Byte Function
                  Param    : code : data to be converted into flipByte
                  Return   :  0:succuess -1:fail

              '''

    def __flipByte(self, code):
        code = float(code)
        code = int(code * 5000.0 / 5.0)
        X = binascii.hexlify(struct.pack('>i', code))

        byte1 = X[0:2]
        byte2 = X[2:4]
        byte3 = X[4:6]
        byte4 = X[6:8]
        finalbyte = byte4 + byte3 + byte2 + byte1
        finalbyte = finalbyte.upper()
        if Interpreter == '3':
            return finalbyte.decode()
        return finalbyte

    # __getValueOfByte function

    '''
              Index    :  23
              Function : get value of byte
              Param    : ByteString : string format of the byte
              Return   :  value:succuess 
              Error    :  call strErrorMessage

     '''
    def __getValueOfByte(self,ByteString):
        try:
            finalbyte = ByteString[6:14]
            byte1 = finalbyte[0:2]
            byte2 = finalbyte[2:4]
            byte3 = finalbyte[4:6]
            byte4 = finalbyte[6:8]
            finalbyte = byte4 + byte3 + byte2 + byte1
            #finalbyte = int(finalbyte, 16)
            #finalbyte = struct.unpack('!i', finalbyte.decode('hex'))[0]
            finalbyte = struct.unpack('!i', binascii.unhexlify(finalbyte))[0]
            finalbyte = float(finalbyte)
            Value = finalbyte * 5.0 / 5000.0
            return Value
        except Exception as e:
            self.strErrorMessage  = '__getValueOfByte except %s' % e
            return -1

    #__readONorOFF function
    '''
               Index    :  24
               Function : read the data of command, on or off
               Param    : command: the command to send
               Return   :  readState:succuess  -1:fail
      '''
    def __readONorOFF(self,command):
        bcc = self.__bccValue(command)
        command = command + bcc + '\r'
        command = command.upper()
        if Interpreter == '3':
            command = command.encode('utf-8')
        self.ser.write(command)
        readString = self.ReadData(0.1)#self.ser.readline()
        if("fail" in readString):
            return -1
        readState = readString[6]
        return int(readState)

    # __getCoordinatesFromFile function


    # def __getCoordinatesFromFile(self,FilePath):
    #     FinalSplitLine = []
    #     with open(FilePath) as f:
    #         for line in f:
    #             splitline = line.rstrip().split('\r')
    #             FinalSplitLine = FinalSplitLine + splitline
    #         f.close()
    #         return FinalSplitLine

    # __readONorOFF function
    '''
                  Index    :  25
                  Function : read software version
                  Return   :  readState:succuess  -1:fail
         '''
    def __readVer(self):
        try:
            ver = 'Ver1.0.2'
            return ver
        except Exception as e:
            pass

    def ReadVer(self):
        return self.__readVer(myBojayPLCCommandClass.ReadPLCVersion)

    # printHello function
    def printHello(self):
        print('Hello')

    #ChooseCOM function

    '''
                  Index    :  26
                  param    :  serial Name             
                  Function : choose com according to plc serial port
                  Return   :  readState:succuess  -1:fail
                  Error    :  call strErrorMessage
         '''
    def ChooseCOM(self,serialName):
        try:
            self.ser = serial.Serial(port=serialName,
                                    timeout=0.01,
                                    baudrate=115200,
                                    parity=serial.PARITY_ODD)
            self.ser.close()
            self.ser.open()
            if(self.ser.is_open):
                command = '%01#RDD0015400155'
                bcc = self.__bccValue(command)
                command = command + bcc + '\r'
                command = command.upper()
                if Interpreter == '3':
                    command = command.encode('utf-8')
                self.ser.write(command)
                readStr = self.ReadData(0.1)
                if("fail" in readStr):
                    self.strErrorMessage = "ChooseCOM read command fail"
                    return -1
                else:
                    err = self.GetAllAxisLimit()
                    if(err == -1):
                        return -1
                    return 0
            else:
                return 1
        except:
            self.strErrorMessage =  "ChooseCOM fail"
            return -1

    '''
                    Index    :  27
                    Function : relative movement to the positive direction according to setting the step value
                    param    :  ofWhatAxis:   ofWhatAxis: specified axis, can be set X_axis/Y_axis/Z_axis/R_axis       
                    Return   :  readState:succuess  -1:fail
                    Error    :  call strErrorMessage
           '''
    # Move Increment
    def MoveIncrement(self,ofWhatAxis):
        if(self.ser.isOpen() == False):
            self.strErrorMessage =  "The serial port is not open"
            return -1
        try:
            if (ofWhatAxis == self.X_axis):
                command = myBojayPLCCommandClass.MoveStep_xAxis #'%01#WCSR00201'
            elif(ofWhatAxis == self.Y_axis):
                command = myBojayPLCCommandClass.MoveStep_yAxis #'%01#WCSR00241'
            elif(ofWhatAxis == self.Z_axis):
                command = myBojayPLCCommandClass.MoveStep_zAxis #'%01#WCSR00281'
            elif (ofWhatAxis == self.R_axis):
                command = myBojayPLCCommandClass.MoveStep_rAxis #'%01#WCSR00391'
            ret = self.__writeRead(command)
            if(ret == -1):
                self.strErrorMessage = "MoveIncrement write command error"
                return -1
            timeout = 10
            if (ofWhatAxis == self.X_axis):
                # wait for x-axis is ready
                mytimeCount = 0
                while (self.GetmoveSignal(self.X_axis) == 1):
                    if (mytimeCount > timeout):
                        self.strErrorMessage = "MoveDecrement time out"
                        return -1
                    time.sleep(self.myWaitTime)
                    mytimeCount += self.myWaitTime

                xCurrentPosition = self.GetCurrentCoordinates(self.X_axis)
                if(xCurrentPosition == -9999):
                    return -1
                if(xCurrentPosition < (self.XAxisMinLimit+1) or xCurrentPosition > (self.XAxisMaxLimit-1)):
                    self.strErrorMessage = "X asix is exceed limit"
                    return -1

            elif (ofWhatAxis == self.Y_axis):
                mytimeCount = 0
                while (self.GetmoveSignal(self.Y_axis) == 1):
                    if (mytimeCount > timeout):
                        self.strErrorMessage = "MoveIncrement time out"
                        return -1
                    time.sleep(self.myWaitTime)
                    mytimeCount += self.myWaitTime

                yCurrentPosition = self.GetCurrentCoordinates(self.Y_axis)
                if(yCurrentPosition == -9999):
                    return -1
                if(yCurrentPosition < (self.YAxisMinLimit+1) or yCurrentPosition > (self.YAxisMaxLimit-1)):
                    self.strErrorMessage = "Y asix is exceed limit"
                    return -1

            elif (ofWhatAxis == self.Z_axis):
                mytimeCount = 0
                while (self.GetmoveSignal(self.Z_axis) == 1):
                    if (mytimeCount > timeout):
                        self.strErrorMessage = "MoveIncrement time out"
                        return -1
                    time.sleep(self.myWaitTime)
                    mytimeCount += self.myWaitTime

                zCurrentPosition = self.GetCurrentCoordinates(self.Z_axis)
                if(zCurrentPosition == -9999):
                    return -1
                if(zCurrentPosition < (self.ZAxisMinLimit+1) or zCurrentPosition > (self.ZAxisMaxLimit-1)):
                    self.strErrorMessage = "Z asix is exceed limit"
                    return -1
            elif (ofWhatAxis == self.R_axis):
                mytimeCount = 0
                while (self.GetmoveSignal(self.R_axis) == 1):
                    if (mytimeCount > timeout):
                        self.strErrorMessage = "MoveIncrement time out"
                        return -1
                    time.sleep(self.myWaitTime)
                    mytimeCount += self.myWaitTime

                rCurrentPosition = self.GetCurrentCoordinates(self.R_axis)
                if(rCurrentPosition == -9999):
                    return -1
                if(rCurrentPosition < (self.RAxisMinLimit) or rCurrentPosition > (self.RAxisMaxLimit)):
                    self.strErrorMessage = "R asix is exceed limit"
                    return -1
            return 0
        except Exception as e:
            self.strErrorMessage = "MoveIncrement except %s" % e
            return -1

    # Move decrement
    '''
                       Index    :  28
                       Function : relative movement to the opposite direction according to setting the step value
                       param    :  ofWhatAxis:   ofWhatAxis: specified axis, can be set X_axis/Y_axis/Z_axis/R_axis       
                       Return   :  readState:succuess  -1:fail
                       Error    :  call strErrorMessage
              '''
    def MoveDecrement(self,ofWhatAxis):
        if(self.ser.isOpen() == False):
            self.strErrorMessage = "the serial port is not open"
            return -1
        try:
            if (ofWhatAxis == self.X_axis):
                command = '%01#WCSR00211'
            elif(ofWhatAxis == self.Y_axis):
                command = '%01#WCSR00251'
            elif(ofWhatAxis == self.Z_axis):
                command = '%01#WCSR00291'
            elif (ofWhatAxis == self.R_axis):
                command = '%01#WCSR00391'
            ret = self.__writeRead(command)
            if(ret == -1):
                self.strErrorMessage = "MoveDecrement write command error"
                return -1
            timeout = 5
            if (ofWhatAxis == self.X_axis):
                # wait for x-axis is ready
                mytimeCount = 0
                while (self.GetmoveSignal(self.X_axis) == 1):
                    if (mytimeCount > timeout):
                        self.strErrorMessage = "MoveDecrement time out"
                        return -1
                    time.sleep(self.myWaitTime)
                    mytimeCount += self.myWaitTime

                xCurrentPosition = self.GetCurrentCoordinates(self.X_axis)
                if(xCurrentPosition == -9999):
                    return -1
                if(xCurrentPosition < (self.XAxisMinLimit+1) or xCurrentPosition > (self.XAxisMaxLimit-1)):
                    self.strErrorMessage = "X asix is exceed limit"
                    return -1
            elif (ofWhatAxis == self.Y_axis):
                mytimeCount = 0
                while (self.GetmoveSignal(self.Y_axis) == 1):
                    if (mytimeCount > timeout):
                        self.strErrorMessage = "MoveDecrement time out"
                        return -1
                    time.sleep(self.myWaitTime)
                    mytimeCount += self.myWaitTime

                yCurrentPosition = self.GetCurrentCoordinates(self.Y_axis)
                if(yCurrentPosition == -9999):
                    return -1
                if(yCurrentPosition < (self.YAxisMinLimit+1) or yCurrentPosition > (self.YAxisMaxLimit-1)):
                    self.strErrorMessage = "Y asix is exceed limit"
                    return -1
            elif (ofWhatAxis == self.Z_axis):
                mytimeCount = 0
                while (self.GetmoveSignal(self.Z_axis) == 1):
                    if (mytimeCount > timeout):
                        self.strErrorMessage = "MoveDecrement time out"
                        return -1
                    time.sleep(self.myWaitTime)
                    mytimeCount += self.myWaitTime

                zCurrentPosition = self.GetCurrentCoordinates(self.Z_axis)
                if(zCurrentPosition == -9999):
                    return -1
                if(zCurrentPosition < (self.ZAxisMinLimit+1) or zCurrentPosition > (self.ZAxisMaxLimit-1)):
                    self.strErrorMessage = "Z asix is exceed limit"
                    return -1
            elif (ofWhatAxis == self.R_axis):
                mytimeCount = 0
                while (self.GetmoveSignal(self.R_axis) == 1):
                    if (mytimeCount > timeout):
                        self.strErrorMessage = "MoveDecrement time out"
                        return -1
                    time.sleep(self.myWaitTime)
                    mytimeCount += self.myWaitTime

                rCurrentPosition = self.GetCurrentCoordinates(self.R_axis)
                if(rCurrentPosition == -9999):
                    return -1
                if(rCurrentPosition < (self.RAxisMinLimit+1) or rCurrentPosition > (self.RAxisMaxLimit-1)):
                    self.strErrorMessage = "Z asix is exceed limit"
                    return -1
            return 0
        except Exception as e:
            self.strErrorMessage = "MoveDecrement except %s" % e
            return -1

    '''
                           Index    :  28
                           Function : Get home finish state
                           param    :  ofWhatAxis:   ofWhatAxis: specified axis, can be set X_axis/Y_axis/Z_axis/R_axis       
                           Return   :  readState:succuess  -1:fail
                           Error    :  call strErrorMessage
                  '''
    def GetHomeFinishState(self,ofWhatAxis):
        if(self.ser.isOpen() == False):
            self.strErrorMessage =  "The serial port is not opened"
            return  -1
        try:
            if (ofWhatAxis == self.XYZ_axis):
                command = myBojayPLCCommandClass.SingleHomeFinish_xyzAxis #'%01#RCSR0104'
            elif(ofWhatAxis == self.X_axis):
                command = myBojayPLCCommandClass.SingleHomeFinish_xAxis #'%01#RCSR0100'
            elif(ofWhatAxis == self.Y_axis):
                command = myBojayPLCCommandClass.SingleHomeFinish_yAxis #'%01#RCSR0101'
            elif(ofWhatAxis == self.Z_axis):
                command = myBojayPLCCommandClass.SingleHomeFinish_zAxis #'%01#RCSR0102'
            elif (ofWhatAxis == self.R_axis):
                command = myBojayPLCCommandClass.SingleHomeFinish_rAxis  # '%01#RCSR0119'

            bcc = self.__bccValue(command)
            command = command + bcc + '\r'
            command = command.upper()
            if Interpreter == '3':
                command = command.encode('utf-8')
            self.ser.write(command)
            readString =  self.ReadData(0.01)
            if("fail" in readString):
                self.strErrorMessage = "GetHomeFinishState read time out"
                return -1
            readString = int(readString[6])
            if (readString == 1):
                return 0
            elif (readString == 0):
                return 1
        except:
            self.strErrorMessage = "GetHomeFinishState error"
            return -1


    '''
                           Index    :  29
                           Function : Get move signal
                           param    :  ofWhatAxis:   ofWhatAxis: specified axis, can be set X_axis/Y_axis/Z_axis/R_axis       
                           Return   :  readState:succuess  -1:fail
                           Error    :  call strErrorMessage
                  '''
    def GetmoveSignal(self,ofWhatAxis):
        if(self.ser.isOpen() == False):
            self.strErrorMessage =  "The serial port is not opened"
            return  -1
        try:
            if (ofWhatAxis == self.XY_axis):
                command = myBojayPLCCommandClass.SingleMoveFinish_xyAxis #'%01#RCSR005C'
            elif(ofWhatAxis == self.X_axis):
                command = myBojayPLCCommandClass.SingleMoveFinish_xAxis #'%01#RCSR0052'
            elif(ofWhatAxis == self.Y_axis):
                command = myBojayPLCCommandClass.SingleMoveFinish_yAxis #'%01#RCSR0054'
            elif(ofWhatAxis == self.Z_axis):
                command = myBojayPLCCommandClass.SingleMoveFinish_zAxis #'%01#RCSR0056'
            elif (ofWhatAxis == self.R_axis):
                command = myBojayPLCCommandClass.SingleMoveFinish_rAxis  # '%01#RCSR0057'
            bcc = self.__bccValue(command)
            command = command + bcc + '\r'
            command = command.upper()
            if Interpreter == '3':
                command = command.encode('utf-8')
            self.ser.write(command)
            readString =  self.ReadData(0.1)
            if("fail" in readString):
                self.strErrorMessage = "GetmoveSignal read time out"
                return -1
            readString = int(readString[6])
            if (readString == 1):
                return 0
            elif (readString == 0):
                return 1
            else:
                self.strErrorMessage = "GetmoveSignal error"
                return -1
        except:
            self.strErrorMessage = "GetmoveSignal error"
            return -1



    #return error message to UI
    def GetErrorMessage(self):
        return self.strErrorMessage

    '''
                              Index    :  30
                              Function : This function is used for Calibration position
                              Return   :  readState:succuess  -1:fail
                              Error    :  call strErrorMessage
                     '''
    def Calibrate(self,ofWhichAxis, incrementOrDecrement):
        try:
            shouldCalibrate = 1
            LoopCounter = 0
            Coordinates = 0.0
            isSuccess = 0

            StepValue = 0.5
            err = self.SetStepValue(self.X_axis,StepValue)
            if(err == -1):
                return -1
            err = self.SetStepValue(self.Y_axis,StepValue)
            if(err == -1):
                return -1
            err = self.SetStepValue(self.Z_axis,StepValue)
            if(err == -1):
                return -1

            while (shouldCalibrate):
                err = self.GetSensorStatus(self.Sensor_Calibrate)
                if(err == -1):
                    return -1
                elif(err == 0):
                    if("increment" in incrementOrDecrement):
                        ret = self.MoveIncrement(ofWhichAxis)
                        if(ret == -1):
                            return -1
                    elif("decrement" in incrementOrDecrement):
                        ret = self.MoveDecrement(ofWhichAxis)
                        if(ret == -1):
                            return -1
                elif(err == 1):
                    # back
                    if("increment" in incrementOrDecrement):
                        err = self.SetStepValue(ofWhichAxis, -3*StepValue)
                        if (err == -1):
                            return -1
                        ret = self.MoveIncrement(ofWhichAxis)
                        if(ret == -1):
                            return -1
                    elif("decrement" in incrementOrDecrement):
                        err = self.SetStepValue(ofWhichAxis, -3*StepValue)
                        if (err == -1):
                            return -1
                        ret = self.MoveDecrement(ofWhichAxis)
                        if(ret == -1):
                            return -1

                    LoopCounter = LoopCounter + 1
                    if (LoopCounter == 1):
                        StepValue = 0.1
                    elif (LoopCounter == 2):
                        StepValue = 0.02
                    elif (LoopCounter == 3):
                        CalPosition = self.GetCurrentCoordinates(ofWhichAxis)
                        if(CalPosition == -9999):
                            return -1
                        else:
                            # back
                            if ("increment" in incrementOrDecrement):
                                err = self.SetStepValue(ofWhichAxis, -3 * StepValue)
                                if (err == -1):
                                    return -1
                                ret = self.MoveIncrement(ofWhichAxis)
                                if (ret == -1):
                                    return -1
                            elif ("decrement" in incrementOrDecrement):
                                err = self.SetStepValue(ofWhichAxis, -3 * StepValue)
                                if (err == -1):
                                    return -1
                                ret = self.MoveDecrement(ofWhichAxis)
                                if (ret == -1):
                                    return -1
                            return round(float(CalPosition),2)
                    err = self.SetStepValue(self.X_axis,StepValue)
                    if(err == -1):
                        return -1
                    err = self.SetStepValue(self.Y_axis,StepValue)
                    if(err == -1):
                        return -1
                    err = self.SetStepValue(self.Z_axis,StepValue)
                    if(err == -1):
                        return -1
        except:
            self.strErrorMessage = "Calibrate error"
            return -1

    '''
                             Index    :  31
                             Function : Calibration position
                             param    :  offset: the offset of z axis 
                                        filename: Calibration file of result
                                        SafePosition : calibration safe positon
                             Return   :  readState:succuess  -1:fail
                             Error    :  call strErrorMessage
                    '''
    def CalibrationPosition(self,offset,filename,SafePosition):
        try:
            ### Declare Variables #####
            Xval_1 = 0.0
            Xval_2 = 0.0
            Xval_Final = 0.0
            Yval_1 = 0.0
            Yval_2 = 0.0
            Yval_Final = 0.0
            Zvsl_Finsl = 0.0
            StepSpeed = 15


            #3:Clynder in
            err = self.Set_CylindeFunction(self.Cylinder_IN)
            if(err == -1):
                return -1

            #4:Set speed
            err = self.SetSpeed(self.X_axis,StepSpeed)
            if(err == -1):
                return -1
            err = self.SetSpeed(self.Y_axis,StepSpeed)
            if(err == -1):
                return -1
            err = self.SetSpeed(self.Z_axis,StepSpeed)
            if(err == -1):
                return -1

            #read each test calibration initial position
            # exeFolderPath = os.getcwd()
            # # exeFolderPath += "\\CalibrationInitial.txt"
            # ###################################################
            # os_name=os.name
            # if(os_name=='posix'):
            #     exeFolderPath += "/CalibrationInitial.txt"
            # elif(os_name=='nt'):
            #     exeFolderPath += "\\CalibrationInitial.txt"
            ###################################################
            # exeFolderPath = os.path.join(exeFolderPath+"\\CalibrationInitial",path)
            exeFolderPath = os.getcwd()
            filepath = exeFolderPath + "/CalibrationInitial/"
            calibraionFile = open(filepath+filename)
            alllines = calibraionFile.readlines()
            for line in alllines:
                strLine = line.strip()
                if ("ZIncrementX=" in strLine):
                    zAxisCalX = float(strLine[strLine.find("=") + 1:])
                elif ("ZIncrementY=" in strLine):
                    zAxisCalY = float(strLine[strLine.find("=") + 1:])

                elif ("XAxisCalIncrementX=" in strLine):
                    XAxisCalIncrementX = float(strLine[strLine.find("=") + 1:])
                elif ("XAxisCalIncrementY=" in strLine):
                    XAxisCalIncrementY = float(strLine[strLine.find("=") + 1:])

                elif ("XAxisCalDecrementX=" in strLine):
                    XAxisCalDecrementX = float(strLine[strLine.find("=") + 1:])
                elif ("XAxisCalDecrementY=" in strLine):
                    XAxisCalDecrementY = float(strLine[strLine.find("=") + 1:])

                elif ("YAxisCalIncrementX=" in strLine):
                    YAxisCalIncrementX = float(strLine[strLine.find("=") + 1:])
                elif ("YAxisCalIncrementY=" in strLine):
                    YAxisCalIncrementY = float(strLine[strLine.find("=") + 1:])

                elif ("YAxisCalDecrementX=" in strLine):
                    YAxisCalDecrementX = float(strLine[strLine.find("=") + 1:])
                elif ("YAxisCalDecrementY=" in strLine):
                    YAxisCalDecrementY = float(strLine[strLine.find("=") + 1:])


            # 3:Z-axis calibration
            zSafeDistance = 0
            err = self.MoveToCoordinates(self.Z_axis,zSafeDistance,10)
            if(err == -1):
                return -1
            else:
                #1:Move to calibration position
                err = self.MoveToCoordinates(self.X_axis,zAxisCalX,10)
                if (err == -1):
                    return -1


                err = self.MoveToCoordinates(self.Y_axis,zAxisCalY,10)
                if (err == -1):
                    return -1

                CalibrationBlockHeight = 0
                Zvsl_Finsl = self.Calibrate(self.Z_axis,"increment")
                Zvsl_Finsl = Zvsl_Finsl + CalibrationBlockHeight
                if(Zvsl_Finsl == -1):
                    print("Z axis calibration error")
                    return -1
                else:
                    print("Z axis calibration value=" + str(Zvsl_Finsl))

            #4:X-axis calibration  X1
            err = self.MoveToCoordinates(self.Z_axis,zSafeDistance,10)
            if(err == -1):
                return -1
            err = self.MoveToCoordinates(self.X_axis, XAxisCalIncrementX,10)
            if (err == -1):
                return -1
            err = self.MoveToCoordinates(self.Y_axis, XAxisCalIncrementY,10)
            if (err == -1):
                return -1
            Zvalue_temp = float(Zvsl_Finsl)
            Zvalue_temp = Zvalue_temp + SafePosition
            err = self.MoveToCoordinates(self.Z_axis, Zvalue_temp, 10)
            if (err == -1):
                return -1
            else:
                Xval_1 = self.Calibrate(self.X_axis, 'increment')
                if (Xval_1 == -1):
                    print("X_axis increment fail")
                    return -1
                else:
                    print("X axis calibration value=" + str(Xval_1))


            # 4:X-axis calibration  X2
            err = self.MoveToCoordinates(self.Z_axis, zSafeDistance, 10)
            if (err == -1):
                return -1
            err = self.MoveToCoordinates(self.X_axis, XAxisCalDecrementX,10)
            if (err == -1):
                return -1
            err = self.MoveToCoordinates(self.Y_axis, XAxisCalDecrementY,10)
            if (err == -1):
                return -1
            err = self.MoveToCoordinates(self.Z_axis,Zvalue_temp,10)
            if(err == -1):
                return -1
            else:
                Xval_2 = self.Calibrate(self.X_axis, 'decrement')
                if(Xval_2 == -1):
                    print("X_axis increment fail")
                    return -1
                else:
                    print("X axis calibration value=" + str(Xval_2))
            Xval_Final = (Xval_1 + Xval_2) / 2
            print("X calibrate result:" + str(Xval_Final))

            # 5:Y-axis calibration Y1
            err = self.MoveToCoordinates(self.Z_axis, zSafeDistance, 10)
            if (err == -1):
                return -1
            err = self.MoveToCoordinates(self.X_axis, YAxisCalIncrementX,10)
            if (err == -1):
                return -1
            err = self.MoveToCoordinates(self.Y_axis, YAxisCalIncrementY,10)
            if (err == -1):
                return -1
            err = self.MoveToCoordinates(self.Z_axis,Zvalue_temp,10)
            if(err == -1):
                return -1
            else:
                Yval_1 = self.Calibrate(self.Y_axis, 'increment')
                if(Yval_1 == -1):
                    print("Y_axis increment fail")
                    return -1
                else:
                    print("Y axis calibration value=" + str(Yval_1))

            # 5:Y-axis calibration Y2
            err = self.MoveToCoordinates(self.Z_axis, zSafeDistance, 10)
            if (err == -1):
                return -1
            err = self.MoveToCoordinates(self.X_axis, YAxisCalDecrementX,10)
            if (err == -1):
                return -1
            err = self.MoveToCoordinates(self.Y_axis, YAxisCalDecrementY,10)
            if (err == -1):
                return -1
            err = self.MoveToCoordinates(self.Z_axis, Zvalue_temp,10)
            if (err == -1):
                return -1
            else:
                Yval_2 = self.Calibrate(self.Y_axis, 'decrement')
                if (Yval_2 == -1):
                    print("Y_axis increment fail")
                    return -1
                else:
                    print("Y axis calibration value=" + str(Yval_2))
            Yval_Final = (Yval_1 + Yval_2) / 2
            print("Y calibrate result:" + str(Yval_Final))


            #Save calibrtion file
            Zvsl_Finsl = Zvsl_Finsl + offset
            exeFolderPath = os.getcwd()
            exeFolderPath += "/Calibration/"
            if not os.path.exists(exeFolderPath):
                os.mkdir(exeFolderPath)
            exeFolderPath = os.path.join(exeFolderPath, filename)
            output = open(exeFolderPath, 'w')
            output.write("X1=" + str(Xval_1) + "\n")
            output.write("X2=" + str(Xval_2) + "\n")
            output.write("X-Finial=" + str(Xval_Final) + "\n")
            output.write("Y1=" + str(Yval_1) + "\n")
            output.write("Y2=" + str(Yval_2) + "\n")
            output.write("Y-Finial=" + str(Yval_Final) + "\n")
            output.write("Z-Finial=" + str(Zvsl_Finsl) + "\n")

            err = self.MoveToCoordinates(self.Z_axis, zSafeDistance,10)
            if (err == -1):
                return -1
            err = self.MoveToCoordinates(self.X_axis, Xval_Final,10)
            if (err == -1):
                return -1
            err = self.MoveToCoordinates(self.Y_axis, Yval_Final,10)
            if (err == -1):
                return -1
            return 0
        except Exception as e:
            self.strErrorMessage = "CalibrationPosition error"
            return -1


    def MoveToCalPosition(self,XPosition,YPosition,ZPosition):
        try:
            ret = self.MoveToCoordinates(self.X_axis,XPosition)
            if ret == -1:
                self.strErrorMessage = "Move XAxis CalPosition error"
                return  -1

            ret = self.MoveToCoordinates(self.Y_axis, YPosition)
            if ret == -1:
                self.strErrorMessage = "Move XAxis CalPosition error"
                return -1

            ret = self.MoveToCoordinates(self.Z_axis, ZPosition)
            if ret == -1:
                self.strErrorMessage = "Move XAxis CalPosition error"
                return -1

            return 0


        except:
            self.strErrorMessage = "MoveToCalPosition error"
            return -1

    def CalZAxisHeight(self):
        try:
            # 3:Clynder in
            err = self.Set_CylindeFunction(self.Cylinder_IN)
            if (err == -1):
                return -1

            # 4:Set speed
            StepSpeed = 15
            err = self.SetSpeed(self.X_axis, StepSpeed)
            if (err == -1):
                return -1
            err = self.SetSpeed(self.Y_axis, StepSpeed)
            if (err == -1):
                return -1
            err = self.SetSpeed(self.Z_axis, StepSpeed)
            if (err == -1):
                return -1

            # read each test calibration initial position
            # exeFolderPath = os.getcwd()
            # # exeFolderPath += "\\CalibrationInitial.txt"
            # ###################################################
            # os_name=os.name
            # if(os_name=='posix'):
            #     exeFolderPath += "/CalibrationInitial.txt"
            # elif(os_name=='nt'):
            #     exeFolderPath += "\\CalibrationInitial.txt"
            ###################################################
            # exeFolderPath = os.path.join(exeFolderPath+"\\CalibrationInitial",path)

            # 3:Z-axis calibration
            zSafeDistance = 5
            ZAxisPosition = ["Left_Up","Left_Down","Right_Down","Right_UP"]
            ZAxisPosition_4Point = [[-45,-80],[90,0],[0,160],[-90,0]]
            exeFolderPath = os.getcwd()
            filename = "ZAxisCalHeight.txt"
            exeFolderPath = os.path.join(exeFolderPath, filename)
            output = open(exeFolderPath, 'w')
            for i in range(0,4):
                err = self.MoveToCoordinates(self.Z_axis, zSafeDistance, 10)
                if (err == -1):
                    return -1
                else:
                    err = self.SetStepValue(self.X_axis, float(ZAxisPosition_4Point[i][0]))
                    if err != 0:
                        self.strErrorMessage = "SetStepValue error"
                        return -1
                    err = self.MoveIncrement(self.X_axis)
                    if err != 0:
                        self.strErrorMessage = "MoveIncrement error"
                        return -1
                    err = self.SetStepValue(self.Y_axis, float(ZAxisPosition_4Point[i][1]))
                    if err != 0:
                        self.strErrorMessage = "SetStepValue error"
                        return -1
                    err = self.MoveIncrement(self.Y_axis)
                    if err != 0:
                        self.strErrorMessage = "SetStepValue error"
                        return -1
                    CalibrationBlockHeight = 0
                    Zvsl_Finsl = self.Calibrate(self.Z_axis, "increment")
                    Zvsl_Finsl = Zvsl_Finsl + CalibrationBlockHeight
                    if (Zvsl_Finsl == -1):
                        print("Z axis calibration error")
                        return -1
                    else:
                        print("Z axis calibration value=" + str(Zvsl_Finsl))
                    output.write(ZAxisPosition[i] + str(Zvsl_Finsl) + "\n")
            # err = self.SinglAxisGoHome(self.X_axis)
            # if err != 0:
            #     self.ShowErroeMessage(self.strErrorMessage)
            #     return -1
            # err = self.SinglAxisGoHome(self.Y_axis)
            # if err != 0:
            #     self.ShowErroeMessage(self.strErrorMessage)
            #     return -1
            # err = self.SinglAxisGoHome(self.Z_axis)
            # if err != 0:
            #     self.ShowErroeMessage(self.strErrorMessage)
            #     return -1
        except Exception as e:
            self.strErrorMessage = "MoveToCalPosition error"
            return -1

    '''
                               Index    :  32
                               Function : made alarm off or on
                               param    :  state：Alarm_On/Alarm_Off
                               Return   :  readState:succuess  -1:fail
                               Error    :  call strErrorMessage
                      '''
    def AlarmBuzzer(self,state):
        if (self.ser.isOpen() == False):
            self.strErrorMessage = "The serial port is not opened"
            return -1
        try:
            if(state == self.Alarm_On):
                command = "%01#WCSR01300"
            elif(state == self.Alarm_Off):
                command = "%01#WCSR01301"
            ret = self.__writeRead(command)
            if (ret != 0):
                self.strErrorMessage = "AlarmBuzzer Red command fail"
                return -1
            return 0
        except Exception as e:
            self.strErrorMessage = "AlarmBuzzer except %s" % e
            return -1

    '''
                                Index    :  33
                                Function : Get all axis of limit
                                param    :  
                                Return   :  readState:succuess  -1:fail
                                Error    :  call strErrorMessage
                       '''
    def GetAllAxisLimit(self):
        try:
            ret = self.GetLimit(self.Z_axis, self.Max_limit)
            if (ret == -9999):
                return -1
            else:
                self.ZAxisMaxLimit = float(ret)
            ret = self.GetLimit(self.Z_axis, self.Min_limit)
            if (ret == -9999):
                return -1
            else:
                self.ZAxisMinLimit = float(ret)

            ret = self.GetLimit(self.X_axis, self.Max_limit)
            if (ret == -9999):
                return -1
            else:
                self.XAxisMaxLimit = float(ret)
            ret = self.GetLimit(self.X_axis, self.Min_limit)
            if (ret == -9999):
                return -1
            else:
                self.XAxisMinLimit = float(ret)
            ret = self.GetLimit(self.Y_axis, self.Max_limit)
            if (ret == -9999):
                return -1
            else:
                self.YAxisMaxLimit = float(ret)
            ret = self.GetLimit(self.Y_axis, self.Min_limit)
            if (ret == -9999):
                return -1
            else:
                self.YAxisMinLimit = float(ret)
            ret = self.GetLimit(self.R_axis, self.Max_limit)
            if (ret == -9999):
                return -1
            else:
                self.RAxisMaxLimit = float(ret)
            ret = self.GetLimit(self.R_axis, self.Min_limit)
            if (ret == -9999):
                return -1
            else:
                self.RAxisMinLimit = float(ret)
            return 0
        except:
            return -1
































