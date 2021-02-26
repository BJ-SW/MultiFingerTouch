from GOEPLCControl import *

ControlInterface = GOEControlClass()
def main():
    try:
        #************************************************#
        '''
        MultiFingerTouch 4 Cases:
            This is sample code for the 4 cases - single probe,multi finger vertical swipes,
        multi finger horizontal swipers and pinch/zoom for D6-L10 
        '''
        #************************************************#


        #Step1: open serial port
        err = ControlInterface.OpenSerial()
        print('open serial port return err {}'.format(err))

        # Step2: Reset Fixture
        err = ControlInterface.SignalReset()
        print('reset return err {}'.format(err))

        #Step3: Lock Dut
        err = ControlInterface.DUTLockOrUnlock(ControlInterface.DUTLock)
        print('lock dut return err {}'.format(err))

        #Step4:check if the DUT is placed on the fixture
        err = ControlInterface.GetSensorStatus(ControlInterface.CheckDUTSensor)
        print('GetSensorStatus return err {}'.format(err))

        # Step5:Get Holder State,execute the corresponding program according to holder
        err = ControlInterface.GetHolderState()
        print('GetHolderState return err {}'.format(err))

        # Step4: Cylinder in
        err = ControlInterface.Set_CylindeFunction(ControlInterface.Cylinder_IN)
        print('Cylinder in return err {}'.format(err))

        # Step5: Set Axis speed
        err = ControlInterface.SetSpeed(ControlInterface.X_axis,60)
        print('SetSpeed return err {}'.format(err))
        err = ControlInterface.SetSpeed(ControlInterface.Y_axis,60)
        print('SetSpeed return err {}'.format(err))
        err = ControlInterface.SetSpeed(ControlInterface.Z_axis, 15)
        print('SetSpeed return err {}'.format(err))
        err = ControlInterface.SetSpeed(ControlInterface.R_axis, 30)
        print('SetSpeed return err {}'.format(err))

        case = 1
        if case == 1:
            Case1_Single_Probe()
        elif case == 2:
            Case2_vertical_swipes()
        elif case == 3:
            Case3_horizontal_swipers()
        elif case == 4:
            Case4_pinch_swipers()
    except Exception as ex:
        print(ex)


def Case1_Single_Probe():
    try:
        '''
        Case1 - Single Probe
            (1) Select Touch Pen A as the center position:
                    open file:MultiFingerTouch\\Calibration\\6mm_校准位置10mm_水平画线后笔头.txt(后笔头为A,前笔头为B)
            (2) Get Calibrated X and Y absolute Position:
                X_Position:-67.4
                Y_Position:-12.26
                Z_Position:13.78
            (3) Made Touch Pen A down
            (4) Drawing rectangle 
                For example:
                    1.make sure the four coordinates of the rectangle:
                        Left_Up = [X_Position - 30, Y_Position - 30]
                        Left_Down = [X_Position + 30, Y_Position - 30]
                        Right_Up = [X_Position - 30, Y_Position + 30]
                        Right_Down = [X_Position + 30, Y_Position + 30]
                    2.Move Axis to LeftUp Position
                    3.Make ToucbPenA down
                    4.Move ZAxis to the touch position
                    5.Move Axis to LeftDown Position
                    6.Move Axis to RightUp Position
                    7.Move Axis to RightDown Position
                    8.Move ZAxis to the safe position
            (5) Finish test

        '''
        # 1.make sure the four coordinates of the rectangle
        X_Position = -67.4
        Y_Position = -12.26
        Z_Position = 13.78
        Left_Up = [X_Position - 30, Y_Position - 30]
        Left_Down = [X_Position + 30, Y_Position - 30]
        Right_Up = [X_Position - 30, Y_Position + 30]
        Right_Down = [X_Position + 30, Y_Position + 30]



        # 2.Move Axis to LeftUp Position
        err = ControlInterface.MoveToCoordinates(ControlInterface.X_axis, Left_Up[0])
        print('X Axis MoveToCoordinates  return err {}'.format(err))

        err = ControlInterface.MoveToCoordinates(ControlInterface.Y_axis, Left_Up[1])
        print('Y Axis MoveToCoordinates  return err {}'.format(err))

        # 3.Make ToucbPenA down
        err = ControlInterface.ActionTouchPen(ControlInterface.PenA_Down)
        print('ActionTouchPen  return err {}'.format(err))


        # 4.Move ZAxis to the touch position
        err = ControlInterface.MoveToCoordinates(ControlInterface.Z_axis, Z_Position)
        print('Z Axis MoveToCoordinates  return err {}'.format(err))

        # 5.Move Axis to LeftDown Position
        err = ControlInterface.MoveToCoordinates(ControlInterface.X_axis, Left_Down[0])
        print('X Axis MoveToCoordinates  return err {}'.format(err))

        err = ControlInterface.MoveToCoordinates(ControlInterface.Y_axis, Left_Down[1])
        print('Y Axis MoveToCoordinates  return err {}'.format(err))

        # 6.Move Axis to RightUp Position
        err = ControlInterface.MoveToCoordinates(ControlInterface.X_axis, Right_Down[0])
        print('X Axis MoveToCoordinates  return err {}'.format(err))

        err = ControlInterface.MoveToCoordinates(ControlInterface.Y_axis, Right_Down[1])
        print('Y Axis MoveToCoordinates  return err {}'.format(err))

        # 7.Move Axis to RightDown Position
        err = ControlInterface.MoveToCoordinates(ControlInterface.X_axis, Right_Up[0])
        print('X Axis MoveToCoordinates  return err {}'.format(err))

        err = ControlInterface.MoveToCoordinates(ControlInterface.Y_axis, Right_Up[1])
        print('Y Axis MoveToCoordinates  return err {}'.format(err))

        # 8.Move ZAxis to the safe position
        err = ControlInterface.MoveToCoordinates(ControlInterface.Z_axis, 0)
        print('Z Axis MoveToCoordinates  return err {}'.format(err))

    except Exception as ex:
        print(ex)

def Case2_vertical_swipes():
    try:
        '''
             Case2 - Multi finger vertical swipes
                 (1) Select Touch Pen D as the center position
                        open file:MultiFingerTouch\\Calibration\\6mm_校准位置10mm_垂直画线左笔头.txt(左笔头为D笔头,右笔头为C笔头)
                 (2) Get Calibrated X and Y absolute Position:
                     X Position:9.0
                     Y Position:-8.56
                     Z Position:29.1
                 (3) Drawing Tow line
                    For example:
                        1.make sure the four coordinates of the tow line:
                            FirstLine_1 = [X_Position - 30, Y_Position - 30]
                            FirstLine_2 = [X_Position + 30, Y_Position - 30]
                            SecondLine_1 = [X_Position - 30, Y_Position + 30]
                            SecondLine_2 = [X_Position + 30, Y_Position + 30]
                        2.Move Axis to FirstLine_1 Position
                        3.Move ZAxis to the touch position
                        4.Move Axis to FirstLine_2 Position
                        5.Move ZAxis to the safe position
                        6.Move Axis to SecondLine_1 Position
                        7.Move ZAxis to the touch position
                        8.Move Axis to SecondLine_2 Position
                        9.Move ZAxis to the safe position
                 (4) Finish test
        '''
        # 1.make sure the four coordinates of the tow line:
        X_Position = 9.0
        Y_Position = -8.56
        Z_Position = 29.1
        FirstLine_1 = [X_Position - 30, Y_Position - 30]
        FirstLine_2 = [X_Position + 30, Y_Position - 30]
        SecondLine_1 = [X_Position - 30, Y_Position + 30]
        SecondLine_2 = [X_Position + 30, Y_Position + 30]

        # 2.Move Axis to FirstLine_1 position
        err = ControlInterface.MoveToCoordinates(ControlInterface.X_axis, FirstLine_1[0])
        print('X Axis MoveToCoordinates  return err {}'.format(err))

        err = ControlInterface.MoveToCoordinates(ControlInterface.Y_axis, FirstLine_1[1])
        print('Y Axis MoveToCoordinates  return err {}'.format(err))

        # 3.Move ZAxis to the touch position
        err = ControlInterface.MoveToCoordinates(ControlInterface.Z_axis, Z_Position)
        print('Y Axis MoveToCoordinates  return err {}'.format(err))

        # 4.Move Axis to FirstLine_2 position
        err = ControlInterface.MoveToCoordinates(ControlInterface.X_axis, FirstLine_2[0])
        print('X Axis MoveToCoordinates  return err {}'.format(err))

        err = ControlInterface.MoveToCoordinates(ControlInterface.Y_axis, FirstLine_2[1])
        print('Y Axis MoveToCoordinates  return err {}'.format(err))

        # 5.Move ZAxis to the safe position
        err = ControlInterface.MoveToCoordinates(ControlInterface.Z_axis, 0)
        print('Y Axis MoveToCoordinates  return err {}'.format(err))

        # 6.Move Axis to SecondLine_1 position
        err = ControlInterface.MoveToCoordinates(ControlInterface.X_axis, SecondLine_1[0])
        print('X Axis MoveToCoordinates  return err {}'.format(err))

        err = ControlInterface.MoveToCoordinates(ControlInterface.Y_axis, SecondLine_1[1])
        print('Y Axis MoveToCoordinates  return err {}'.format(err))

        # 7.Move ZAxis to the touch position
        err = ControlInterface.MoveToCoordinates(ControlInterface.Z_axis, Z_Position)
        print('Y Axis MoveToCoordinates  return err {}'.format(err))

        # 8.Move Axis to SecondLine_2 position
        err = ControlInterface.MoveToCoordinates(ControlInterface.X_axis, SecondLine_2[0])
        print('X Axis MoveToCoordinates  return err {}'.format(err))

        err = ControlInterface.MoveToCoordinates(ControlInterface.Y_axis, SecondLine_2[1])
        print('Y Axis MoveToCoordinates  return err {}'.format(err))

        # 9.Move ZAxis to the safe position
        err = ControlInterface.MoveToCoordinates(ControlInterface.Z_axis, 0)
        print('Z Axis MoveToCoordinates  return err {}'.format(err))
    except Exception as ex:
        print(ex)

def Case3_horizontal_swipers():
    try:

        '''
             Case3 - Multi finger horizontal swipes
                 (1) Select Touch Pen A as the center
                        open file:MultiFingerTouch\\Calibration\\6mm_校准位置10mm_水平画线后笔头.txt(后笔头为A笔头,前笔头为B笔头)
                 (2) Get Calibrated X and Y absolute Position:
                     X_Position:-67.4
                     Y_Position:-12.26
                     Z_Position:13.78
                 (3) Drawing Tow line 
                     For example:
                        1.make sure the four coordinates of the tow line:
                            FirstLine_1 = [X_Position - 30, Y_Position - 30]
                            FirstLine_2 = [X_Position + 30, Y_Position - 30]
                            SecondLine_1 = [X_Position - 30, Y_Position + 30]
                            SecondLine_2 = [X_Position + 30, Y_Position + 30]
                        2.Move Axis to FirstLine_1 Position
                        3.Make TouchPenA and TouchPenB down
                        4.Move ZAxis to the touch position
                        5.Move Axis to FirstLine_2 Position
                        6.Move ZAxis to the safe position
                        7.Move Axis to SecondLine_1 Position
                        8.Move ZAxis to the touch position
                        9.Move Axis to SecondLine_2 Position
                        10.Move ZAxis to the safe position
                 (4) Finish test
        '''
        # 1.make sure the four coordinates of the tow line:
        X_Position = 9.0
        Y_Position = -8.56
        Z_Position = 13.78
        FirstLine_1 = [X_Position - 30, Y_Position - 30]
        FirstLine_2 = [X_Position - 30, Y_Position + 30]
        SecondLine_1 = [X_Position + 30, Y_Position - 30]
        SecondLine_2 = [X_Position + 30, Y_Position + 30]

        # 2.Move Axis to FirstLine_1 Position
        err = ControlInterface.MoveToCoordinates(ControlInterface.X_axis, FirstLine_1[0])
        print('X Axis MoveToCoordinates  return err {}'.format(err))

        err = ControlInterface.MoveToCoordinates(ControlInterface.Y_axis, FirstLine_1[1])
        print('Y Axis MoveToCoordinates  return err {}'.format(err))

        # 3.Made TouchPenA and TouchPenB dow
        err = ControlInterface.ActionTouchPen(ControlInterface.PenA_Down)
        print('ActionTouchPen  return err {}'.format(err))
        err = ControlInterface.ActionTouchPen(ControlInterface.PenB_Down)
        print('ActionTouchPen  return err {}'.format(err))

        # 4.Move ZAxis to the touch position
        err = ControlInterface.MoveToCoordinates(ControlInterface.Z_axis, Z_Position)
        print('Y Axis MoveToCoordinates  return err {}'.format(err))

        # 5.Move Axis to FirstLine_2 Position
        err = ControlInterface.MoveToCoordinates(ControlInterface.X_axis,FirstLine_2[0])
        print('X Axis MoveToCoordinates  return err {}'.format(err))

        err = ControlInterface.MoveToCoordinates(ControlInterface.Y_axis, FirstLine_2[1])
        print('Y Axis MoveToCoordinates  return err {}'.format(err))

        # 6.Move ZAxis to the safe position
        err = ControlInterface.MoveToCoordinates(ControlInterface.Z_axis, 0)
        print('Y Axis MoveToCoordinates  return err {}'.format(err))

        # 7.Move Axis to SecondLine_1 Position
        err = ControlInterface.MoveToCoordinates(ControlInterface.X_axis, SecondLine_1[0])
        print('X Axis MoveToCoordinates  return err {}'.format(err))

        err = ControlInterface.MoveToCoordinates(ControlInterface.Y_axis, SecondLine_1[1])
        print('Y Axis MoveToCoordinates  return err {}'.format(err))

        # 8.Move ZAxis to the touch position
        err = ControlInterface.MoveToCoordinates(ControlInterface.Z_axis, Z_Position)
        print('Y Axis MoveToCoordinates  return err {}'.format(err))

        # 9.Move Axis to SecondLine_2 Position
        err = ControlInterface.MoveToCoordinates(ControlInterface.X_axis, SecondLine_2[0])
        print('X Axis MoveToCoordinates  return err {}'.format(err))

        err = ControlInterface.MoveToCoordinates(ControlInterface.Y_axis, SecondLine_2[1])
        print('Y Axis MoveToCoordinates  return err {}'.format(err))

        # 10.Move ZAxis to the safe position
        err = ControlInterface.MoveToCoordinates(ControlInterface.Z_axis, 0)
        print('Z Axis MoveToCoordinates  return err {}'.format(err))
    except Exception as ex:
        print(ex)

def Case4_pinch_swipers():
    try:
        '''
             Case3 - Multi finger horizontal swipes
                 (1) Select Touch Pen D as the center
                        open file:MultiFingerTouch\\Calibration\\6mm_校准位置10mm_垂直画线左笔头.txt(左笔头为D笔头,右笔头为C笔头)
                 (2) Get Calibrated X and Y absolute Position:
                     X Position:9.0
                     Y Position:-8.56
                     Z Position:29.1
                 (3) Drawing One line:
                        1.make sure the two coordinates of the One line::
                        2. Move to Calibration position
                        3. Move ZAxis to the touch position
                        4. Move RAxis to Pinch_2 position
                        5. Move RAxis to Pinch_1 position
                 (4) Finish test
        '''
        X_Position = 9.0
        Y_Position = -8.56
        Z_Position = 29.1

        OneLine_pinch_1 = 0
        OneLine_pinch_2 = 60
        # Move Axis to Calibration position
        err = ControlInterface.MoveToCoordinates(ControlInterface.X_axis, X_Position)
        print('X Axis MoveToCoordinates  return err {}'.format(err))

        err = ControlInterface.MoveToCoordinates(ControlInterface.Y_axis, Y_Position)
        print('Y Axis MoveToCoordinates  return err {}'.format(err))

        # Move ZAxis to the touch position
        err = ControlInterface.MoveToCoordinates(ControlInterface.Z_axis, Z_Position)
        print('Y Axis MoveToCoordinates  return err {}'.format(err))

        # Move RAxis to Pinch_2 position
        err = ControlInterface.MoveToCoordinates(ControlInterface.R_axis, OneLine_pinch_2)
        print('X Axis MoveToCoordinates  return err {}'.format(err))

        # Move RAxis to Pinch_1 position
        err = ControlInterface.MoveToCoordinates(ControlInterface.R_axis, OneLine_pinch_1)
        print('X Axis MoveToCoordinates  return err {}'.format(err))

        # Move ZAxis to the safe position
        err = ControlInterface.MoveToCoordinates(ControlInterface.Z_axis, 0)
        print('Z Axis MoveToCoordinates  return err {}'.format(err))
    except Exception as ex:
        print(ex)

if __name__ == '__main__':
    main()