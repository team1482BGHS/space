#!/usr/bin/env python3
"""
This is a program showing the use of the RobotDrive class,
specifically it contains the code necessary to operate a robot with
a single joystick
"""

import math

import wpilib
import wpilib.drive
import wpilib.cameraserver as cameraserver
# from cscore import CameraServer
import ctre

def float_round(x, prec=2, base=.05):
  """Directly copied from StackOverflow"""
  return round(base * round(float(x) / base), prec)

def sigmoid(x):
  """1 / (1 + e ^ -x)"""
  return 1 / (1 + math.exp(-x))

class Robot(wpilib.IterativeRobot):
  """Deep space 2019 robot code"""

  # stick 1
  AXIS_THROTTLE = 1  # l stick
  AXIS_STEER = 0  # l stick
  # AXIS_YAW = 4
  # AXIS_PITCH = 5
  BUTTON_SHIFT = 6  # r bumper

  # stick 2
  AXIS_LIFT = 1  # r stick
  AXIS_REAR_LIFT = 5  # l stick
  AXIS_REAR_FORWARD = 3  # l trigger
  AXIS_REAR_REVERSE = 2  # r trigger 
  BUTTON_STALL = 6  # r bumper
  BUTTON_FIRE = 1  # a
  BUTTON_UNLOCK = 2  # b
  BUTTON_RESET = 8  # start button
  POV_UP = 0  # pov up
  POV_DOWN = 180  # pov down

  def robotInit(self):
    """Robot initialization function"""

    self.front_left_drive = ctre.WPI_TalonSRX(7)
    self.rear_left_drive = ctre.WPI_TalonSRX(6)
    self.front_right_drive = ctre.WPI_TalonSRX(4)
    self.rear_right_drive = ctre.WPI_TalonSRX(5)
    self.rear_drive = ctre.WPI_TalonSRX(3)

    self.front_left_drive.setInverted(True)
    self.rear_left_drive.setInverted(False)
    self.front_right_drive.setInverted(False)
    self.rear_right_drive.setInverted(True)

    self.front_lift = ctre.WPI_TalonSRX(1)
    self.rear_lift = ctre.WPI_TalonSRX(2)
    self.encoder = self.front_lift

    # self.front_lift.setInverted(True)

    self.drive = wpilib.drive.DifferentialDrive(
      wpilib.SpeedControllerGroup(self.front_left_drive, self.rear_left_drive),
      wpilib.SpeedControllerGroup(self.front_right_drive, self.rear_right_drive)
    )
    self.drive.setExpiration(0.5)

    self.rear_drive = wpilib.drive.DifferentialDrive(self.rear_drive, self.rear_drive)
    self.rear_drive.setExpiration(0.5)

    self.lift = wpilib.drive.DifferentialDrive(self.front_lift, self.front_lift)
    self.lift.setExpiration(0.5)

    self.rear_lift = wpilib.drive.DifferentialDrive(self.rear_lift, self.rear_lift)
    self.rear_lift.setExpiration(0.5)

    self.stick_drive = wpilib.Joystick(0)
    self.stick_lift = wpilib.Joystick(1)

    self.shift = wpilib.DoubleSolenoid(0, 1)
    self.unlock = wpilib.DoubleSolenoid(4, 5)
    self.arm_fire = wpilib.DoubleSolenoid(7, 6)

    # self.camera_pitch = wpilib.Servo(2)
    # self.camera_yaw = wpilib.Servo(3)

    self.log_timer = wpilib.Timer()
    self.log_timer.start()

    self.compressor = wpilib.Compressor(0)


    # wpilib.CameraServer.launch()
    # camera_alpha = cameraserver.getInstance().startAutomaticCapture(0)
    # camera_beta = cameraserver.getInstance().startAutomaticCapture(1)
    # cameraserver.CameraServer.launch()
    cameraserver.CameraServer.launch("camera.py:main")

    # camera.setResolution(426, 240)
    # camera.setFPS(15)

    self.stage = 0
    self.stages = [0, 26000]  # 37 inches to travel, 4.76 in/rotation, 4096 units/rotation
    self.encoder.setSelectedSensorPosition(0)
    # self.encoder.setSensorPhase(True)

  def autonomousInit(self):
    self.teleopInit()

  def autonomousPeriodic(self):
    self.teleopPeriodic()

  def teleopInit(self):
    """Executed at the start of teleop mode"""
    # self.drive.setSafetyEnabled(True)
    self.compressor.start()
    pass

  def teleopPeriodic(self):
    """Does stuff"""
    distance = self.encoder.getSelectedSensorPosition()

    # distance = real_distance - self.init_distance
    # distance *= 1
    
    # self.logger(repr(self.stage))
    desired_distance = self.stages[self.stage]
    
    self.drive.arcadeDrive(-self.stick_drive.getRawAxis(self.AXIS_THROTTLE), self.stick_drive.getRawAxis(self.AXIS_STEER))

    input_forward = self.stick_drive.getRawAxis(self.AXIS_REAR_FORWARD)
    input_reverse = self.stick_drive.getRawAxis(self.AXIS_REAR_REVERSE)

    self.rear_drive.arcadeDrive(input_forward if input_forward > input_reverse else -input_reverse, 0)

    self.shift.set(self.shift.Value.kForward if self.stick_drive.getRawButton(self.BUTTON_SHIFT) else self.shift.Value.kReverse)
    self.arm_fire.set(self.arm_fire.Value.kForward if self.stick_lift.getRawButton(self.BUTTON_FIRE) else self.arm_fire.Value.kReverse)
    self.unlock.set(self.unlock.Value.kForward if self.stick_lift.getRawButton(self.BUTTON_UNLOCK) else self.unlock.Value.kReverse)

    if self.stick_lift.getPOV() == self.POV_UP:
      self.stage = 1
    elif self.stick_lift.getPOV() == self.POV_DOWN:
      self.stage = 0
    elif self.stick_lift.getRawButton(self.BUTTON_RESET):
      self.encoder.setSelectedSensorPosition(0)

    lift_auto_power = max(-1, min(1, (desired_distance - distance) / 1024))

    if abs(self.stick_lift.getRawAxis(self.AXIS_LIFT)) > 0.15 or self.stick_lift.getRawButton(self.BUTTON_STALL):
      self.lift.arcadeDrive(-self.stick_lift.getRawAxis(self.AXIS_LIFT), 0)
    # auto code
    elif abs(desired_distance - distance) > 200:
      self.lift.arcadeDrive(lift_auto_power * 0.8, 0)
    else:
      self.lift.arcadeDrive(0, 0)
    self.rear_lift.arcadeDrive(-self.stick_lift.getRawAxis(self.AXIS_REAR_LIFT), 0)

    # current_pitch = self.camera_pitch.get()
    # current_yaw = self.camera_yaw.get()
    # input_pitch = -self.stick_drive.getRawAxis(self.AXIS_PITCH)
    # input_yaw = self.stick_drive.getRawAxis(self.AXIS_YAW)

    if self.log_timer.hasPeriodPassed(0.5):
      self.logger.info(repr({
        "init_ditance": 0, # self.init_distance,
        # "real_distance": real_distance,
        "distance": distance,
        "desired_distance": desired_distance,
        "stage": self.stage,
        "delta": lift_auto_power,
      }))
      self.log_timer.reset()

    # if abs(input_pitch) > 0.15:
    #   self.camera_pitch.set(max(0, min(1, current_pitch + input_pitch * 0.05)))
    # if abs(input_yaw) > 0.15:
    #   self.camera_yaw.set(max(0, min(1, current_yaw + input_yaw * 0.05)))

  def testInit(self):
    self.compressor.stop()

  testPeriodic = teleopPeriodic

if __name__ == "__main__":
  wpilib.run(Robot)
