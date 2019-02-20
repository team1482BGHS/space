#!/usr/bin/env python3
"""
This is a program showing the use of the RobotDrive class,
specifically it contains the code necessary to operate a robot with
a single joystick
"""

import math

import wpilib
import wpilib.drive
import wpilib.cameraserver
# from cscore import CameraServer
import ctre

def float_round(x, prec=2, base=.05):
  """Directly copied from StackOverflow"""
  return round(base * round(float(x) / base), prec)

def sigmoid(x):
  return 1 / (1 + math.exp(-x))

class Robot(wpilib.IterativeRobot):
  # stick 1
  AXIS_THROTTLE = 1
  AXIS_STEER = 0
  AXIS_YAW = 4
  AXIS_PITCH = 5
  BUTTON_SHIFT = 6

  # stick 2
  AXIS_LIFT = 1
  AXIS_REAR_LIFT = 5
  AXIS_REAR_FORWARD = 3
  AXIS_REAR_REVERSE = 2
  BUTTON_STALL = 6
  BUTTON_FIRE = 1
  POV_UP = 0
  POV_DOWN = 180

  def robotInit(self):
    """Robot initialization function"""

    self.front_left_drive = ctre.WPI_TalonSRX(7)
    self.rear_left_drive = ctre.WPI_TalonSRX(6)
    self.front_right_drive = ctre.WPI_TalonSRX(4)
    self.rear_right_drive = ctre.WPI_TalonSRX(5)
    self.rear_drive = ctre.WPI_TalonSRX(3)

    self.front_right_drive.setInverted(True)
    self.rear_right_drive.setInverted(True)

    self.front_lift = ctre.WPI_TalonSRX(1)
    # self.front_left_lift = ctre.WPI_TalonSRX(0)
    # self.front_right_lift = ctre.WPI_TalonSRX(1)
    self.rear_lift = ctre.WPI_TalonSRX(2)

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
    self.arm_fire = wpilib.DoubleSolenoid(6, 7)

    self.camera_pitch = wpilib.Servo(2)
    self.camera_yaw = wpilib.Servo(3)

    self.log_timer = wpilib.Timer()

    # self.compressor = wpilib.Compressor(0);

    wpilib.CameraServer.launch()
    # camera = CameraServer.getInstance().startAutomaticCapture()
    # camera.setResolution(426, 240)
    # camera.setFPS(15)

    self.stage = 0
    self.stages = [0, 10000, 20000] 
    self.init_distance = self.front_lift.getSelectedSensorPosition()

  def autonomousInit(self):
    self.teleopInit()

  def autonomousPeriodic(self):
    self.teleopPeriodic()

  def teleopInit(self):
    """Executed at the start of teleop mode"""
    self.drive.setSafetyEnabled(True)

  def teleopPeriodic(self):
    """Runs the motors with tank steering"""
    distance = self.front_lift.getSelectedSensorPosition()
    delta_distance = distance - self.init_distance
    
    self.drive.arcadeDrive(self.stick_drive.getRawAxis(self.AXIS_THROTTLE), self.stick_drive.getRawAxis(self.AXIS_STEER))

    input_forward = self.stick_lift.getRawAxis(self.AXIS_REAR_FORWARD)
    input_reverse = self.stick_lift.getRawAxis(self.AXIS_REAR_REVERSE)

    self.rear_drive.arcadeDrive((input_forward if input_forward > input_reverse else input_reverse) * 0.5, 0)

    self.shift.set(self.stick_drive.getRawButton(self.BUTTON_SHIFT))
    self.arm_fire.set(self.stick_lift.getRawButton(self.BUTTON_FIRE))

    if self.stick_lift.getPOV() == self.POV_UP and self.stage < len(self.stages):
      self.stage += 1
    elif self.stick_lift.getPOV() == self.POV_DOWN and self.stage > 0:
      self.stage -= 1

    if abs(self.stick_lift.getRawAxis(self.AXIS_LIFT)) > 0.25 or self.stick_lift.getRawButton(self.BUTTON_STALL):
      self.lift.arcadeDrive(self.stick_lift.getRawAxis(self.AXIS_LIFT), 0)
    elif abs(delta_distance) > 200:
      self.lift.arcadeDrive(2 * sigmoid(delta_distance / 100) - 1, 0)

    self.rear_lift.arcadeDrive(self.stick_lift.getRawAxis(self.AXIS_REAR_LIFT), 0)

    current_pitch = self.camera_pitch.get()
    current_yaw = self.camera_yaw.get()
    input_pitch = -self.stick_drive.getRawAxis(self.AXIS_PITCH)
    input_yaw = self.stick_drive.getRawAxis(self.AXIS_YAW)

    #if self.log_timer.hasPeriodPassed(0.5):
    self.logger.info(repr({
      "input_pitch": input_pitch,
      "current_pitch": current_pitch,
      "delta_pitch": current_pitch + input_pitch,
      "input_yaw": input_yaw,
      "current_yaw": current_yaw,
      "delta_yaw": current_yaw + input_yaw
    }))
      #self.log_timer.reset()

    if abs(input_pitch) > 0.15:
      self.camera_pitch.set(max(0, min(1, current_pitch + input_pitch * 0.05)))
    if abs(input_yaw) > 0.15:
      self.camera_yaw.set(max(0, min(1, current_yaw + input_yaw * 0.05)))

if __name__ == "__main__":
  wpilib.run(Robot)
