#!/usr/bin/env python3
"""
This is a demo program showing the use of the RobotDrive class,
specifically it contains the code necessary to operate a robot with
a single joystick
"""

import wpilib
from wpilib import drive
import ctre

def float_round(x, prec=2, base=.05):
  """Directly copied from StackOverflow"""
  return round(base * round(float(x) / base), prec)

class Robot(wpilib.IterativeRobot):
  def robotInit(self):
    """Robot initialization function"""

    self._right_talons = [ctre.WPI_TalonSRX(0), ctre.WPI_TalonSRX(1)]
    self._left_talons = [ctre.WPI_TalonSRX(2), ctre.WPI_TalonSRX(3)]

    self._right = wpilib.SpeedControllerGroup(*self._right_talons)
    self._left = wpilib.SpeedControllerGroup(*self._left_talons)

    self.drive = drive.DifferentialDrive(self._left, self._right)
    self.drive.setExpiration(0.1)

    self.stick = wpilib.Joystick(0)

  def autonomousInit(self):
    pass

  def autonomousPeriodic(self):
    pass

  def teleopInit(self):
    """Executed at the start of teleop mode"""
    self.drive.setSafetyEnabled(True)

  def teleopPeriodic(self):
    """Runs the motors with tank steering"""
    self.drive.arcadeDrive(self.stick.getY(), self.stick.getX())

if __name__ == "__main__":
  wpilib.run(Robot)
