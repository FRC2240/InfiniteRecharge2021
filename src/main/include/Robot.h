// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include <frc/drive/DifferentialDrive.h>
#include "rev/CANSparkMax.h"
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/WPILib.h>
#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/Path.h>
#include <wpi/SmallString.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
 
 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
  static const int backrightMotorDeviceID = 1, frontrightMotorDeviceID = 2, backleftMotorDeviceID = 3, frontleftMotorDeviceID = 4, hopperMotorDeviceID = 5, rightshooterMotorDeviceID = 6, leftshooterMotorDeviceID = 7, turretMotorDeviceID = 8, raisingMotorDeviceID = 9, rightelevatingMotorDeviceID = 10, leftelevatingMotorDeviceID = 11, intakeMotorDeviceID = 12, colorwheelMotorDeviceID = 13;

//drivebase motors (looking from the front)//
  rev::CANSparkMax m_backrightMotor{backrightMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_frontrightMotor{frontrightMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_backleftMotor{backleftMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_frontleftMotor{backleftMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANEncoder m_backrightEncoder = m_backrightMotor.GetEncoder();
  rev::CANEncoder m_frontrightEncoder = m_frontrightMotor.GetEncoder();
  rev::CANEncoder m_backleftEncoder = m_backleftMotor.GetEncoder();
  rev::CANEncoder m_frontleftEncoder = m_frontleftMotor.GetEncoder();
  frc::DifferentialDrive m_robotDrive{m_frontleftMotor, m_frontrightMotor};
 
 //hopper and shooter and turret//
  rev::CANSparkMax m_hopperMotor{hopperMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightshooterMotor{rightshooterMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftshooterMotor{leftshooterMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_turretMotor{turretMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANEncoder m_hopperEncoder = m_hopperMotor.GetEncoder();
  rev::CANEncoder m_rightshooterEncoder = m_rightshooterMotor.GetEncoder();
  rev::CANEncoder m_leftshooterEncoder = m_leftshooterMotor.GetEncoder();
  rev::CANEncoder m_turretEncoder = m_turretMotor.GetEncoder();
  
  //climber//
  rev::CANSparkMax m_raisingMotor{raisingMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightelevatingMotor{rightelevatingMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftelevatingMotor{leftelevatingMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANEncoder m_raisingEncoder = m_raisingMotor.GetEncoder();
  rev::CANEncoder m_rightelevatingEncoder = m_rightelevatingMotor.GetEncoder();
  rev::CANEncoder m_leftelevatingEncoder = m_leftelevatingMotor.GetEncoder();
  
  //intake//
  rev::CANSparkMax m_intakeMotor{intakeMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANEncoder m_intakeEncoder = m_intakeMotor.GetEncoder();
  
  //colorwheel//
  rev::CANSparkMax m_colorwheelMotor{colorwheelMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANEncoder m_colorwheelEncoder = m_colorwheelMotor.GetEncoder();
  frc::Joystick m_stick{0};
  
  //pneumatics//
  frc::DoubleSolenoid m_uptake{0, 7};
  frc::DoubleSolenoid m_intakeleft{1, 6};
  frc::DoubleSolenoid m_intakeright{2, 5};
  frc::DoubleSolenoid m_colorwheel{3, 4};
  frc::Compressor m_compressor;
};
