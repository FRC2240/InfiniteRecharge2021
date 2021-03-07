// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit()
{

wpi::SmallString<64> deployDirectory;
frc::filesystem::GetDeployDirectory(deployDirectory);
wpi::sys::path::append(deployDirectory, "paths");
wpi::sys::path::append(deployDirectory, "TestPath.wpilib.json");

frc::Trajectory trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);

  m_backleftMotor.RestoreFactoryDefaults();
  m_backrightMotor.RestoreFactoryDefaults();
  m_frontleftMotor.RestoreFactoryDefaults();
  m_frontrightMotor.RestoreFactoryDefaults();

  // Set back motors to follow front motors
  m_backleftMotor.Follow(m_frontleftMotor, true);
  m_backrightMotor.Follow(m_frontrightMotor, true);
  m_backleftMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  m_backrightMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  //Set shooter motors equal
  m_leftshooterMotor.Follow(m_rightshooterMotor, true);
  // Start compressor
  m_compressor.Start();
  // Initialize solenoids
  m_intakeright.Set(frc::DoubleSolenoid::Value::kForward);
  m_intakeleft.Set(frc::DoubleSolenoid::Value::kForward);
  m_colorwheel.Set(frc::DoubleSolenoid::Value::kForward);
  m_uptake.Set(frc::DoubleSolenoid::Value::kForward);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit()
{
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom)
  {
    // Custom Auto goes here
  }
  else
  {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic()
{
  if (m_autoSelected == kAutoNameCustom)
  {
    // Custom Auto goes here
  }
  else
  {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {

    // read drive input from joystick
  double move   = m_stick.GetRawAxis(1);
  double rotate = m_stick.GetRawAxis(2);
  m_robotDrive.ArcadeDrive(move, rotate);

  if (m_stick.GetRawButton(1))
  {
    m_intakeleft.Set(frc::DoubleSolenoid::Value::kReverse);
    m_intakeright.Set(frc::DoubleSolenoid::Value::kReverse);
  } else {
    m_intakeleft.Set(frc::DoubleSolenoid::Value::kForward);
    m_intakeright.Set(frc::DoubleSolenoid::Value::kForward);
  }

  if (m_stick.GetRawButton(2)) {
    m_uptake.Set(frc::DoubleSolenoid::Value::kReverse);
  } else {
    m_uptake.Set(frc::DoubleSolenoid::Value::kForward);
  }

  if (m_stick.GetRawButton(3)) {
    m_colorwheel.Set(frc::DoubleSolenoid::Value::kReverse);
  } else {
    m_colorwheel.Set(frc::DoubleSolenoid::Value::kForward);
  }  
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic()
{
  //shooter//

  bool shooterButton = m_stick.GetRawButton(1);

  if (shooterButton)
  {
    m_rightshooterMotor.Set(1.0);
  }
  else
  {
    m_rightshooterMotor.Set(0.0);
  }


//hopper//

bool hopperButton = m_stick.GetRawButton(2);

if (hopperButton)
{
  m_hopperMotor.Set(1.0);
}
else
{
  m_hopperMotor.Set(0.0);
}

//turrett//

bool turretButton = m_stick.GetRawButton(3);

if (turretButton)
{
  m_turretMotor.Set(1.0);
}
else
{
  m_turretMotor.Set(0.0);
}

//uptake//

bool uptakeButton = m_stick.GetRawButton(4);

if (uptakeButton)
{
  m_uptake.Set(frc::DoubleSolenoid::Value::kForward);
}
else
{
  m_uptake.Set(frc::DoubleSolenoid::Value::kReverse);
}
  // color wheel
  bool colorWheelTest = m_stick.GetRawButton(1);
  if (colorWheelTest){
    m_colorwheelMotor.Set(1.0);
  }
  else {
    m_colorwheelMotor.Set(0);
  }
  // intake
  bool intakeButton = m_stick.GetRawButton(2);
  if (intakeButton) {
    m_intakeMotor.Set(1.0);
  }
  else {
    m_intakeMotor.Set(0);
  }
  // climber
  bool raisingButton = m_stick.GetRawButton(3);
  if (raisingButton) {
    m_rightelevatingMotor.Set(1.0);
  }
  else {
    m_rightelevatingMotor.Set(0);
  }
  bool leftClimberButton = m_stick.GetRawButton(4);
  if (leftClimberButton){
    m_leftelevatingMotor.Set(1.0);
  }
  else {
    m_leftelevatingMotor.Set(0);
  }
  bool rightClimberButton = m_stick.GetRawButton(5);
  if (rightClimberButton) {
    m_rightelevatingMotor.Set(1.0);
  }
  else {
    m_rightelevatingMotor.Set(0);
  }
}
#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
