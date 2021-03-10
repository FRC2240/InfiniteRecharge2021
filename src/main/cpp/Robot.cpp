// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit()
{
  /* Add this section back whrn ready for Pathweaver
  wpi::SmallString<64> deployDirectory;
  frc::filesystem::GetDeployDirectory(deployDirectory);
  wpi::sys::path::append(deployDirectory, "paths");
  wpi::sys::path::append(deployDirectory, "TestPath.wpilib.json");

  frc::Trajectory trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);*/

  m_backleftMotor.RestoreFactoryDefaults();
  m_backrightMotor.RestoreFactoryDefaults();
  m_frontleftMotor.RestoreFactoryDefaults();
  m_frontrightMotor.RestoreFactoryDefaults();

  m_backleftMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  m_backrightMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  m_frontleftMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  m_frontrightMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

  // Set back motors to follow front motors
  m_backleftMotor.Follow(m_frontleftMotor);
  m_backrightMotor.Follow(m_frontrightMotor);

  // Set shooter motors equal (and inverted)
  m_leftshooterMotor.Follow(m_rightshooterMotor, true);

  // Initialize PID coeffiecients
  InitializePIDControllers();

  // Turret soft stops
  m_turretEncoder.SetPosition(0.0);
  m_turretMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
  m_turretMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);
  m_turretMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, -15.0);
  m_turretMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, 15.0);

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

void Robot::TeleopPeriodic()
{
  // read drive input from joystick
  double move = m_stick.GetRawAxis(1);
  double rotate = m_stick.GetRawAxis(4);
  m_robotDrive.ArcadeDrive(move, rotate);

  // Shooting?
  if (fabs(m_stick.GetRawAxis(3)) > 0.75) {
    m_table->PutNumber("pipeline", 0); // Enable targeting pipeline of Limelight

    // Is target locked?
    if (LimelightTracking()) {
      // Calculate distance to target from Limelight data
      double ty = m_table->GetNumber("ty", 0.0);
      double distance = ((heightOfTarget - heightLimelight) / tan((constantLimelightAngle + ty) * (3.141592653 / 180)));
      double rpm = CalculateRPM(distance);

      m_shooterPID.SetReference(rpm, rev::ControlType::kVelocity); // Set shooter motor speed (based on distance)

      // Enable uptake and hopper if we're at 99.5% of desired shooter speed
      if (m_rightshooterEncoder.GetVelocity() > (rpm * 0.995)) {
          m_uptake.Set(frc::DoubleSolenoid::Value::kReverse);
          m_hopperMotor.Set(1.0);
      } else {
          m_uptake.Set(frc::DoubleSolenoid::Value::kForward);
          m_hopperMotor.Set(0.0);
      }
    }
  } else { // Not shooting
    m_table->PutNumber("pipeline", 1); // Set driving pipeline of Limelight

    // Shooter off, Hopper off, Uptake down, and Turret centered
    m_rightshooterMotor.Set(0.0);
    m_hopperMotor.Set(0.0);
    m_uptake.Set(frc::DoubleSolenoid::Value::kForward);
    m_turretPID.SetReference(0, rev::ControlType::kPosition);
  }

  //pipeline code

  bool gatherButton = m_stick.GetRawButton(4);

  if (gatherButton)
  {
    m_intakeleft.Set(frc::DoubleSolenoid::Value::kForward);
    m_intakeright.Set(frc::DoubleSolenoid::Value::kForward);
    m_intakeMotor.Set(1.0);
    m_hopperMotor.Set(1.0);
  }
  else
  {
    m_intakeleft.Set(frc::DoubleSolenoid::Value::kReverse);
    m_intakeright.Set(frc::DoubleSolenoid::Value::kReverse);
    m_intakeMotor.Set(0.0);
    m_hopperMotor.Set(0.0);
  }

  bool releasegatherButton = m_stick.GetRawButtonReleased(4);

  if (releasegatherButton)
  {
    m_intakeleft.Set(frc::DoubleSolenoid::Value::kReverse);
    m_intakeright.Set(frc::DoubleSolenoid::Value::kReverse);
    m_intakeMotor.Set(0.0);

    //hopper runs for some amount of time after button is released
    for (int time = 0; time < 200; time++)
    {
      m_hopperMotor.Set(1.0);
    }
    m_hopperMotor.Set(0.0);
  }
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic()
{
  std::cout << "turret: " << m_turretEncoder.GetPosition() << std::endl;
  //shooter//

  bool shooterButton = m_stick.GetRawButton(1);

  if (shooterButton)
  {
    m_rightshooterMotor.Set(-1.0);
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
    m_uptake.Set(frc::DoubleSolenoid::Value::kReverse);
  }
  else
  {
    m_uptake.Set(frc::DoubleSolenoid::Value::kForward);
  }
  // color wheel
  bool colorWheelTest = m_stick.GetRawButton(1);
  if (colorWheelTest)
  {
    m_colorwheelMotor.Set(1.0);
  }
  else
  {
    m_colorwheelMotor.Set(0);
  }
  // intake
  bool intakeButton = m_stick.GetRawButton(2);
  if (intakeButton)
  {
    m_intakeMotor.Set(1.0);
  }
  else
  {
    m_intakeMotor.Set(0);
  }
  // climber
  /*
  bool raisingButton = m_stick.GetRawButton(3);
  if (raisingButton) {
    m_raisingMotor.Set(1.0);
  }
  else {
    m_raisingMotor.Set(0);
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

*/
  //frontright

  bool frontrightButton = m_stick.GetRawButton(6);

  if (frontrightButton)
  {
    m_frontrightMotor.Set(1.0);
  }
  else
  {
    m_frontrightMotor.Set(0.0);
  }
  //backright

  bool backrightButton = m_stick.GetRawButton(7);

  if (backrightButton)
  {
    m_backrightMotor.Set(1.0);
  }
  else
  {
    m_backrightMotor.Set(0.0);
  }
  //frontleft

  bool frontleftButton = m_stick.GetRawButton(8);

  if (frontleftButton)
  {
    m_frontleftMotor.Set(1.0);
  }
  else
  {
    m_frontleftMotor.Set(0.0);
  }

  //backleft
  bool backleftButton = m_stick.GetRawButton(9);

  if (backleftButton)
  {
    m_backleftMotor.Set(1.0);
  }
  else
  {
    m_backleftMotor.Set(0.0);
  }
}

bool Robot::LimelightTracking()
{
  bool shoot = false;

  // Proportional Steering Constant:
  // If your robot doesn't turn fast enough toward the target, make this number bigger
  // If your robot oscillates (swings back and forth past the target) make this smaller
  const double STEER_K   = 0.03;
  const double MAX_STEER = 0.2;

  double tx = m_table->GetNumber("tx", 0.0);
  double tv = m_table->GetNumber("tv", 0.0);

  double limelightTurnCmd = 0.0;

  if (tv > 0.0)
  {
    // Proportional steering
    limelightTurnCmd = (tx + tx_OFFSET) * STEER_K;
    limelightTurnCmd = std::clamp(limelightTurnCmd, -MAX_STEER, MAX_STEER);
    if (tx < 0.25) {
      shoot = true;
    }
  }

  m_turretMotor.Set(limelightTurnCmd);
  return shoot;
}

// Calculate the RPM from the distance
double Robot::CalculateRPM(double d) {
  double rpm;

  if (d < 125.0) {
    rpm = (-45.671 * d) + 14322.0;
  } else {
    rpm = 0.028052 * d * d - 8.5977 * d + 2946.0;
  }
  return rpm;
}

// Initialize the PID coefficients
void Robot::InitializePIDControllers() {
  m_shooterPID.SetP(m_shooterPIDCoeff.kP);
  m_shooterPID.SetI(m_shooterPIDCoeff.kI);
  m_shooterPID.SetD(m_shooterPIDCoeff.kD);
  m_shooterPID.SetIZone(m_shooterPIDCoeff.kIz);
  m_shooterPID.SetFF(m_shooterPIDCoeff.kFF);
  m_shooterPID.SetOutputRange(m_shooterPIDCoeff.kMinOutput, m_shooterPIDCoeff.kMaxOutput);

  m_turretPID.SetP(m_turretPIDCoeff.kP);
  m_turretPID.SetI(m_turretPIDCoeff.kI);
  m_turretPID.SetD(m_turretPIDCoeff.kD);
  m_turretPID.SetIZone(m_turretPIDCoeff.kIz);
  m_turretPID.SetFF(m_turretPIDCoeff.kFF);
  m_turretPID.SetOutputRange(m_turretPIDCoeff.kMinOutput, m_turretPIDCoeff.kMaxOutput); 
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
