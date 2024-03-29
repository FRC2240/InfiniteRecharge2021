// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <frc/controller/RamseteController.h>
#include <frc/trajectory/TrajectoryGenerator.h>

#include <wpi/Path.h>
#include <wpi/SmallString.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

#include "rev/CANSparkMax.h"

#include "Drivetrain.h"

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

  bool LimelightTracking();
  void InitializePIDControllers();
  double CalculateRPM(double d);
  void AutoSimpleGame();
  void AutoFollowPath();

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameSimpleGame     = "SimpleGame";     // Simple Game Auto: Back up and shoot
  const std::string kAutoNamePathweaverGame = "PathweaverGame"; // Pathweaver for INFINITE RECHARGE Game
  const std::string kAutoNameSimplePath     = "SimplePath";     // Simple test path
  const std::string kAutoErikTest           = "ErikTest";       // Test path

  const std::string kAutoBarrelRacing       = "BarrelRacing";   // AutoNav Challenge
  const std::string kAutoSlalom             = "Slalom";         // AutoNav Challenge
  const std::string kAutoBounce             = "Bounce";         // AutoNav Challenge
  const std::string kAutoRedA               = "RedA";           // Galactic Search
  const std::string kAutoRedB               = "RedB";           // Galactic Search
  const std::string kAutoBlueA              = "BlueA";          // Galactic Search
  const std::string kAutoBlueB              = "BlueB";          // Galactic Search

  const std::string kAutoNameDefault        = "Default";        // Default: Do nothing

  std::string m_autoSelected;

  double constantLimelightAngle = 13;      // degrees (old 22)
  double heightLimelight        = 23.0;    // inches
  double heightOfTarget         = 89.75;   // inches

  static const int intakeMotorDeviceID         =  1;
  static const int hopperMotorDeviceID         =  2;
  static const int colorwheelMotorDeviceID     =  3;
  static const int raisingMotorDeviceID        =  4;
  static const int rightelevatingMotorDeviceID =  5;
  static const int rightshooterMotorDeviceID   =  6;
  static const int leftelevatingMotorDeviceID  =  7;
  static const int leftshooterMotorDeviceID    =  8;
  static const int frontrightMotorDeviceID     =  9;
  static const int backrightMotorDeviceID      = 10;
  static const int turretMotorDeviceID         = 11;
  static const int frontleftMotorDeviceID      = 12;
  static const int backleftMotorDeviceID       = 13;

  //drivebase motors (looking from the front)//
  rev::CANSparkMax m_backrightMotor{backrightMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_frontrightMotor{frontrightMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_backleftMotor{backleftMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_frontleftMotor{frontleftMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};
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

  rev::CANPIDController m_turretPID = m_turretMotor.GetPIDController();
  rev::CANPIDController m_shooterPID = m_leftshooterMotor.GetPIDController();
  
  //climber//
  rev::CANSparkMax m_raisingMotor{raisingMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightelevatingMotor{rightelevatingMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftelevatingMotor{leftelevatingMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANEncoder m_raisingEncoder = m_raisingMotor.GetEncoder();
  rev::CANEncoder m_rightelevatingEncoder = m_rightelevatingMotor.GetEncoder();
  rev::CANEncoder m_leftelevatingEncoder = m_leftelevatingMotor.GetEncoder();

  rev::CANPIDController m_raisingPID = m_raisingMotor.GetPIDController();
  
  //intake//
  rev::CANSparkMax m_intakeMotor{intakeMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANEncoder m_intakeEncoder = m_intakeMotor.GetEncoder();
  
  //colorwheel//
  rev::CANSparkMax m_colorwheelMotor{colorwheelMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANEncoder m_colorwheelEncoder = m_colorwheelMotor.GetEncoder();

  // Alternate Encoder for Hopper
  rev::CANEncoder m_hopperAltEncoder = m_hopperMotor.GetAlternateEncoder(rev::CANEncoder::AlternateEncoderType::kQuadrature, 8192);

  frc::Joystick m_stick{0};
  
  //pneumatics//
  frc::DoubleSolenoid m_intakeleft{0, 7};
  frc::DoubleSolenoid m_intakeright{1, 6};
  frc::DoubleSolenoid m_colorwheel{2, 5};
  frc::DoubleSolenoid m_uptake{3, 4};

  frc::Compressor m_compressor;

  //gathering//
  int  m_hopperTimer  = 0;
  int  m_reverseTimer = 0;
  bool m_isGathering   = false;
  bool m_isShooting    = false;
  bool m_hopperReverse = false;

  //climbing//
/*
  bool m_isClimbing = false;
  bool m_isElevated = false;
*/

  // Limelight
  std::shared_ptr<NetworkTable> m_table = nt::NetworkTableInstance::GetDefault().GetTable("limelight-scorpio");

  // Override Shooter speed for calibration/testing
  double m_overrideShooterSpeed = 0.0;

  // PID coefficient structure
  struct pidCoeff {
    double kP;
    double kI;
    double kD;
    double kIz;
    double kFF;
    double kMinOutput;
    double kMaxOutput;
  };

  // DETERMINE THESE EXPERIMENTALLY!!!!!!!
  // pidCoeff ----------------{kP, kI, kD, kIz, kFF, kMinOutput, kMaxOutput}
  pidCoeff m_shooterPIDCoeff {0.0005, 0.00000004, 0.03, 0.0, 0.000202, -1.0, 1.0};
  pidCoeff m_turretPIDCoeff {0.02, 0.0, 0.0, 0.0, 0.0, -0.5, 0.5};
  pidCoeff m_raisingPIDCoeff {7.0e-4, 1e-6, 0.0, 0.0, 0.0, -1.0, 1.0};

  frc::Timer m_autoTimer;

  double m_txOFFSET = 3.0;

  // **** RAMSETE Control **** //
  Drivetrain* m_drive;

  // The trajectory to follow
  frc::Trajectory m_trajectory;

  // The Ramsete Controller to follow the trajectory
  frc::RamseteController m_ramseteController;

  // The timer to use during the autonomous period
  frc2::Timer m_timer;
};