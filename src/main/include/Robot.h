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
#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>

#include <wpi/Path.h>
#include <wpi/SmallString.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
// These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
// These characterization values MUST be determined either experimentally or
// theoretically for *your* robot's drive. The Robot Characterization
// Toolsuite provides a convenient tool for obtaining these values for your
// robot.
constexpr auto ks = 0.22_V;
constexpr auto kv = 1.98 * 1_V * 1_s / 1_m;
constexpr auto ka = 0.2 * 1_V * 1_s * 1_s / 1_m;

// Example value only - as above, this must be tuned for your drive!
constexpr double kPDriveVel = 8.5;

constexpr auto kTrackwidth = 0.69_m;
//Error here: extern const frc::DifferentialDriveKinematics kDriveKinematics;

constexpr auto kMaxSpeed = 11.5_mps;
constexpr auto kMaxAcceleration = 3.06_mps_sq;

// Reasonable baseline values for a RAMSETE follower in units of meters and
// seconds
constexpr double kRamseteB = 2;
constexpr double kRamseteZeta = 0.7;

#include "rev/CANSparkMax.h"

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

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

  double constantLimelightAngle = 17;      // degrees (old 22)
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
  
  //intake//
  rev::CANSparkMax m_intakeMotor{intakeMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANEncoder m_intakeEncoder = m_intakeMotor.GetEncoder();
  
  //colorwheel//
  rev::CANSparkMax m_colorwheelMotor{colorwheelMotorDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANEncoder m_colorwheelEncoder = m_colorwheelMotor.GetEncoder();
  frc::Joystick m_stick{0};
  
  //pneumatics//
  frc::DoubleSolenoid m_intakeleft{0, 7};
  frc::DoubleSolenoid m_intakeright{1, 6};
  frc::DoubleSolenoid m_colorwheel{2, 5};
  frc::DoubleSolenoid m_uptake{3, 4};

  frc::Compressor m_compressor;

  //gathering//
  int m_gatherTimer = 0;
  bool m_isGathering = false;

  // Limelight
  std::shared_ptr<NetworkTable> m_table = nt::NetworkTableInstance::GetDefault().GetTable("limelight-scorpio");
  double tx_OFFSET = 0.0; // old = 3.0

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
  pidCoeff m_shooterPIDCoeff {0.0007, 0.0, 1.0, 0.0, 0.000207, -1.0, 1.0};
  pidCoeff m_turretPIDCoeff {0.02, 0.0, 0.0, 0.0, 0.0, -0.5, 0.5};
};
