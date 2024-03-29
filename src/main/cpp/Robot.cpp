// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit()
{
  std::cout << "Robot Init" << std::endl;

  m_chooser.SetDefaultOption(kAutoNameSimpleGame, kAutoNameSimpleGame);
  m_chooser.AddOption(kAutoNameSimpleGame, kAutoNameSimpleGame);
  m_chooser.AddOption(kAutoNamePathweaverGame, kAutoNamePathweaverGame);

  m_chooser.AddOption(kAutoBarrelRacing, kAutoBarrelRacing);
  m_chooser.AddOption(kAutoSlalom, kAutoSlalom);
  m_chooser.AddOption(kAutoBounce, kAutoBounce);

  m_chooser.AddOption(kAutoRedA, kAutoRedA);
  m_chooser.AddOption(kAutoRedB, kAutoRedB);
  m_chooser.AddOption(kAutoBlueA, kAutoBlueA);
  m_chooser.AddOption(kAutoBlueB, kAutoBlueB);

  m_chooser.AddOption(kAutoNameSimplePath, kAutoNameSimplePath);
  m_chooser.AddOption(kAutoErikTest, kAutoErikTest);

  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  m_backleftMotor.RestoreFactoryDefaults();
  m_backrightMotor.RestoreFactoryDefaults();
  m_frontleftMotor.RestoreFactoryDefaults();
  m_frontrightMotor.RestoreFactoryDefaults();
  m_leftelevatingMotor.RestoreFactoryDefaults();
  m_rightelevatingMotor.RestoreFactoryDefaults();


  m_backleftMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  m_backrightMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  m_frontleftMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  m_frontrightMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);



  // Set back motors to follow front motors
  m_backleftMotor.Follow(m_frontleftMotor);
  m_backrightMotor.Follow(m_frontrightMotor);

  // Set shooter motors equal (and inverted)
  m_rightshooterMotor.Follow(m_leftshooterMotor, true);

  m_hopperMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

  // Initialize PID coeffiecients
  InitializePIDControllers();

  // Turret soft stops
  m_turretEncoder.SetPosition(0.0);
  m_turretMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
  m_turretMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);
  m_turretMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, -67.0);
  m_turretMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, 67.0);

  m_hopperAltEncoder.SetPosition(0.0);

  
  // set climber encoders to zero
  m_raisingEncoder.SetPosition(0.0);
  m_leftelevatingEncoder.SetPosition(0.0);
  m_rightelevatingEncoder.SetPosition(0.0);

  // Climber soft stops (1 rotation = 2 in and 9/16)

  // Enable Climber Soft Limits
  m_leftelevatingMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse,true);
  m_leftelevatingMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward,true);

  m_rightelevatingMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
  m_rightelevatingMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);

  // Set Climber Soft Limits
  m_leftelevatingMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, 0.0);
  m_leftelevatingMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, 1400.0);

  m_rightelevatingMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, 0.0);
  m_rightelevatingMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, 1400.0);

  // Start compressor
  m_compressor.Start();

  // Initialize solenoids
  m_intakeright.Set(frc::DoubleSolenoid::Value::kForward);
  m_intakeleft.Set(frc::DoubleSolenoid::Value::kForward);
  m_colorwheel.Set(frc::DoubleSolenoid::Value::kForward);
  m_uptake.Set(frc::DoubleSolenoid::Value::kForward);

  frc::SmartDashboard::PutNumber("Override Speed", m_overrideShooterSpeed);
  frc::SmartDashboard::PutNumber("P Gain", m_shooterPIDCoeff.kP);
  //frc::SmartDashboard::PutNumber("I Gain", m_shooterPIDCoeff.kI);
  frc::SmartDashboard::PutNumber("D Gain", m_shooterPIDCoeff.kD);
  frc::SmartDashboard::PutNumber("I Zone", m_shooterPIDCoeff.kIz);
  frc::SmartDashboard::PutNumber("Feed Forward", m_shooterPIDCoeff.kFF);
  frc::SmartDashboard::PutNumber("Max Output", m_shooterPIDCoeff.kMaxOutput);
  frc::SmartDashboard::PutNumber("Min Output", m_shooterPIDCoeff.kMinOutput);

  frc::SmartDashboard::PutNumber("tx offset", m_txOFFSET);

  m_robotDrive.SetSafetyEnabled(false);

  // Initialize auto driver
  auto leftGroup  = new frc::SpeedControllerGroup(m_frontleftMotor, m_backleftMotor);
  auto rightGroup = new frc::SpeedControllerGroup(m_frontrightMotor, m_backrightMotor);
  m_drive = new Drivetrain(leftGroup, rightGroup, &m_frontleftEncoder, &m_frontrightEncoder);
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
  std::cout << "Auto Mode: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameSimpleGame)
  {
    std::cout << "Start Simple Game" << std::endl;
    m_autoTimer.Start();
    return;
  }

  if (m_autoSelected == kAutoNameDefault)
  {
    std::cout << "Default Auto: Do nothing\n";
    return;
  }

  // Pathweaver path directory
  wpi::SmallString<64> deployDirectory;
  frc::filesystem::GetDeployDirectory(deployDirectory);
  wpi::sys::path::append(deployDirectory, "output");

  // Pathweaver Modes...
  if (m_autoSelected == kAutoNamePathweaverGame)
  {
    // Read path
    wpi::sys::path::append(deployDirectory, "TestPath.wpilib.json");
    m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);
  }
  else if (m_autoSelected == kAutoNameSimplePath)
  {
    // Create path
    m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      frc::Pose2d(0_m, 0_m, 0_rad),
      {frc::Translation2d(1_m, 1_m), frc::Translation2d(2_m, -1_m)},
      frc::Pose2d(3_m, 0_m, 0_rad), frc::TrajectoryConfig(6.0_fps, 4.0_fps_sq));
      //frc::Pose2d(0_m, 0_m, 0_rad),
      //{frc::Translation2d(1_m, 0_m)},
      //frc::Pose2d(2_m, 0_m, 0_rad), frc::TrajectoryConfig(8_fps, 4_fps_sq));
  }
  else if (m_autoSelected == kAutoErikTest)
  {
    // Read path
    wpi::sys::path::append(deployDirectory, "ErikTest.wpilib.json");
    m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);
  }
  else if (m_autoSelected == kAutoBarrelRacing)
  {
    // Read path
    wpi::sys::path::append(deployDirectory, "BarrelRacing.wpilib.json");
    m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);
  }
  else if (m_autoSelected == kAutoSlalom)
  {
    // Read path
    wpi::sys::path::append(deployDirectory, "Slalom.wpilib.json");
    m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);
  }
  else if (m_autoSelected == kAutoBounce)
  {
    // Read path
    wpi::sys::path::append(deployDirectory, "Bounce.wpilib.json");
    m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);
  }
  else if (m_autoSelected == kAutoRedA)
  {
    // Read path
    wpi::sys::path::append(deployDirectory, "RedA.wpilib.json");
    m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);
  }
  else if (m_autoSelected == kAutoRedB)
  {
    // Read path
    wpi::sys::path::append(deployDirectory, "RedB.wpilib.json");
    m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);
  }
  else if (m_autoSelected == kAutoBlueA)
  {
    // Read path
    wpi::sys::path::append(deployDirectory, "BlueA.wpilib.json");
    m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);
  }
  else if (m_autoSelected == kAutoBlueB)
  {
    // Read path
    wpi::sys::path::append(deployDirectory, "BlueB.wpilib.json");
    m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);
  }

  // Start the timer
  m_timer.Start();

  // Reset the drivetrain's odometry to the starting pose of the trajectory
  m_drive->ResetOdometry(m_trajectory.InitialPose());
}

void Robot::AutonomousPeriodic()
{
  if (m_autoSelected == kAutoNameSimpleGame)
  {
    AutoSimpleGame();
    return;
  }

  if (m_autoSelected == kAutoNameDefault) {
    return;
  }

  AutoFollowPath();
}

void Robot::TeleopInit() {
  m_overrideShooterSpeed       = frc::SmartDashboard::GetNumber("Override Speed", 0);
  m_txOFFSET                   = frc::SmartDashboard::GetNumber("tx offset", m_txOFFSET);
  m_shooterPIDCoeff.kP         = frc::SmartDashboard::GetNumber("P Gain", m_shooterPIDCoeff.kP);
  //m_shooterPIDCoeff.kI         = frc::SmartDashboard::GetNumber("I Gain", m_shooterPIDCoeff.kI);
  m_shooterPIDCoeff.kD         = frc::SmartDashboard::GetNumber("D Gain", m_shooterPIDCoeff.kD);
  m_shooterPIDCoeff.kIz        = frc::SmartDashboard::GetNumber("I Zone", m_shooterPIDCoeff.kIz);
  m_shooterPIDCoeff.kFF        = frc::SmartDashboard::GetNumber("Feed Forward", m_shooterPIDCoeff.kFF);
  m_shooterPIDCoeff.kMaxOutput = frc::SmartDashboard::GetNumber("Max Output", m_shooterPIDCoeff.kMaxOutput);
  m_shooterPIDCoeff.kMinOutput = frc::SmartDashboard::GetNumber("Min Output", m_shooterPIDCoeff.kMinOutput);

  InitializePIDControllers();
}

void Robot::TeleopPeriodic()
{
  // read drive input from joystick
  double move = m_stick.GetRawAxis(1);
  double rotate = m_stick.GetRawAxis(4);

  //bool isQuickTurn = true;
  //m_robotDrive.CurvatureDrive(move, -rotate, isQuickTurn);
  m_robotDrive.ArcadeDrive(move, -0.75*rotate);

  // Shooting?
  if (fabs(m_stick.GetRawAxis(3)) > 0.75)
  {
    m_table->PutNumber("pipeline", 0); // Enable targeting pipeline of Limelight
    m_intakeleft.Set(frc::DoubleSolenoid::Value::kReverse);
    m_intakeright.Set(frc::DoubleSolenoid::Value::kReverse);

    // Is target locked?
    if (LimelightTracking())
    {
      // Calculate distance to target from Limelight data
      double ty = m_table->GetNumber("ty", 0.0);
      double distance = ((heightOfTarget - heightLimelight) / tan((constantLimelightAngle + ty) * (3.141592653 / 180)));
      double rpm = CalculateRPM(distance);

      // Override for test/calibration?
      if (m_overrideShooterSpeed > 1.0) {
        rpm = m_overrideShooterSpeed;
        frc::SmartDashboard::PutNumber("Shooter Speed", m_leftshooterEncoder.GetVelocity());
      }

      if ((distance < 420) && (distance > 96)) {
        m_shooterPID.SetReference(rpm, rev::ControlType::kVelocity); // Set shooter motor speed (based on distance)
      }
      std::cout << "distance = " << distance  << " want = " << rpm << " got = " << m_leftshooterEncoder.GetVelocity() << std::endl;

      // Enable uptake and hopper if we're at 98% of desired shooter speed
      if (m_leftshooterEncoder.GetVelocity() > (rpm * 0.98))
      {
        // Start hopper only when it's in the correct position
        auto hopperPosition = 5.0*m_hopperAltEncoder.GetPosition();
        auto remainder = std::fabs(hopperPosition - std::round(hopperPosition));
        if (remainder < 0.06) {
          m_uptake.Set(frc::DoubleSolenoid::Value::kReverse);
        }
        if (!m_isShooting) {
          m_hopperMotor.Set(0.25);
          m_hopperTimer = 0;
          m_isShooting = true;
        }
      }
    }
  }
  else
  {
    // Not shooting: Shooter off, Uptake down, and Turret centered
    m_table->PutNumber("pipeline", 1); // Set driving pipeline of Limelight
    m_leftshooterMotor.Set(0.0);
    m_uptake.Set(frc::DoubleSolenoid::Value::kForward);
    m_isShooting = false;

    //pipeline code
    if (m_stick.GetRawButtonPressed(4))
    {
      if (!m_isGathering)
      {
        m_intakeleft.Set(frc::DoubleSolenoid::Value::kReverse);
        m_intakeright.Set(frc::DoubleSolenoid::Value::kReverse);
        m_intakeMotor.Set(-0.75);
        m_hopperMotor.Set(0.25);
        m_turretPID.SetReference(30.0, rev::ControlType::kPosition);
        m_isGathering = true;
        m_hopperTimer = 0;
      }
      else
      {
        m_intakeleft.Set(frc::DoubleSolenoid::Value::kForward);
        m_intakeright.Set(frc::DoubleSolenoid::Value::kForward);
        m_intakeMotor.Set(0.0);
        m_isGathering = false;
        m_hopperTimer = 0;
        m_hopperReverse = false;
      }
    }

    if (!m_isGathering) {
      m_turretPID.SetReference(0, rev::ControlType::kPosition);
      m_intakeleft.Set(frc::DoubleSolenoid::Value::kForward);
      m_intakeright.Set(frc::DoubleSolenoid::Value::kForward);
    }
  }

  //frc::SmartDashboard::PutNumber("Hopper Speed", m_hopperEncoder.GetVelocity());

  // If gathering/shooting and the velocity drops, it's a jam!
  // Reverse the hopper for a short period
  if ((m_isGathering || m_isShooting) &&
      !m_hopperReverse &&
      (m_hopperEncoder.GetVelocity() < 500.0) &&
      (m_hopperTimer > 20)) {
    m_hopperMotor.Set(0.0);
    m_hopperReverse = true;
    m_reverseTimer = 0;
    m_hopperMotor.Set(-0.4);
  }

  // Hopper reverse complete?
  if ((m_reverseTimer > 30) && m_hopperReverse) {
    m_hopperMotor.Set(0.25);
    m_hopperReverse = false;
    m_hopperTimer = 0;
  }

  if (!m_isGathering && !m_isShooting)
  {
    m_hopperMotor.Set(0.0);
  }

  ++m_reverseTimer;
  ++m_hopperTimer;


  // Raise arms
  if (m_stick.GetRawButtonPressed(7)) {
    m_raisingPID.SetReference(18.9, rev::ControlType::kSmartMotion);
  }

  // Lower arms
  if (m_stick.GetRawButtonPressed(8)) {
    m_raisingPID.SetReference(0.0, rev::ControlType::kSmartMotion);
  }

  // Extend or retract climb arms
  if (m_stick.GetRawButton(3)) {
    if ((m_leftelevatingEncoder.GetPosition() < 100.0) || (m_leftelevatingEncoder.GetPosition() > 1300.0)) {
      m_leftelevatingMotor.Set(-0.25);
    } else {
      m_leftelevatingMotor.Set(-1.0);
    }
    if ((m_rightelevatingEncoder.GetPosition() < 100.0) || (m_rightelevatingEncoder.GetPosition() > 1300.0)) {
      m_rightelevatingMotor.Set(-0.25);
    } else {
      m_rightelevatingMotor.Set(-1.0);
    }
    //std::cout << "enc: " << m_leftelevatingEncoder.GetPosition() << "  " << m_rightelevatingEncoder.GetPosition() << std::endl;
  }
  else if (m_stick.GetRawButton(1)) {
    if ((m_leftelevatingEncoder.GetPosition() < 100.0) || (m_leftelevatingEncoder.GetPosition() > 1300.0)) {
      m_leftelevatingMotor.Set(0.25);
    } else {
      m_leftelevatingMotor.Set(1.0);
    }
    if ((m_rightelevatingEncoder.GetPosition() < 100.0) || (m_rightelevatingEncoder.GetPosition() > 1300.0)) {
      m_rightelevatingMotor.Set(0.25);
    } else {
      m_rightelevatingMotor.Set(1.0);
    }
    //std::cout << "enc: " << m_leftelevatingEncoder.GetPosition() << "  " << m_rightelevatingEncoder.GetPosition() << std::endl;
  }
  else {
    m_leftelevatingMotor.Set(0.0);
    m_rightelevatingMotor.Set(0.0);
  }

  // color wheel (ask Erik about specific button and how to get a hold down command)//
  if (m_stick.GetRawButtonPressed(5))
  {
    if (m_colorwheel.Get() == frc::DoubleSolenoid::Value::kForward)
    {
      m_colorwheel.Set(frc::DoubleSolenoid::Value::kReverse);
      m_colorwheelMotor.Set(1.0);
    }
    else {
      m_colorwheel.Set(frc::DoubleSolenoid::Value::kForward);
      m_colorwheelMotor.Set(0.0);
    }
  }
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {
    m_frontrightEncoder.SetPosition(0.0);
    m_backrightEncoder.SetPosition(0.0);
    m_frontleftEncoder.SetPosition(0.0);
    m_backleftEncoder.SetPosition(0.0);
}

void Robot::TestPeriodic()
{
  
}

bool Robot::LimelightTracking()
{
  bool shoot = false;

  // Proportional Steering Constant:
  // If your robot doesn't turn fast enough toward the target, make this number bigger
  // If your robot oscillates (swings back and forth past the target) make this smaller
  const double STEER_K = 0.04;
  const double MAX_STEER = 0.5;

  double tx = m_table->GetNumber("tx", 0.0);
  double tv = m_table->GetNumber("tv", 0.0);

  double limelightTurnCmd = 0.0;

  if (tv > 0.0)
  {
    // Proportional steering
    limelightTurnCmd = (tx + m_txOFFSET) * STEER_K;
    limelightTurnCmd = std::clamp(limelightTurnCmd, -MAX_STEER, MAX_STEER);
    if (tx < 0.25)
    {
      shoot = true;
    }
  }

  m_turretMotor.Set(limelightTurnCmd);
  return shoot;
}

// Calculate the RPM from the distance
double Robot::CalculateRPM(double d)
{
  //double rpm = 0.0169 * d * d - 4.12 * d + 2614.5;
  //double rpm = 0.01474 * d * d - 3.573 * d + 2588.0;
  //double rpm = 0.0273 * d * d - 6.27 * d + 2901.3;
  double rpm = 0.01132 * d * d - 0.762 * d + 2290.1;
  return rpm;
}

// Initialize the PID coefficients
void Robot::InitializePIDControllers()
{
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

  m_raisingPID.SetP(m_raisingPIDCoeff.kP);
  m_raisingPID.SetI(m_raisingPIDCoeff.kI);
  m_raisingPID.SetD(m_raisingPIDCoeff.kD);
  m_raisingPID.SetIZone(m_raisingPIDCoeff.kIz);
  m_raisingPID.SetFF(m_raisingPIDCoeff.kFF);
  m_raisingPID.SetOutputRange(m_raisingPIDCoeff.kMinOutput, m_raisingPIDCoeff.kMaxOutput);

  // default smart motion coefficients
  double kMaxVel = 550, kMinVel = 0, kMaxAcc = 250, kAllErr = 0;

  m_raisingPID.SetSmartMotionMaxVelocity(kMaxVel);
  m_raisingPID.SetSmartMotionMinOutputVelocity(kMinVel);
  m_raisingPID.SetSmartMotionMaxAccel(kMaxAcc);
  m_raisingPID.SetSmartMotionAllowedClosedLoopError(kAllErr);
}

void Robot::AutoSimpleGame()
{
  if (m_autoTimer.Get() <= 0.25) {
    m_robotDrive.ArcadeDrive(0.5, 0);
  }

  else {
    m_robotDrive.ArcadeDrive(0, 0);
  }

  if (m_autoTimer.Get() >= 2 && m_autoTimer.Get() <= 10) {
    m_table->PutNumber("pipeline", 0); // Enable targeting pipeline of Limelight
    m_intakeleft.Set(frc::DoubleSolenoid::Value::kReverse);
    m_intakeright.Set(frc::DoubleSolenoid::Value::kReverse);

    // Is target locked?
    if (LimelightTracking())
    {
      // Calculate distance to target from Limelight data
      double ty = m_table->GetNumber("ty", 0.0);
      double distance = ((heightOfTarget - heightLimelight) / tan((constantLimelightAngle + ty) * (3.141592653 / 180)));
      double rpm = CalculateRPM(distance);

      m_shooterPID.SetReference(rpm, rev::ControlType::kVelocity); // Set shooter motor speed (based on distance)
      std::cout << "distance = " << distance  << " want = " << rpm << " got = " << m_leftshooterEncoder.GetVelocity() << std::endl;

      // Enable uptake and hopper if we're at 98% of desired shooter speed
      if (m_leftshooterEncoder.GetVelocity() > (rpm * 0.97))
      {
        m_uptake.Set(frc::DoubleSolenoid::Value::kReverse);
        m_hopperMotor.Set(0.3);
      }
    }
  } else {
    m_hopperMotor.Set(0.0);
    m_turretPID.SetReference(0, rev::ControlType::kPosition);
    m_uptake.Set(frc::DoubleSolenoid::Value::kForward);
    m_shooterPID.SetReference(0.0, rev::ControlType::kVelocity);
  }
}

void Robot::AutoFollowPath()
{
  // Update odometry
    m_drive->UpdateOdometry();

    if (m_timer.Get() < m_trajectory.TotalTime()) {
      // Get the desired pose from the trajectory
      auto desiredPose = m_trajectory.Sample(m_timer.Get());

      // Get the reference chassis speeds from the Ramsete Controller
      //std::cout << "x = " << m_drive->GetPose().X() 
      //          <<  "y = " << m_drive->GetPose().Y() << " rot = " << m_drive->GetPose().Rotation().Degrees() << std::endl;
      //std::cout << "dx = " << desiredPose.pose.X() 
      //          << " dy = " << desiredPose.pose.Y() << " drot = " << desiredPose.pose.Rotation().Degrees() << std::endl;
      
      auto refChassisSpeeds = m_ramseteController.Calculate(m_drive->GetPose(), desiredPose);

      // Set the linear and angular speeds
      m_drive->Drive(refChassisSpeeds.vx, refChassisSpeeds.omega);
    } else {
      m_drive->Drive(0_mps, 0_rad_per_s);
    }
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
