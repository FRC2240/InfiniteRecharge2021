// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/DriverStation.h>
#include <frc/Encoder.h>
#include <frc/PWMVictorSPX.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <units/angular_velocity.h>
#include <units/acceleration.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <wpi/math>

#include "rev/CANSparkMax.h"
#include "AHRS.h"

/**
 * Represents a differential drive style drivetrain.
 */
class Drivetrain {
 public:
  Drivetrain(
      frc::SpeedControllerGroup* leftGroup,
      frc::SpeedControllerGroup* rightGroup,
      rev::CANEncoder* leftEncoder,
      rev::CANEncoder* rightEncoder
      ) :
  m_leftGroup(leftGroup),
  m_rightGroup(rightGroup),
  m_leftEncoder(leftEncoder),
  m_rightEncoder(rightEncoder)
  {
    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_leftEncoder->SetPositionConversionFactor(kDistancePerRotation);
    m_rightEncoder->SetPositionConversionFactor(kDistancePerRotation);
    m_leftEncoder->SetVelocityConversionFactor(kDistancePerRotation/60.0);
    m_rightEncoder->SetVelocityConversionFactor(kDistancePerRotation/60.0);

    m_leftEncoder->SetPosition(0.0);
    m_rightEncoder->SetPosition(0.0);

    // Instantiate gyro
    try {
			m_gyro = new AHRS(frc::SPI::Port::kMXP);
		} catch (std::exception& ex ) {
			std::string err_string = "Error instantiating navX MXP:  ";
			err_string += ex.what();
			frc::DriverStation::ReportError(err_string.c_str());
		}

    m_gyro->Reset();
  }

  void SetSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds);
  void Drive(units::meters_per_second_t xSpeed,
             units::radians_per_second_t rot);
  void UpdateOdometry();
  void ResetOdometry(const frc::Pose2d& pose);
  frc::Pose2d GetPose() const;

 private:

  static constexpr double kP = 2.42;                                // measured
  static constexpr auto kS = 0.314_V;                               // measured
  static constexpr auto kV = 1.57 * 1_V * 1_s / 1_m;                // measured
  static constexpr auto kA = 0.156 * 1_V * 1_s * 1_s / 1_m;         // measured
  static constexpr units::meter_t kTrackWidth = 0.381_m * 2;        // measured

  static constexpr double kWheelDiameter = 0.1524;                  // meters (6 inches)

  static constexpr auto kMaxAcceleration = 1.0_mps_sq;              // estimate
  static constexpr units::meters_per_second_t kMaxSpeed = 3.5_mps;  // estimate
  static constexpr double kDistancePerRotation = wpi::math::pi * kWheelDiameter;

  frc::SpeedControllerGroup* m_leftGroup;
  frc::SpeedControllerGroup* m_rightGroup;

  rev::CANEncoder* m_leftEncoder;
  rev::CANEncoder* m_rightEncoder;

  frc2::PIDController m_leftPIDController{kP, 0.0, 0.0};
  frc2::PIDController m_rightPIDController{kP, 0.0, 0.0};

  AHRS* m_gyro;

  frc::DifferentialDriveKinematics m_kinematics{kTrackWidth};
  frc::DifferentialDriveOdometry m_odometry{m_gyro->GetRotation2d()};

  // Gains are for example purposes only - must be determined for your own
  // robot!
  frc::SimpleMotorFeedforward<units::meters> m_feedforward{kS, kV, kA};
};