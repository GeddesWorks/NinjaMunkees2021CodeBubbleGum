#pragma once

#include "frc/WPILib.h"
#include <string>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/spark.h>
#include <cameraserver/CameraServer.h>

#include <frc/DigitalInput.h>
#include <wpi/raw_ostream.h>
#include <frc/encoder.h>
#include <frc/TimedRobot.h>
#include <frc/Joystick.h>

#include <frc/util/color.h>
#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"

#include <frc/SpeedControllerGroup.h>

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

#include <frc/Talon.h>
#include "rev/CANSparkMax.h"
#include "ctre/Phoenix.h"
#include <frc/timer.h>
#include <frc2/timer.h>


class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
// Robot scripts setup

  void Drive();
  void Shooter();
  void Intake();
  void Climber();
  void ColorPizza();
  void Index();
  void LED();
  void Testing();

  // Auto scripts setup

  void AutoNav1();
  void AutoNav2();
  double rotationsLeftMotors;
  double rotationsRightMotors;
  // Input

  frc::Joystick JLeft{0};
  frc::Joystick JRight{1};
  frc::Joystick buttonBoard{2};
  frc::Joystick testJ{3};
  frc::DigitalInput upSwitch{1};
  frc::DigitalInput downSwitch{2};
  frc::DigitalInput ballSwitch{4};
  
  static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
  rev::ColorSensorV3 m_colorSensor{i2cPort};
  rev::ColorMatch m_colorMatcher;
 
  static constexpr frc::Color kBlueTarget = frc::Color(0.143, 0.427, 0.429);
  static constexpr frc::Color kGreenTarget = frc::Color(0.197, 0.561, 0.240);
  static constexpr frc::Color kRedTarget = frc::Color(0.561, 0.232, 0.114);
  static constexpr frc::Color kYellowTarget = frc::Color(0.361, 0.524, 0.113);
  bool haveHitColor = false;
  frc::Timer T1;
  frc::Timer T2;
  int rotations = 0;
  bool rot;
  bool isYellow = false;
// drive train setup

  float deadZone = .25;
 
  rev::CANSparkMax frontLeftMotor1{1, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax frontLeftMotor2{2, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax frontRightMotor1{3, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax frontRightMotor2{4, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax rearLeftMotor1{5, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax rearLeftMotor2{6, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax rearRightMotor1{7, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax rearRightMotor2{8, rev::CANSparkMax::MotorType::kBrushless};


  rev::CANEncoder frontLeftEncoder1 = frontLeftMotor1.GetEncoder();
  rev::CANEncoder frontRightEncoder1 = frontRightMotor1.GetEncoder();
  rev::CANEncoder rearLeftEncoder1 = rearLeftMotor1.GetEncoder();
  rev::CANEncoder rearRightEncoder1 = rearRightMotor1.GetEncoder();
  rev::CANEncoder frontLeftEncoder2 = frontLeftMotor2.GetEncoder();
  rev::CANEncoder frontRightEncoder2 = frontRightMotor2.GetEncoder();
  rev::CANEncoder rearLeftEncoder2 = rearLeftMotor2.GetEncoder();
  rev::CANEncoder rearRightEncoder2 = rearRightMotor2.GetEncoder();


  frc::SpeedControllerGroup m_left{frontLeftMotor1, frontLeftMotor2, rearLeftMotor1, rearLeftMotor2};
  frc::SpeedControllerGroup m_right{frontRightMotor1, frontRightMotor2, rearRightMotor1, rearRightMotor2};
  
  

// Aiming------------------
  std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
 
  float Kp = -.1f;
  float min_command = 0.05f;
  bool aimed;
  float distanceFromTarget;
  float hightOfTarget = 7.5623;
  float hightOfCamera = 0.8645833;
  float angleOfCamera = 122;
  float angleOfCameraFromTarget;
  
 // Talon encoder shooter-----------
  TalonFX * shoot1 = new TalonFX(9);
  TalonFX * shoot2 = new TalonFX(12);
  double speed;
  double Ospeed;
  const float shooterdeadzone = 300;

  double shooterTargetSpeed;
  double shooterActualSpeed;
  double shooterActualSpeed2;

  const float WRIST_kP = 0.001;
  const float WRIST_kI = 0;
  const float WRIST_kD = 0;
  const float WRIST_kF = 0;

  const int kPIDLoopIdx = 0;

  const int kTimeoutMs = 30;

  const float  wrist_low = 0;
  const float  wrist_pickup = 0;
  const float  wrist_mid = 4096 / 4;
  const float  wrist_high = 0.15 * 4096;

  const int kSlotIdx = 0;

// Intake-------------------
  VictorSPX * intakeMove = new VictorSPX(10);
  TalonSRX * intakeRun = new TalonSRX(13);
  TalonSRX * ballUp = new TalonSRX(18);
  bool buttonPressed = false; //Toggles intake on(true) & off(false)
  bool isRun = false;
  
// Climber------------------
  rev::CANSparkMax climber{14, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANEncoder climbEnc = climber.GetEncoder();
  rev::CANPIDController climbPID = climber.GetPIDController();
  TalonSRX * barDrive = new TalonSRX(15);  

  double pos;
  double posUp = 0;
  double posDown = 0;

  double kPe = 0.2, 
    kI = 0, 
    kD = 1, 
    kIz = 0, 
    kFF = 0, 
    kMaxOutput = 1, 
    kMinOutput = -1;

// Index-------------------
  TalonSRX * index = new TalonSRX(11);
  double indexDirection = 0;
  bool intakeIsRunning;
  bool shooterIsRunning;
  bool indexShift = false;
  bool button2Pressed = false;
  bool is2Run = false;
  bool IndexRun = false;

  bool buttonHit;

  frc::DigitalInput indexClick{0};


// LED Set-up
  frc::Spark LEDcontrol{0};
  bool canShoot;


 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
  
  float LBd;
  float RBd;
  float Ld;
  float Rd;

 

  
};