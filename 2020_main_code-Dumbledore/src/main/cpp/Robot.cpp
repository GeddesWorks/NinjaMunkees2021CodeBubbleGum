
#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  frc::Color detectedColor = m_colorSensor.GetColor();m_colorMatcher.AddColorMatch(kBlueTarget);

  m_colorMatcher.AddColorMatch(kGreenTarget);
  m_colorMatcher.AddColorMatch(kRedTarget);
  m_colorMatcher.AddColorMatch(kYellowTarget);

  // ECODERS
  shoot1->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, kTimeoutMs);
  shoot1->ConfigPeakOutputForward(1);
  shoot1->ConfigPeakOutputReverse(-1);
  shoot1->Config_kF(kPIDLoopIdx, 0.1097, kTimeoutMs);
  shoot1->Config_kP(kPIDLoopIdx, 0.22, kTimeoutMs);
  shoot1->Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
  shoot1->Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);
  shoot1->SetSensorPhase(false);
  shoot1->SetInverted(false);
  
  shoot2->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, kTimeoutMs);
  shoot2->ConfigPeakOutputForward(1);
  shoot2->ConfigPeakOutputReverse(-1);
  shoot2->Config_kF(kPIDLoopIdx, 0.1097, kTimeoutMs);
  shoot2->Config_kP(kPIDLoopIdx, 0.22, kTimeoutMs);
  shoot2->Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
  shoot2->Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);
  shoot2->SetSensorPhase(false);
  shoot2->SetInverted(false);

  climbPID.SetP(kPe);
  climbPID.SetI(kI);
  climbPID.SetD(kD);
  climbPID.SetIZone(kIz);
  climbPID.SetFF(kFF);
  climbPID.SetOutputRange(kMinOutput, kMaxOutput);

  index->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, kTimeoutMs);

  T1.Start();
  T2.Start();

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
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here


  } else {
    // Default Auto goes here
    /*shooterActualSpeed = shoot1->GetSelectedSensorVelocity();
    shooterTargetSpeed = 4500; //Replace this with real number

    shoot1->Set(ControlMode::Velocity, shooterTargetSpeed); 
    shoot2->Set(ControlMode::Velocity, shooterTargetSpeed * -1);
    if(shooterActualSpeed < shooterTargetSpeed + shooterdeadzone && shooterActualSpeed > shooterTargetSpeed - shooterdeadzone){
      shooterIsRunning = true;
    }
    else{
      shooterIsRunning = false;
    }

    if (shooterIsRunning == true){
      index->Set(ControlMode::Velocity, -1); 
    }
    else{
      index->Set(ControlMode::Velocity, 0);
    }
    */
    //AutoNav1();
    
    
    rotationsLeftMotors += 800;
  rotationsRightMotors += 800;
    frontLeftEncoder1.SetPosition(rotationsLeftMotors);
    frontRightEncoder1.SetPosition(rotationsRightMotors);
    rearLeftEncoder1.SetPosition(rotationsLeftMotors); 
    rearRightEncoder1.SetPosition(rotationsRightMotors); 
    frontLeftEncoder2.SetPosition(rotationsLeftMotors); 
    frontRightEncoder2.SetPosition(rotationsRightMotors); 
    rearLeftEncoder2.SetPosition(rotationsLeftMotors);  
    rearRightEncoder2.SetPosition(rotationsRightMotors);
  
  }
}

void Robot::AutoNav1()
{

  //drive stright towards first target
  rotationsLeftMotors += 20;
  rotationsRightMotors += 20;
  frc::Wait(.5);
  
  // after pass target start to turn right and loop
  rotationsLeftMotors += 60;
  rotationsRightMotors += 10;
  frc::Wait(.5);

  // drive towards second target
  rotationsLeftMotors += 20;
  rotationsRightMotors += 20;
  frc::Wait(.5);

  // drive left around target
  rotationsLeftMotors += 10;
  rotationsRightMotors += 60;
  frc::Wait(.5);

  //drive towards last target
  rotationsLeftMotors += 20;
  rotationsRightMotors += 20;
  frc::Wait(.5);

  // drive left around target but not fully
  rotationsLeftMotors += 10;
  rotationsRightMotors += 30;
  frc::Wait(.5);

  // drive tpowards end 
  rotationsLeftMotors += 70;
  rotationsRightMotors += 70;
  frc::Wait(.5);
  
  // done

  // drive motors
   
}

void Robot::AutoNav2(){
  //drive forwards and to the left
  frontLeftEncoder1.SetPosition(rotationsLeftMotors);
  frontRightEncoder1.SetPosition(rotationsRightMotors);
  rearLeftEncoder1.SetPosition(rotationsLeftMotors); 
  rearRightEncoder1.SetPosition(rotationsRightMotors); 
  frontLeftEncoder2.SetPosition(rotationsLeftMotors); 
  frontRightEncoder2.SetPosition(rotationsRightMotors); 
  rearLeftEncoder2.SetPosition(rotationsLeftMotors);  
  rearRightEncoder2.SetPosition(rotationsRightMotors); 
}
void Robot::TeleopInit() {
 
}

void Robot::TeleopPeriodic() {

  //ColorPizza();
  Drive();
  Shooter();
  Intake();
  Climber();
  Index();
  LED();
  //Testing();

  frc::SmartDashboard::PutBoolean("ballIn", ballSwitch.Get());
  frc::SmartDashboard::PutNumber("frontRightEncoder", frontRightEncoder1.GetVelocity());
  frc::SmartDashboard::PutNumber("frontLeftEncoder", frontLeftEncoder1.GetVelocity());
  frc::SmartDashboard::PutNumber("rearRightEncoder", rearRightEncoder1.GetVelocity());
  frc::SmartDashboard::PutNumber("rearLeftEncoder", rearLeftEncoder1.GetVelocity());
  frc::SmartDashboard::PutNumber("climber", climbEnc.GetPosition());


  std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  double targetArea = table->GetNumber("ta",0.0);

}

  void Robot::ColorPizza() {
    double confidence = 0.0;
    frc::Color detectedColor = m_colorSensor.GetColor();
    std::string colorString;
    std::string seenColor;
    frc::Color matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);
    
    
    if (matchedColor == kBlueTarget) {
      colorString = "Red";
      seenColor = "blue";
    } else if (matchedColor == kRedTarget) {
      colorString = "Blue";
      seenColor = "red";
    } else if (matchedColor == kGreenTarget) {
      colorString = "Yellow";
      seenColor = "green";
    } else if (matchedColor == kYellowTarget) {
      colorString = "Green";
      seenColor = "yellow";
    } else {
      colorString = "Unknown";
    }

    

    std::string gameData;
    gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
    if(gameData.length() > 0){
      switch (gameData[0]){
        case 'B' :
          //Blue case code
          break;
        case 'G' :
          //Green case code
          break;
        case 'R' :
          //Red case code
          break;
        case 'Y' :
          //Yellow case code
          break;
        default :
          //This is corrupt data
          break;
      }
    } 
    else {
      //Code for no data received yet
    }
    if(buttonBoard.GetRawButton(8)){
      if(gameData == "G"){
        // Checks green---------
        if(colorString == "Green" || haveHitColor == true){ 
          haveHitColor = true;
          if(seenColor == "blue"){
            barDrive->Set(ControlMode::PercentOutput, -.25);
          }
          else if(seenColor == "red"){
            barDrive->Set(ControlMode::PercentOutput, .25);
          }
          else if(seenColor == "yellow"){
            barDrive->Set(ControlMode::PercentOutput, 0);
          }
          else{
            haveHitColor = false;
          }
        }
        
        else{
          barDrive->Set(ControlMode::PercentOutput, -.75);
        }
      }

      else if (gameData == "R"){
        // Checks red---------
        if(colorString == "Red" || haveHitColor == true){ 
          haveHitColor = true;
          if(seenColor == "green"){
            barDrive->Set(ControlMode::PercentOutput, -.25);
          }
          else if(seenColor == "yellow"){
            barDrive->Set(ControlMode::PercentOutput, .25);
          }
          else if(seenColor == "blue"){
            barDrive->Set(ControlMode::PercentOutput, 0);
          }
          else{
            haveHitColor = false;
          }
        }
        else{
          barDrive->Set(ControlMode::PercentOutput, -.75);
        }
      }
      else if (gameData == "B"){
        // Checks blue---------
        if(colorString == "Blue" || haveHitColor == true){ 
          haveHitColor = true;
          if(seenColor == "yellow"){
            barDrive->Set(ControlMode::PercentOutput, -.25);
          }
          else if(seenColor == "green"){
            barDrive->Set(ControlMode::PercentOutput, .25);
          }
          else if(seenColor == "red"){
            barDrive->Set(ControlMode::PercentOutput, 0);
          }
          else{
            haveHitColor = false;
          }
        }
        else{
          barDrive->Set(ControlMode::PercentOutput, -.75);
        }
      }
      else if (gameData == "Y"){
        // Checks yellow---------
        if(colorString == "Yellow" || haveHitColor == true){ 
          haveHitColor = true;
          if(seenColor == "red"){
            barDrive->Set(ControlMode::PercentOutput, -.25);
          }
          else if(seenColor == "blue"){
            barDrive->Set(ControlMode::PercentOutput, .25);
          }
          else if(seenColor == "green"){
            barDrive->Set(ControlMode::PercentOutput, 0);
          }
          else{
            haveHitColor = false;
          }
        }
        else{
          barDrive->Set(ControlMode::PercentOutput, -.75);
        }
      }
    }
    else{
      barDrive->Set(ControlMode::PercentOutput, 0);
    }
    frc::SmartDashboard::PutString("Detected color", colorString);
    frc::SmartDashboard::PutString("Target color", gameData);
    //frc::SmartDashboard::PutNumber("time", T1.Get());
    /*if(T1.Get() == .15){
      T1.Reset();
      
      if(seenColor == "yellow" && isYellow == false){
        rotations += 1;
        isYellow = true;
      }
      else if(seenColor != "yellow" && isYellow == true){
        isYellow == false;
      }
    }
    if(rotations >= 8){
      frc::SmartDashboard::PutBoolean("rotations", true);
    }
    else{
      frc::SmartDashboard::PutBoolean("rotations", false);
    }*/
  }

  void Robot::Intake() {
    bool up = upSwitch.Get();
    bool down = downSwitch.Get();
    if(up == true){
      frc::SmartDashboard::PutString("Intake is", "UP");
    }
    else if(down == true){
      frc::SmartDashboard::PutString("Intake is", "DOWN");
    }
    else{
      frc::SmartDashboard::PutString("Intake is", "In Between");
    }
    

    // upSwitch = 1 when intake is up
    // downSwitch = 1 when intake is down

    if(buttonBoard.GetRawButton(1) == 1 && up == false){
      intakeMove->Set(ControlMode::PercentOutput, -.6);
    }
    else if(buttonBoard.GetRawButton(5) == 1 && down == false){
      intakeMove->Set(ControlMode::PercentOutput, .2);
    }
    else{
      intakeMove->Set(ControlMode::PercentOutput, 0);
    }
    
    if(buttonBoard.GetRawButton(9)){  //Has intake button been pushed?
      if(buttonPressed == false){
        buttonPressed = true;
        if(isRun == false){
          intakeRun->Set(ControlMode::PercentOutput, -.8);
          isRun = true;
        }
        else{
          isRun = false;
          intakeRun->Set(ControlMode::PercentOutput, 0);
        }
      } 
      
    } 
    else{
      buttonPressed = false;
    }
  }

  void Robot::Drive() {
    LBd = JLeft.GetY();
    RBd = JRight.GetY();

    if (buttonBoard.GetRawButton(10) == 0){

      //Left deadZone
      if (LBd > deadZone){
        Ld = ((LBd - deadZone) * (1 / (1 - deadZone)));
      }
      else if (LBd < - deadZone){
        Ld = ((LBd + deadZone) * (-1 / (-1 + deadZone)));
      }
      else{
        Ld = 0;
      }

      //Right deadZone
      if (RBd > deadZone){
        Rd = ((RBd - deadZone) * (1 / (1 - deadZone)));
      }
      else if (RBd < - deadZone){
        Rd = ((RBd + deadZone) * (-1 / (-1 + deadZone)));
      }
      else{
        Rd = 0;
      }
    }

    

    //Aiming-
    std::shared_ptr<NetworkTable> table = NetworkTable::GetTable("limelight");
    float tx = table->GetNumber("tx",0.0);
    //double targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
    double targetOffsetAngle_Vertical = table->GetNumber("ty",0.0);
    //double targetArea = table->GetNumber("ta",0.0);
    //double targetSkew = table->GetNumber("ts",0.0);

    if (buttonBoard.GetRawButton(10)){
      float heading_error = -tx;
      float steering_adjust = 0.0f;
      if (tx > 1.0){
              steering_adjust = Kp*heading_error - min_command;
      }
      else if (tx < 1.0){
              steering_adjust = Kp*heading_error + min_command;
      }
      Ld -= steering_adjust ;
      Rd += steering_adjust;
      frc::SmartDashboard::PutNumber("steering", steering_adjust);
      frc::SmartDashboard::PutNumber("tx", tx);
    }
    if(tx < .1 && tx > -.1){
      aimed = true;
    }
    else{
      aimed = false;
    }
    
    angleOfCameraFromTarget = targetOffsetAngle_Vertical;
    distanceFromTarget = 500; // (hightOfTarget - hightOfCamera) / tan(angleOfCamera + angleOfCameraFromTarget);

    frc::SmartDashboard::PutNumber("Distance From Target", distanceFromTarget); 

    if(JLeft.GetRawButton(1) == 1){
      Ld = Ld;
    }
    else{
      Ld = Ld * .5;
    }
    if(JRight.GetRawButton(1) == 1){
      Rd = Rd;
    }
    else{
      Rd = Rd * .5;
    }


    m_left.Set(Ld * -1);
    m_right.Set(Rd);
  }

  void Robot::Shooter(){

		double leftYstick = JLeft.GetY();

		shooterActualSpeed = shoot1->GetSelectedSensorVelocity();
    shooterActualSpeed2 = shoot2->GetSelectedSensorVelocity();
    frc::SmartDashboard::PutNumber("Shooter Actual Speed", shooterActualSpeed);
    //frc::SmartDashboard::PutNumber("Shooter Actual Speed2", shooterActualSpeed);

    shooterTargetSpeed = 8000; //distanceFromTarget; // * some number;

    if(buttonBoard.GetRawButton(11)){
      shoot2->Config_kP(kPIDLoopIdx, 0.22, kTimeoutMs);
      shoot1->Config_kP(kPIDLoopIdx, 0.22, kTimeoutMs);
      shoot1->Set(ControlMode::Velocity, shooterTargetSpeed); 
      shoot2->Set(ControlMode::Velocity, shooterTargetSpeed * -1);
      ballUp->Set(ControlMode::PercentOutput, 1); 
      if(shooterActualSpeed < (shooterTargetSpeed + shooterdeadzone) && shooterActualSpeed > (shooterTargetSpeed - shooterdeadzone)){
        shooterIsRunning = true;
      }
      else{
        shooterIsRunning = false;
      }
    }
    else{
      shoot2->Config_kP(kPIDLoopIdx, 0.01, kTimeoutMs);
      shoot1->Config_kP(kPIDLoopIdx, 0.01, kTimeoutMs);
      shoot1->Set(ControlMode::Velocity, 0); 
      shoot2->Set(ControlMode::Velocity, 0); 
      ballUp->Set(ControlMode::PercentOutput, 0);
      shooterIsRunning = false;
    }
    frc::SmartDashboard::PutNumber("Shooter Target Speed", shooterTargetSpeed);
    if(shooterIsRunning == true){
      frc::SmartDashboard::PutBoolean("Shooter Is Shooting", true);
    }
    else{
      frc::SmartDashboard::PutBoolean("Shooter Is Shooting", false);
    }
  }

  void Robot::Climber(){
    bool goUp = buttonBoard.GetRawButton(2);
    bool goDown = buttonBoard.GetRawButton(6);

    if(goUp == 1){
      pos = posUp;
    }
    else if(goDown == 1){
     
      pos = posDown;
    }
    else{}

    if(buttonBoard.GetRawButton(3)){
      barDrive->Set(ControlMode::PercentOutput, -1);
    }
    else if(buttonBoard.GetRawButton(7)){
      barDrive->Set(ControlMode::PercentOutput, 1);
    }
    else{
      barDrive->Set(ControlMode::PercentOutput, 0);
    }

    //climbPID.SetReference(pos, rev::ControlType::kPosition);
    
    climber.Set(testJ.GetY());
  }

  void Robot::Index(){
    if (buttonBoard.GetRawButton(11) && shooterIsRunning == true){
      index->Set(ControlMode::PercentOutput, -.3f); 
      //ballUp->Set(ControlMode::PercentOutput, 1);
    }
    else if(buttonBoard.GetRawButton(4) && indexShift == false){
      T2.Reset();
      indexShift = true;
      
    }
    else if(indexShift == false && buttonBoard.GetRawButton(11) == false){
      index->Set(ControlMode::PercentOutput, 0); // 0
      
    }
    else{
      //ballUp->Set(ControlMode::PercentOutput, 0); 
    }

    
    if (indexShift == true) {
      
      if(indexClick.Get() && T2.Get() > .3){
        indexShift = false;
      }
      else{
        index->Set(ControlMode::PercentOutput, -.5); 
      }
    }
    

    
  }

  void Robot::LED(){
    
    if(aimed == true){
      LEDcontrol.Set(-0.91);
      frc::SmartDashboard::PutBoolean("Aimed", true);
    }
    else{
      LEDcontrol.Set(-0.79);
      frc::SmartDashboard::PutBoolean("Aimed", false);
    }

  }

  void Robot::Testing(){
    speed = frc::SmartDashboard::GetNumber("DB/Slider 0", 0);
    Ospeed = speed / 100;
    

    if(buttonBoard.GetRawButton(11)){
        shoot1->Set(ControlMode::PercentOutput, Ospeed);
        shoot2->Set(ControlMode::PercentOutput, Ospeed * -1);
        ballUp->Set(ControlMode::PercentOutput, 1);
        if(shooterActualSpeed < shooterTargetSpeed + shooterdeadzone && shooterActualSpeed > shooterTargetSpeed - shooterdeadzone){
          shooterIsRunning = true;
        }
        else{
          shooterIsRunning = false;
        }
      }
      else{
        shoot1->Set(ControlMode::Velocity, 0); 
        shoot2->Set(ControlMode::Velocity, 0); 
        shooterIsRunning = false;
        ballUp->Set(ControlMode::PercentOutput, 0);
      }
      frc::SmartDashboard::PutNumber("Shooter Target Speed", shooterTargetSpeed);
      if(shooterIsRunning == true){
        frc::SmartDashboard::PutBoolean("Shooter Is Shooting", true);
      }
      else{
        frc::SmartDashboard::PutBoolean("Shooter Is Shooting", false);
      }
  }

void Robot::TestPeriodic() {
  
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
