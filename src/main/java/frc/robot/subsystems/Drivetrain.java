// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new test2. */

  AHRS gyro = new AHRS(SPI.Port.kMXP);

  WPI_TalonFX frontLeftDriveMotor = new WPI_TalonFX(SwerveConstants.frontLeftDriveMotorID);
  WPI_TalonSRX frontLeftTurnMotor = new WPI_TalonSRX(SwerveConstants.frontLeftTurnMotorID);
  // Encoder frontLeftTurnEncoder = new Encoder(SwerveConstants.frontLeftTurnEncoderPin1, SwerveConstants.frontLeftTurnEncoderPin2, false, Encoder.EncodingType.k2X);

  WPI_TalonFX frontRightDriveMotor = new WPI_TalonFX(SwerveConstants.frontRightDriveMotorID);
  WPI_TalonSRX frontRightTurnMotor = new WPI_TalonSRX(SwerveConstants.frontRightTurnMotorID);
  // Encoder frontRightTurnEncoder = new Encoder(SwerveConstants.frontRightTurnEncoderPin1, SwerveConstants.frontRightTurnEncoderPin2, false, Encoder.EncodingType.k2X);

  WPI_TalonFX backLeftDriveMotor = new WPI_TalonFX(SwerveConstants.backLeftDriveMotorID);
  WPI_TalonSRX backLeftTurnMotor = new WPI_TalonSRX(SwerveConstants.backLeftTurnMotorID);
  // Encoder backLeftTurnEncoder = new Encoder(SwerveConstants.backLeftTurnEncoderPin1, SwerveConstants.backLeftTurnEncoderPin2, false, Encoder.EncodingType.k2X);

  WPI_TalonFX backRightDriveMotor = new WPI_TalonFX(SwerveConstants.backRightDriveMotorID);
  WPI_TalonSRX backRightTurnMotor = new WPI_TalonSRX(SwerveConstants.backRightTurnMotorID);
  // Encoder backRightTurnEncoder = new Encoder(SwerveConstants.backRightTurnEncoderPin1, SwerveConstants.backRightTurnEncoderPin2, false, Encoder.EncodingType.k2X);

  double forwardSpeed, rightSpeed, rotation, temp, ratio, frspeed, flspeed, brspeed, blspeed, frangle, flangle, brangle, blangle, a, b, c, d, max;
  // fwd speed, rightspeed, radians rotation, temporary, ratio, module speeds x4, module angles x4, temp x4, maximum speed

  int wheelbase = 1;    // robot width/height (does not matter units, just the ratios in the end)
  int trackbase = 1;    // square bot

  /*
          NOTES
  forward speed = joystick's -Y             (forward)
  strafe right speed = joystick's X         (right)
  rotate clockwise speed = joystick 2's X   (right)

  random note: on Windows, hold alt and type: 0 1 7 6. then release alt. it will type a degree symbol: ° 

  */

  public void initTalons(WPI_TalonSRX motor, boolean isBack){
    // motor.setSelectedSensorPosition(0);
    // motor.setSafetyEnabled(true);
    motor.setInverted(false);
    motor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
    motor.setSensorPhase(false);
    // motor.config_kP(0, 0.0025);
    motor.config_kP(0, 0.9);
    //between 0.015 and 0.0015
    motor.config_kI(0, 0.0);
    motor.config_kD(0, 0.0);
    motor.config_kF(0, 0.0);
    motor.configMotionCruiseVelocity(3000);
    motor.configMotionAcceleration(2250);
    motor.configNominalOutputForward(0);
    motor.configNominalOutputReverse(0);
    motor.configPeakOutputForward(1);
    motor.configPeakOutputReverse(-1);
    motor.configAllowableClosedloopError(0, 0, 75);
    motor.setSelectedSensorPosition(0);
  }

  public Drivetrain() {
    frontLeftTurnMotor.configFactoryDefault();
    frontRightTurnMotor.configFactoryDefault();
    backLeftTurnMotor.configFactoryDefault();
    backRightTurnMotor.configFactoryDefault();

    frontLeftDriveMotor.configFactoryDefault();
    frontRightDriveMotor.configFactoryDefault();
    backRightDriveMotor.configFactoryDefault();
    backLeftDriveMotor.configFactoryDefault();

    initTalons(frontLeftTurnMotor, false);
    initTalons(frontRightTurnMotor, false);
    initTalons(backRightTurnMotor, true);
    initTalons(backLeftTurnMotor, true);

  }

  public void stopMotors(){
    frontLeftTurnMotor.set(ControlMode.PercentOutput, 0);
    // frontRightTurnMotor.set(ControlMode.PercentOutput, 0);
    // backLeftTurnMotor.set(ControlMode.PercentOutput, 0);
    // backRightTurnMotor.set(ControlMode.PercentOutput, 0);
  }

  public void spinMotorsTest(double speed){
    frontLeftTurnMotor.set(ControlMode.PercentOutput, speed);
    frontRightTurnMotor.set(ControlMode.PercentOutput, speed);
    backLeftTurnMotor.set(ControlMode.PercentOutput, speed);
    backRightTurnMotor.set(ControlMode.PercentOutput, speed);

    frontLeftDriveMotor.set(ControlMode.PercentOutput, speed);
    frontRightDriveMotor.set(ControlMode.PercentOutput, speed);
    backLeftDriveMotor.set(ControlMode.PercentOutput, speed);
    backRightDriveMotor.set(ControlMode.PercentOutput, speed);
  }
  
  public boolean isMotorAtPos(boolean isFront, boolean isRight, int position){
    if (isFront){
      if (isRight){
        return (frontRightTurnMotor.getSelectedSensorPosition() > (position - 100) && (frontRightTurnMotor.getSelectedSensorPosition() < (position + 100)));
      }
      else{
        return (frontLeftTurnMotor.getSelectedSensorPosition() > (position - 100) && (frontLeftTurnMotor.getSelectedSensorPosition() < (position + 100)));
      }
    }
    else{
      if (isRight){
        return (backRightTurnMotor.getSelectedSensorPosition() > (position - 100) && (backRightTurnMotor.getSelectedSensorPosition() < (position + 100)));
      }
      else{
        return (backLeftTurnMotor.getSelectedSensorPosition() > (position - 100) && (backLeftTurnMotor.getSelectedSensorPosition() < (position + 100)));
      }
    }
    // return false;
  }
  double ticksPerDegree = 0.0;
  public void swerveDrive(double fwdspeed, double rspeed, double cRotation){
    forwardSpeed = fwdspeed;
    rightSpeed = rspeed;
    rotation = cRotation;

    temp = (forwardSpeed * Math.cos(gyro.getAngle())) + (rightSpeed * Math.sin(gyro.getAngle()));
    rightSpeed = -(forwardSpeed * Math.sin(gyro.getAngle())) + (rightSpeed * Math.cos(gyro.getAngle()));
    forwardSpeed = temp;

    ratio = Math.sqrt((wheelbase*wheelbase) + (trackbase*trackbase));

    a = (forwardSpeed - rotation * (wheelbase/ratio));
    b = (forwardSpeed + rotation * (wheelbase/ratio));

    c = (forwardSpeed - rotation * (trackbase/ratio));
    d = (forwardSpeed + rotation * (trackbase/ratio));

    frspeed = Math.sqrt((b*b) + (c*c));

    flspeed = Math.sqrt((b*b) + (d*d));

    brspeed = Math.sqrt((a*a) + (c*c));

    blspeed = Math.sqrt((a*a) + (d*d));


    frangle = (Math.atan2(b, c) * (180 / Math.PI));

    flangle = (Math.atan2(b, d) * (180 / Math.PI));

    blangle = (Math.atan2(a, d) * (180 / Math.PI));

    brangle = (Math.atan2(a, c) * (180 / Math.PI));

    max = frspeed;
    if (flspeed > max){
      max = flspeed;
    }

    if (blspeed > max){
      max = blspeed;
    }

    if (brspeed > max){
      max = brspeed;
    }

    if (max > 1){
      frspeed /= max;
      flspeed /= max;
      blspeed /= max;
      brspeed /= max;
    }

    // wheel speeds are now 0-1
    // NOT -1 - 1
    // this is because each speed is in the direction of the module

    ticksPerDegree = (1024 / 360);
    // ticksPerDegree = 1024/360;
    // double ticksPerDegree = 360 / 1024;
    SmartDashboard.putNumber("Ticks Per Degree", ticksPerDegree);
    // 1024 = ticks per whole rotation
    // 360 = degrees in circle
    // double ticksPerDegree = 1024 / 360;

    frontLeftTurnMotor.set(ControlMode.MotionMagic, flangle * ticksPerDegree);
    frontRightTurnMotor.set(ControlMode.MotionMagic, frangle * ticksPerDegree);
    backLeftTurnMotor.set(ControlMode.MotionMagic, blangle * ticksPerDegree);
    backRightTurnMotor.set(ControlMode.MotionMagic, brangle * ticksPerDegree);

    frontLeftDriveMotor.set(ControlMode.PercentOutput, flspeed);
    frontRightDriveMotor.set(ControlMode.PercentOutput, frspeed);
    backLeftDriveMotor.set(ControlMode.PercentOutput, blspeed);
    backRightDriveMotor.set(ControlMode.PercentOutput, brspeed);

  }

  // public void updateSmartDashboard(){
  //   SmartDashboard.putNumber("FL pos", frontLeftTurnMotor.getSelectedSensorPosition());
  //   SmartDashboard.putNumber("FR pos", frontRightTurnMotor.getSelectedSensorPosition());
  //   SmartDashboard.putNumber("BL pos", backLeftTurnMotor.getSelectedSensorPosition());
  //   SmartDashboard.putNumber("BR pos", backRightTurnMotor.getSelectedSensorPosition());
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("FL turn degrees: ", 360 * frontLeftTurnMotor.getSelectedSensorPosition() / 1024);
    SmartDashboard.putNumber("FR turn degrees: ", 360 * frontRightTurnMotor.getSelectedSensorPosition() / 1024);
    SmartDashboard.putNumber("BL turn degrees: ", 360 * backLeftTurnMotor.getSelectedSensorPosition() / 1024);
    SmartDashboard.putNumber("BR turn degrees: ", 360 * frontLeftTurnMotor.getSelectedSensorPosition() / 1024);
    // SmartDashboard.putNumber("FL turn pos degrees: ", 360 * frontLeftTurnMotor.getSelectedSensorPosition() / 1024);
  }
}
