// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;

public class Sensors extends SubsystemBase {
   private SparkMax climbMotor = new SparkMax(ClimbConstants.ClimbMotorId, MotorType.kBrushless);

   private PIDController pidControllerClimbReady = new PIDController(1, 0, 0);

  private PIDController pidControllerClimbNailedIt = new PIDController(6, 0, 0);
  
  private Servo climbServo = new Servo(ClimbConstants.climbServo);
   private SparkMaxConfig ClimbConfig = new SparkMaxConfig();

      public Command KeepClimbSafeDefaultCommand() {
    return Commands.run(() -> { if (true) { 
      climbServo.set(Constants.ClimbConstants.climbLockServoPosition);
      climbMotor.set(0);
    }}, this);
  }
  /** Creates a new Sensors. */
  public Sensors() {
      ClimbConfig
    .inverted(true)
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(40);

   ClimbConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(1.0, 0.0, 0.0);
  }
  public void goodClimbInCommand() {//.05
    if (climbMotor.getEncoder().getPosition() > 1.5) {// 1.4
      climbMotor.set(0);
    }
    else {//1.8457
      climbServo.set(Constants.ClimbConstants.climbLockServoPosition);
      climbMotor.set(Constants.ClimbConstants.climbForwardSpeed);}
  } 

  public void climbOutCommandpart1() {
    climbServo.set(Constants.ClimbConstants.climbUnlockServoPosition);
  }

  public void climbOutCommandpart2() {
    climbServo.set(Constants.ClimbConstants.climbUnlockServoPosition);
    climbMotor.set(Constants.ClimbConstants.ClimbBackwardSpeed);
  }
   public void climbToSetpointPID() {
    climbServo.set(Constants.ClimbConstants.climbUnlockServoPosition);
    climbMotor.set(pidControllerClimbReady.calculate(climbMotor.getEncoder().getPosition(), 
    Constants.ClimbConstants.ClimbReadySetpoint));
  }

  public void climbToNailItPID() {
    climbServo.set(Constants.ClimbConstants.climbLockServoPosition);
    climbMotor.set(pidControllerClimbNailedIt.calculate(climbMotor.getEncoder().getPosition(), 
    Constants.ClimbConstants.ClimbLockedInSetpoint));
  }

    // Same Command?? cody fix it
  public void climbStopCommand() {
    climbMotor.set(Constants.ClimbConstants.ClimbStop);
  }
  public void disableClimbMotor() {
    climbMotor.set(0);
  }

  // Untested lol prolly dont need it (STANLEY!!)
  // public void ClimbJoystick(double joystick) { //MIGHT NEED TO CHANGE THIS TO SUPPLIER
  //   climbMotorNeo.set(joystick * .3); //NERFED SPEED CHANGE LATER
  // }
  // public void ClimbJoystickServo(DoubleSupplier joystick) { //MIGHT NEED TO CHANGE THIS TO SUPPLIER
  //   climbServo.set(joystick.getAsDouble() * .3); //NERFED SPEED CHANGE LATER
  // }

  // Servo

  public Command climbUnlock() {
    return Commands.run(() -> climbServo.set(Constants.ClimbConstants.climbUnlockServoPosition), this);
  }

  public Command climbLock() {
    return Commands.run(() -> climbServo.set(Constants.ClimbConstants.climbLockServoPosition), this);
  }

  public void disableClimbServo() {
    climbServo.set(0);
  }

  public void resetMotorPosition() {
    climbMotor.getEncoder().setPosition(0);
  }

  public void PutTheServoInTheRightSpotPlease() {

    climbServo.set(Constants.ClimbConstants.climbLockServoPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
