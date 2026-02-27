// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final SparkMax shooter2;
  private PIDController flywheelPID;
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    shooter2 = new SparkMax(39, MotorType.kBrushless);
    flywheelPID = new PIDController(ShooterConstants.FLYWHEEL_P, ShooterConstants.FLYWHEEL_I, ShooterConstants.FLYWHEEL_D); 
  }
  public void shooterTestBeforePID(double speed){
    shooter2.set(speed);
  }
   //public void flywheelSetpoint(double speed) {
     //flywheelPID.setSetpoint(speed);
  //}

  //public Command flywheelRevUp(double setpoint, double voltage) {
    //return Commands.run(()-> shooter2.set(flywheelPID.calculate(shooter2.get(), setpoint)));
  //}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
