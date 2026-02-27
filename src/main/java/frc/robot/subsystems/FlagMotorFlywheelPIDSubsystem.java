// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class FlagMotorFlywheelPIDSubsystem extends SubsystemBase {
  private TalonFX shooterMotor; 
  
  private PIDController flywheelPID;
  /** Creates a new FlagMotorFlywheelPIDSubsystem. */
  public FlagMotorFlywheelPIDSubsystem() {
   shooterMotor = new TalonFX(ShooterConstants.SHOOTER_MOTOR_ID);
   flywheelPID = new PIDController(ShooterConstants.FLYWHEEL_P, ShooterConstants.FLYWHEEL_I, ShooterConstants.FLYWHEEL_D); 
  }
     public void setShooterMotor(double speed) {
    shooterMotor.set(speed);
  }
  public void flywheelSetpoint(double speed) {
    flywheelPID.setSetpoint(speed);
  }
   public Command flywheelRevUp(double setpoint) {
    return Commands.run(()-> shooterMotor.set(flywheelPID.calculate(shooterMotor.getVelocity().getValueAsDouble() * 60, setpoint)));
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("RPM",flywheelPID.calculate(shooterMotor.getVelocity().getValueAsDouble() * 60));
    // This method will be called once per scheduler run
  }
}
