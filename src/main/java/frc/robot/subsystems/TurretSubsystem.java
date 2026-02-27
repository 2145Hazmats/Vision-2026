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
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
  private Visionsubsystem m_vision;  private TalonFX motor;  
  private PIDController turretPID = new PIDController(0.45, 0.02, 0.016);  
  
  /** Creates a new TurretSubsystem. */
  public TurretSubsystem(Visionsubsystem vision) {
    motor = new TalonFX(TurretConstants.TURRET_MOTOR_ID);
    m_vision = vision;
  }  

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Motor Angle", motor.getPosition().getValueAsDouble() / TurretConstants.TURRET_GEAR_RATIO * 360 % 360);
    SmartDashboard.putNumber("Turret Setpoint", -m_vision.calculateAngleToHub());
  }
  
  public void setTurretMotor(double speed) {
    motor.set(speed);
  }  
  
  public Command turnTurretToHub() {
    if(m_vision.getEstimatedPose() != null) {
      return Commands.run(
        () -> setTurretMotor(turretPID.calculate(motor.getPosition().getValueAsDouble() / TurretConstants.TURRET_GEAR_RATIO * 2 * Math.PI, -m_vision.calculateAngleToHub())),
        this,
        m_vision
      ).finallyDo(
        () -> setTurretMotor(0)
      );
    }    
    
    return Commands.run(() -> setTurretMotor(0), this);
  }
}
