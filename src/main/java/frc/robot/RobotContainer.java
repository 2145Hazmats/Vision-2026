// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ShooterConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FlagMotorFlywheelPIDSubsystem;
import frc.robot.subsystems.Sensors;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.Visionsubsystem;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric pointRobotAt = new SwerveRequest.RobotCentric();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain;
    public final Visionsubsystem vision;
    public final TurretSubsystem turret;
    private Sensors m_sensors = new Sensors(); 
    // public final ShooterSubsystem shooter;
    // public final FlagMotorFlywheelPIDSubsystem pidFlywheel;
    

    public RobotContainer() {
        drivetrain = TunerConstants.createDrivetrain();
        vision = new Visionsubsystem(drivetrain);
        turret = new TurretSubsystem(vision);
        // shooter = new ShooterSubsystem();
        // pidFlywheel = new FlagMotorFlywheelPIDSubsystem();
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        
        m_sensors.setDefaultCommand(m_sensors.KeepClimbSafeDefaultCommand());
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
 
        joystick.leftTrigger().whileTrue(Commands.run(() -> m_sensors.goodClimbInCommand(), m_sensors));
        //P2controller.rightTrigger().whileTrue(Commands.run(() -> m_ClimbSubsystemNeo.PutTheServoInTheRightSpotPlease(), m_ClimbSubsystemNeo).until(()->m_ClimbSubsystemNeo.ReadyToStickTheClimbOutIGuess()).andThen(Commands.waitSeconds(4)).andThen(()-> m_ClimbSubsystemNeo.climbForwardCommand()));//.andThen(() ->m_ClimbSubsystemNeo.climbForwardCommand()));
        //P2controller.x().whileTrue(Commands.run(() -> m_ClimbSubsystemNeo.PutTheServoInTheRightSpotPlease(), m_ClimbSubsystemNeo));
        
        /*
        joystick.rightTrigger().whileTrue(
            Commands.run(() -> m_sensors.climbOutCommandpart1(), m_sensors)
            .withTimeout(0.8).andThen(Commands.run(() -> m_sensors.climbToSetpointPID(), m_sensors)));
        */

        turret.setDefaultCommand(turret.turnTurretToHub());


        // joystick.x().whileTrue(drivetrain.applyRequest(() -> pointRobotAt
        //     .withRotationalRate(vision.getPIDControllerOutput() * MaxAngularRate)
        // ));
        // joystick.y().whileTrue(drivetrain.applyRequest(() -> pointRobotAt
        // .withRotationalRate(vision.getGenericAngle() *MaxAngularRate)));

        //slow mode
        joystick.rightBumper().whileTrue( drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * 0.5 * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX()* 0.5 * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX()* 0.5 * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
        
        //joystick.povDown().whileTrue(Commands.run(()-> shooter.shooterTestBeforePID(1), shooter)
        //.finallyDo(()-> shooter.shooterTestBeforePID(0)));  
        
        // joystick.rightTrigger().whileTrue(pidFlywheel.flywheelRevUp(2000)
        // .finallyDo(() -> pidFlywheel.setShooterMotor(0)));
    }


    public Command pointTurretToHub() {
        return null;
    }
     

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
}
