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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import frc.robot.subsystems.EffectorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.KickerSubsystem;

public class RobotContainer {
    private EffectorSubsystem effectorSubsystem = new EffectorSubsystem();
    private KickerSubsystem kickerSubsystem = new KickerSubsystem();
    private ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final CommandXboxController joystick = new CommandXboxController(0);

    private Command effectorFastForward           = effectorSubsystem.makeEffectorSetSpeedCmd(60.5); // Top speed = 60.5
    private Command effectorSlowReverse           = effectorSubsystem.makeEffectorSetSpeedCmd(-5.0);
    private Command effectorStop                  = effectorSubsystem.makeEffectorSetSpeedCmd(0.0);
    private Command effectorwaitUntilCoralPresent = effectorSubsystem.makeEffectorWaitUntilCoralPresent();
    private Command effectorWaitUntilCoralMissing = effectorSubsystem.makeEffectorWaitUntilCoralMissing();
    private Command effectorCenterCoral           = effectorSubsystem.makeEffectorSetPositionCmd(6.0);
    private Command effectorZeroEncoder           = effectorSubsystem.makeEffectorZeroEncoderCmd();
    private Command effectorRotateToIntake        = effectorSubsystem.makeEffectorRoateToPosition(+20.0);
    private Command effectorRotateToLevel4        = effectorSubsystem.makeEffectorRoateToPosition(+40.0);
    private Command effectorRotateToLevel3        = effectorSubsystem.makeEffectorRoateToPosition(+0.0);
    private Command effectorRotateToLevel2        = effectorSubsystem.makeEffectorRoateToPosition(-20.0);
    private Command effectorRotateToLevel1        = effectorSubsystem.makeEffectorRoateToPosition(-90.0);
    private Command elevatorGoToIntakeAngle       = elevatorSubsystem.makeGoToAngleCmd(-10.0);
    private Command elevatorGoToZero              = elevatorSubsystem.makeGoToPositionCmd(0.0);
    private Command elevatorGoToL4                = elevatorSubsystem.makeGoToPositionCmd(46.5);

    private Command intakeCoral           = elevatorGoToIntakeAngle
                                                    .andThen(effectorRotateToIntake)
                                                    .andThen(effectorFastForward)
                                                    .andThen(effectorwaitUntilCoralPresent)
                                                    .andThen(effectorSlowReverse)
                                                    .andThen(effectorWaitUntilCoralMissing)
                                                    .andThen(effectorZeroEncoder)
                                                    .andThen(effectorCenterCoral)
                                                    .finallyDo((interrupted)->{if (interrupted) effectorStop.schedule();});

                                                        

    private Command kickerGoToPosition = kickerSubsystem.makeGoToPositionCmd(() -> joystick.getRightTriggerAxis());

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive `tors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);


    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        initializeSubsystems();
        configureBindings();
    }

    private void initializeSubsystems() {
        effectorSubsystem.init();
        kickerSubsystem.init();
        elevatorSubsystem.init();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * 0.4) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * 0.4) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        kickerSubsystem.setDefaultCommand(kickerGoToPosition);
        
        //joystick.povUp().onTrue(effectorRotateToLevel4.andThen(elevatorGoToL4Angle));
        //joystick.povRight().onTrue(elevatorGoToZero.andThen(effectorRotateToLevel3));
        //joystick.povDown().onTrue(effectorRotateToLevel2);
        //joystick.povLeft().onTrue(effectorRotateToLevel1);
        joystick.povUp().onTrue(elevatorGoToL4);
        joystick.povDown().onTrue(elevatorGoToZero);
        joystick.x().onTrue(intakeCoral);
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

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
