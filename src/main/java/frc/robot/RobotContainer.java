// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import javax.sound.sampled.SourceDataLine;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import frc.robot.subsystems.EffectorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.KickerSubsystem;

public class RobotContainer {

    private enum PoseState {SAFE, INTAKE, L1, L2, L3, L4, LIFT};

    private static PoseState previousPoseState = PoseState.SAFE;

    private EffectorSubsystem effectorSubsystem = new EffectorSubsystem();
    private KickerSubsystem kickerSubsystem = new KickerSubsystem();
    private ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final CommandXboxController controllerDriver   = new CommandXboxController(0);
    private final CommandXboxController controllerOperator = new CommandXboxController(1);

    private final Command makeSetNewPoseState(PoseState newState) { return Commands.runOnce(() -> previousPoseState = newState); }
    private final Command makeEffectorFastForward()           { return effectorSubsystem.makeSetSpeed(60.5);} // Top speed = 60.5
    private final Command makeEffectorStop()                  { return effectorSubsystem.makeSetSpeed(0.0);} 
    private final Command makeEffectorCenterCoral()           { return effectorSubsystem.makeMoveTo(6.0);} 
    private final Command makeEffectorZeroEncoder()           { return effectorSubsystem.makeZeroEncoder();}
    private final Command makeEffectorRotateToSafe()          { return effectorSubsystem.makeRotateTo(+ 0.0);}
    private final Command makeEffectorSubsystem()             { return effectorSubsystem.makeRunSideways(); }



    private final Command makeElevatorRotateToSafe()              { return elevatorSubsystem.makeGoToAngleCmd(0);}

    private final Command makeElevatorGoToSafe()                  { return elevatorSubsystem.makeGoToPositionCmd(0.0);}

    private final Command makeEffectorRotateToIntake()        { return effectorSubsystem.makeRotateTo(+55.0);}
    private final Command makeElevatorRotateToIntake()            { return elevatorSubsystem.makeGoToAngleCmd(-14.0);}
    private final Command makeElevatorGoToIntake()                  { return elevatorSubsystem.makeGoToPositionCmd(1.5);}
    private final Command makeEffectorSlowReverse()           { return effectorSubsystem.makeSetSpeed(-10.0);} 
    private final Command makeEffectorWaitUntilCoralPresent() { return effectorSubsystem.makeWaitUntilCoralPresent();} 
    private final Command makeEffectorWaitUntilCoralMissing() { return effectorSubsystem.makeWaitUntilCoralMissing();} 

    private final Command makeEffectorDepositCoral()          { return  effectorSubsystem.makeZeroEncoder()
        .andThen(effectorSubsystem.makeMoveTo(20.0));} 
    private final Command makeEffectorSidewaysDepositCoral()          { return  effectorSubsystem.makeZeroEncoder()
        .andThen(makeEffectorSubsystem()
        .andThen(Commands.waitSeconds(2))
        .andThen(makeEffectorStop()));}
    
    private final Command makeEffectorRotateToL4()        { return effectorSubsystem.makeRotateTo(+40.0);} 
    private final Command makeEffectorMoveToL4()              { return effectorSubsystem.makeMoveTo(3.0);} 
    private final Command makeElevatorRotateToPreL4()              { return elevatorSubsystem.makeGoToAngleCmd(12);}
    private final Command makeElevatorRotateToL4()              { return elevatorSubsystem.makeGoToAngleCmd(17);}
    private final Command makeElevatorGoToL4()                    { return elevatorSubsystem.makeGoToPositionCmd(26.5);}

    private final Command makeEffectorRotateToL3()        { return effectorSubsystem.makeRotateTo(+8.0);} 
    private final Command makeElevatorRotateToPreL3()              { return elevatorSubsystem.makeGoToAngleCmd(19);}
    private final Command makeElevatorRotateToL3()              { return elevatorSubsystem.makeGoToAngleCmd(23);}
    private final Command makeElevatorGoToL3()                    { return elevatorSubsystem.makeGoToPositionCmd(13);}

    private final Command makeEffectorRotateToL2()        { return effectorSubsystem.makeRotateTo(0);} 
    private final Command makeElevatorRotateToPreL2()              { return elevatorSubsystem.makeGoToAngleCmd(32);}
    private final Command makeElevatorRotateToL2()              { return elevatorSubsystem.makeGoToAngleCmd(37);}
    private final Command makeElevatorGoToL2()                    { return elevatorSubsystem.makeGoToPositionCmd(4.5);}

    private final Command makeEffectorRotateToL1()        { return effectorSubsystem.makeRotateTo(-59);} 
    private final Command makeElevatorRotateToPreL1()              { return elevatorSubsystem.makeGoToAngleCmd(42);}
    private final Command makeElevatorRotateToL1()              { return elevatorSubsystem.makeGoToAngleCmd(47);}  // 40 was about 2 inches too high
    private final Command makeElevatorGoToL1()                    { return elevatorSubsystem.makeGoToPositionCmd(1);}

    
    private final Command makeEffectorRotateToLift()        { return effectorSubsystem.makeRotateTo(90);} 
    private final Command makeElevatorRotateToLift()              { return elevatorSubsystem.makeGoToAngleCmd(80);}
    private final Command makeElevatorGoToLift()                    { return elevatorSubsystem.makeGoToPositionCmd(0);}

    private final Command say(String msg) {return Commands.runOnce(() -> System.out.println(msg));}

    private final Command makeRobotSafe() {
        return makeElevatorRotateToPreL4()
                .andThen(makeElevatorGoToSafe())
                .andThen(makeElevatorRotateToSafe())
                .andThen(makeEffectorRotateToSafe())
                .andThen(makeSetNewPoseState(PoseState.SAFE));
    };

    private final Command robotIntake                   =  makeRobotSafe()
                                                          .andThen(makeEffectorRotateToIntake())
                                                          .andThen(makeElevatorGoToIntake())
                                                          .andThen(makeElevatorRotateToIntake())
                                                          .andThen(makeEffectorFastForward()) // Go fast
                                                          .andThen(makeEffectorWaitUntilCoralPresent())  // Wait until coral present
                                                          .andThen(makeEffectorSlowReverse()) // Go slow
                                                          .andThen(makeEffectorWaitUntilCoralMissing())
                                                          .andThen(makeEffectorZeroEncoder())
                                                          .andThen(makeEffectorCenterCoral())
                                                          .andThen(makeSetNewPoseState(PoseState.INTAKE))
                                                          .finallyDo((interrupted)->{if (interrupted) makeEffectorStop().schedule();});

                                                    
    private final Command robotToL4 = makeRobotSafe()
        .andThen(makeEffectorRotateToL4())
        .andThen(makeEffectorMoveToL4())
        .andThen(makeElevatorRotateToPreL4())
        .andThen(makeElevatorGoToL4())
        .andThen(makeElevatorRotateToL4())
        .andThen(makeSetNewPoseState(PoseState.L4));

    private final Command robotToL3 = makeRobotSafe()
        .andThen(makeEffectorRotateToL3())
        .andThen(makeElevatorRotateToPreL3())
        .andThen(makeElevatorGoToL3())
        .andThen(makeElevatorRotateToL3())
        .andThen(makeSetNewPoseState(PoseState.L3));

    private final Command robotToL2 = makeRobotSafe()
        .andThen(makeEffectorRotateToL2())
        .andThen(makeElevatorRotateToPreL2())
        .andThen(makeElevatorGoToL2())
        .andThen(makeElevatorRotateToL2())
        .andThen(makeSetNewPoseState(PoseState.L2));
                                                      
        private final Command robotToL1 = makeRobotSafe()
        .andThen(makeEffectorRotateToL1())
        .andThen(makeElevatorRotateToPreL1())
        .andThen(makeElevatorGoToL1())
        .andThen(makeElevatorRotateToL1())
        .andThen(makeSetNewPoseState(PoseState.L1));                                                      

        private final Command robotToLift = makeRobotSafe()
        .andThen(makeEffectorRotateToLift())
        .andThen(makeElevatorGoToLift())
        .andThen(makeElevatorRotateToLift())
        .andThen(makeSetNewPoseState(PoseState.LIFT));                                                      

    private final Command robotDrive = makeRobotSafe();
    
    private final Command robotDepositCoral = new ConditionalCommand(makeEffectorDepositCoral(), say("sideways").andThen(makeEffectorSidewaysDepositCoral()), ()->{return previousPoseState!=PoseState.L1;});
    
    private final Command kickerGoToPosition = kickerSubsystem.makeGoToPositionCmd(() -> controllerDriver.getRightTriggerAxis());

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 2% deadband
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
                drive.withVelocityX(-controllerDriver.getLeftY() * Math.abs(controllerDriver.getLeftY()) * MaxSpeed * (controllerDriver.rightBumper().getAsBoolean()?0.2:1.0)) // Drive forward with negative Y (forward)
                    .withVelocityY(-controllerDriver.getLeftX() * Math.abs(controllerDriver.getLeftX()) * MaxSpeed * (controllerDriver.rightBumper().getAsBoolean()?0.2:1.0)) // Drive left with negative X (left)
                    .withRotationalRate(-controllerDriver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        kickerSubsystem.setDefaultCommand(kickerGoToPosition);
        
        ////joystick.povUp().onTrue(effectorRotateToLevel4.andThen(elevatorGoToL4Angle));
        ////joystick.povRight().onTrue(elevatorGoToZero.andThen(effectorRotateToLevel3));
        ////joystick.povDown().onTrue(effectorRotateToLevel2);
        ////joystick.povLeft().onTrue(effectorRotateToLevel1);
        //controllerDriver.povUp()  .onTrue(elevatorGoToL4);
        //controllerDriver.povDown().onTrue(elevatorGoToSafe);
        controllerDriver.x()      .onTrue(robotDrive);
        controllerDriver.a()      .onTrue(robotDepositCoral);
        controllerDriver.b()      .whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-controllerDriver.getLeftY(), -controllerDriver.getLeftX()))
        ));


        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        controllerDriver.back().and(controllerDriver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        controllerDriver.back().and(controllerDriver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        controllerDriver.start().and(controllerDriver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        controllerDriver.start().and(controllerDriver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        controllerDriver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        controllerOperator.leftBumper().onTrue(robotIntake);
        controllerOperator.y().onTrue(robotToL4);
        controllerOperator.x().onTrue(robotToL3);
        controllerOperator.b().onTrue(robotToL3);
        controllerOperator.a().onTrue(robotToL2);
        controllerOperator.rightBumper().onTrue(robotToL1);
        controllerOperator.back().onTrue(robotToLift);

        drivetrain.registerTelemetry(logger::telemeterize);


    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("path1");
    }
}
