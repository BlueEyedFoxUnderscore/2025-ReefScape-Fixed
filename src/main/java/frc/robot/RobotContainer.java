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

    private final Command makeSetNewPoseState(PoseState newState)               { return Commands.runOnce(() -> previousPoseState = newState); }
    private final Command say(String msg)                                       { return Commands.runOnce(() -> System.out.println(msg)); }
    private final Command makeRobotNOP ()                                       { return Commands.runOnce(() -> {}); } 


    private final Command makeEffectorDepositCoral()                            { return 
                                                                                    effectorSubsystem.makeZeroEncoder().andThen(
                                                                                    effectorSubsystem.makeMoveTo(20.0)); } 
    private final Command makeEffectorSidewaysDepositCoral()                    { return
                                                                                    effectorSubsystem.makeZeroEncoder().andThen(
                                                                                    effectorSubsystem.makeSetSpeed(15, 25),
                                                                                    Commands.waitSeconds(2),
                                                                                    effectorSubsystem.makeSetSpeed(0.0, 0.0)); }


    private final Command makeSafe_Step1_ArmLength()                            { return elevatorSubsystem.makeGoToPositionCmd(0);}
    private final Command makeSafe_Step2_ArmAngle()                             { return elevatorSubsystem.makeGoToAngleCmd(0);}
    private final Command makeSafe_Step3_EffectorAngle()                        { return effectorSubsystem.makeRotateTo(+ 0.0);}
    private final Command makeRobotSafe() { return
        makeL4_Step3_Preposition_ArmAngle().andThen(
        makeSafe_Step1_ArmLength(),
        makeSafe_Step2_ArmAngle(),
        makeSafe_Step3_EffectorAngle(),
        makeSetNewPoseState(PoseState.SAFE));
    };

    private final Command makeIntake_CenterCoral()                              { return effectorSubsystem.makeMoveTo(6.0);} 
    private final Command makeIntake_ZeroEncoder()                              { return effectorSubsystem.makeZeroEncoder();}
    private final Command makeIntake_WaitUntilCoralPresent()                    { return effectorSubsystem.makeWaitUntilCoralPresent();} 
    private final Command makeIntake_WaitUntilCoralMissing()                    { return effectorSubsystem.makeWaitUntilCoralMissing();} 
    private final Command makeIntake_Step1_EffectorAngle()                      { return effectorSubsystem.makeRotateTo(+60.0);}
    private final Command makeIntake_Step2_ArmLength()                          { return elevatorSubsystem.makeGoToPositionCmd(2.66);}
    private final Command makeIntake_Step3_ArmAngle()                           { return elevatorSubsystem.makeGoToAngleCmd(-18.0);} // -14
    private final Command makeIntake_Step4_EffectorRapidIntake()                { return effectorSubsystem.makeSetSpeed(60.5, 60.5);} // Top speed = 60.5
    private final Command makeIntake_Step5_EffectorSlowReverse()                { return effectorSubsystem.makeSetSpeed(-10.0, -10.0);} 
    private final Command makeIntake_Stop()                                     { return effectorSubsystem.makeSetSpeed(0.0, 0.0);}
    private final Command robotIntake = 
        makeRobotSafe().andThen(
        makeIntake_Step1_EffectorAngle(),
        makeSetNewPoseState(PoseState.INTAKE),
        makeIntake_Step2_ArmLength(),
        makeIntake_Step3_ArmAngle(),
        makeIntake_Step4_EffectorRapidIntake(),
        makeIntake_WaitUntilCoralPresent(),
        makeIntake_Step5_EffectorSlowReverse(),
        makeIntake_WaitUntilCoralMissing(),
        makeIntake_ZeroEncoder(),
        makeIntake_CenterCoral())
        .finallyDo((interrupted)->{if (interrupted) makeIntake_Stop().schedule();});


    private final Command makeRemoveEitherAlgae_Deposit_ArmLength()             { return elevatorSubsystem.makeGoToPositionCmd(0);}
    private final Command makeRemoveEitherAlgae_Deposit_ArmAngle()              { return elevatorSubsystem.makeGoToAngleCmd(18);}
    private final Command makeRemoveHighAlgae_Step1_PreRemove_ArmAngle()        { return elevatorSubsystem.makeGoToAngleCmd(12);} //12
    private final Command makeRemoveHighAlgae_Step2_PreRemove_EffectorAngle()   { return effectorSubsystem.makeRotateTo(0.0);} 
    private final Command makeRemoveHighAlgae_Step3_PreRemove_ArmLength()       { return elevatorSubsystem.makeGoToPositionCmd(26.6);}
    private final Command makeRemoveHighAlgae_Step4_ReadyRemove_ArmAngle()      { return elevatorSubsystem.makeGoToAngleCmd(22);}
    private final Command makeRemoveHighAlgae_Step5_Remove_ArmAngle()           { return elevatorSubsystem.makeGoToAngleCmd(12);}
    private final Command robotRemoveHighAlgae = new ConditionalCommand(
        makeRemoveHighAlgae_Step1_PreRemove_ArmAngle().andThen(
        makeRemoveHighAlgae_Step2_PreRemove_EffectorAngle(),
        makeRemoveHighAlgae_Step3_PreRemove_ArmLength(),
        makeRemoveHighAlgae_Step4_ReadyRemove_ArmAngle(),
        makeSetNewPoseState(PoseState.L3),
        makeRemoveEitherAlgae_EffectorGrab(),
        Commands.waitSeconds(.8),
        makeRemoveHighAlgae_Step5_Remove_ArmAngle(),
        makeRemoveEitherAlgae_Deposit_ArmLength(),
        makeRemoveEitherAlgae_Deposit_ArmAngle(),
        makeIntake_ZeroEncoder(),
        makeRemoveEitherAlgae_Deposit_EffectorRelease(),
        Commands.waitSeconds(.5),
        makeL1_Step3_ArmAngle(),
        makeSetNewPoseState(PoseState.L1)), makeRobotNOP(), ()->previousPoseState==PoseState.L4);   

    private final Command makeRemoveEitherAlgae_EffectorGrab()                  { return effectorSubsystem.makeSetSpeed(-50, -50);} // Top speed = 60.5
    private final Command makeRemoveEitherAlgae_Deposit_EffectorRelease()       { return effectorSubsystem.makeMoveTo(3.0);} 
    private final Command makeRemoveLowAlgae_Step1_BackUpFromL4_ArmAngle()      { return elevatorSubsystem.makeGoToAngleCmd(12);}
    private final Command makeRemoveLowAlgae_Step2_PreRemove_EffectorAngle()    { return effectorSubsystem.makeRotateTo(-80);}
    private final Command makeRemoveLowAlgae_Step3_PreRemove_ArmLength()        { return elevatorSubsystem.makeGoToPositionCmd(0);}
    private final Command makeRemoveLowAlgae_Step4_PreRemove_ArmAngle()         { return elevatorSubsystem.makeGoToAngleCmd(40);} //45
    private final Command makeRemoveLowAlgae_Step5_Grab_ArmLength()             { return elevatorSubsystem.makeGoToPositionCmd(9);}
    private final Command makeRemoveLowAlgae_Step6_PartialRemove_ArmLength()    { return elevatorSubsystem.makeGoToPositionCmd(7);}
    private final Command makeRemoveLowAlgae_Step7_PartialRemove_ArmAngle()     { return elevatorSubsystem.makeSlowGoToAngleCmd(35);}
    private final Command makeRemoveLowAlgae_Step7_2_PreRemove_EffectorAngle()  { return effectorSubsystem.makeRotateTo(-70);}
    private final Command makeRemoveLowAlgae_Step8_PartialRemove2_ArmLength()    { return elevatorSubsystem.makeGoToPositionCmd(6.25);}
    private final Command makeRemoveLowAlgae_Step9_Remove_ArmAngle()            { return elevatorSubsystem.makeSlowGoToAngleCmd(12);}
    private final Command makeRemoveLowAlgae_Step10_PrepareDrop_EffectorAngle()  { return effectorSubsystem.makeRotateTo(30);}
    private final Command robotRemoveLowAlgae = new ConditionalCommand(
        makeRemoveLowAlgae_Step1_BackUpFromL4_ArmAngle().andThen(
        makeRemoveLowAlgae_Step2_PreRemove_EffectorAngle(),
        makeRemoveLowAlgae_Step3_PreRemove_ArmLength(),
        makeSetNewPoseState(PoseState.L3),
        makeRemoveLowAlgae_Step4_PreRemove_ArmAngle(),
        makeRemoveEitherAlgae_EffectorGrab(),
        makeRemoveLowAlgae_Step5_Grab_ArmLength(),
        Commands.waitSeconds(.8),
        makeRemoveLowAlgae_Step6_PartialRemove_ArmLength(),
        makeRemoveLowAlgae_Step7_PartialRemove_ArmAngle(),
        makeRemoveLowAlgae_Step7_2_PreRemove_EffectorAngle(),
        makeRemoveLowAlgae_Step8_PartialRemove2_ArmLength(),
        makeRemoveLowAlgae_Step9_Remove_ArmAngle(),
        makeRemoveLowAlgae_Step10_PrepareDrop_EffectorAngle(),
        makeRemoveEitherAlgae_Deposit_ArmLength(),
        makeRemoveEitherAlgae_Deposit_ArmAngle(),
        makeIntake_ZeroEncoder(),
        makeRemoveEitherAlgae_Deposit_EffectorRelease(),
        Commands.waitSeconds(.5),
        makeL1_Step3_ArmAngle(),
        makeSetNewPoseState(PoseState.L1)), makeRobotNOP(), ()->previousPoseState==PoseState.L4);

    private final Command makeL4_Step1_Preposition_EffectorAngle()              { return effectorSubsystem.makeRotateTo(0.0);} 
    private final Command makeL4_Step2_Preposition_Coral()                      { return effectorSubsystem.makeMoveTo(3.0);} 
    private final Command makeL4_Step3_Preposition_ArmAngle()                   { return elevatorSubsystem.makeGoToAngleCmd(12);}
    private final Command makeL4_Step4_Preposition_ArmLength()                  { return elevatorSubsystem.makeGoToPositionCmd(46.5);}
    private final Command makeL4_Step5_Position_EffectorAngle()                 { return effectorSubsystem.makeRotateTo(+40.0);} 
    private final Command makeL4_Step6_Position_ArmAngle()                      { return elevatorSubsystem.makeGoToAngleCmd(17);}
    private final Command robotToL4 =
        makeRobotSafe().andThen(
        makeL4_Step1_Preposition_EffectorAngle(),
        makeL4_Step2_Preposition_Coral(),
        makeL4_Step3_Preposition_ArmAngle(),
        makeL4_Step4_Preposition_ArmLength(),
        makeSetNewPoseState(PoseState.L4),
        makeL4_Step5_Position_EffectorAngle(),
        makeL4_Step6_Position_ArmAngle());

    private final Command makeL3_Step1_Preposition_EffectorAngle()              { return effectorSubsystem.makeRotateTo(+8.0);} 
    private final Command makeL3_Step2_Preposition_ArmAngle()                   { return elevatorSubsystem.makeGoToAngleCmd(19);}
    private final Command makeL3_Step3_Preposition_ArmLength()                  { return elevatorSubsystem.makeGoToPositionCmd(23);}
    private final Command makeL3_Step4_Position_ArmAngle()                      { return elevatorSubsystem.makeGoToAngleCmd(23);}
    private final Command robotToL3 =
        makeRobotSafe().andThen(
        makeL3_Step1_Preposition_EffectorAngle(),
        makeL3_Step2_Preposition_ArmAngle(),
        makeL3_Step3_Preposition_ArmLength(),
        makeSetNewPoseState(PoseState.L3),
        makeL3_Step4_Position_ArmAngle());

    private final Command makeL2_Step1_Preposition_EffectorAngle()              { return effectorSubsystem.makeRotateTo(0);} 
    private final Command makeL2_Step2_Preposition_ArmAngle()                   { return elevatorSubsystem.makeGoToAngleCmd(30);}
    private final Command makeL2_Step4_Position_ArmAngle()                      { return elevatorSubsystem.makeGoToAngleCmd(35);}
    private final Command makeL2_Step3_Preposition_ArmLength()                  { return elevatorSubsystem.makeGoToPositionCmd(9.75);}
    private final Command robotToL2 = makeRobotSafe().andThen(
        makeL2_Step1_Preposition_EffectorAngle(),
        makeL2_Step2_Preposition_ArmAngle(),
        makeL2_Step3_Preposition_ArmLength(),
        makeSetNewPoseState(PoseState.L2),
        makeL2_Step4_Position_ArmAngle());
                                                      
    private final Command makeL1_Step1_EffectorAngle()                          { return effectorSubsystem.makeRotateTo(-59);} 
    private final Command makeL1_Step2_ArmLength()                              { return elevatorSubsystem.makeGoToPositionCmd(1.8);}
    private final Command makeL1_Step3_ArmAngle()                               { return elevatorSubsystem.makeGoToAngleCmd(47);}
    private final Command robotToL1 = makeRobotSafe()
        .andThen(makeL1_Step1_EffectorAngle())
        .andThen(makeL1_Step2_ArmLength())
        .andThen(makeL1_Step3_ArmAngle())
        .andThen(makeSetNewPoseState(PoseState.L1));                                                      

    private final Command makeLiftPosition_Step1_EffectorAngle()                { return effectorSubsystem.makeRotateTo(90);} 
    private final Command makeLiftPosition_Step2_ArmLength()                    { return elevatorSubsystem.makeGoToPositionCmd(0);}
    private final Command makeLiftPosition_Step3_ArmAngle()                     { return elevatorSubsystem.makeGoToAngleCmd(70);}
    private final Command robotToLift = makeRobotSafe()
        .andThen(makeLiftPosition_Step1_EffectorAngle())
        .andThen(makeLiftPosition_Step2_ArmLength())
        .andThen(makeLiftPosition_Step3_ArmAngle())
        .andThen(makeSetNewPoseState(PoseState.LIFT));                

    private final Command robotReleaseBrake = elevatorSubsystem.makeReleaseElevatorBrakeCmd();
       
    private final Command robotDrive = makeRobotSafe();

    private final Command robotDepositCoral = new ConditionalCommand(makeEffectorDepositCoral(), say("sideways").andThen(makeEffectorSidewaysDepositCoral()), ()->{return previousPoseState!=PoseState.L1;});
    
    private final Command kickerGoToPosition = kickerSubsystem.makeGoToPositionCmd(() -> controllerDriver.getRightTriggerAxis());

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.0).withRotationalDeadband(MaxAngularRate * 0.0) // Add a 2% deadband
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
                drive.withVelocityX(-controllerDriver.getLeftY() * Math.abs(controllerDriver.getLeftY()) *  MaxSpeed * (controllerDriver.rightBumper().getAsBoolean()?0.2:1.0)) // Drive forward with negative Y (forward)
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
        controllerOperator.povUp().onTrue(robotRemoveHighAlgae);
        controllerOperator.povDown().onTrue(robotRemoveLowAlgae);
        controllerOperator.start().toggleOnTrue(robotReleaseBrake);

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
