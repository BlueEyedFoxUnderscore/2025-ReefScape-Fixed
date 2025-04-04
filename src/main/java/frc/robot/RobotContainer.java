// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.EffectorEatSubsystem;
import frc.robot.subsystems.EffectorLookSubsystem;
import frc.robot.subsystems.ArmReachSubsystem;
import frc.robot.subsystems.ArmTiltSubsystem;
import frc.robot.subsystems.KickerSubsystem;

public class RobotContainer {

    private final PWM pwm0 = new PWM(0);

    private enum PoseState {
        L1, L2, L3, L4, LIFT, NONE
    };

    private static PoseState previousPoseState = PoseState.NONE;

    private EffectorLookSubsystem effectorSubsystem = new EffectorLookSubsystem();
    private EffectorEatSubsystem effectorEatSubsystem = new EffectorEatSubsystem();
    private KickerSubsystem kickerSubsystem = new KickerSubsystem();
    private ArmReachSubsystem armReachSubsystem = new ArmReachSubsystem();
    private ArmTiltSubsystem armTiltSubsystem = new ArmTiltSubsystem();
    private CommandXboxController controllerDriver = new CommandXboxController(0);
    private CommandXboxController controllerOperator = new CommandXboxController(1);

    private Command makeSetNewPoseState(PoseState newState) {
        return Commands.runOnce(() -> previousPoseState = newState);
    }

    private Command say(String msg) {
        return Commands.runOnce(() -> System.out.println(msg));
    }

    private Command makeRobotNOP() {
        return Commands.runOnce(() -> {
        });
    }

    private Command useSwerveBrake() {
        return Commands.runEnd(() -> {enableSwerveBrake=true;}, () -> {enableSwerveBrake=false;});
    }
    private Command makeEffectorDepositCoral() {
        return effectorEatSubsystem.makeZeroEncoder().andThen(
                effectorEatSubsystem.makeEatTo(20.0));
    }

    private Command makeEffectorSidewaysDepositCoral() {
        return effectorEatSubsystem.makeZeroEncoder().andThen(
                effectorEatSubsystem.makeEatAt(15, 60),
                Commands.waitSeconds(2),
                effectorEatSubsystem.makeEatAt(0.0, 0.0));
    }

    private Command makeEmergencyExtend() {
        return Commands.runOnce(() -> {
            switch (previousPoseState) {
                case L1:
                    makeL1First_Step2_ArmLength()       .schedule();
                    break;
                case L2:
                    makeL2_Step3_Preposition_ArmLength().schedule();
                    break;
                case L3:
                    makeL3_Step3_Preposition_ArmLength().schedule();
                    break;
                case L4:
                    makeL4_reach().schedule();
                    break;
                default:break;
            }
        });
    }

    private final Command robotEmergencyExtend = makeEmergencyExtend();

    private Command makeRobotSafe() {
        return makeSetNewPoseState(PoseState.NONE).andThen(armTiltSubsystem.makeSafe());
    };

    private final Command stopIntake = effectorEatSubsystem.makeEatAt(0.0, 0.0);

    private final Command robotIntake =
            makeRobotSafe().andThen(
            //effectorSubsystem.makeLook(+53.0), 58 snake hits
            effectorSubsystem.makeLook(+55.0),
            //armReachSubsystem.makeReach(1.0).alongWith(armTiltSubsystem.makeTilt(-13.5), effectorEatSubsystem.makeEatAt(60.5, 60.5)),
            armReachSubsystem.makeReach(2.25).alongWith(armTiltSubsystem.makeTilt(-15), effectorEatSubsystem.makeEatAt(60.5, 60.5)),
            effectorEatSubsystem.makeWaitUntilCoralPresent(),
            //effectorEatSubsystem.makeEatAt(-10.0, -10.0),
            //makeIntake_WaitUntilCoralMissing(),
            effectorEatSubsystem.makeZeroForIntake(),
            effectorEatSubsystem.makeEatTo(6.0))
            .finallyDo((interrupted) -> {if (interrupted) stopIntake.schedule();});

    private final Command robotReIntake =
            effectorEatSubsystem.makeEatAt(-5, -5).andThen(
            Commands.waitUntil(() -> {return !controllerOperator.leftTrigger().getAsBoolean();}),
            effectorEatSubsystem.makeEatAt(-10.0, -10.0).andThen(
            effectorEatSubsystem.makeWaitUntilCoralMissing(),
            effectorEatSubsystem.makeZeroEncoder(),
            effectorEatSubsystem.makeEatTo(6.0))
            );

    private final Command robotRemoveHighAlgaeAuto = 
        useSwerveBrake().raceWith(
            makeRobotSafe().andThen(
                effectorSubsystem.makeLook(-15).alongWith(
                armTiltSubsystem.makeTilt(12),
                armReachSubsystem.makeReach(26.6),
                effectorEatSubsystem.makeEatAt(-60, -60)),
            //Grab
            armTiltSubsystem.makeTilt(27),
            Commands.waitSeconds(.5),
            //Remove
                armTiltSubsystem.makeTilt(12).alongWith(
                armReachSubsystem.makeReach(28)),
            //Release
            armReachSubsystem.makeReach(0),
            armTiltSubsystem.makeTilt(18),
            effectorEatSubsystem.makeZeroEncoder(),
            effectorEatSubsystem.makeEatTo(3.0),
            Commands.waitSeconds(.8),
                effectorSubsystem.makeLook(-55).alongWith(
                armTiltSubsystem.makeTilt(50))        
            )
        );


    private final Command robotRemoveHighAlgae =
        useSwerveBrake().raceWith(
            makeRobotSafe().andThen(
                effectorSubsystem.makeLook(-15).alongWith(
                armTiltSubsystem.makeTilt(12),
                armReachSubsystem.makeReach(26.6),
                effectorEatSubsystem.makeEatAt(-60, -60)),
            //Grab
            armTiltSubsystem.makeTilt(27),
            Commands.waitSeconds(.5),
            //Remove
                armTiltSubsystem.makeTilt(12).alongWith(
                armReachSubsystem.makeReach(28)),
            //Release
            armReachSubsystem.makeReach(0),
            armTiltSubsystem.makeTilt(18),
            effectorEatSubsystem.makeZeroEncoder(),
            effectorEatSubsystem.makeEatTo(3.0))
        );

    private final Command robotRemoveLowAlgaeAuto = 
        useSwerveBrake().raceWith(
            makeRobotSafe().andThen(
            armReachSubsystem.makeReach(0),
                effectorSubsystem.makeLook(-50).alongWith( //-80
                armTiltSubsystem.makeTilt(40),
                effectorEatSubsystem.makeEatAt(-60, -60)),
            armReachSubsystem.makeReach(9.5),
            Commands.waitSeconds(.5),
            armReachSubsystem.makeReach(10.5).alongWith(
                armTiltSubsystem.makeTilt(12)),
            armReachSubsystem.makeReach(0),
            armTiltSubsystem.makeTilt(18),
            effectorEatSubsystem.makeZeroEncoder(),
            effectorEatSubsystem.makeEatTo(5.0),
            Commands.waitSeconds(.8),
                effectorSubsystem.makeLook(-55).alongWith(
                armTiltSubsystem.makeTilt(50))        
            )
        );
    

    private final Command robotRemoveLowAlgae =
        useSwerveBrake().raceWith(
            makeRobotSafe().andThen(
            armReachSubsystem.makeReach(0),
                effectorSubsystem.makeLook(-50).alongWith( //-80
                armTiltSubsystem.makeTilt(40),
                effectorEatSubsystem.makeEatAt(-60, -60)),
            armReachSubsystem.makeReach(9.5),
            Commands.waitSeconds(.5),
            armReachSubsystem.makeReach(10.5).alongWith(
                armTiltSubsystem.makeTilt(12)),
            armReachSubsystem.makeReach(0),
            armTiltSubsystem.makeTilt(18),
            effectorEatSubsystem.makeZeroEncoder(),
            effectorEatSubsystem.makeEatTo(3.0)
        ));
    

            private final Command makeL4_reach()                  { return armReachSubsystem.makeReach(46.5);}
    private final Command robotToL4 =
        say("ROBOT TO L4").andThen(
        makeRobotSafe(),
        effectorSubsystem.makeLook(0.0).andThen(
        effectorEatSubsystem.makeEatTo(3.0),
            effectorSubsystem.makeLook(+40.0)).alongWith(
            armTiltSubsystem.makeTilt(12),
            makeL4_reach()),
        armTiltSubsystem.makeTilt(17),
        makeSetNewPoseState(PoseState.L4),
        say("----------------------------------"));


    private final Command makeL3_Step3_Preposition_ArmLength() {
        return armReachSubsystem.makeReach(23);
    }

    private final Command robotToL3 =
        say("ROBOT TO L3").andThen(
        makeRobotSafe(),
            effectorSubsystem.makeLook(+8.0).alongWith(
            armTiltSubsystem.makeTilt(5),
            armReachSubsystem.makeReach(23)
        ),
        armTiltSubsystem.makeTilt(23),
        makeSetNewPoseState(PoseState.L3),
        say("----------------------------------"));
    
    private final Command makeL2_Step3_Preposition_ArmLength() {
        return armReachSubsystem.makeReach(9.75);
    }

    private final Command robotToL2 =
    say("ROBOT TO L2").andThen(
    makeRobotSafe(),
        effectorSubsystem.makeLook(-7.0).alongWith(
        armTiltSubsystem.makeTilt(12),
        armReachSubsystem.makeReach(9)
    ),
    armTiltSubsystem.makeTilt(35),
    makeSetNewPoseState(PoseState.L2),
    say("----------------------------------"));

    private final Command robotToCaptureAlgae =
    say("ROBOT TO FIRST L1").andThen(
    makeRobotSafe(),
    armReachSubsystem.makeReach(0),
    effectorSubsystem.makeLook(-55),
    armTiltSubsystem.makeTilt(50),
    say("----------------------------------"));


    private final Command robotToL1First =
    say("ROBOT TO FIRST L1").andThen(
    makeRobotSafe(),
        effectorSubsystem.makeLook(-55).alongWith(
        armTiltSubsystem.makeTilt(23),
        armReachSubsystem.makeReach(0)
    ),
    armTiltSubsystem.makeTilt(57),
    makeSetNewPoseState(PoseState.L1),
    say("----------------------------------"));

    private final Command robotToL1Second =
    say("ROBOT TO FIRST L1").andThen(
    makeRobotSafe(),
        effectorSubsystem.makeLook(-55).alongWith(
        armTiltSubsystem.makeTilt(12),
        armReachSubsystem.makeReach(1.8)
    ),
    armTiltSubsystem.makeTilt(47),
    makeSetNewPoseState(PoseState.L1),
    say("----------------------------------"));



    private final Command makeL1First_Step2_ArmLength() {
        return armReachSubsystem.makeReach(0);
    }

    private final Command robotToLift = makeRobotSafe().andThen(
        effectorSubsystem.makeLook(120),
        armReachSubsystem.makeReach(0),
        armTiltSubsystem.makeTilt(68.5),
        makeSetNewPoseState(PoseState.LIFT),
        kickerSubsystem.makeKickerPrepareLift()
        );               
    
    private final Command robotLift = new ConditionalCommand (
        say("Lifting").andThen(
        kickerSubsystem.makeKickerLift(controllerOperator::getLeftTriggerAxis)),
        say("Not Lifting"),
        ()->{return previousPoseState==PoseState.LIFT;}
    );

    private final Command robotAutoKick = Commands.race(
        kickerSubsystem.makeKickerAutoKick(),
        Commands.waitSeconds(1) // WAS 3
    );



    private final Command robotReleaseBrake = armTiltSubsystem.makeReleaseBrakeCmd();

    private final Command robotDrive = 
        makeRobotSafe().andThen(
            armTiltSubsystem.makeTilt(0).alongWith(armReachSubsystem.makeReach(0), effectorSubsystem.makeLook(0))
        );

    private final Command robotDepositCoral = new ConditionalCommand(makeEffectorDepositCoral(),
            say("sideways").andThen(makeEffectorSidewaysDepositCoral()), () -> {
                return previousPoseState != PoseState.L1;
            });

    private final Command kickerGoToPosition = kickerSubsystem
            .makeGoToPositionCmd(() -> controllerDriver.getRightTriggerAxis());

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.0).withRotationalDeadband(MaxAngularRate * 0.0) // Add a 2% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive `tors

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
        .withDeadband(MaxSpeed * 0.0).withRotationalDeadband(MaxAngularRate * 0.0) // Add a 2% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive `tors

    
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain;

    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    public RobotContainer() {
        drivetrain = TunerConstants.createDrivetrain();

        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        {
            robotDepositCoral.          setName("depositCoral");
            robotDrive.                 setName("drive");
            robotIntake.                setName("intake");
            robotReleaseBrake.          setName("releaseBrake");
            robotRemoveHighAlgaeAuto.   setName("removeHighAlgae");
            robotRemoveLowAlgaeAuto.    setName("removeLowAlgae");
            robotToCaptureAlgae.        setName("toCaptureAlgae");
            robotToL1First.             setName("toL1");
            robotToL2.                  setName("toL2");
            robotToL3.                  setName("toL3");
            robotToL4.                  setName("toL4");
            robotToLift.                setName("toLift");
            robotAutoKick.              setName("autoKick");
        }

        register(
            robotDepositCoral,
            robotDrive,
            robotIntake,
            robotReleaseBrake,
            robotRemoveHighAlgaeAuto,
            robotRemoveLowAlgaeAuto,
            robotToL1First,
            robotToL2,
            robotToL3,
            robotToL4,
            robotToLift,
            robotAutoKick
        );

        AutoBuilder.configure(
                () -> {return drivetrain.getState().Pose;}, // Supplier<Pose2d> current robot pose
                drivetrain::resetPose, // Consumer<Pose2d> reset robot pose
                () -> {return drivetrain.getState().Speeds;}, // Supplier<ChassisSpeeds> robot-relative speeds
                (speeds, ffs) -> {
                    drivetrain.setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(speeds)
                            .withWheelForceFeedforwardsX(ffs.robotRelativeForcesXNewtons())
                            .withWheelForceFeedforwardsY(ffs.robotRelativeForcesYNewtons()));
                        }, // Consumer<ChassisSpeeds>
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
                                                // holonomic drive trains
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ),
                config, // PathFollowerConfig path follower configuration
                () -> {
                    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
                }, // BooleanSupplier is robot on red alliance
                drivetrain // Subsystem requirement
        );
        initializeSubsystems();
        configureBindings();

        m_chooser.setDefaultOption("Mid (L4 A PROC STATION)", "Mid (L4 A PROC STATION)");
        m_chooser.addOption("Mid (L1)", "Mid (L1)");
        m_chooser.addOption("Mid (L4 A)", "Mid (L4 A)");
        m_chooser.addOption("Red (L4 A PROC STATION)", "Blue (L4 A PROC STATION)");
        m_chooser.addOption("Blue (L4 A PROC STATION)", "Blue (L4 A PROC STATION)");
        m_chooser.addOption("SCORE to STATION test", "Copy of RL4 LA SCORE"); 
        m_chooser.addOption("DO NOT USE", "$RL4 LA PROC STAT NEL4 SWRL4 SWA PROC STAT");
        SmartDashboard.putData("Auto choices", m_chooser);
        
    }

    private void register(Command... commands){
        for (Command command: commands) NamedCommands.registerCommand(command.getName(), command);
    }  



    private void initializeSubsystems() {
        pwm0.setAlwaysHighMode();
        effectorSubsystem.init();
        effectorEatSubsystem.init();
        kickerSubsystem.init();
        armReachSubsystem.init();
        armTiltSubsystem.init();
    }

    static boolean enableSwerveBrake = false;
    private void configureBindings() {

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> enableSwerveBrake?brake:
                controllerDriver.leftTrigger().getAsBoolean()? 
                    robotCentricDrive
                        .withVelocityX(-controllerDriver.getLeftY() * Math.abs(controllerDriver.getLeftY()) * MaxSpeed
                                * (controllerDriver.rightBumper().getAsBoolean() ? 0.2 : 1.0)) // Drive forward with
                                                                                               // negative Y (forward)
                        .withVelocityY(-controllerDriver.getLeftX() * Math.abs(controllerDriver.getLeftX()) * MaxSpeed
                                * (controllerDriver.rightBumper().getAsBoolean() ? 0.2 : 1.0)) // Drive left with
                                                                                               // negative X (left)
                        .withRotationalRate(-controllerDriver.getRightX() * MaxAngularRate 
                                * (controllerDriver.rightBumper().getAsBoolean() ? 0.2 : 1.0)) // Drive counterclockwise
                                                                                            // with negative X (left)
                    :
                    fieldCentricDrive
                        .withVelocityX(-controllerDriver.getLeftY() * Math.abs(controllerDriver.getLeftY()) * MaxSpeed
                                * (controllerDriver.rightBumper().getAsBoolean() ? 0.2 : 1.0)) // Drive forward with
                                                                                               // negative Y (forward)
                        .withVelocityY(-controllerDriver.getLeftX() * Math.abs(controllerDriver.getLeftX()) * MaxSpeed
                                * (controllerDriver.rightBumper().getAsBoolean() ? 0.2 : 1.0)) // Drive left with
                                                                                               // negative X (left)
                        .withRotationalRate(-controllerDriver.getRightX() * MaxAngularRate 
                                * (controllerDriver.rightBumper().getAsBoolean() ? 0.2 : 1.0)) // Drive counterclockwise
                                                                                            // with negative X (left)
                ));

        controllerDriver.x().onTrue(robotDrive);
        controllerDriver.a().onTrue(robotDepositCoral);
        //controllerDriver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        controllerDriver.b().whileTrue(drivetrain.applyRequest(() -> point
                .withModuleDirection(new Rotation2d(-controllerDriver.getLeftY(), -controllerDriver.getLeftX()))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        /*
        //controllerDriver.back().and(controllerDriver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //controllerDriver.back().and(controllerDriver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        //controllerDriver.start().and(controllerDriver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //controllerDriver.start().and(controllerDriver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        */
        // reset the field-centric heading on left bumper press
        controllerDriver.povDown().onTrue(armTiltSubsystem.makeTrimDownCmd());
        controllerDriver.povUp().onTrue(armTiltSubsystem.makeTrimUpCmd());

        controllerDriver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        controllerOperator.leftBumper().onTrue(robotIntake);
        controllerOperator.y().onTrue(robotToL4);
        controllerOperator.x().onTrue(robotToL3);
        controllerOperator.b().onTrue(robotToL3);
        controllerOperator.a().onTrue(robotToL2);
        controllerOperator.rightBumper().onTrue(robotToL1First);
        controllerOperator.rightTrigger().onTrue(robotEmergencyExtend);
        controllerOperator.rightStick().onTrue(robotToL1Second);
        controllerOperator.povUp().onTrue(robotRemoveHighAlgae);
        controllerOperator.povDown().onTrue(robotRemoveLowAlgae);
        controllerOperator.start().toggleOnTrue(robotReleaseBrake);

        kickerSubsystem.setDefaultCommand(kickerGoToPosition);
        controllerOperator.back().onTrue(robotToLift);
        controllerOperator.leftStick().onTrue(robotLift);
        controllerOperator.leftTrigger().onTrue(robotReIntake);
        controllerOperator.povRight().onTrue(robotToCaptureAlgae);


        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto(m_chooser.getSelected());
    }
}
