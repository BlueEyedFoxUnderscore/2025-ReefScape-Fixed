package frc.robot.subsystems;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase{

    private static final TalonFX leaderTilt = new TalonFX(51, "Default Name");
    private static final TalonFX followerTilt = new TalonFX(52, "Default Name");
    private static final Follower followLeaderTiltRequest = new Follower(51, true);

    private static final TalonFXConfiguration followerTiltConfig = new TalonFXConfiguration();
    private static final TalonFXConfiguration leaderTiltConfig = new TalonFXConfiguration();
    private static final MotionMagicVoltage requestMotionMagicVoltage = new MotionMagicVoltage(0);
    private static final CoastOut requestCoast = new CoastOut();
    private static final NeutralOut requestNeutral = new NeutralOut();

    private static final CANdi lampryCANdi = new CANdi(53, "Default Name");

    private static final double minAllowedAngle = -20.0; 
    private static final double maxAllowedAngle = 75.0;
    private static final double minAllowedPosition = 0.0;
    private static final double maxAllowedPosition = 46.5;

    private SparkMax leftExtensionSparkMax = new SparkMax(31, MotorType.kBrushless);
    private SparkMax rightExtensionSparkMax = new SparkMax(32, MotorType.kBrushless);
    private SparkClosedLoopController leftExtensionController = leftExtensionSparkMax.getClosedLoopController();
    private RelativeEncoder leftExtensionEncoder = leftExtensionSparkMax.getEncoder();

    private double trim = 0;

    public void init() {

        followerTiltConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        followerTiltConfig.CurrentLimits.StatorCurrentLimit = 15;
        followerTiltConfig.CurrentLimits.SupplyCurrentLimit = 15;
        followerTiltConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        followerTiltConfig.CurrentLimits.SupplyCurrentLowerTime = 0; // Disabled since zero
        followerTiltConfig.Voltage.PeakForwardVoltage=9.0; //9.0;
        followerTiltConfig.Voltage.PeakReverseVoltage=-9.0; // -9.0;
        followerTiltConfig.Feedback.SensorToMechanismRatio = 4.*3.*3.*(80./12.); // Sets the gear ratio between rotations of the motor and rotations of the mechnism

        leaderTiltConfig.CurrentLimits.StatorCurrentLimit = 15;
        leaderTiltConfig.CurrentLimits.SupplyCurrentLimit = 15;
        leaderTiltConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        leaderTiltConfig.CurrentLimits.SupplyCurrentLowerTime = 0; // Disabled since zero
        leaderTiltConfig.Voltage.PeakForwardVoltage=9.0; //9.0;
        leaderTiltConfig.Voltage.PeakReverseVoltage=-9.0; // -9.0;
        leaderTiltConfig.Slot0.kP = 600.0; // An error of 1 results in 0.1 V output
        leaderTiltConfig.Slot0.kI = 0.0; // no output for integrated error
        leaderTiltConfig.Slot0.kD = 0.0; // no output for error derivative
        leaderTiltConfig.Slot0.kS = 0.1399; // Add output to overcome static friction
        leaderTiltConfig.Slot0.kV = 25.92402963116587; // V / rpm
        leaderTiltConfig.Slot0.kA = 3.2; // V / rotion/sec^2
        leaderTiltConfig.Slot0.kG = -.14; // -.10 is barely enough to stop it from going down when horizontal
        leaderTiltConfig.Slot0.GravityType=GravityTypeValue.Arm_Cosine;
        leaderTiltConfig.Slot1.kP = 100.0; // An error of 1 results in 0.1 V output
        leaderTiltConfig.Slot1.kI = 0.0; // no output for integrated error
        leaderTiltConfig.Slot1.kD = 0.0; // no output for error derivative
        leaderTiltConfig.Slot1.kS = 0.1399; // Add output to overcome static friction
        leaderTiltConfig.Slot1.kV = 10; // Intentionally underspped
        leaderTiltConfig.Slot1.kA = 3.2; // V / rotion/sec^2
        leaderTiltConfig.Slot1.kG = -.14; // -.10 is barely enough to stop it from going down when horizontal
        leaderTiltConfig.Slot1.GravityType=GravityTypeValue.Arm_Cosine;
        leaderTiltConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; // No gravity compensation (yet)
        leaderTiltConfig.MotionMagic.MotionMagicCruiseVelocity = .2; // Rot / sec
        leaderTiltConfig.MotionMagic.MotionMagicAcceleration = 1; // Rot / sec^2
        leaderTiltConfig.MotionMagic.MotionMagicJerk = 4; // Rot/sec^3
        leaderTiltConfig.Feedback.SensorToMechanismRatio = 4.*3.*3.*(80./12.); // Sets the gear ratio between rotations of the motor and rotations of the mechnism
        leaderTiltConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        leaderTiltConfig.HardwareLimitSwitch.ReverseLimitRemoteSensorID = 53;
        leaderTiltConfig.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyClosed;
        leaderTiltConfig.HardwareLimitSwitch.ReverseLimitEnable = true;
        leaderTiltConfig.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.RemoteCANdiS2;

        followerTilt.getConfigurator().apply(followerTiltConfig);
        leaderTilt.getConfigurator().apply(leaderTiltConfig);
        leaderTilt.setPosition(-.25, 1); // When vertical, we are actually at -.25
        followerTilt.setControl(followLeaderTiltRequest);
        leaderTilt.setControl(requestMotionMagicVoltage.withPosition(-0.25));

        // Configure left extension first
        SparkMaxConfig leftExtensionSparkMaxConfig = new SparkMaxConfig();
        leftExtensionSparkMaxConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)
            .inverted(true)
            .voltageCompensation(10.0);
        leftExtensionSparkMaxConfig.encoder
            // positionConversionFactor sets the position that encoder reports when making one full revolution.
            .positionConversionFactor(1.7716535329818726)
            .velocityConversionFactor(0.029527559876441956);
        leftExtensionSparkMaxConfig.closedLoop
        .pidf(.9, 0, 0, .00001, ClosedLoopSlot.kSlot0)
        .pidf(0, 0, 0, 0, ClosedLoopSlot.kSlot1)
        .outputRange(-.4, .5, ClosedLoopSlot.kSlot0);  // -0.3 and 0.4 are really fast

        leftExtensionSparkMax.configure(leftExtensionSparkMaxConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        // Configure right extension%
        SparkMaxConfig rightExtensionSparkMaxConfig = new SparkMaxConfig();
        rightExtensionSparkMaxConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)
            .voltageCompensation(10.0)
            .follow(leftExtensionSparkMax, true);
        rightExtensionSparkMaxConfig.encoder
            // positionConversionFactor sets the position that encoder reports when making one full revolution.
            .positionConversionFactor(1.7716535329818726)
            .velocityConversionFactor(0.029527559876441956);
        rightExtensionSparkMax.configure(rightExtensionSparkMaxConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
                

        SmartDashboard.putData("Elevator", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addDoubleProperty("Lampry angle", () -> {return lampryCANdi.getPWM1Position().getValueAsDouble()*360;}, null);
                builder.addDoubleProperty("Elevator angle", () -> {return leaderTilt.getPosition().getValueAsDouble()*360;}, null);
                builder.addDoubleProperty("Elevator extension", leftExtensionSparkMax.getEncoder()::getPosition, null);
            }
        });

        leftExtensionEncoder.setPosition(0);
        leftExtensionController.setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    double previousArmAngle = 0;

    public Command makeGoToAngleCmd(double angle){
        if (angle<minAllowedAngle || angle > maxAllowedAngle) {
            System.out.println("ASSERTION FAILED: Requested command would set elevator angle out of bounds to " + angle);
            return this.runOnce(
                () -> {
                    System.out.println("This command sets the elevator angle out of bounds to " + angle);
                }
            );
        }
        return new FunctionalCommand (
            () -> {
                previousArmAngle = (angle/360.0) - .25;
                leaderTilt.setControl(requestMotionMagicVoltage.withPosition(previousArmAngle + trim)); // Position is set in rotations, not angle, and has an offset of 1/4 rotation
                System.out.println("Elevator angle set to " + angle);
            },
            () -> {}, //onExecute
            (Boolean wasCanceled) -> {}, //onEnd
            () -> {
                Boolean arrived = (Math.abs(leaderTilt.getPosition().getValueAsDouble()-((angle/360.0)-.25)))<(5.0/360.0);
                //System.out.println("Elevator angle arrived? Delta " + (Math.abs(leaderTilt.getPosition().getValueAsDouble()-((angle/360.0)-.25))) + " so " + arrived); 
                return arrived; 
            },
            this);
    }

    public Command makeSlowGoToAngleCmd(double angle){
        if (angle<minAllowedAngle || angle > maxAllowedAngle) {
            System.out.println("ASSERTION FAILED: Requested command would set elevator angle out of bounds to " + angle);
            return this.runOnce(
                () -> {
                    System.out.println("This command sets the elevator angle out of bounds to " + angle);
                }
            );
        }
        return new FunctionalCommand (
            () -> {
                previousArmAngle = (angle/360.0) - .25;
                leaderTilt.setControl(requestMotionMagicVoltage.withPosition(previousArmAngle + trim).withSlot(1)); // Position is set in rotations, not angle, and has an offset of 1/4 rotation
                System.out.println("Elevator angle set to " + angle);
            },
            () -> {}, //onExecute
            (Boolean wasCanceled) -> {}, //onEnd
            () -> {
                Boolean arrived = (Math.abs(leaderTilt.getPosition().getValueAsDouble()-((angle/360.0)-.25)))<(5.0/360.0);
                //System.out.println("Elevator angle arrived? Delta " + (Math.abs(leaderTilt.getPosition().getValueAsDouble()-((angle/360.0)-.25))) + " so " + arrived); 
                return arrived; 
            },
            this);
    }


    public Command makeReleaseElevatorBrakeCmd(){
        return this.startEnd(
            () -> {
                leaderTilt.setControl(requestCoast);
                followerTilt.setControl(requestCoast);
                System.out.println("Elevator set to coast");
            },
            () -> {
                leaderTilt.setControl(requestNeutral);
                followerTilt.setControl(requestNeutral);
                System.out.println("Elevator set to neutral");
            });
    }

    public Command makeGoToPositionCmd(double position){
        if (position<minAllowedPosition || position > maxAllowedPosition) {
            System.out.println("ASSERTION FAILED: Requested command would set elevator position out of bounds to " + position);
        }
        return new FunctionalCommand (
            () -> {
                leftExtensionController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
                System.out.println("ELEVATOR TRAVELING TO " + position);
            }, // onInit
            () -> {}, //onExecute
            (Boolean wasCanceled) -> {}, //onEnd
            () -> {
                Boolean arrived = (Math.abs(((leftExtensionSparkMax.getEncoder().getPosition())-position))<.25);
                //System.out.println("at position? " + Math.abs(((leftExtensionSparkMax.getEncoder().getPosition())-position)) + " so " + arrived); 
                if (arrived) System.out.println("ELEVATOR ARRIVED.");
                return arrived; 
            },
            this);
    }

    public Command makeTrimUpCmd() {
        return this.runOnce(
            () -> {
                if (!(leaderTilt.getPosition().getValueAsDouble() + .25 >= 0)) {
                    trim += 0.25/360.0;
                }
                else {
                    trim -= 0.25/360.0;
                }
                leaderTilt.setControl(requestMotionMagicVoltage.withPosition(previousArmAngle + trim)); // Position is set in rotations, not angle, and has an offset of 1/4 rotationc
            }
        );
    }

    public Command makeTrimDownCmd() {
        return this.runOnce(
            () -> {
                if (!(leaderTilt.getPosition().getValueAsDouble() + .25 >= 0)) {
                    trim -= 0.25/360.0;
                }
                else {
                    trim += 0.25/360.0;
                }
                leaderTilt.setControl(requestMotionMagicVoltage.withPosition(previousArmAngle + trim)); // Position is set in rotations, not angle, and has an offset of 1/4 rotationc
            }
        );
    }
}