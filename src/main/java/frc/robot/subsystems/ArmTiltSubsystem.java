package frc.robot.subsystems;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmTiltSubsystem extends SubsystemBase{

    private static final TalonFX leaderTilt = new TalonFX(51, "Default Name");
    private static final TalonFX followerTilt = new TalonFX(52, "Default Name");
    private static final Follower followLeaderTiltRequest = new Follower(51, true);

    private static final TalonFXConfiguration followerTiltConfig = new TalonFXConfiguration();
    private static final TalonFXConfiguration leaderTiltConfig = new TalonFXConfiguration();
    private static final MotionMagicVoltage requestMotionMagicVoltage = new MotionMagicVoltage(0);
    private static final CoastOut requestCoast = new CoastOut();
    private static final NeutralOut requestNeutral = new NeutralOut();

    private static final double minAllowedAngle = -20.0; 
    private static final double maxAllowedAngle = 75.0;

    private double angleTrim = 0;

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

        SmartDashboard.putData("Arm Tilt", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addDoubleProperty("Arm angle", () -> getAngle(), null);
            }
        });
    }

    public double getAngle()
    {
        return leaderTilt.getPosition().getValueAsDouble()*360;
    }

    public boolean isStopped()
    {
        return (leaderTilt.getVelocity().getValueAsDouble()*360)<2.0;
    }



    double currentArmAngle = 0;

    public Command makeTilt(double angle){
        if (angle<minAllowedAngle || angle > maxAllowedAngle) {
            System.out.println("ARM TILT REQUEST OF " + angle + " IS OUT OF BOUNDS");
            return this.runOnce(
                () -> {
                    System.out.println("BAD ARM TILT COMMAND EXECUTED");
                }
            );
        }
        return new FunctionalCommand (
            () -> {
                currentArmAngle = (angle/360.0) - .25;
                leaderTilt.setControl(requestMotionMagicVoltage.withPosition(currentArmAngle + angleTrim)); // Position is set in rotations, not angle, and has an offset of 1/4 rotation
                System.out.println("Arm angle set to " + angle);
            },
            () -> {}, //onExecute
            (Boolean wasCanceled) -> {}, //onEnd
            () -> {
                Boolean arrived = (Math.abs(leaderTilt.getPosition().getValueAsDouble()-((angle/360.0)-.25)))<(5.0/360.0);
                return arrived; 
            },
            this);
    }

    public Command makeSlowTiltCmd(double angle){
        if (angle<minAllowedAngle || angle > maxAllowedAngle) {
            System.out.println("ARM ANGLE REQUEST OF " + angle + " IS OUT OF BOUNDS");
            return this.runOnce(
                () -> {
                    System.out.println("BAD ANGLE REQUST EXECUTED");
                }
            );
        }
        return new FunctionalCommand (
            () -> {
                currentArmAngle = (angle/360.0) - .25;
                leaderTilt.setControl(requestMotionMagicVoltage.withPosition(currentArmAngle + angleTrim).withSlot(1)); // Position is set in rotations, not angle, and has an offset of 1/4 rotation
            },
            () -> {}, //onExecute
            (Boolean wasCanceled) -> {}, //onEnd
            () -> {
                Boolean arrived = (Math.abs(leaderTilt.getPosition().getValueAsDouble()-((angle/360.0)-.25)))<(5.0/360.0);
                if (!arrived && isStopped()) {
                    System.out.println("ARM ANGLE JAMMED AT " + getAngle() + " WHILE TRAVELING TO " + currentArmAngle);   
                }
                return arrived; 
            },
            this);
    }


    public Command makeReleaseBrakeCmd(){
        return this.startEnd(
            () -> {
                leaderTilt.setControl(requestCoast);
                followerTilt.setControl(requestCoast);
                System.out.println("ARM SET TO COAST");
            },
            () -> {
                leaderTilt.setControl(requestNeutral);
                followerTilt.setControl(requestNeutral);
                System.out.println("ARM SET TO NEUTRAL");
            });
    }

    public Command makeTrimUpCmd() {
        return this.runOnce(
            () -> {
                if (!(leaderTilt.getPosition().getValueAsDouble() + .25 >= 0)) {
                    angleTrim += 0.25/360.0;
                }
                else {
                    angleTrim -= 0.25/360.0;
                }
                leaderTilt.setControl(requestMotionMagicVoltage.withPosition(currentArmAngle + angleTrim)); // Position is set in rotations, not angle, and has an offset of 1/4 rotationc
            }
        );
    }

    public Command makeTrimDownCmd() {
        return this.runOnce(
            () -> {
                if (!(leaderTilt.getPosition().getValueAsDouble() + .25 >= 0)) {
                    angleTrim -= 0.25/360.0;
                }
                else {
                    angleTrim += 0.25/360.0;
                }
                leaderTilt.setControl(requestMotionMagicVoltage.withPosition(currentArmAngle + angleTrim)); // Position is set in rotations, not angle, and has an offset of 1/4 rotationc
            }
        );
    }
}