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

public class ArmReachSubsystem extends SubsystemBase{

    private static final double minAllowedPosition = 0.0;
    private static final double maxAllowedPosition = 46.5;

    private SparkMax leftReachSparkMax = new SparkMax(31, MotorType.kBrushless);
    private SparkMax rightReachSparkMax = new SparkMax(32, MotorType.kBrushless);
    private SparkClosedLoopController leftReachController = leftReachSparkMax.getClosedLoopController();
    private RelativeEncoder leftReachEncoder = leftReachSparkMax.getEncoder();

    private double trim = 0;

    public void init() {

        // Configure left extension first
        SparkMaxConfig leftReachSparkMaxConfig = new SparkMaxConfig();
        leftReachSparkMaxConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)
            .inverted(true)
            .voltageCompensation(10.0);
        leftReachSparkMaxConfig.encoder
            // positionConversionFactor sets the position that encoder reports when making one full revolution.
            .positionConversionFactor(1.7716535329818726)
            .velocityConversionFactor(0.029527559876441956);
        leftReachSparkMaxConfig.closedLoop
        .pidf(.9, 0, 0, .00001, ClosedLoopSlot.kSlot0)
        .pidf(0, 0, 0, 0, ClosedLoopSlot.kSlot1)
        .outputRange(-.4, .5, ClosedLoopSlot.kSlot0);  // -0.3 and 0.4 are really fast

        leftReachSparkMax.configure(leftReachSparkMaxConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        // Configure right reach
        SparkMaxConfig rightReachSparkMaxConfig = new SparkMaxConfig();
        rightReachSparkMaxConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)
            .voltageCompensation(10.0)
            .follow(leftReachSparkMax, true);
        rightReachSparkMaxConfig.encoder
            // positionConversionFactor sets the position that encoder reports when making one full revolution.
            .positionConversionFactor(1.7716535329818726)
            .velocityConversionFactor(0.029527559876441956);
        rightReachSparkMax.configure(rightReachSparkMaxConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
                

        SmartDashboard.putData("Arm", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addDoubleProperty("Arm reach", leftReachEncoder::getPosition, null);
                builder.addDoubleProperty("Arm speed", leftReachEncoder::getVelocity, null);
            }
        });

        leftReachEncoder.setPosition(0);
        leftReachController.setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public double getReach()
    {
        return leftReachEncoder.getPosition();
    }

    public boolean isStopped()
    {
        return (Math.abs(leftReachEncoder.getVelocity())<0.1);
    }

    public Command makeReach(double position){
        if (position<minAllowedPosition || position > maxAllowedPosition) {
            System.out.println("ARM REACH REQUEST OF " + position + " IS OUT OF BOUNDS");
            return this.runOnce(
                () -> {
                    System.out.println("BAD ARM REACH COMMAND EXECUTED");
                }
            );
        }
        else {
            return new FunctionalCommand (
                () -> {
                    leftReachController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
                }, // onInit
                () -> {}, //onExecute
                (Boolean wasCanceled) -> {}, //onEnd
                () -> {
                    Boolean arrived = (Math.abs(((leftReachSparkMax.getEncoder().getPosition())-position))<.25);
                    if (!arrived && isStopped()) {
                        System.out.println("ARM REACH JAMMED AT " + getReach() + " WHILE TRAVELING TO " + position);   
                    }
                    return arrived; 
                },
                this);
        }
    }
}