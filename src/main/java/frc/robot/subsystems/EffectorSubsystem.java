package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class EffectorSubsystem extends SubsystemBase {
    private SparkMax leftIntakeSparkMax  = new SparkMax(1, SparkLowLevel.MotorType.kBrushless);
    private SparkMax rightIntakeSparkMax  = new SparkMax(2, SparkLowLevel.MotorType.kBrushless);
    private SparkMax pivotSparkMax = new SparkMax(3, SparkLowLevel.MotorType.kBrushless);
    private RelativeEncoder leftIntakeEncoder = leftIntakeSparkMax.getEncoder();
    private RelativeEncoder rightIntakeEncoder = rightIntakeSparkMax.getEncoder();
    private AbsoluteEncoder pivotAbsoluteEncoder = pivotSparkMax.getAbsoluteEncoder();
    private AnalogInput coralEffectorSensor = new AnalogInput(0);
    private SparkClosedLoopController leftIntakeSparkMaxClosedLoopController = leftIntakeSparkMax.getClosedLoopController();
    private SparkClosedLoopController rightIntakeSparkClosedLoopController = rightIntakeSparkMax.getClosedLoopController();
    private SparkClosedLoopController pivotSparkMaxClosedLoopController = pivotSparkMax.getClosedLoopController();

    private SparkMaxConfig sparkMaxIntakeConfig = new SparkMaxConfig();
    private SparkMaxConfig sparkMaxPivotConfig = new SparkMaxConfig();

    private static final double positionWhenZeroed = .5;

    public void init() {

        sparkMaxIntakeConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(20)
            .inverted(false)
            .voltageCompensation(10.0);
        sparkMaxIntakeConfig.closedLoop
            .pidf(0,0.0,0.0,0.0168079390796268, ClosedLoopSlot.kSlot0) // 0.0168 v/rpm, so at 10v = 584 max 
            .pidf(.05, 0, 0, 0, ClosedLoopSlot.kSlot1)
            .outputRange(-1, 1);
        sparkMaxIntakeConfig.encoder
            // positionConversionFactor sets the position that encoder reports when making one full revolution.
            // Distance traveled = output revolutions * circumfrence of wheel * correction factor
            .positionConversionFactor((1.0 / 25.0) /*output revolutions*/ * (3.15159 * 3 /*inches*/))
            // velocityConversionFactor sets the velocity that encoder reports when running at 1 rpm.
            .velocityConversionFactor((1.0 / 25.0) /*output revolutions*/ * (3.15159 * 3 /*inches*/) / 60.0); 
       
        rightIntakeSparkMax.configure(sparkMaxIntakeConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        sparkMaxIntakeConfig.inverted(true);
        leftIntakeSparkMax.configure(sparkMaxIntakeConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        
        sparkMaxPivotConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(20)
            .inverted(false)
            .voltageCompensation(10.0);
        sparkMaxPivotConfig.absoluteEncoder
            // 
            .zeroOffset(0.331)
            // positionConversionFactor sets the position that encoder reports when making one full revolution.
            //x In this case, we will use degrees           
            .positionConversionFactor(360);
        sparkMaxPivotConfig.closedLoop
            .pidf(.007, 0, 0, 0, ClosedLoopSlot.kSlot0)
            .outputRange(-.2, .2, ClosedLoopSlot.kSlot0)
            .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder);
        
        pivotSparkMax.configure(sparkMaxPivotConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        rightIntakeSparkClosedLoopController.setReference(0, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        leftIntakeSparkMaxClosedLoopController.setReference(0, ControlType.kVelocity, ClosedLoopSlot.kSlot0);

        pivotSparkMaxClosedLoopController.setReference(180, ControlType.kPosition, ClosedLoopSlot.kSlot0);

        SmartDashboard.putData("Effector", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addDoubleProperty("Effector angle from down", pivotAbsoluteEncoder::getPosition, null);
                builder.addDoubleProperty("Left effector velocity", leftIntakeEncoder::getVelocity, null);
                builder.addDoubleProperty("Right effector velocity", rightIntakeEncoder::getVelocity, null);
            }
        });
    }

    public Command makeRotateTo(double position)
    {  
        if (position > (130) || position < (-130)) {
            System.out.println("ASSERTION FAILURE: position for elevator pivot requested outside of allowed range!");
            return this.runOnce(() -> System.out.println("ASSERTION FAILURE: This pivot command has a position outside of the allowed range!"));
        }
        else return this.runOnce(() -> 
            {
                pivotSparkMaxClosedLoopController.setReference(180+position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
                System.out.println("Effector angle set to " + position);
            }
        );
    }

    public Command makeSetSpeed(double ips)
    {
        return this.runOnce(() -> {
                rightIntakeSparkClosedLoopController.setReference(ips, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
                leftIntakeSparkMaxClosedLoopController.setReference(ips, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
                System.out.println("Effector speed set to " + ips);
            }); 
    }
    
    public Command makeMoveTo(double position)
    {
        return this.runOnce(() -> { // onInit
            rightIntakeSparkClosedLoopController.setReference(position-positionWhenZeroed, ControlType.kPosition, ClosedLoopSlot.kSlot1);
            leftIntakeSparkMaxClosedLoopController.setReference(position-positionWhenZeroed, ControlType.kPosition, ClosedLoopSlot.kSlot1);
            System.out.println("Effector coral position set to " + position);
        });
    }

    public Command makeWaitUntilCoralPresent() {
        return new WaitUntilCommand(() -> coralEffectorSensor.getVoltage() > 2.1);
    }

    public Command makeWaitUntilCoralMissing() {
        return new WaitUntilCommand(() -> coralEffectorSensor.getVoltage() < 2.1);
    }

    public Command makeZeroEncoder() {
        return this.runOnce(() -> {
            rightIntakeEncoder.setPosition(0);
            leftIntakeEncoder.setPosition(0);
            System.out.println("Effector encoders zeroed");
        });
    }
}