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

public class EffectorLookSubsystem extends SubsystemBase {
    private SparkMax lookSparkMax = new SparkMax(3, SparkLowLevel.MotorType.kBrushless);
    private AbsoluteEncoder lookAbsoluteEncoder = lookSparkMax.getAbsoluteEncoder();
    private SparkClosedLoopController lookSparkMaxClosedLoopController = lookSparkMax.getClosedLoopController();

    private SparkMaxConfig sparkMaxLookConfig = new SparkMaxConfig();

    private static final double positionWhenZeroed = .5;

    public void init() {

        sparkMaxLookConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(20)
            .inverted(false)
            .voltageCompensation(10.0);
        sparkMaxLookConfig.absoluteEncoder
            // 
            .zeroOffset(0.331)
            // positionConversionFactor sets the position that encoder reports when making one full revolution.
            //x In this case, we will use degrees           
            .positionConversionFactor(360);
        sparkMaxLookConfig.closedLoop
            .pidf(.007, 0, 0, 0, ClosedLoopSlot.kSlot0)
            .outputRange(-.2, .2, ClosedLoopSlot.kSlot0)
            .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder);
        
        lookSparkMax.configure(sparkMaxLookConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        lookSparkMaxClosedLoopController.setReference(180, ControlType.kPosition, ClosedLoopSlot.kSlot0);

        SmartDashboard.putData("Effector", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addDoubleProperty("Effector look", ()-> {return lookAbsoluteEncoder.getPosition()-180;}, null);
            }
        });
    }

    public Command makeLook(double look)
    {  
        if (look > (130) || look < (-55)) {
            System.out.println("EFFECTOR LOOK REQUEST OF " + look + " IS OUT OF BOUNDS");
            return this.runOnce(
                () -> {
                    System.out.println("BAD EFFECOTOR LOOK COMMAND EXECUTED");
                }
            );
        }
        else return this.runOnce(() -> 
            {
                lookSparkMaxClosedLoopController.setReference(180+look, ControlType.kPosition, ClosedLoopSlot.kSlot0);
                System.out.println("Effector angle set to " + look);
            }
        );
    }
} 