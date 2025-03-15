package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KickerSubsystem extends SubsystemBase {
    private SparkMax kickerSparkMax = new SparkMax(21, MotorType.kBrushless);
    private SparkClosedLoopController kickerController;
    private AbsoluteEncoder kickerAbsoluteEncoder = kickerSparkMax.getAbsoluteEncoder();

    private static double reqang=0; 
    public static double getReqang() {
        return reqang;
    }

    private final static double
        kickerPosRetracted = 92 /* degrees */,
        kickerPosPrepped = 105 /* degrees */,
        kickerPosExtended = 200 /* degrees */;
    
    public void init() {
        SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
        kickerController = kickerSparkMax.getClosedLoopController();

        
        sparkMaxConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)
            .inverted(false)
            .voltageCompensation(10.0);
        sparkMaxConfig.absoluteEncoder
            .zeroOffset(0.5922214984893799)
            // positionConversionFactor sets the position that encoder reports when making one full revolution.
            // In this case, we will use degrees
            .positionConversionFactor(360.0);


        sparkMaxConfig.closedLoop
        .pidf(.01, 0, 0, 0, ClosedLoopSlot.kSlot0)
        .outputRange(-.2, .2, ClosedLoopSlot.kSlot0)
        .pidf(.01, 0, 0, 0, ClosedLoopSlot.kSlot1)
        .outputRange(-1.0, 1.0, ClosedLoopSlot.kSlot1)
        .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder);
       
        kickerSparkMax.configure(sparkMaxConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        kickerController.setReference(0.25, ControlType.kPosition, ClosedLoopSlot.kSlot0);

        SmartDashboard.putData("Kicker", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addDoubleProperty("Kicker angle from down", kickerAbsoluteEncoder::getPosition, null);
                builder.addDoubleProperty("Kicker requested angle", KickerSubsystem::getReqang, null);
            }
        });
    }  

    public Command makeGoToPositionCmd(DoubleSupplier supplier){
        return this.run(
            () -> {
                double setPoint = Math.min(Math.max(supplier.getAsDouble(), 0.0), 1.0);
                setPoint = setPoint * (kickerPosExtended-kickerPosRetracted) + kickerPosRetracted;
                reqang = setPoint;

                kickerController.setReference(setPoint, ControlType.kPosition, kickerAbsoluteEncoder.getPosition()<140.0?ClosedLoopSlot.kSlot0:ClosedLoopSlot.kSlot1);
            }
        );
    }
}
