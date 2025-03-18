package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.TrapeziodVelocityPositionPID.PIDType;


public class TrapeziodVelocityPositionPID extends SubsystemBase{

    
    TrapezoidProfile trapezoidProfile =  new TrapezoidProfile(new TrapezoidProfile.Constraints(0, 0));

    private double maxVelocity=0.0, maxAcceleration=0.0;
    private double minPosition=0.0, maxPosition=0.0;
    private double lag=0.0;
    private DoubleSupplier getVelocity;
    private DoubleSupplier getPosition;
    private TrapezoidProfile.State goalState = null;


    private final TrapezoidProfile.State currentState = new TrapezoidProfile.State();
    private final TrapezoidProfile.State accelerationState = new TrapezoidProfile.State();
    private final TrapezoidProfile.State decelerationState = new TrapezoidProfile.State();

    private double directionSign = 1.0; // 1 if goal position > start position, -1 otherwise

    /* Sets the maximum velocity the device will travel at */
    TrapeziodVelocityPositionPID setConstraings(double maxVelocity) 
    {
        this.maxVelocity=maxVelocity;
        trapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
        return this;
    }
    /* Sets the minimum velocity the device will travel at */
    TrapeziodVelocityPositionPID setMinVelocity(double maxAcceleration) {this.maxAcceleration=maxAcceleration; return this;}
    /* Sets the minimum position the device can reach */
    TrapeziodVelocityPositionPID setMaxPosition(double maxPosition) {this.maxPosition=maxPosition; return this;}
    /* Sets the maximum position the device can reach */
    TrapeziodVelocityPositionPID setMinPosition(double minPosition) {this.minPosition=minPosition; return this;}
    /* Sets the lag between the time that the device is commanded to be at zero velocity and the time it actually achieves zero velocity */
    TrapeziodVelocityPositionPID setLag(double lag) {this.lag=lag; return this;}
    TrapeziodVelocityPositionPID setVelocityDoubleSupplier(DoubleSupplier getVolocity) {this.getVelocity=getVolocity; return this;}
    TrapeziodVelocityPositionPID setPositionDoubleSupplier(DoubleSupplier getPosition) {this.getVelocity=getPosition; return this;}
    TrapeziodVelocityPositionPID setGoal(TrapezoidProfile.State goalState) {this.goalState = goalState; return this;}


    enum PIDType {
        Velocity,
        Position
    };
    public PIDType usePID = PIDType.Velocity;

    private double maxCommandedVel=0;
    private double maxActualVel=0;

    Bool periodic() {
        currentState.position = getPosition.getAsDouble();
        currentState.velocity = getVelocity.getAsDouble();    
        accelerationState = trapezoidProfile.calculate(.02, currentState, goalState);
        // There is some lag between when we command the motor to slow down and it actually
        // slowing down, so send the slow down commands early.
        decelerationState = trapezoidProfile.calculate(goalEndLag, currentState, goalState);

        if(accelerationState.velocity>maxCommandedVel) maxCommandedVel = accelerationState.velocity;

        // If the decleration velocity, which is slightly ahead in time than the
        // acceleration velocity, is smaller, then we must be decelerating.
        boolean declerating = (Math.abs(decelerationState.velocity)-Math.abs(accelerationState.velocity))<0; 
        if (declerating) {
            setpointState.position = decelerationState.position;
            setpointState.velocity = decelerationState.velocity;
        }
        else {
            setpointState.position = accelerationState.position;
            setpointState.velocity = accelerationState.velocity;
        }


        switch(usePID) {
            case Velocity:
            boolean endedByPosition = (goalState.position-currentState.position)*directionSign<=0; 
            boolean endedByVelocity = declerating&&Math.abs(sparkAbsoluteEncoder.getVelocity())*360.0/60.0<5.0; 
            if(endedByPosition||endedByVelocity) {
                    usePID=PIDType.Position;
                    System.out.format("Ended by "+(endedByVelocity?"velocity":"position")+"."+
                        "\n%+06.1f degrees left to go @ velocity %+06.1f"+
                        "\nmax velocity was:%+06.1f\n",
                        ((goalState.position-currentState.position)*directionSign),
                        sparkAbsoluteEncoder.getVelocity(),
                        maxCommandedVel);
            }
            break;
            case Position:
                if(Math.abs(currentState.position-goalState.position)>20) {
                    usePID=PIDType.Velocity;
                }        
                break;
        }

        switch(usePID) {
            case Velocity:
                sparkClosedLoopController.setReference(setpointState.velocity, ControlType.kVelocity,0);
                if (printVelocity) System.out.format("sv:%+06.1f av:%+06.1f ap:%+06.1f\n",
                    setpointState.velocity,
                    sparkAbsoluteEncoder.getVelocity()*360.0/60.0,
                    sparkAbsoluteEncoder.getPosition());
                break;
            case Position:
                sparkClosedLoopController.setReference(goalState.position, ControlType.kPosition,1);
                break;
        }
    }

    boolean printVelocity=true;

    public Command makeDeployHatCmd ()
    {
        return this.runOnce(() -> {
            System.out.println("delpoyed");
            goalState = deployedState;
            goalEndLag = deployedStateEndLag;
            accelerationState.velocity = sparkAbsoluteEncoder.getVelocity();
            directionSign = (goalState.position>currentState.position)?1:-1;
        });
    }
    
    public Command makeRetractHatCmd ()
    {
        return this.runOnce(() -> {
            System.out.println("retracted");
            goalState = retractedState;
            goalEndLag = retractedStateEndLag;
            accelerationState.velocity = sparkAbsoluteEncoder.getVelocity();
            directionSign = (goalState.position>currentState.position)?1:-1;
        });
    }
}
