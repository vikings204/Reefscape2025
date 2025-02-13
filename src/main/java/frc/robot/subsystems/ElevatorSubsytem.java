package frc.robot.subsystems;


import static frc.robot.Constants.Elevator.*;



import java.util.Map;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator.Positions;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.ReduceCANUsage;
import frc.robot.util.ReduceCANUsage.CANCoderUtil;
import frc.robot.util.ReduceCANUsage.CANCoderUtil.CCUsage;
import frc.robot.util.ReduceCANUsage.Spark_Max.Usage;



public class ElevatorSubsytem extends SubsystemBase {
    public int moduleNumber;
    private final SparkMax angleMotor;
    private final SparkMaxConfig angleConfig;
    private final RelativeEncoder integratedAngleEncoder;
    private CANcoder angleEncoder;
    private final SparkClosedLoopController angleController;

    public int moduleNumber2;
    private final SparkMax angleMotor2;
    private final SparkMaxConfig angleConfig2;
    private final RelativeEncoder integratedAngleEncoder2;
    private CANcoder angleEncoder2;
    private final SparkClosedLoopController angleController2;
   


    public ElevatorSubsytem() {
      
        angleMotor = new SparkMax(ANGLE_MOTOR_ID_ONE, MotorType.kBrushless);
        angleConfig = new SparkMaxConfig();
        integratedAngleEncoder = angleMotor.getEncoder();
        angleController = angleMotor.getClosedLoopController();
        configAngleMotor();
        

       
        angleMotor2 = new SparkMax(ANGLE_MOTOR_ID_TWO, MotorType.kBrushless);
        angleConfig2 = new SparkMaxConfig();
        integratedAngleEncoder2 = angleMotor2.getEncoder();
        angleController2 = angleMotor2.getClosedLoopController();
        configAngleMotor2();
       
    }

   
   
    public void setAngle(Positions targetposition) {
        // Prevent rotating module if speed is less then 1%. Prevents jittering.
       angleController.setReference(targetposition.position, ControlType.kPosition);

       angleController2.setReference(targetposition.position, ControlType.kPosition);
    }
        
    public void setAngle(boolean b) {
        // Prevent rotating module if speed is less then 1%. Prevents jittering.
       if(b){
        System.out.println("I AM MOVING THE ARM to : "+integratedAngleEncoder.getPosition()+1);
        angleMotor.set(.25);
        angleMotor2.set(.25);

        //angleController.setReference(integratedAngleEncoder.getPosition()+1, ControlType.kPosition);
       }
       if(!b){
       // System.out.println("I AM MOVING THE ARM to : "+integratedAngleEncoder.getPosition()1);
        angleMotor.set(0);
        angleMotor2.set(0);

       }
    }
    public void setNAngle(boolean b) {
        // Prevent rotating module if speed is less then 1%. Prevents jittering.
       if(b){
        System.out.println("I AM MOVING THE ARM to : "+integratedAngleEncoder.getPosition()+1);
        angleMotor.set(-.25);
        angleMotor2.set(-.25);

       }
       if(!b){
        System.out.println("I AM MOVING THE ARM to : "+integratedAngleEncoder.getPosition()+1);
        angleMotor.set(0);
        angleMotor2.set(0);
       }
    }
 
public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    } 
public Rotation2d getAngle2(){
    return Rotation2d.fromDegrees(integratedAngleEncoder2.getPosition());
}    
 
 //implement later
private void configAngleMotor() {
       
        
       ReduceCANUsage.Spark_Max.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly,angleConfig);
       angleConfig.smartCurrentLimit(ANGLE_CURRENT_LIMIT) ;
       angleConfig.inverted(ANGLE_INVERT); 
       angleConfig.idleMode(ANGLE_IDLE_MODE); 
        //angleConfig.encoder.positionConversionFactor(1/ANGLE_POSITION_CONVERSION_FACTOR);
        angleConfig.encoder.positionConversionFactor(1.0/Constants.Elevator.ANGLE_POSITION_CONVERSION_FACTOR);
        angleConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(1,0,0)
            .outputRange(-1,1)
            .positionWrappingEnabled(false)
            .positionWrappingInputRange(0, 1)
            .minOutput(-1)
            .maxOutput(1);
            angleConfig.closedLoop.apply(angleConfig.closedLoop);
            angleConfig.apply(angleConfig);
        angleConfig.voltageCompensation(VOLTAGE_COMPENSATION);
        angleMotor.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
 
        //Timer.delay(2);
        integratedAngleEncoder.setPosition(0);
    }
    private void configAngleMotor2() {
       
        ReduceCANUsage.Spark_Max.setCANSparkMaxBusUsage(angleMotor2, Usage.kPositionOnly,angleConfig2);
        angleConfig2.smartCurrentLimit(ANGLE_CURRENT_LIMIT) ;
   
        angleConfig2.inverted(ANGLE_INVERT_2); 
     
        angleConfig2.idleMode(ANGLE_IDLE_MODE); 
     
        // angleConfig2.encoder.positionConversionFactor(1/ANGLE_POSITION_CONVERSION_FACTOR);
         angleConfig2.encoder.positionConversionFactor(1.0/ANGLE_POSITION_CONVERSION_FACTOR);
    
         angleConfig2.closedLoop
             .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
             .pid(1,0,0)
             .outputRange(-1, 1)
             .positionWrappingEnabled(false)
             .positionWrappingInputRange(0, 1)
             .minOutput(-1)
             .maxOutput(1);
     angleConfig2.closedLoop.apply(angleConfig2.closedLoop);
     angleConfig2.apply(angleConfig2);
         angleConfig2.voltageCompensation(VOLTAGE_COMPENSATION);
       
        angleMotor2.configure(angleConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  

        //   Timer.delay(2);

         integratedAngleEncoder2.setPosition(0);
         
     }

     //implement later
     /*public void CurrentLimit(){
        while(int i<hugenumber){
            if (angleMotor.getOutputCurrent())
        }
    
     }
        */
    
    }




