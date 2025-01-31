package frc.robot.subsystems;

import static frc.robot.Constants.Elevator.*;
/*import static frc.robot.Constants.Elevator.ANGLE_CURRENT_LIMIT;
import static frc.robot.Constants.Elevator.ANGLE_IDLE_MODE;
import static frc.robot.Constants.Elevator.ANGLE_INVERT;
import static frc.robot.Constants.Elevator.ANGLE_INVERT_2;
import static frc.robot.Constants.Elevator.ANGLE_MOTOR_ID_ONE;
import static frc.robot.Constants.Elevator.ANGLE_MOTOR_ID_TWO;
import static frc.robot.Constants.Elevator.ANGLE_PID_D;
import static frc.robot.Constants.Elevator.ANGLE_PID_FF;
import static frc.robot.Constants.Elevator.ANGLE_PID_I;
import static frc.robot.Constants.Elevator.ANGLE_PID_P;
import static frc.robot.Constants.Elevator.ANGLE_POSITION_CONVERSION_FACTOR;
import static frc.robot.Constants.Elevator.MAX_SPEED;
import static frc.robot.Constants.Elevator.VOLTAGE_COMPENSATION;*/
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
      //  this.angleOffset = angleOffset;

        /* Angle Encoder Config */
         //angleEncoder = new CANcoder(canCoderID);
        //configAngleEncoder();
        angleMotor = new SparkMax(ANGLE_MOTOR_ID_ONE, MotorType.kBrushless);
        angleConfig = new SparkMaxConfig();
        integratedAngleEncoder = angleMotor.getEncoder();
        angleController = angleMotor.getClosedLoopController();
        configAngleMotor();
        

       // Shuffleboard.getTab("swervetest").addNumber("angleEncoderCurrent Reading " + moduleNumber, integratedAngleEncoder::getPosition).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", 0, "max", 360));
        //Shuffleboard.getTab("swervetest").addNumber("angleMotorAbsEncoder Reading " + moduleNumber, angleMotor.getAnalog()::getVoltage);

        //angleOffset2 = angleOffset;

        //angleEncoder2 = new CANcoder(canCoderID);
      //  configAngleEncoder();
        angleMotor2 = new SparkMax(ANGLE_MOTOR_ID_TWO, MotorType.kBrushless);
        angleConfig2 = new SparkMaxConfig();
        integratedAngleEncoder2 = angleMotor2.getEncoder();
        angleController2 = angleMotor2.getClosedLoopController();
        configAngleMotor2();
       
    //    Shuffleboard.getTab("swervetest").addNumber("angleEncoderCurrent Reading " + moduleNumber2, integratedAngleEncoder2::getPosition).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", 0, "max", 360));
       // Shuffleboard.getTab("swervetest").addNumber("angleMotorAbsEncoder Reading " + moduleNumber2, angleMotor2.getAnalog()::getVoltage);
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
        angleMotor.set(.5);
        angleMotor2.set(-.5);

        //angleController.setReference(integratedAngleEncoder.getPosition()+1, ControlType.kPosition);
       }
       if(!b){
       // System.out.println("I AM MOVING THE ARM to : "+integratedAngleEncoder.getPosition()1);
        angleMotor.set(0);
        angleMotor2.set(0);

        //angleController.setReference(integratedAngleEncoder.getPosition()-1, ControlType.kPosition);
       }
      // angleController2.setReference(targetposition.position, ControlType.kPosition);
    }
    public void setNAngle(boolean b) {
        // Prevent rotating module if speed is less then 1%. Prevents jittering.
       if(b){
        System.out.println("I AM MOVING THE ARM to : "+integratedAngleEncoder.getPosition()+1);
        angleMotor.set(-.5);
        angleMotor2.set(.5);

        //angleController.setReference(integratedAngleEncoder.getPosition(), ControlType.kPosition);
       }
       if(!b){
        System.out.println("I AM MOVING THE ARM to : "+integratedAngleEncoder.getPosition()+1);
        angleMotor.set(0);
        angleMotor2.set(0);
        //angleController.setReference(integratedAngleEncoder.getPosition(), ControlType.kPosition);
       }
      // angleController2.setReference(targetposition.position, ControlType.kPosition);
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
        angleConfig.encoder.positionConversionFactor(4);
        angleConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(1, ANGLE_PID_I, ANGLE_PID_D, ANGLE_PID_FF)
            .positionWrappingEnabled(true)
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
         angleConfig2.encoder.positionConversionFactor(ANGLE_POSITION_CONVERSION_FACTOR);
    
         angleConfig2.closedLoop
             .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
             .pidf(1, ANGLE_PID_I, ANGLE_PID_D, ANGLE_PID_FF)
             .outputRange(-1, 1)
             .positionWrappingEnabled(true)
             .positionWrappingInputRange(0, 1)
             .minOutput(-1)
             .maxOutput(1);
     angleConfig2.closedLoop.apply(angleConfig.closedLoop);
     angleConfig2.apply(angleConfig);
         angleConfig2.voltageCompensation(VOLTAGE_COMPENSATION);
       
        angleMotor2.configure(angleConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  

        //   Timer.delay(2);

         integratedAngleEncoder2.setPosition(0);
         
     }
    
    }




