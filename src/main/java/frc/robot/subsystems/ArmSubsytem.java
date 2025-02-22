package frc.robot.subsystems;

import static frc.robot.Constants.Arm.*;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ArmSubsytem extends SubsystemBase {
    public int moduleNumber;
    private final SparkMax angleMotor;
    private final SparkMaxConfig angleConfig;
    private final RelativeEncoder integratedAngleEncoder;
    private CANcoder angleEncoder;
    private final SparkClosedLoopController angleController;

   


    public ArmSubsytem() {
      //  this.angleOffset = angleOffset;

        /* Angle Encoder Config */
         //angleEncoder = new CANcoder(canCoderID);
        //configAngleEncoder();
        angleMotor = new SparkMax(43, MotorType.kBrushless);
        angleConfig = new SparkMaxConfig();
        integratedAngleEncoder = angleMotor.getEncoder();
        configAngleMotor();
        angleController = angleMotor.getClosedLoopController();


        

       // Shuffleboard.getTab("swervetest").addNumber("angleEncoderCurrent Reading " + moduleNumber, integratedAngleEncoder::getPosition).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", 0, "max", 360));
        //Shuffleboard.getTab("swervetest").addNumber("angleMotorAbsEncoder Reading " + moduleNumber, angleMotor.getAnalog()::getVoltage);

        //angleOffset2 = angleOffset;

        //angleEncoder2 = new CANcoder(canCoderID);
      //  configAngleEncoder();

       
    //    Shuffleboard.getTab("swervetest").addNumber("angleEncoderCurrent Reading " + moduleNumber2, integratedAngleEncoder2::getPosition).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", 0, "max", 360));
       // Shuffleboard.getTab("swervetest").addNumber("angleMotorAbsEncoder Reading " + moduleNumber2, angleMotor2.getAnalog()::getVoltage);
    }

   
   
    public void setAngle (Positions targetposition) {

       //angleController.setReference(targetposition.position, ControlType.kPosition);
       System.out.println("Set Angle is Working");

    }   
    public void ShootArm(boolean b){
      if(b==true){
        angleController.setReference(integratedAngleEncoder.getPosition()+.05,ControlType.kPosition);
      }
      else
        angleController.setReference(integratedAngleEncoder.getPosition(), ControlType.kPosition);
    }
    
    public void NegativeShootArm(boolean b){
      if(b==true){
        angleController.setReference(integratedAngleEncoder.getPosition()-.05,ControlType.kPosition);
      }
      else
      angleController.setReference(integratedAngleEncoder.getPosition(), ControlType.kPosition);
    }

    public void setNAngle(boolean b) {
        // Prevent rotating module if speed is less then 1%. Prevents jittering.
        System.out.println("Current Arm Postioin = "+integratedAngleEncoder.getPosition());
        System.out.println("I AM MOVING THE ARM to : "+1);
       //angleController.setReference(5, ControlType.kPosition);
       //angleMotor.set(.1);
       angleController.setReference(-2,ControlType.kPosition);
   
    }

    /*public void setNAngle(boolean b) {
       if(b){
        System.out.println("I AM MOVING THE ARM to : "+(integratedAngleEncoder.getPosition()+1));
       angleController.setReference(integratedAngleEncoder.getPosition()+1, ControlType.kPosition);
        //angleMotor.set(-.5);
        //angleMotor2.set(.5);

        //angleController.setReference(integratedAngleEncoder.getPosition(), ControlType.kPosition);
       }
       if(!b){
        //System.out.println("I AM MOVING THE ARM to : "+integratedAngleEncoder.getPosition()+1);
        //angleMotor.set(0);
        //angleMotor2.set(0);
        //angleController.setReference(integratedAngleEncoder.getPosition(), ControlType.kPosition);
       }
      // angleController2.setReference(targetposition.position, ControlType.kPosition);
    }*/
 
public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    }   

 //implement later
private void configAngleMotor() {
       
        
      // ReduceCANUsage.Spark_Max.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly,angleConfig);
            angleConfig
                    .idleMode(ANGLE_IDLE_MODE)
                    .smartCurrentLimit(40)
                    .inverted(false);
                    //.setInverted(true);
            angleConfig.encoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering moton the MAXSwerve Module.
                    //.inverted(true)
                    .positionConversionFactor(1.0/100.0) // radians
                    .velocityConversionFactor(1); // radians per second
            angleConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(10, 0, 0)
                    .outputRange(-1,1)
                    .positionWrappingEnabled(false)
                    .positionWrappingInputRange(0,1)
                    .minOutput(-1)
                    .maxOutput(1);
            angleConfig.closedLoop.apply(angleConfig.closedLoop);
            angleConfig.apply(angleConfig);
           System.out.println("I CONFIGURED THE ELEVATOR MOTOR");
        
            // angleController
        // Replaced above 
        //angleController.SetP(ANGLE_PID_P);
        //angleController.
        //angleController.setI(ANGLE_PID_I);
        //angleController.setD(ANGLE_PID_D);
        //angleController.setFF(ANGLE_PID_FF);
       // angleController.setFeedbackDevice(angleMotor.getAnalog(SparkAnalogSensor.Mode.kAbsolute));
        angleConfig.voltageCompensation(VOLTAGE_COMPENSATION);
       //REPLACED ABOVE angleMotor.enableVoltageCompensation(VOLTAGE_COMPENSATION);
       angleMotor.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
 
       //angleMotor.burnFlash();
        Timer.delay(2);
        integratedAngleEncoder.setPosition(0);

    }    
  }
