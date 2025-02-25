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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ShooterSubsystem extends SubsystemBase {
    public int moduleNumber;
    private final SparkMax angleMotor;
    private final SparkMaxConfig angleConfig;
    private final RelativeEncoder integratedAngleEncoder;
    private CANcoder angleEncoder;
    private final SparkClosedLoopController angleController;

   


    public ShooterSubsystem() {
        angleMotor = new SparkMax(44, MotorType.kBrushless);
        angleConfig = new SparkMaxConfig();
        integratedAngleEncoder = angleMotor.getEncoder();
        configAngleMotor();
        angleController = angleMotor.getClosedLoopController();

           }
    
    public void ShootArm(boolean b){
      if(b==true){
      angleMotor.set(.3);
    }
      else
        angleMotor.set(0);
    }
    

    public void setNAngle(boolean b) {
       angleMotor.set(-.1);
   
    }

       
 
public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    }   

 //implement later
private void configAngleMotor() {
       
        
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
        
        angleConfig.voltageCompensation(VOLTAGE_COMPENSATION);
       angleMotor.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
 
        Timer.delay(.5);
        integratedAngleEncoder.setPosition(0);

    }    
  }
