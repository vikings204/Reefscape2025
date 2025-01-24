package frc.robot.subsystems;


import static frc.robot.Constants.Swerve.ANGLE_CURRENT_LIMIT;
import static frc.robot.Constants.Swerve.ANGLE_GEAR_RATIO;
import static frc.robot.Constants.Swerve.ANGLE_IDLE_MODE;
import static frc.robot.Constants.Swerve.ANGLE_INVERT;
import static frc.robot.Constants.Swerve.ANGLE_PID_D;
import static frc.robot.Constants.Swerve.ANGLE_PID_FF;
import static frc.robot.Constants.Swerve.ANGLE_PID_I;
import static frc.robot.Constants.Swerve.ANGLE_PID_P;
import static frc.robot.Constants.Swerve.ANGLE_POSITION_CONVERSION_FACTOR;
import static frc.robot.Constants.Swerve.MAX_SPEED;
import static frc.robot.Constants.Swerve.VOLTAGE_COMPENSATION;
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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants;
import frc.robot.Constants.Elevator.*;
import frc.robot.Robot;
import frc.robot.util.ReduceCANUsage;
import frc.robot.util.ReduceCANUsage.CANCoderUtil;
import frc.robot.util.ReduceCANUsage.CANCoderUtil.CCUsage;
import frc.robot.util.ReduceCANUsage.Spark_Max.Usage;



public class ElevatorSubsytem {

    public int moduleNumber;
    private Rotation2d lastAngle;
    private Rotation2d angleOffset;
    private double turningPDeg = 0;
    private int turningPQuad = 0; // top left is 1 counterclockwise
    private double turningTotalDeg = 0.0;
    private final SparkMax angleMotor;
    private final SparkMaxConfig angleConfig;
    
    private final RelativeEncoder integratedAngleEncoder;
    private CANcoder angleEncoder;
    private final SparkClosedLoopController angleController;
    


    public ElevatorSubsytem(int moduleNumber, int angleMotorID, Rotation2d angleOffset, int canCoderID) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = angleOffset;

        /* Angle Encoder Config */
         angleEncoder = new CANcoder(canCoderID);
        configAngleEncoder();

        angleMotor = new SparkMax(angleMotorID, MotorType.kBrushless);
        angleConfig = new SparkMaxConfig();
        integratedAngleEncoder = angleMotor.getEncoder();
        angleController = angleMotor.getClosedLoopController();
        configAngleMotor();


        Shuffleboard.getTab("swervetest").addNumber("angleEncoderCurrent Reading " + moduleNumber, integratedAngleEncoder::getPosition).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", 0, "max", 360));
        //REMOVED MOD.KABS AS I THINK IT WAS DEPRECATED
        Shuffleboard.getTab("swervetest").addNumber("angleMotorAbsEncoder Reading " + moduleNumber, angleMotor.getAnalog()::getVoltage);
    }

   
   
    private void setAngle(Positions targetposition) {
        // Prevent rotating module if speed is less then 1%. Prevents jittering.
       angleController.setReference(targetposition.position, ControlType.kPosition);
    }
     private void configAngleEncoder() {
         CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
         angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCanCoderConfig);
    }

 
public Rotation2d getAngle() {
        //SmartDashboard.putNumber("getAngleCall position Mod" + moduleNumber, (angleMotor.getSelectedSensorPosition()/1023)*360-angleOffset.getDegrees());
        //System.out.println("Encoder Position Mod "+moduleNumber+": "+(angleMotor.getSelectedSensorPosition()/1023)*360);
        //return Rotation2d.fromDegrees(((angleMotor.getSelectedSensorPosition()/1023)*360)-angleOffset.getDegrees());
        return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
        //return Rotation2d.fromDegrees(getCanCoder().getDegrees() - angleOffset.getDegrees());
    } 
 
 //implement later
private void configAngleMotor() {
       
       //2025 This is Deprecated angleMotor.restoreFactoryDefaults();
        
       ReduceCANUsage.Spark_Max.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly,angleConfig);
       angleConfig.smartCurrentLimit(ANGLE_CURRENT_LIMIT) ;
       // replaced above angleMotor.setSmartCurrentLimit(ANGLE_CURRENT_LIMIT);
       angleConfig.inverted(ANGLE_INVERT); 
       // replaced above angleMotor.setInverted(ANGLE_INVERT);
       angleConfig.idleMode(ANGLE_IDLE_MODE); 
       // replaced above angleMotor.setIdleMode(ANGLE_IDLE_MODE);
        angleConfig.encoder.positionConversionFactor(ANGLE_POSITION_CONVERSION_FACTOR);
        // replaced above integratedAngleEncoder.setPositionConversionFactor(ANGLE_POSITION_CONVERSION_FACTOR);
        angleConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(ANGLE_PID_P, ANGLE_PID_I, ANGLE_PID_D, ANGLE_PID_FF)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, ANGLE_POSITION_CONVERSION_FACTOR);
        /* Replaced above 
        angleController.setP(ANGLE_PID_P);
        angleController.setI(ANGLE_PID_I);
        angleController.setD(ANGLE_PID_D);
        angleController.setFF(ANGLE_PID_FF);*/
       // angleController.setFeedbackDevice(angleMotor.getAnalog(SparkAnalogSensor.Mode.kAbsolute));
        angleConfig.voltageCompensation(VOLTAGE_COMPENSATION);
       //REPLACED ABOVE angleMotor.enableVoltageCompensation(VOLTAGE_COMPENSATION);
       angleMotor.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
 
       //angleMotor.burnFlash();
        Timer.delay(2);
        resetToAbsolute();
    }
public void resetToAbsolute() {
        System.out.println("Encoder "+moduleNumber+ " is set to Absolute Position");
        System.out.println("The offset is "+ angleOffset.getRotations());
        System.out.println("The Absolute Position is "+ angleEncoder.getAbsolutePosition().getValueAsDouble());
        double absolutePosition = angleEncoder.getAbsolutePosition().getValueAsDouble()-angleOffset.getRotations();
        if(absolutePosition<0){
            absolutePosition = 1+absolutePosition;
        }
        System.out.println("The Integrated encoder is reading: "+integratedAngleEncoder.getPosition());
        
        Timer.delay(2);

        integratedAngleEncoder.setPosition(0);
        Timer.delay(2);

        System.out.println("Now the Integrated encoder is reading: "+integratedAngleEncoder.getPosition());


    }

}




