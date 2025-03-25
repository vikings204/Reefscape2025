package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.servohub.config.ServoChannelConfig;
import com.revrobotics.servohub.config.ServoHubConfig;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.ColorSensorV3;

public class TongueSubsystem extends SubsystemBase {
    ServoHubConfig config = new ServoHubConfig();
    ServoHub hub = new ServoHub(3);
    ServoChannel pegServo;
    ServoChannel extenderServo;
    private final ColorSensorV3 sensor1;


    public TongueSubsystem() {
        config.channel0
                .pulseRange(500, 1500, 2500)
                .disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kSupplyPower);
        config.channel1
                .pulseRange(500, 1500, 2500)
                .disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kSupplyPower);
        hub.configure(config, ServoHub.ResetMode.kResetSafeParameters);
        hub.setBankPulsePeriod(ServoHub.Bank.kBank0_2, 5000);

        pegServo = hub.getServoChannel(ChannelId.kChannelId0);
        pegServo.setPowered(true);
        pegServo.setEnabled(true);

        extenderServo = hub.getServoChannel(ChannelId.kChannelId1);
        extenderServo.setPowered(true);
        extenderServo.setEnabled(true); // moved from extend(), should work
        sensor1 = new ColorSensorV3(I2C.Port.kOnboard);
        //sensor1.configureProximitySensor(ColorSensorV3.ProximitySensorResolution.kProxRes11bit, ColorSensorV3.ProximitySensorMeasurementRate.kProxRate6ms);

    }

    public void readSensor(){
  Color detectedColor = sensor1.getColor();

    /**
     * The sensor returns a raw IR value of the infrared light detected.
     */
    double IR = sensor1.getIR();

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("IR", IR);

    /**
     * In addition to RGB IR values, the color sensor can also return an 
     * infrared proximity value. The chip contains an IR led which will emit
     * IR pulses and measure the intensity of the return. When an object is 
     * close the value of the proximity will be large (max 2047 with default
     * settings) and will approach zero when the object is far away.
     * 
     * Proximity can be used to roughly approximate the distance of an object
     * or provide a threshold for when an object is close enough to provide
     * accurate color values.
     */
    int proximity = sensor1.getProximity();

    SmartDashboard.putNumber("Proximity", proximity);

    }
    public void setPosScore() {
        
        int pw = pegServo.getPulseWidth();
        //int pw = 1350;
        while (pw > 645) {
            pegServo.setPulseWidth(pw);
            pw -= .025;
            Timer.delay(.0015);
        }
    }

    public void setPosCarrying() {
        pegServo.setPulseWidth(1665);
    }

    public void setPosAuto() {
        pegServo.setPulseWidth(900);
    }

    public void setPosReceive() {
        pegServo.setPulseWidth(1050);
    }
public void setPosL4() {
        pegServo.setPulseWidth(1505);
    }
    public void extend() {
        extenderServo.setPulseWidth(1008);
    }

    public void retract() {
        extenderServo.setPulseWidth(500);
    }

}
