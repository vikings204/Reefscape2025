package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import com.revrobotics.servohub.config.ServoChannelConfig;
import com.revrobotics.servohub.config.ServoHubConfig;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.ServoChannel;

public class TongueSubsystem extends SubsystemBase {
    ServoHubConfig config = new ServoHubConfig();
    ServoHub hub = new ServoHub(3);
    ServoChannel pegServo;
    ServoChannel extenderServo;

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
    }

    public void setPosScore() {
        int pw = 1350;
        while (pw > 700) {
            pegServo.setPulseWidth(pw);
            pw -= .025;
            Timer.delay(.0015);
        }
    }

    public void setPosCarrying() {
        pegServo.setPulseWidth(1600);
    }

    public void setPosReceive() {
        pegServo.setPulseWidth(1000);
    }

    public void extend() {
        extenderServo.setPulseWidth(950);
    }

    public void retract() {
        extenderServo.setPulseWidth(500);
    }

}
