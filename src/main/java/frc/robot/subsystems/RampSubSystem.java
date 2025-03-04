package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import com.revrobotics.servohub.config.ServoChannelConfig;
import com.revrobotics.servohub.config.ServoHubConfig;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.ServoChannel;

public class RampSubSystem extends SubsystemBase{
    ServoHubConfig config = new ServoHubConfig(); 
    ServoHub hub = new ServoHub(3);
    ServoChannel m_channel0;
    ServoChannel m_channel1;

    public RampSubSystem(){
    config.channel0
        .pulseRange(500,1500,2500)
        //.pulseRange(1000,1500,2000)
        .disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kSupplyPower);
    config.channel1
        .pulseRange(500,1500,2500)
        //.pulseRange(1000,1500,2000)
        .disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kSupplyPower);



        // their defaults.
        hub.configure(config, ServoHub.ResetMode.kResetSafeParameters);
        m_channel0 =hub.getServoChannel(ChannelId.kChannelId0);
        hub.setBankPulsePeriod(ServoHub.Bank.kBank0_2, 5000);
        m_channel0.setPowered(true);
        m_channel0.setEnabled(true);
        //m_channel0.setPulseWidth(1500);
        m_channel1 =hub.getServoChannel(ChannelId.kChannelId1);
        m_channel1.setPowered(true);
       // Timer.delay(2);
        //m_channel1.setEnabled(true);
 

    }
    public void setPosScore(boolean b) {
      /*   m_channel0.setPulseWidth(1700);
        Timer.delay(.2);
        m_channel.setPulseWidth(1200);
        Timer.delay(.2);*/
       int pw = 1350 ;
       while (pw>700){
        m_channel0.setPulseWidth(pw);
        pw-=.025;
        Timer.delay(.0015);
       }

    }

    public void setPosCarrying(boolean b) {
        
        m_channel0.setPulseWidth(1600);
    }
    public void setPosReceive(boolean b) {
        m_channel0.setPulseWidth(1000);
    }

    public void extend(){
        m_channel1.setEnabled(true);
        m_channel1.setPulseWidth(950);
    }
    public void retract(){
        m_channel1.setPulseWidth(   500);
    }

}
