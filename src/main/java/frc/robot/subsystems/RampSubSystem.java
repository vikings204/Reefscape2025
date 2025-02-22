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

    public RampSubSystem(){
    config.channel0
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

    }
    public void setPosIn(boolean b) {
      /*   m_channel0.setPulseWidth(1700);
        Timer.delay(.2);
        m_channel0.setPulseWidth(1200);
        Timer.delay(.2);*/
       int pw = 2000 ;
       while (pw>1100){
        m_channel0.setPulseWidth(pw);
        pw-=.5;
        Timer.delay(.001);
       }

    }
        public void setPosiReceive(boolean b) {
        
        m_channel0.setPulseWidth(1400);
    }
    public void setPosOut(boolean b) {
        m_channel0.setPulseWidth(2000);
    }

}
