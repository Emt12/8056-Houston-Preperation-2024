package frc.robot.custom;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class MZ80 extends SubsystemBase{

    DigitalInput dio = new DigitalInput(IntakeConstants.MZ80Port);
    public MZ80(){}

    public boolean get(){
        return dio.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("mz80 dio", get());
        super.periodic();
    }
}
