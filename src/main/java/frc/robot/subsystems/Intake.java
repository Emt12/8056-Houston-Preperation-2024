package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.CanIds;

public class Intake extends SubsystemBase{

    private CANSparkMax mRoller = new CANSparkMax(CanIds.kRollerCanId, MotorType.kBrushless);

    public Intake(){}

    /**
     * Starts the movement of the roller at given speed
     * @param speed
     *      *(+)  -> In
     *      *(-)  -> Out
     */ 
    public void setRoller(double speed){
        mRoller.set(speed);
        
    }


    @Override
    public void periodic() {

    }
}
