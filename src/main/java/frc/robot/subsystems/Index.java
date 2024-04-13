package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;

public class Index extends SubsystemBase{
    private CANSparkMax mIndexLeft    = new CANSparkMax(CanIds.kIndexLeftCanId, MotorType.kBrushless);
    private CANSparkMax mIndexRight   = new CANSparkMax(CanIds.kIndexRightCanId, MotorType.kBrushless);
    /**
     * Starts the movement of the index motor at given speed
     * @param speed
     *      *(+)  -> In
     *      *(-)  -> Out
     */ 
    public void setIndex(double speed){
        mIndexLeft.set(speed);
        mIndexRight.set(-speed);
    }
}
