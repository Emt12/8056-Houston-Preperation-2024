package frc.robot.commands.CombinedCommands;


import frc.robot.commands.ShooterCommands.WristPIDCmd;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;


public class Stabilize extends ParallelCommandGroup {
    
    Shooter shooter;
    double distance;
    public Stabilize(Shooter shooter){
        this.shooter = shooter;

        double distance = shooter.getDistance();

        this.distance = distance;

        addCommands(
            new WristPIDCmd(shooter, distance)
        );
    }
}
