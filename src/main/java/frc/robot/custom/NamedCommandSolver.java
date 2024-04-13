package frc.robot.custom;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;

public class NamedCommandSolver {
    public NamedCommandSolver(Command cmd1, Command cmd2){
        NamedCommands.registerCommand("p1", cmd1);
        NamedCommands.registerCommand("p2", cmd2);
    }
}
