package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;

//Commad Class
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

//Subsystems
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.commands.ShooterCommands.IndexCmd;

//Custom Library
import frc.robot.custom.MZ80;
import frc.robot.custom.NamedCommandSolver;
//Commands
import frc.robot.commands.ShooterCommands.ShootCmd;
import frc.robot.commands.ShooterCommands.WristPIDCmd;
import frc.robot.commands.DriveCommands.ResetRelative;
import frc.robot.commands.DriveCommands.SetX;
import frc.robot.commands.IntakeCommands.IntakeCmd;
import frc.robot.commands.IntakeCommands.MZ80Cmd;
import frc.robot.commands.CombinedCommands.Amp;
//Combined Commands
import frc.robot.commands.CombinedCommands.Shoot;
import frc.robot.commands.CombinedCommands.Spit;
import frc.robot.commands.CombinedCommands.Stabilize;
import frc.robot.commands.CombinedCommands.Take;

public class RobotContainer {
    
    NamedCommandSolver solver = new NamedCommandSolver(Commands.print("enes"), Commands.print("murat"));
    SendableChooser<Command> autoChooser;

    //Custom Library
    MZ80    mz80                            = new MZ80();

    //Subsystems
    Swerve  swerve                          = new Swerve();
    Shooter shooter                         = new Shooter();
    Intake  intake                          = new Intake();
    Index   index                           = new Index();
    Vision  vision                          = new Vision();


    //Joysticks
    XboxController driverJoystick           = new XboxController(0);
    Joystick       operatorJoystick         = new Joystick(1);
    Joystick       testJoystick             = new Joystick(2);
    boolean        useTest                  = false; // if true use test joystick

    //Driver Button Definitions
    JoystickButton driverA                  = new JoystickButton(driverJoystick, 1);
    JoystickButton driverB                  = new JoystickButton(driverJoystick, 2);
    JoystickButton driverX                  = new JoystickButton(driverJoystick, 3);
    JoystickButton driverY                  = new JoystickButton(driverJoystick, 4);
    JoystickButton driverRB                 = new JoystickButton(driverJoystick, 5);
    

    
    //Operator Button Definitions
    JoystickButton operatorSquare           = new JoystickButton(operatorJoystick, 1);
    JoystickButton operatorX                = new JoystickButton(operatorJoystick, 2);
    JoystickButton operatorCircle           = new JoystickButton(operatorJoystick, 3);
    JoystickButton operatorTriangle         = new JoystickButton(operatorJoystick, 4);
    JoystickButton operatorRB               = new JoystickButton(operatorJoystick, 5);
    
    
    //Test Button Definitions
    JoystickButton testA                    = new JoystickButton(testJoystick, 1);
    JoystickButton testB                    = new JoystickButton(testJoystick, 2);
    JoystickButton testX                    = new JoystickButton(testJoystick, 3);
    JoystickButton testY                    = new JoystickButton(testJoystick, 4);
    JoystickButton testRB                   = new JoystickButton(testJoystick, 5);
 
    //Commands
    //--Shoot
    ShootCmd       ShootCmd                 = new ShootCmd(shooter, index);
    IndexCmd       IndexCmdInShoot          = new IndexCmd(index, .9);
    IndexCmd       IndexCmdOutSlow          = new IndexCmd(index, -.1);
    IntakeCmd      IntakeFeed               = new IntakeCmd(intake, .9);
    //--Spit
    IntakeCmd      IntakeGiveSpit           = new IntakeCmd(intake, -.9);
    IndexCmd       IndexCmdOutSpit          = new IndexCmd(index, -.9);
    //--Take
    IndexCmd       IndexCmdInTake           = new IndexCmd(index, .9);
    IntakeCmd      IntakeTake               = new IntakeCmd(intake, .9);
    MZ80Cmd        MZ80Cmd                  = new MZ80Cmd(mz80);
    //--Drive
    ResetRelative  ResetRelative            = new ResetRelative(swerve);
    SetX           SetX                     = new SetX(swerve);
    //--Amp
    IndexCmd       indexCmdOutAmp           = new IndexCmd(index, -.9);
    WristPIDCmd    wristUp                  = new WristPIDCmd(shooter, 200);//milimeter
    WristPIDCmd    wristDown                = new WristPIDCmd(shooter, 0);//milimeter
    
    //Combined Commands
    Shoot          Shoot                    = new Shoot(IndexCmdInShoot ,IndexCmdOutSlow, ShootCmd, IntakeFeed);
    Take           Take                     = new Take(IntakeTake, IndexCmdInTake, MZ80Cmd);
    Spit           Spit                     = new Spit(IntakeGiveSpit, IndexCmdOutSpit);
    Stabilize      Stabilize                = new Stabilize(shooter);
    Amp            Amp                      = new Amp(indexCmdOutAmp, wristUp, wristDown);

    //Robot
    Robot          robot                    = new Robot();



    public RobotContainer() {

        autoChooser = AutoBuilder.buildAutoChooser();
        configureButtonBindings();
        SmartDashboard.putData("Auto Mode", autoChooser);

        shooter.setDefaultCommand(
                new RunCommand(
                        () -> shooter.driveWithJoystick(
                                -MathUtil.applyDeadband(operatorJoystick.getRawAxis(5)/4, OIConstants.kWristDeadband)),
                        shooter)
        );               
                        
        if(useTest){
            swerve.setDefaultCommand(
                new RunCommand(
                        () -> swerve.drive(
                                -MathUtil.applyDeadband(testJoystick.getRawAxis(5) / 1.5, OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(testJoystick.getRawAxis(4) / 1.5, OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband((testJoystick.getRawAxis(3) - testJoystick.getRawAxis(2)) / 1.5, OIConstants.kDriveDeadband),
                                true, true),
                        swerve)
            );

        }
        else{
            swerve.setDefaultCommand(
                new RunCommand(
                        () -> swerve.drive(
                                -MathUtil.applyDeadband(driverJoystick.getRightY() / 1.5, OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(driverJoystick.getRightX() / 1.5, OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband((driverJoystick.getRightTriggerAxis() - driverJoystick.getLeftTriggerAxis()) / 1.5, OIConstants.kDriveDeadband),
                                true, true),
                        swerve)
            );
        }
    }

    private void configureButtonBindings() {

        // Driver
        if(useTest){
            testX.whileTrue(SetX);
            testRB.whileTrue(ResetRelative);

        }
        else{
            driverX.whileTrue(SetX);
            driverRB.whileTrue(ResetRelative);
        }
        
        // Operator
        operatorX.whileTrue(Take);
        operatorCircle.whileTrue(Spit);
        operatorTriangle.whileTrue(Shoot);
        operatorSquare.whileTrue(Amp);
        operatorRB.whileTrue(Stabilize);

    }
    public void registerNamedCommands(){
        NamedCommands.registerCommand("p1", Commands.print("print it 1"));
        NamedCommands.registerCommand("p1", Commands.print("print it 1"));
    }

    /**
     * Just for disappearing the error: <b>m_robotContainer not used</b>
     */
    public void errorSolver() {}

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
