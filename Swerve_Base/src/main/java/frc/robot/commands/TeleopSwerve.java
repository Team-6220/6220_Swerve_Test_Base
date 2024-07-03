package frc.robot.commands;

import frc.lib.util.DriverInputCleanup;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private BooleanSupplier robotCentricSup;
    private XboxController driver;

    public TeleopSwerve(Swerve s_Swerve, XboxController driver, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        this.driver = driver;
        this.robotCentricSup = robotCentricSup;

    }

    @Override
    public void initialize() {
        s_Swerve.resetModulesToAbsolute();
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double[] driverInputs = DriverInputCleanup.getDriverInputs(driver);
        
        /* Drive */
        s_Swerve.drive(
            new Translation2d(driverInputs[0], driverInputs[1]), 
            driverInputs[2], 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}