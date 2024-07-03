package frc.lib.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;

public class DriverInputCleanup {
    public static double modifyMoveAxis(double value) {
            // Deadband
            if(Math.abs(value) < OperatorConstants.kDeadband) {
                return 0;
            }
        
            // Change the axis
            double b = .1;
            double a = .5;
            double x = value;
            if(value >=0) {
                return b + (1-b)*(a*Math.pow(x, 3) + (1-a)*x);
            } else{
                return -b + (1-b)*(a*Math.pow(x, 3) + (1-a)*x);
            }
            //value = Math.copySign(value * value, value);
        
            //return value;
          }
        public static double modifyRotAxis(double value) {
            // Deadband
            if(Math.abs(value) < OperatorConstants.kDeadband) {
                return 0;
            }
        
            // Change the axis
            double b = .05;
            double a = .2;
            if(value >=0) {
                return b + (1-b)*(a*Math.pow(value, 3) + (1-a)*value);
            } else{
                return -b + (1-b)*(a*Math.pow(value, 3) + (1-a)*value);
            }
          }

        public static double[] getDriverInputs(XboxController driver) {
            double[] inputs = new double[3];

            inputs[0] = modifyMoveAxis(-driver.getRawAxis(OperatorConstants.translationAxis));
            inputs[1] = modifyMoveAxis(-driver.getRawAxis(OperatorConstants.strafeAxis));
            inputs[2] = modifyRotAxis(-driver.getRawAxis(OperatorConstants.rotationAxis));

            inputs[0] = MathUtil.applyDeadband(inputs[0], OperatorConstants.kDeadband);
            inputs[1] = MathUtil.applyDeadband(inputs[1], OperatorConstants.kDeadband);
            inputs[2] = MathUtil.applyDeadband(inputs[2], OperatorConstants.kDeadband);

            int invert =  (Constants.isRed) ? -1 : 1; 

            inputs[0] *= invert;
            inputs[1] *= invert;

            inputs[0] *= SwerveConstants.maxSpeed;
            inputs[1] *= SwerveConstants.maxSpeed;
            inputs[2] *= SwerveConstants.maxAngularVelocity;
            return inputs;
        }
}
