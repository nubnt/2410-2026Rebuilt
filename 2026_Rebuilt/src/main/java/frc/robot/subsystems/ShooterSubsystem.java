package frc.robot.subsystems;

import java.util.logging.Logger;
import java.util.logging.Level;

import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.SubsystemBase;


/**
 * Shoots balls currently loaded in the machine.
 * @version v1.0.0
 */
public class ShooterSubsystem extends SubsystemBase{
    private static final Logger logger = Logger.getLogger("ShooterSubsystem");
    private static final double MOTOR_POWER_MULT = Constants.LiterallyTheOnlyShooterConstant.motorPowerConversion;
    private SparkMax flywheel = new SparkMax(0, MotorType.kBrushless);
    private SparkMax backRollers = new SparkMax(0, MotorType.kBrushless);
    private SparkMax feedRollers = new SparkMax(0, MotorType.kBrushless);

    /**
     * Shoots at default power level of 50%.
     * @since v0.0.0
     * @version v1.0.0
     */
    public void shoot(){
        flywheel.set(0.5);
        backRollers.set(0.5);
        setFeedRollers();
    }
    
    /**
     * Shoots balls at manually set power level.
     * @param powerLevel The power level to set the flywheels to, range 0.0-1.0.
     * @since v1.0.0
     * @version v1.0.0
     */
    public void shoot(double powerLevel){
        // Create power level variable that we can actually change, since java no like when parameter changed
        double power = powerlevel;
        
        if(powerLevel > 0){
            logger.info("Power level exceeds maximum output, set to 1.0");
            power = 1;
        }
        if(powerLevel < 0){
            logger.warning("for some reason you passed a negative value so i set the power to 0 since i dont want it to get jammed");
            power = 0;
        }
        
        flywheel.set(powerLevel);
        backRollers.set(powerLevel);
        setFeedRollers();
    }
    
    /**
     * Shoots balls at power level dependent on distance, [insert maximum shooting distance here].
     * @param distance The distance from the thing you're trying to shoot.
     * @since v1.0.0
     * @version v1.0.0
     */
    public void shootAtDistance(double distance){
        double power = Math.sqrt(distance)*MOTOR_POWER_MULT;
        // TODO: add a fancy log here telling you about the maximum distance if you exceed it, and also set power to 1.0
        
        shoot(power);
    }

    /**
     * Sets feedRollers to 0 if not firing so that it doesn't get jammed (will it get jammed otherwise?? idk, probably).
     * @since v1.0.0
     * @version v1.0.0
     */
    public void setFeedRollers(){
        if(flywheel.get() < 0.05 || backRollers.get() < 0.05)
            feedRollers.set(0);
        else
           feedRollers.set(0.5); 
    }
}
