package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.SubsystemBase;


/**
 * Shoots balls currently loaded in the machine.
 * @version v0.0.0
 */
public class ShooterSubsystem extends SubsystemBase{
    private SparkMax flywheel = new SparkMax(0, MotorType.kBrushless);
    private SparkMax backRollers = new SparkMax(0, MotorType.kBrushless);
    private SparkMax feedRollers = new SparkMax(0, MotorType.kBrushless);

    /**
     * Shoots at default power level of 50%.
     * @since v0.0.0
     * @version v0.0.0
     */
    public void shoot(){
        flywheel.set(0.5);
        backRollers.set(0.5);
        feedRollers.set(0.5);
    }
}
