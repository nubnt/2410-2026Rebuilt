package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{

    private SparkMax intakeRoller = new SparkMax(0, MotorType.kBrushless);

    /**
     * Spins intake roller to accept balls into the robot.
     */
    public void intake(){
        intakeRoller.set(1);
    }
    /**
     * Spins intake roller to unload balls currently inside the robot.
     */
    public void outake(){
        intakeRoller.set(-1);
    }

    /**
     * Stops the intake roller.
     */
    public void stop(){
        intakeRoller.set(0);
    }

}
