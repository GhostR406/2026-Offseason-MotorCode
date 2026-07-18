package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.mechanisms.rotary.RotaryMechanism;
import frc.lib.util.LoggerHelper;
import frc.robot.Constants.DoubleMotorConstants;
import frc.robot.Constants.SingleMotorConstants;

public class Intake extends SubsystemBase {
    private final FlywheelMechanism m_roller;
    private final RotaryMechanism m_pivot;

    public Intake(FlywheelMechanism roller, RotaryMechanism pivot) {
        m_roller = roller;
        m_pivot = pivot;
    }

    // Roller Methods
    public Command rollerSetVelocity(SingleMotorConstants.Setpoint setpoint)
    {

        return this.run(
            () -> m_roller.runVelocity(setpoint.getSetpoint(),
                SingleMotorConstants.ACCELERATION, PIDSlot.SLOT_1))
            .withName("Go To "  + setpoint.name() + " Setpoint - "+setpoint.getSetpoint().in(RotationsPerSecond));
            
    }
    public Command rollerCoast()
    {

        return this.run(
            () -> m_roller.runCoast() )
            .withName("Coasting");
            
    }
    public Command rollerSetpoint(Angle postion)
    {
        return this.runOnce(
            () -> m_roller.runPosition(postion, PIDSlot.SLOT_0))
            .withName("Go To "  + postion.in(Rotation) + " Setpoint");
    }
    public AngularVelocity rollerGetVelocity()
    {
        return m_roller.getVelocity();
    }

    // Pivot Methods
    public Command pivotSetpoint(DoubleMotorConstants.Setpoint setpoint)
    {
        return this.runOnce(
            () -> m_pivot.runPosition(setpoint.getSetpoint(), PIDSlot.SLOT_0))
            .withName("Go To "  + setpoint.name() + " Setpoint");
    };

    // Intake Methods
    public Command stow() {
        return rollerCoast()
        .alongWith(pivotSetpoint(DoubleMotorConstants.DEFAULT_SETPOINT))
        .withName("Stow");
    }

    public Command intake() {
        return rollerSetVelocity(SingleMotorConstants.Setpoint.INTAKE)
        .alongWith(pivotSetpoint(DoubleMotorConstants.Setpoint.RAISED))
        .withName("Intake");
    }

    public Command unjam() {
        return rollerSetVelocity(SingleMotorConstants.Setpoint.UNJAM)
        .alongWith(pivotSetpoint(DoubleMotorConstants.Setpoint.RAISED))
        .withName("Unjam");
    }

    public Command turbob() {
        return rollerSetVelocity(SingleMotorConstants.Setpoint.FUNBOB)
        .alongWith(pivotSetpoint(DoubleMotorConstants.Setpoint.FUNBOB))
        .withName("TURBOB");
    }

    @Override
    public void periodic()
    {
        LoggerHelper.recordCurrentCommand("Intake", this);

        m_pivot.periodic();
        Logger.recordOutput(
            "3D_Feild/2_DoubleMotor",
            new Pose3d[] {new Pose3d(-0.127,0,0.1143, new Rotation3d(0, m_pivot.getPosition().in(Radian), 0))}
        );

        m_roller.periodic();
            Logger.recordOutput("3D_Feild/0_Zero",
            new Pose3d[] {new Pose3d(0,0,0, new Rotation3d(0, 0, -0))}
        );
        Logger.recordOutput(
            "3D_Feild/1_SingleWheel",
            new Pose3d[] {new Pose3d(0.04445,-0.127,0.6223,new Rotation3d(0, 0, m_roller.getPosition().in(Radian)))}
        );
        
    }
}
