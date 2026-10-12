package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;
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
    private final FlywheelMechanism<?> m_roller;
    private final RotaryMechanism<?,?> m_pivot;

    public Intake(FlywheelMechanism<?> roller, RotaryMechanism<?,?> pivot) {
        m_roller = roller;
        m_pivot = pivot;
    }

    // Roller Methods
    public Command rollerSetVelocity(SingleMotorConstants.Setpoint setpoint)
    {

        return this.run(
            () -> m_roller.runVelocity(setpoint.getSetpoint(),
                SingleMotorConstants.ACCELERATION, PIDSlot.SLOT_3))
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
            () -> m_roller.runUnprofiledPosition(postion, PIDSlot.SLOT_3))
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
            () -> m_pivot.runUnprofiledPosition(setpoint.getSetpoint(), PIDSlot.SLOT_0))
            .withName("Go To "  + setpoint.name() + " Setpoint");
    };

    // Intake Methods
    public Command stow() {
        return this.run(() -> {
            m_pivot.runUnprofiledPosition(DoubleMotorConstants.DEFAULT_SETPOINT.getSetpoint(), PIDSlot.SLOT_0);
            m_roller.runCoast();
        })
        .withName("Stow");
    }

    public Command intake() {
        return this.run(() -> {
            m_roller.runVelocity(SingleMotorConstants.Setpoint.INTAKE.getSetpoint(), PIDSlot.SLOT_3);
            m_pivot.runUnprofiledPosition(DoubleMotorConstants.Setpoint.RAISED.getSetpoint(), PIDSlot.SLOT_0);
        })
        .withName("Intake");
    }

    public Command unjam() {
        return this.run(() -> {
            rollerSetVelocity(SingleMotorConstants.Setpoint.UNJAM);
            pivotSetpoint(DoubleMotorConstants.Setpoint.RAISED);
        })
        .withName("Unjam");
    }

    public Command turbob() {
        return this.run(() -> {
            rollerSetVelocity(SingleMotorConstants.Setpoint.FUNBOB);
            pivotSetpoint(DoubleMotorConstants.Setpoint.FUNBOB);
        })
        .withName("TURBOB");
    }

    @Override
    public void periodic()
    {
        LoggerHelper.recordCurrentCommand("Intake", this);

        m_pivot.periodic();

        m_roller.periodic();
        

        Logger.recordOutput("3D_Feild/0_2022_RobotPose",
            new Pose3d[] {
                new Pose3d(3.75,3.25,0.075,
                    new Rotation3d(0, 0, 0)
                )
            }
        );
        Logger.recordOutput(
            "3D_Feild/1_2022_Turret_Yaw",
            new Pose3d[] {
                new Pose3d(-0.0762,0,0,
                    new Rotation3d(0, 0, m_roller.getPosition().in(Radians))
            )
            }
        );
        Logger.recordOutput(
            "3D_Feild/2_2022_Turret_Pitch",
            new Pose3d[] {
                new Pose3d(-0.0775-0.0975*Math.cos(m_roller.getPosition().in(Radians)),-0.0975*Math.sin(m_roller.getPosition().in(Radians)),0.48, new Rotation3d(0, m_pivot.getPosition().in(Radian), m_roller.getPosition().in(Radians)))});
        Logger.recordOutput("3D_Feild/3_2022_Intake", new Pose3d[] {new Pose3d(0.19,0,0.37, new Rotation3d(0, m_pivot.getPosition().in(Radians), 0))});
    }
}
