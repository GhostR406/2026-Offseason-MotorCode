package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.mechanisms.rotary.RotaryMechanism;

public class Turret extends SubsystemBase {
    private final RotaryMechanism<?, ?> m_base;
    private final RotaryMechanism<?, ?> m_head;
    private final FlywheelMechanism<?> m_shooter;



    public Turret(RotaryMechanism<?, ?> base, RotaryMechanism<?, ?> head, FlywheelMechanism<?> shooter) {
        this.m_base = base;
        this.m_head = head;
        this.m_shooter = shooter;
    }

    public Command auto_shoot(Supplier<Angle> base_angle, Supplier<Angle> head_angle, Supplier<AngularVelocity> shooter_speed) {
        return Commands.run(() -> {
            m_base.runPosition(angle_filter(base_angle.get()), PIDSlot.SLOT_0);
            m_head.runPosition(angle_filter(head_angle.get()), PIDSlot.SLOT_0);
            m_shooter.runVelocity(shooter_speed.get(), PIDSlot.SLOT_0);
        }, this);
    }

    public Angle angle_filter(Angle theta) {
        double thetaDegreesWrapped = theta.in(Degrees)%360;
        return thetaDegreesWrapped > 0 ?
        Angle.ofBaseUnits(thetaDegreesWrapped, Degrees) : Angle.ofBaseUnits(360+thetaDegreesWrapped, Degrees);
    }

    public Command coast_shooter() {
        return Commands.run(() -> {
            m_shooter.runCoast();
        }, this);
    }

    @Override
    public void periodic() {
        super.periodic();
        m_base.periodic();
        m_head.periodic();
        m_shooter.periodic();
    }

    class ShooterCALculator {

    }
}
