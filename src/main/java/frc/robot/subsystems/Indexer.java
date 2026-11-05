package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import frc.robot.Robot;

public class Indexer extends SubsystemBase {
    private final FlywheelMechanism<?> m_wheel_left;
    private final FlywheelMechanism<?> m_wheel_right;
    private final FlywheelMechanism<?> m_wheel_bottom;
    private final Timer switch_timer;

    public Indexer(FlywheelMechanism<?> left_wheel, FlywheelMechanism<?> right_wheel, FlywheelMechanism<?> bottom_wheel) {
        m_wheel_left = left_wheel;
        m_wheel_right = right_wheel;
        m_wheel_bottom = bottom_wheel;
        switch_timer = new Timer();
    }

    
    public Command runWheelsPerColor(Supplier<Color> color) {
        return Commands.run(() -> {
            double modifier;
            Color alliance = Color.kBlue;
            // This code works
            if (DriverStation.getAlliance().isPresent()) {
                if (DriverStation.getAlliance().get() == Alliance.Blue) {
                    alliance = Color.kBlue;
                    switch_timer.start();
                }
                else if (DriverStation.getAlliance().get() == Alliance.Blue) {
                    if (switch_timer.isRunning() && switch_timer.get() == 1.0)
                    switch_timer.stop();
                    alliance = Color.kRed;
                }
            }
            else alliance = Color.kBlue;
            if (color.get().equals(alliance)) {
                modifier = 1.0;
            }
            else modifier = -1.0;
            int speed = 23000;
            AngularVelocity velo = AngularVelocity.ofBaseUnits(speed, DegreesPerSecond);
            m_wheel_left.runVelocity(velo.times(modifier), PIDSlot.SLOT_0);
            m_wheel_right.runVelocity(velo.times(-1.0), PIDSlot.SLOT_0);
            m_wheel_bottom.runVelocity(velo, PIDSlot.SLOT_0);
        }, this);
    }

    @Override
    public void periodic() {
        super.periodic();
        m_wheel_left.periodic();
        m_wheel_right.periodic();
        m_wheel_bottom.periodic();
    }
}
