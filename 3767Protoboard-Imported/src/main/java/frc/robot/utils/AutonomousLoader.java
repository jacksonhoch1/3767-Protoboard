package frc.robot.utils;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Protoboard;

import frc.robot.commands.auton.*;

public class AutonomousLoader {
    Map<Route, Command> autoRoutes = new HashMap<>();

    private SendableChooser chooser;

    public AutonomousLoader(Protoboard protoboard, Map<String, Trajectory> paths) {
        autoRoutes.put(Route.MobilityEngage, new Test1(protoboard));

        this.chooser = createSendableChooser();
    }

    private SendableChooser createSendableChooser() {
        SendableChooser chooser = new SendableChooser();
        for (Map.Entry<Route, Command> e : autoRoutes.entrySet()) {
            chooser.addOption(e.getKey().name(), e.getKey());
        }

        return chooser;
    }

    public SendableChooser getSendableChooser() {
        return chooser;
    }

    public Command getCurrentSelection() {
        return autoRoutes.get(chooser.getSelected());
    }

    public enum Route {
        MobilityEngage
    }
}
