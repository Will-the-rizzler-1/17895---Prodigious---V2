package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drivetrain.BasicDrive;
import org.firstinspires.ftc.teamcode.drivetrain.PinpointDrive;
import org.firstinspires.ftc.teamcode.teleop.subsystem.Collector;
import org.firstinspires.ftc.teamcode.teleop.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.teleop.subsystem.Whisk;

import java.util.ArrayList;

public class BrainSTEMRobot {
    private final Telemetry telemetry;
    HardwareMap map;
    public Collector collector;
    public Shooter shooter;
    public Whisk whisk;

    public PinpointDrive drive;


    private ArrayList<Component> subsystem;
    public BrainSTEMRobot(HardwareMap map, Telemetry telemetry) {
        this.map = map;
        this.telemetry = telemetry;
        subsystem = new ArrayList<>();
        collector = new Collector(map, telemetry);
        drive = new BasicDrive(map, telemetry);
        shooter = new Shooter(map, telemetry);
        whisk = new Whisk(map, telemetry);
        subsystem.add(collector);
        subsystem.add(drive);
        subsystem.add(shooter);
        subsystem.add(whisk);
    }

    public BrainSTEMRobot(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Telemetry telemetry1) {
        this.telemetry = telemetry1;
    }

    public BrainSTEMRobot(Telemetry telemetry, HardwareMap hardwareMap, Pose2d beginPose) {
    }


    public void update() {
        for (Component c : subsystem) {
            c.update();
        }
        telemetry.update();
    }
}
