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
   Telemetry telemetry;
    HardwareMap map;
    Pose2d pose;
    public Collector collector;
    public Shooter shooter;
    public Whisk whisk;

    public PinpointDrive drive;


    private ArrayList<Component> subsystem;
    public BrainSTEMRobot(HardwareMap map, Telemetry telemetry, Pose2d pose) {
        this.map = map;
        this.telemetry = telemetry;
        subsystem = new ArrayList<>();
        collector = new Collector(map, telemetry);
        drive = new PinpointDrive(map, pose);
        shooter = new Shooter(map, telemetry);
        whisk = new Whisk(map, telemetry);
        subsystem.add(collector);
//        subsystem.add(drive);
        subsystem.add(shooter);
        subsystem.add(whisk);
    }


    public void update() {
        for (Component c : subsystem) {
            c.update();
        }
        telemetry.update();
    }
}
