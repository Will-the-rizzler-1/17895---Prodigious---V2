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
    HardwareMap hardwareMap;
    Pose2d pose;
    public Collector collector;
    public LimeLight limelight;
    public Shooter shooter;
    public Whisk whisk;

    public PinpointDrive drive;


    private ArrayList<Component> subsystem;
    public BrainSTEMRobot(HardwareMap hardwareMap, Telemetry telemetry, Pose2d pose) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.pose = pose;
        subsystem = new ArrayList<>();
        collector = new Collector(hardwareMap, telemetry);
        limelight = new LimeLight(hardwareMap, telemetry);
        drive = new PinpointDrive(hardwareMap, telemetry,pose);
        shooter = new Shooter(hardwareMap, telemetry, this);
        whisk = new Whisk(hardwareMap, telemetry);
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
    }
}
