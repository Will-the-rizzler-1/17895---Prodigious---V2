package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.teamcode.drivetrain.PinpointDrive;
import org.firstinspires.ftc.teamcode.teleop.BrainSTEMRobot;

@Config
@Disabled
@Autonomous(name="BlueSide", group="BlueSideAuto")
public class BlueSide extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Pose2d BeginPose = new Pose2d(64.5, 16.5, Math.toRadians(0));
        Pose2d ShootPose = new Pose2d(36,40, Math.toRadians(120));

        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, BeginPose);
        PinpointDrive drive = robot.drive;
        AutoCommands autoCommands = new AutoCommands(robot, telemetry);

        TrajectoryActionBuilder depositPreloadTrajectory = drive.actionBuilder(BeginPose)
                .setReversed(true)
                .splineToLinearHeading(ShootPose, Math.toRadians(235));




    }
    }