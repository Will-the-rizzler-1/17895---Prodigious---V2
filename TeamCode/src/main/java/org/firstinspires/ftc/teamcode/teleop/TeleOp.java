package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teleop.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.util.GamepadTracker;
import org.firstinspires.ftc.teamcode.util.PIDController;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp", group="Tele")
public class TeleOp extends LinearOpMode {

    public double goalX = 0, goalY = 0;
    public static double kP = 0.1, kI = 0, kD = 0;
    public PIDController pid = new PIDController(kP, kI, kD);

    GamepadTracker gp1;

    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry);

        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, new Pose2d(0, 0, 0));
        gp1 = new GamepadTracker(gamepad1);

        waitForStart();

        while (opModeIsActive()) {
            robot.update();
            updateDrive(robot);
            updateDriver1(robot);
            gp1.update();
            robot.whisk.getWhiskTelemetry();
            telemetry.update();

        }
    }

    private void updateDriver1(BrainSTEMRobot robot) {
        driver1CollectorControls(robot);
    }

    private void updateDrive(BrainSTEMRobot robot) {
        if (gamepad1.dpad_down) {
            double targetAngle = Math.atan2(goalY - robot.drive.pinpoint.getVelY(DistanceUnit.INCH), goalX - robot.drive.pinpoint.getVelX(DistanceUnit.INCH));
            double robotHeading = robot.drive.pinpoint.getHeading(AngleUnit.RADIANS);
            pid.setTarget(targetAngle);
            double power = pid.update(robotHeading);
            robot.drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -power
            ));
        }
        else {
            robot.drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));
        }

    }

    private void driver1CollectorControls(BrainSTEMRobot robot) {

                if (gamepad1.left_trigger>0.1) {
                    robot.shooter.HoodClosePos();
                }

                if (gamepad1.right_trigger>0.1) {
                    robot.shooter.HoodFarPos();
                }
                if (gamepad1.leftBumperWasPressed()) {
                    robot.shooter.HoodDec();
                }
                if (gamepad1.yWasPressed()) {
                    robot.shooter.HoodInc();
                }


                if (gamepad1.dpad_up) {
                    robot.whisk.LiftFlick();
                }


                if (gp1.isFirstDpadLeft()) {// rotates 60 counter clockwise
                    robot.whisk.incWhiskPos();
                } else if (gp1.isFirstDpadRight()) {
                    robot.whisk.decWhiskPos();
                }


                if (gamepad1.right_bumper) {
                    robot.collector.setIn();
                } else {
                    robot.collector.setOff();
                }

                if (gamepad1.a) {
                    if (robot.shooter.shooterState == Shooter.ShooterState.OFF) {
                        robot.shooter.setShoot();
                    } else {
                        robot.shooter.setOff();
                    }
                }

                if (gamepad1.b && (robot.shooter.shooterState == Shooter.ShooterState.OFF || robot.shooter.shooterState == Shooter.ShooterState.SHOOTBASE)) {
                    robot.shooter.setShoot90();
                }
                if (gamepad1.x && (robot.shooter.shooterState == Shooter.ShooterState.OFF || robot.shooter.shooterState == Shooter.ShooterState.SHOOTBASE || robot.shooter.shooterState == Shooter.ShooterState.SHOOT90)) {
                    robot.shooter.setShoot80();
                }


            }
        }