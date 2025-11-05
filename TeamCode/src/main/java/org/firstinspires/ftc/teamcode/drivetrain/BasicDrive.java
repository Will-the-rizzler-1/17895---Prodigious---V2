package org.firstinspires.ftc.teamcode.drivetrain;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleop.Component;


public class BasicDrive implements Component {

    Telemetry telemetry;
    HardwareMap map;

    public MecanumDrive drive;

    private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    double leftStickX, leftStickY, rightStickX;

    public BasicDrive(HardwareMap map, Telemetry telemetry){
        this.map = map;
        this.telemetry = telemetry;

        frontLeftMotor = map.get(DcMotor.class, "fl");
        frontRightMotor = map.get(DcMotor.class, "fr");
        backLeftMotor = map.get(DcMotor.class, "bl");
        backRightMotor = map.get(DcMotor.class, "br");

        drive = new MecanumDrive(map, new Pose2d(0,0,0));
    }
    private double driveSpeed = 0.99;  // Default speed multiplier

    // Initialize motors

    // Set motor directions (adjust based on your robot)
//    frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//    frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//    backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//    backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

    // Set motor modes
//    setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    public void setInputs(double left_stick_x, double left_stick_y, double right_stick_x) {
        leftStickX = left_stick_x;
        leftStickY = left_stick_y;
        rightStickX = left_stick_x;
    }

    @Override
    public void reset() {

    }

    @Override
    public void update() {
//        drive.setDrivePowers(new PoseVelocity2d(
//                new Vector2d(
//                        -leftStickY,
//                        -leftStickX
//                ),
//                -rightStickX
//        ));

        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
    }

    @Override
    public String test() {
        return "passed";
    }
}
