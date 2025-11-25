package org.firstinspires.ftc.teamcode.teleop.subsystem;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.teleop.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.teleop.Component;

@Config
public class Shooter implements Component {

    Telemetry telemetry;

    HardwareMap map;
    public ServoImplEx hood;

    public enum ShooterState {
        SHOOTBASE,
        SHOOT90,
        SHOOT80,
        OFF
    }

    public ShooterState shooterState;
    public DcMotorEx shooterMotorL;
    public DcMotorEx shooterMotorR;
    public BrainSTEMRobot robot;

    public static class Params {
        public double SHOOTER_POWER = 0.70;
        public double HOODCLOSE = 1400;
        public double HOODFAR = 650;
        public double HOODINC = 0.05;
        public double SHOOT37 = 0.55;
    }

    public static Params SHOOTER_PARAMS = new Params();

    @SuppressLint("NotConstructor")
    public Shooter(HardwareMap map, Telemetry telemetry, BrainSTEMRobot robot) {
        this.map = map;
        this.telemetry = telemetry;

        shooterMotorL = map.get(DcMotorEx.class, "shooterL");
        shooterMotorR = map.get(DcMotorEx.class, "shooterR");
        hood = map.get(ServoImplEx.class, "hood");
        hood.setPwmRange(new PwmControl.PwmRange(SHOOTER_PARAMS.HOODCLOSE, SHOOTER_PARAMS.HOODFAR));
        shooterState = ShooterState.OFF;
    }

    @Override
    public void reset() {
        // nothing yet
    }

    public void update() {
//        Pose3D botPose = robot.limelight.update();
//        robot.drive.pinpoint.setPosition(new Pose2d(
//                botPose.getPosition().x,
//                botPose.getPosition().y,
//                botPose.getOrientation().getYaw(AngleUnit.RADIANS))
//        );
        switch (shooterState) {
            case OFF:
                shooterOff();
                break;
            case SHOOTBASE:
                shooterSpeed();
                break;
            case SHOOT90:
                shooterSpeed90();
                break;
            case SHOOT80:
                shooterSpeed80();
                break;
        }

        double velocityTicksPerSecondL = shooterMotorL.getVelocity();
        double velocityTicksPerSecondR = shooterMotorR.getVelocity();


        telemetry.addData("Shooter Power L", getShooterPowerL());
        telemetry.addData("Shooter Power R", getShooterPowerR());
        telemetry.addData("Velocity L (ticks/s)", velocityTicksPerSecondL);
        telemetry.addData("Velocity R (ticks/s)", velocityTicksPerSecondR);
        telemetry.addData("hoodpos", getHoodPos());
    }

    public void setShooterPower(double power) {
        shooterMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotorL.setPower(power);
        shooterMotorR.setPower(power);
    }

    @Override
    public String test() {
        return "";
    }

    public double getShooterPowerL() {
        return shooterMotorL.getPower();
    }

    public double getShooterPowerR() {
        return shooterMotorR.getPower();
    }

    public double getHoodPos() {
        return hood.getPosition();
    }

    private void shooterOff() {
        shooterMotorL.setPower(0.0);
        shooterMotorR.setPower(0.0);
    }

    private void shooterSpeed() {
        shooterMotorL.setPower(-SHOOTER_PARAMS.SHOOTER_POWER);
        shooterMotorR.setPower(SHOOTER_PARAMS.SHOOTER_POWER);
    }

    private void shooterSpeed90() {
        shooterMotorL.setPower(-SHOOTER_PARAMS.SHOOT37);
        shooterMotorR.setPower(SHOOTER_PARAMS.SHOOT37);
    }

    private void shooterSpeed80() {
        shooterMotorL.setPower(-0.70);
        shooterMotorR.setPower(0.70);
    }

    public void HoodFarPos() {
        hood.setPosition(0.99);
    }
    public void HoodInc() {
        hood.setPosition(hood.getPosition() + SHOOTER_PARAMS.HOODINC);
    }
    public void HoodDec() {
        hood.setPosition(hood.getPosition() - SHOOTER_PARAMS.HOODINC);
    }

    public void HoodClosePos() {
        hood.setPosition(0.01);
    }

    public void setShoot() {
        shooterState = ShooterState.SHOOTBASE;
    }

    public void setOff() {
        shooterState = ShooterState.OFF;
    }

    public void setShoot90() {
        shooterState = ShooterState.SHOOT90;
    }

    public void setShoot80() {
        shooterState = ShooterState.SHOOT80;
    }
}