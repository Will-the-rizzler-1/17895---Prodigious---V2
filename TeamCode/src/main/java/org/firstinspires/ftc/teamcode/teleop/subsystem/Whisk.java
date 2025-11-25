package org.firstinspires.ftc.teamcode.teleop.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleop.Component;
import org.firstinspires.ftc.teamcode.util.PIDController;


@Config
public class Whisk implements Component {

    public ServoImplEx Flick;
    public ElapsedTime FlickTimer;

    private final HardwareMap hardwareMap;

    @Override
    public String test() {
        return "";
    }

    public static class Params {
        public double FlickMax = 0.99;
        public double FlickMin = 0.01;
        public double FlickLow = 620;
        public double FlickBallHeight = 1120;
        public double WhiskKp = 0.0355;
        public double WhiskKd = 0.001;
        public double WhiskKi = 0;
        public double WhiskKs = 0;
        public double MAX_POWER_COUNTER_CLOCKWISE = -0.25;
        public double MAX_POWER_CLOCKWISE = 0.25;
        public double encodersPerRev = 145;
    }
    private int whiskPos;
    private double targetEncoder;
    public static Params WHISK_PARAMS = new Params();


        Telemetry telemetry;
        HardwareMap map;
        PIDController WhiskController;
        public DcMotorEx WhiskMotor;


    public Whisk(HardwareMap hardwareMap, Telemetry telemetry) {
        WhiskController = new PIDController(WHISK_PARAMS.WhiskKp, WHISK_PARAMS.WhiskKi, WHISK_PARAMS.WhiskKd, telemetry);
        WhiskController.setOutputBounds(0, 0.2);
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        WhiskMotor = hardwareMap.get(DcMotorEx.class, "whisk");
        Flick = hardwareMap.get(ServoImplEx.class, "flick");
        Flick.setPwmRange(new PwmControl.PwmRange(WHISK_PARAMS.FlickLow, WHISK_PARAMS.FlickBallHeight));
        WhiskMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WhiskMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FlickTimer = new ElapsedTime();
        FlickTimer.reset();

        whiskState = WhiskState.COLLECT1;
        whiskPos = 0;
        targetEncoder = 0;


    }


    public enum WhiskState {
        COLLECT1,
        COLLECT2,
        COLLECT3,
        SHOOT1,
        SHOOT2,
        SHOOT3,
    }
    private WhiskState whiskState;
    public void setMotorPower(double power) {
        WhiskMotor.setPower(power);
    }
    @Override
    public void reset() {
        WhiskMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WhiskMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setTarget(double target){
        Whisk whiskController = null;
        whiskController.setTarget(target);}

    private void WhiskStartPos(){
        setTarget(0);
        setMotorPower(WHISK_PARAMS.MAX_POWER_CLOCKWISE);
    }

    public void update() {
        setMotorPower(-WhiskController.update(WhiskMotor.getCurrentPosition()));
        if(FlickTimer.seconds()>0.15){
            setFlickBase();
        }
    }

    public double getWhiskTelemetry() {
        double pidPower = -WhiskController.update(WhiskMotor.getCurrentPosition());

        telemetry.addData("WhiskController Target", WhiskController.getTarget());
        telemetry.addData("WhiskMotor Power", WhiskMotor.getPower());
        telemetry.addData("Whisk State", whiskState);
        telemetry.addData("Whisk Encoders", WhiskMotor.getCurrentPosition());
        telemetry.addData("Flick Position", Flick.getPosition());
        return WHISK_PARAMS.WhiskKs;
    }

    public void incWhiskPos() {
        whiskPos = (whiskPos + 1) % 6;
        targetEncoder += WHISK_PARAMS.encodersPerRev/6;
        WhiskController.setTarget(targetEncoder);
    }
    public void decWhiskPos() {
        whiskPos = (whiskPos + 1 + 6) % 6;
        targetEncoder -= WHISK_PARAMS.encodersPerRev/6;
        WhiskController.setTarget(targetEncoder);
    }
    public void turnWhisk60() {
        whiskPos = (whiskPos + 1 + 1) % 6;
        targetEncoder += WHISK_PARAMS.encodersPerRev/6;
        WhiskController.setTarget(targetEncoder);

    }


    public void setFlickUp() {
        Flick.setPosition(0.99);
        FlickTimer.reset();
    }
    public void setFlickBase() {
        Flick.setPosition(0.01);
    }
    public int getWhiskPos() {
        return whiskPos;
    }




}

