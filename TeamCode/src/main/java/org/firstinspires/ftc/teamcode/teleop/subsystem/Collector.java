package org.firstinspires.ftc.teamcode.teleop.subsystem;

import android.text.style.UpdateAppearance;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleop.Component;

public class Collector implements Component {
    Telemetry telemetry;
    HardwareMap map;
    UpdateAppearance update;
    public enum CollectorState {
        IN, OFF, OUT
    }
    CollectorState collectorState;
    private DcMotor collectorMotor;
    public Collector(HardwareMap map, Telemetry telemetry){
        this.map = map;
        this.telemetry = telemetry;
        collectorMotor = map.get(DcMotor.class, "collector");
        collectorState = CollectorState.OFF;
    }
    public void setCollectorPower(double power){
        collectorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        collectorMotor.setPower(power);
    }
    public void reset () {

    }

    public void update () {
        switch(collectorState){
            case OFF:{
                collectorOff();
                break;
            }
            case IN:{
                collectorIn();
                break;
            }
            case OUT:{
                collectorOut();
                break;
            }

        }
    }
    public double getCollectorPower(){
        return collectorMotor.getPower();
    }
    public CollectorState getState(){
        return collectorState;
    }
    public String test () {
        return null;
    }
    private void collectorOff() {
        collectorMotor.setPower(0.0);
    }
    private void collectorIn() {
        collectorMotor.setPower(0.7);
    }
    private void collectorOut() {
        collectorMotor.setPower(-0.7);
    }
    public void setIn() {
        collectorState = CollectorState.IN;
    }

    public void setOut() {
        collectorState = CollectorState.OUT;
    }

    public void setOff() {
        collectorState = CollectorState.OFF;
    }
}
