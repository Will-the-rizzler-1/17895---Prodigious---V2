package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.teleop.subsystem.Shooter;

public class LimeLight {
//    public final Limelight3A limelight;
    Telemetry telemetry;

    public LimeLight(HardwareMap hardwareMap, Telemetry telemetry) {
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.pipelineSwitch(0);
//        limelight.start();
//
//        LLResult result = limelight.getLatestResult();


    }

    public Pose3D update() {
//        LLResult result = limelight.getLatestResult();
//
//        if (result != null && result.getBotpose() != null) {
//            return result.getBotpose();
//        }
        return null;
    }
}