package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class LimeLight  implements Component {

    public Limelight3A limelight;
    private LLResult result;
    private Telemetry telemetry;

    public LimeLight(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    @Override
    public void reset() {

    }

    public Pose3D getLimelightResults() {
        result = limelight.getLatestResult();

        if (result != null && result.getBotpose() != null) {
            return result.getBotpose();
        }

        return new Pose3D(new Position(DistanceUnit.INCH, 0,0,0, 0), new YawPitchRollAngles(AngleUnit.DEGREES,0,60,0,0));
    }

    @Override
    public void update() {
        }

    @Override
    public String test() {
        return "";
    }


    public boolean hasTarget() {
        return result != null && result.isValid();
    }

    public double getTx() {
        return (result != null) ? result.getTx() : 0.0;
    }

    public double getTy() {
        return (result != null) ? result.getTy() : 0.0;
    }

    public double getTa() {
        return (result != null) ? result.getTa() : 0.0;
    }

    public Pose3D getBotPose() {
        return (result != null) ? result.getBotpose() : null;
    }
}
