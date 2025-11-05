package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class SwyftDrive {
    public static class Params {
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        public double inPerTick = 1; // SparkFun OTOS Note: you can probably leave this at 1
        public double lateralInPerTick = inPerTick;
        public double trackWidthTicks = 13.328324551637913;

        // feedforward parameters (in tick units)
        public double kS = 1.1374369749541797;
        public double kV = 0.13506168240564792;
        public double kA = 0.05;

        // path profile parameters (in inches)
        public double maxWheelVel = 50;
        public double minProfileAccel = -30;
        public double maxProfileAccel = 50;

        // turn profile parameters (in radians)
        public double maxAngVel = Math.PI; // shared with path
        public double maxAngAccel = Math.PI;

        // path controller gains
        public double axialGain = 15.0;
        public double lateralGain = 10.0;
        public double headingGain = 10.0; // shared with turn

        public double axialVelGain = 0.0;
        public double lateralVelGain = 0.0;
        public double headingVelGain = 0.0; // shared with turn
    }
    public static Params PARAMS = new Params();

}
