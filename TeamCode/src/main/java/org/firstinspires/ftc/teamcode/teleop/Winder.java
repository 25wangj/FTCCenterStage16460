package org.firstinspires.ftc.teamcode.teleop;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@Photon
@TeleOp(name = "Winder")
public class Winder extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor climb = hardwareMap.dcMotor.get("climb");
        while (!isStarted() && !isStopRequested()) {}
        while (opModeIsActive()) {
            if (gamepad1.a) {
                climb.setPower(0.5);
            } else if (gamepad1.b) {
                climb.setPower(-0.5);
            } else {
                climb.setPower(0);
            }
        }
    }
}