package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.hardware.Robot;

@TeleOp(name = "OneDriver")
public class TeleopOneDriver extends CommandOpMode {
    Robot robot = new Robot();
    @Override
    public void initOpMode() {
        robot.init(hardwareMap, true);
    }
}
