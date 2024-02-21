package org.firstinspires.ftc.teamcode.autonomous;
import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.autonomous.AbstractAutonomous.Case.*;
import static org.firstinspires.ftc.teamcode.hardware.RobotStateMachine.robotStates.*;
import static org.firstinspires.ftc.teamcode.hardware.Intake.*;
import static org.firstinspires.ftc.teamcode.hardware.Lift.*;
import android.util.Pair;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.command.Command;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.command.SeqCommand;
import org.firstinspires.ftc.teamcode.command.SwitchCommand;
import org.firstinspires.ftc.teamcode.control.AsymConstraints;
import org.firstinspires.ftc.teamcode.hardware.ValueStorage.Side;
import org.firstinspires.ftc.teamcode.movement.Pose;
import org.firstinspires.ftc.teamcode.movement.TrajCommandBuilder;
import org.firstinspires.ftc.teamcode.movement.Vec;
import org.firstinspires.ftc.teamcode.vision.Vision;
@Photon
@Autonomous(name = "BlueNear")
public class BlueNear extends AbstractAutonomous {
    private AsymConstraints boardConstraints = new AsymConstraints(60, 80, 40);
    private Pose start = new Pose(17, 62, PI / 2);
    private Pose dropLeft = new Pose(30, 34, 0);
    private Pose dropCenter = new Pose(24, 28, 0.2);
    private Pose dropRight = new Pose(9, 34, 0);
    private Pose board = new Pose(55, 36, 0);
    private Pose park = new Pose(44, 60, 0);
    @Override
    public void initAutonomous() {
    }
}
