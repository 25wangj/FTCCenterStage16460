package org.firstinspires.ftc.teamcode.autonomous;
import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.autonomous.AbstractAutonomous.Case.*;
import static org.firstinspires.ftc.teamcode.hardware.RobotStateMachine.robotStates.*;
import static org.firstinspires.ftc.teamcode.hardware.Intake.*;

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
@Autonomous(name = "BlueFar")
public class BlueFar extends AbstractAutonomous {
    private AsymConstraints boardConstraints = new AsymConstraints(20, 30, 30);
    private Pose start = new Pose(-40, 62, PI / 2);
    private Pose dropLeft = new Pose(-30, 28, 3.8);
    private Pose dropCenter = new Pose(-36, 16, -1);
    private Pose dropRight = new Pose(-43, 28, -1);
    private Pose mid1 = new Pose(-24, 10, 0);
    private Pose mid2 = new Pose(12, 10, 0);
    private Pose board = new Pose(52, 36, 0);
    private Pose park = new Pose(44, 12, 0);
    @Override
    public void initAutonomous() {
    }
}
