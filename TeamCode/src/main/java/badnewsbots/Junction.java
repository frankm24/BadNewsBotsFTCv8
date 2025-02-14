package badnewsbots;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.PowerPlayCompBotMecanumDrive;

public final class Junction {
    // implements comparable
    public enum JunctionType {
        LOW,
        MIDDLE,
        HIGH
    }
    public final double x;
    public final double y;
    public final JunctionType type;

    public Junction(double x, double y, JunctionType type) {
        this.x = x;
        this.y = y;
        this.type = type;
    }

    public Vector2d getPositionVec2d() {return new Vector2d(x, y);}

}

