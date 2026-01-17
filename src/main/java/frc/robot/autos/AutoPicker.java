package frc.robot.autos;

import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoPicker {
    private Autos[] autoList;
    public AutoPicker(){
       autoList = new Autos[1]; //TODO Change this number when adding paths
       autoList[0] = new Autos("Intake", intakePath());
    }

    public Command intakePath(){
        try{
            PathPlannerPath intake = PathPlannerPath.fromChoreoTrajectory("Intake1");
            return AutoBuilder.followPath(intake);
        } catch (Exception e){
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }
    
    public Autos[] getAutoList(){
        return autoList;
    }
}
