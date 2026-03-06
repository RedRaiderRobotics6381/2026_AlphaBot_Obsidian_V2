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
       autoList = new Autos[2]; //TODO Change this number when adding paths
       autoList[0] = new Autos("JustShoot", shootPath());
       autoList[1] = new Autos("Simple Path", simpleAuto());
    }

    public Command shootPath(){
        try{
            PathPlannerPath shoot = PathPlannerPath.fromChoreoTrajectory("JustShoot");
            return AutoBuilder.followPath(shoot);
        } catch (Exception e){
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }
    public Command simpleAuto(){
        try{
            PathPlannerPath shoot = PathPlannerPath.fromChoreoTrajectory("Intake1");
            return AutoBuilder.followPath(shoot);
        } catch (Exception e){
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }
    
    public Autos[] getAutoList(){
        return autoList;
    }
}
