package team;

import april.lcm.*;
import april.lcmtypes.*;
import april.config.*;

import lcm.lcm.*;

import java.util.*;
import java.io.*;

public class Predator implements LCMSubscriber {

    gamepad_t gp;
    public static final double MAX_VEL = 1;
    public static final double BASELINE  = .5;
    public static final double MAX_TURN = Math.PI/16*BASELINE;

    public SLAM slam;

    public Predator(Config c, double[] initialState){
	LCM.getSingleton().subscribe("GAMEPAD",this);
	slam = new SLAM(c, initialState);
	System.out.println("Predator created");
    }

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream dins){
	try{
	    if (channel.equals("GAMEPAD")){
		if(gp == null){
		    gp = new gamepad_t(dins);
		}else{
		    synchronized(gp){
			gp = new gamepad_t(dins);
		    }
		}
	    }
	} catch(IOException e){}
    }


    public double[] getDesiredMove(double timeStep){
	double distance;
	double turn;
	if(gp != null){
	    synchronized(gp){
		distance = -gp.axes[3] * MAX_VEL * timeStep;
		turn = -gp.axes[2] * MAX_TURN * timeStep;
	    }
	}else{
	    distance = 0; turn = 0;
	}
	return new double[]{distance-turn,distance+turn};
    }

    public void move(double[] dldr, ArrayList<double[]> laserscan){
	slam.motion(dldr,laserscan,true);
    }

}