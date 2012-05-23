package team;

import lcm.logging.*;
import java.awt.*;
import java.awt.event.*;
import java.io.*;
import java.util.*;
import javax.swing.*;

import april.vis.*;
import april.jmat.*;
import april.jmat.geom.*;
import april.util.*;
import april.lcmtypes.*;
import april.config.*;
import april.sim.*;
import april.lcm.*;




public class Project{
        
	static double[] prey_pos = new double[3];
	static double[] pred_pos = new double[3];
	Vector<nodes>  path = new Vector<nodes>();
	static double theta;
	static double theta_tan;
	static double[] dldr_tran = new double[2];
	static double[] dldr_rot = new double[2];
	static int cnt = 5;
	static double preyBaseLine = 20;
	static double preyMovement = 10; /*fixed distance for robot*/
	static double[] dldrPred = new double[2];
	static double thetaRunPoint = 0;

	static int nranges = 720;
	static double[] laserRangePred = new double[nranges];
	static double[] rpy = {0,0,pred_pos[3]};
	static double[][] R = LinAlg.rollPitchYawToMatrix(rpy);
	
	static double rad0 = -Math.PI/2;
	static double radstep = Math.PI/nranges;
	public static final double MAX_LASER_RANGE = 500;

	static double[] laserRangePrey = new double[nranges];
	static double[] rpyPrey = {0,0,prey_pos[3]};   
	static double[][] RPrey = LinAlg.rollPitchYawToMatrix(rpy);

	static double timestep = 0.01;
	
	
 	public static final double LASER_RANGE_COV = 1e-8;
 

 	Config c;	
	Prey prey;
	Predator predator;
	SimWorld world;	

	public  void Project(){

		dldr_tran[0]=5;
        	dldr_tran[1]=5;
		dldr_rot[0]=0;
		dldr_rot[1]=0;
	
        	/*SimWorld sw = SimWorld(config);*/
			

		prey_pos[0]=0;
		prey_pos[1]=3;
		prey_pos[2]=Math.PI/2;
		pred_pos[0]=18;
		pred_pos[1]=27;
		pred_pos[2]=Math.PI/3;

		R[0][3] = pred_pos[0];
	    	R[1][3] = pred_pos[1];

		RPrey[0][3] = prey_pos[0];
	    	RPrey[1][3] = prey_pos[1];

		Config c = new Config();
		GetOpt gopt;
		c = new ConfigFile(EnvUtil.expandVariables(gopt.getString("config")));
		predator = new Predator(c);
        	prey = new Prey(prey_pos,pred_pos,c);

		
	 	SimWorld world = new SimWorld(c);
		
		ArrayList<double[]> laserPointsPred = LaserScan(laserRangePred);
		prey.move(dldr_tran, laserPointsPred);
        }

        public void main(){

		while(true){
			
			
	    		
	  		ArrayList<double[]> laserPointsPred = LaserScan(laserRangePred);
			predator.move(predator.getDesiredMove(timestep), laserPointsPred);
			dldrPred = predator.getDesiredMove(timestep);
			
			ArrayList<double[]> laserPointsPrey = LaserScan(laserRangePrey);
			prey.move(dldr_tran, laserPointsPrey);
			
			cnt = cnt-1;
			if (cnt==0){

				
				path = prey.ping_prey_status(prey_pos,pred_pos,prey.slam.getLines());
				theta_tan = (path.get(0).node_y - prey_pos[1])/(path.get(0).node_x - prey_pos[0]);
				theta = Math.atan(theta_tan);
				prey_pos[2]=prey_pos[2]+theta;
				dldr_rot[0] = - theta*preyBaseLine/2;
				dldr_rot[1] = theta*preyBaseLine/2;
				prey.move(dldr_rot, laserPointsPrey);
				cnt = 5;
			}
			
			prey_pos[0]=prey_pos[0]+preyMovement*Math.cos(prey_pos[2]);
			prey_pos[1]=prey_pos[1]+preyMovement*Math.sin(prey_pos[2]);
			pred_pos[0]=pred_pos[0]+(dldrPred[0]+dldrPred[1])/2*Math.cos((dldrPred[1]-dldrPred[0])/2);
			pred_pos[1]=pred_pos[1]+(dldrPred[0]+dldrPred[1])/2*Math.sin((dldrPred[1]-dldrPred[0])/2);
                        
                        if (Math.pow(prey_pos[0]-pred_pos[0],2)+Math.pow(prey_pos[1]-pred_pos[1],2)<100)
				break;
		}
	}
	public ArrayList<double[]> LaserScan(double[] laserRange){

			Random r  =new Random(1);	
			double curTheta = rad0;
			ArrayList<double[]> laserPoints = new ArrayList<double[]>();
			for(int i = 0; i < nranges; i++){
		    		if(laserRange[i] == MAX_LASER_RANGE){
					curTheta += radstep;
					continue;
		    		}
		    	laserRangePred[i] = laserRangePred[i] + r.nextGaussian()*LASER_RANGE_COV;
		    	double[] pt = {laserRangePred[i] *Math.cos(curTheta),
				   laserRangePred[i]*Math.sin(curTheta)};
		    	laserPoints.add(pt);
		    	curTheta += radstep;
			}
		return laserPoints;
	    	}
	
}



