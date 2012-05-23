package team;

import javax.swing.*;
import java.awt.*;

import java.util.*;
import java.io.*;

import april.vis.*;
import april.util.*;
import april.lcmtypes.*;

import lcm.lcm.*;


public class Task2 implements LCMSubscriber, ParameterListener
{

    static LCM lcm = LCM.getSingleton();

    JFrame jf = new JFrame("Task2");

    VisWorld vw = new VisWorld();
    VisLayer vl = new VisLayer(vw);
    VisCanvas vc = new VisCanvas(vl);

    ParameterGUI pg = new ParameterGUI();

    laser_t laser; // synchronize on 'this'
    pose_t pose;

    //    ArrayList <Line> lines = new ArrayList<Line>();
    //ArrayList <double[]> points = new ArrayList<double[]>();
  
    int NUM_LINES; 
    double MIN_ERROR;	
    int best_list_index;	
    int NO_OF_STEPS = 100;
    int NUM_ITERS;
	
    public Task2()
    {
	//        pg.addDoubleSlider("thresh","Thresh",0,1,.5);
	//pg.addInt("steps","Max_Steps",0,Integer.MAX_VALUE,NO_OF_STEPS);

        jf.setLayout(new BorderLayout());
        jf.add(vc, BorderLayout.CENTER);
        jf.add(pg, BorderLayout.SOUTH);

        jf.setSize(800,600);
        jf.setVisible(true);
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        lcm.subscribe("LIDAR_FRONT",this);
        lcm.subscribe("POSE",this);
	System.out.println("finished constructor");
    }

    public synchronized void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
	System.out.println("messageReceived");
        try {
            if (channel.equals("LIDAR_FRONT")) {
                laser = new laser_t(ins);
		System.out.println("lidar");

		//                update();
            } else if (channel.equals("POSE")) {
		pose = new pose_t(ins);
		System.out.println("pose: "+ pose.pos[0] + " " + pose.pos[1] + " " + pose.pos[2]);
                // do nothing for Task 2
            }
        } catch(IOException e) {
            System.out.println("Failed to decode message on channel "+channel);
        }
    }


    /*    public synchronized void update()
    {
        {
            VisWorld.Buffer vb = vw.getBuffer("laser-points");
            vb.addBack(new VisPoints(new VisVertexData(laserToPoints(laser)),
                                     new VisConstantColor(Color.green),
                                     2));
                    vb.swap();
        }

	MIN_ERROR = pg.gd("thresh");
	NUM_ITERS = pg.gi("steps");
	lines.clear();

	// Pass laser object to laserToPoints and get a double list of points
	points = laserToPoints (laser);
	lines = Line.extractLines(points,MIN_ERROR,NUM_ITERS,Line.DRAW_THRESHOLD);

        NUM_LINES = points.size()-1;	  

	/* Fill the Lines List 
  	  for(int i = 0; i < NUM_LINES; i++){ 
		Line temp = new Line(points.get(i),points.get(i+1));    
		lines.add(temp); 
	} 

	// Iterate with do-while until some condition
	do{

	// Iterate over the list of lines
	    MIN_ERROR = pg.gd("thresh");

	    double mse = 0;
	    Line templine = null;
	    best_list_index = -1;
	    for(int j = 0; j< NUM_LINES-1; j++)	
		{
			// Find minimum error from array
			templine = Line.merge(lines.get(j),lines.get(j+1));		
			mse = templine.getMSE();		

			if(mse < MIN_ERROR)
			{
				MIN_ERROR=mse;
				best_list_index = j;
				//System.out.println("Min error found");
			}
		}
	    if(best_list_index == -1)
		break;
	// Replace templine instead of 2 lines from lines list
	    //	    System.out.println("best index: " + best_list_index);
		templine = Line.merge(lines.get(best_list_index),lines.get(best_list_index+1));
		lines.remove(best_list_index);
		lines.set(best_list_index, templine);	
		NUM_LINES--;	
	NUM_ITERS--;

	}while(NUM_ITERS>0);	
	VisWorld.Buffer vb = vw.getBuffer("lines");
 	    for(int j = 0; j< NUM_LINES; j++)	
		{
		lines.get(j).drawLine(vb);
	}
	    vb.swap();

    }*/

    

    public void parameterChanged(ParameterGUI pg, String name)
    {
        if (name.equals("thresh")){
	    //   update();
	}
    }

    public static ArrayList<double[]> laserToPoints(laser_t laser)
    {
        ArrayList<double[]> points = new ArrayList<double[]>();
        for (int i = 0; i < laser.nranges; i++) {
            double theta = laser.rad0 + laser.radstep*i;
            points.add(new double[] { laser.ranges[i] * Math.cos(theta),
                                      laser.ranges[i] * Math.sin(theta) });
        }
        return points;
    }

    public static void main(String args[])
    {
	new Task2();
	
    }


}
