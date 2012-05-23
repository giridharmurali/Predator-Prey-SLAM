package team;

import javax.swing.*;
import java.awt.*;
import java.util.*;
import java.io.*;
import april.vis.*;
import april.sim.*;
import april.config.*;
import april.util.*;
import april.jmat.*;
//import april.lcmtypes.*;
//import lcm.lcm.*;

public class SLAMTester {

    public static final double PRED_TASK_RATE = .05; 
    public static final double PRED_NOISE_COV = 5e-3;
    public static final double MAX_LASER_RANGE = 500;
    public static final double LASER_RANGE_COV = 1e-2;

    JFrame jf;

    VisWorld vw = new VisWorld();
    VisLayer vl = new VisLayer(vw);
    VisCanvas vc = new VisCanvas(vl);

    SimWorld world;
    String worldFilePath = "/tmp/world.world";

    SimBox robotBox;

    Predator p;

    PeriodicTasks tasks;

    public SLAMTester(GetOpt gopt){
	try{
            Config config = new Config();
            if (gopt.wasSpecified("config"))
                config = new ConfigFile(EnvUtil.expandVariables(gopt.getString("config")));

            if (gopt.getString("world").length() > 0) {
                worldFilePath = EnvUtil.expandVariables(gopt.getString("world"));
                this.world = new SimWorld(worldFilePath, config);
            } else {
                this.world = new SimWorld(config);
            }
	    p= new Predator(config, Final.PRED_INIT);
	    //p.slam.vw = vw;

	}catch(IOException ex){
	    System.out.println("ex:" + ex);
	    return;
	}

	robotBox = new SimBox(world);
	robotBox.sxyz = new double[]{.4,.2,.2};

	tasks = new PeriodicTasks();
	tasks.addFixedRate(new PredatorTask(),PRED_TASK_RATE);
	
	jf = new JFrame("SLAM Tester");
	jf.setLayout(new BorderLayout());
	jf.add(vc,BorderLayout.CENTER);
	
	jf.setSize(800,600);
	jf.setVisible(true);
	jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        VzGrid.addGrid(vw, new VzGrid(new VzLines.Style(Color.gray,1),
                                      new VzMesh.Style(Color.white)));// = Color.white;//new java.awt.Color(255,255,255,0);
        vl.backgroundColor = Color.white;
	//VisWorld.Buffer vb = vw.getBuffer("SimWorld");
	//vb.addBack(new VisSimWorld());
	//	vb.swap();
	//	new Thread(new ThreadRunner()).start();
	tasks.setRunning(true);
    }


    public static void main(String[] args){
	GetOpt gopt = new GetOpt();
        gopt.addBoolean('h', "help", false, "Show this help");
        gopt.addString('w', "world", "", "World file");
        gopt.addString('c', "config", "", "Configuration file");
	
        if (!gopt.parse(args) || gopt.getBoolean("help") || gopt.getExtraArgs().size() > 0) {
            gopt.doHelp();
            return;
        }

	new SLAMTester(gopt);
    }

    class VisSimWorld implements VisObject
    {
        public void render(VisCanvas vc, VisLayer layer, VisCanvas.RenderInfo rinfo, GL gl)
        {
            synchronized(world) {
                for (SimObject obj : world.objects) {
                    VisChain v = new VisChain(obj.getPose(), obj.getVisObject());
                    v.render(vc, layer, rinfo, gl);
                }
            }
        }
    }


    public class PredatorTask implements PeriodicTasks.Task {
	
	double[] xyt = LinAlg.copy(Final.PRED_INIT);
	ArrayList<double[]> realTraj = new ArrayList<double[]>();

	Random r  =new Random(1);
	int laserCounter = LASER_COUNTER_MAX;

	public static final int LASER_COUNTER_MAX = 4;

	boolean first = true;

	public void run(double dt){
	    realTraj.add(LinAlg.resize(xyt,2));
	    //	    System.out.println("Beginning Predator Task");
	    double[] desMove = p.getDesiredMove(dt);
	    boolean noMove = false;
	    if(desMove[0] == 0.0 && desMove[1] == 0.0 && !first){
		noMove = true;
	    }
	    first = false;
	    double[] NoisyMove = {desMove[0]*(1+r.nextGaussian()*PRED_NOISE_COV), desMove[1]*(1+r.nextGaussian()*PRED_NOISE_COV)};
	    double[] xytMove = {(NoisyMove[0] + NoisyMove[1])/2,0,(NoisyMove[1] - NoisyMove[0])/Predator.BASELINE};
	    double[] xytTmp = LinAlg.xytMultiply(xyt,xytMove);
	    double[][] T = LinAlg.xyzrpyToMatrix(new double[]{xytTmp[0],xytTmp[1],0,0,0,xytTmp[2]});
	    robotBox.setPose(T);
	    synchronized(world){
		for (SimObject so : world.objects){
		    if(Collisions.collision(so.getShape(),so.getPose(),robotBox.getShape(),T)){
			noMove = true;
			break;
		    }
		}
	    }
	    if(!noMove){
	    xyt = xytTmp;
	    double[] rpy = {0,0,xyt[2]};
	    double[][] R = LinAlg.rollPitchYawToMatrix(rpy);
	    R[0][3] = xyt[0];
	    R[1][3] = xyt[1];
	    ArrayList<double[]> laserPoints = null;
	    if(laserCounter == LASER_COUNTER_MAX){
		laserCounter = -1;
		double rad0 = -Math.PI/2;
		int nranges = 720;
		double radstep = Math.PI/nranges;
		double[] laserRange = Sensors.laser(world,new HashSet(),R,nranges,rad0,radstep,MAX_LASER_RANGE);
		double curTheta = rad0;
		//laser_t laser = new laser_t();
		//laser.utime = TimeUtil.utime();
		//laser.nranges = nranges;
		//laser.rad0 = (float) rad0;
		//laser.radstep = (float) radstep;
		//laser.ranges = LinAlg.copyFloats(laserRange);
		//LCM.getSingleton().publish("LIDAR",laser);
		laserPoints = new ArrayList<double[]>();
		for(int i = 0; i < nranges; i++){
		    if(laserRange[i] == MAX_LASER_RANGE){
			curTheta += radstep;
			continue;
		    }
		    laserRange[i] = laserRange[i] + r.nextGaussian()*LASER_RANGE_COV;
		    double[] pt = {laserRange[i] *Math.cos(curTheta),
				   laserRange[i]*Math.sin(curTheta)};
		    laserPoints.add(pt);
		    curTheta += radstep;
		}
	    }
	    laserCounter++;
	    p.move(desMove,laserPoints);
	    

	    VisWorld.Buffer vb = vw.getBuffer("robot trajectory");
	    p.slam.drawTrajectory(vb);
	    vb = vw.getBuffer("laser points");
	    p.slam.drawCompiledLaserScan(vb);
	    vb = vw.getBuffer("map lines");
	    p.slam.drawScanLines(vb,true);
	    vb = vw.getBuffer("actualTraj");
	    vb.addBack(new VzLines(new VisVertexData(realTraj),VzLines.LINE_STRIP,new VzLines.Style(Color.blue,1)));
	    vb.swap();
	    }
	    VisWorld.Buffer vb = vw.getBuffer("actual Robot");
	    vb.addBack(new VisChain(T,new VzRobot()));
	    vb.swap();
	}
    }

}