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

public class Final {

    public static final double MOVE_TASK_RATE = .05; 
    public static final double PRED_NOISE_COV = 5e-3;
    public static final double MAX_LASER_RANGE = 500;
    public static final double LASER_RANGE_COV = 1e-2;
    public static final double[] PRED_INIT = {0,0,0};
    public static final double[] PREY_INIT = {20,13,Math.PI};
    public static final double PREY_SLAMMING_VELOCITY = .75;
    public static final double PREY_RUNNING_VELOCITY = .75;
    public static final double PREY_BASELINE = .5;
    public static final double PREY_DIST_THRESH = .5;

    JFrame jfpred;

    VisWorld vwpred = new VisWorld();
    VisLayer vlpred = new VisLayer(vwpred);
    VisCanvas vcpred = new VisCanvas(vlpred);

    JFrame jfprey;

    VisWorld vwprey = new VisWorld();
    VisLayer vlprey = new VisLayer(vwprey);
    VisCanvas vcprey = new VisCanvas(vlprey);

    SimWorld world;
    String worldFilePath = "/tmp/world.world";

    SimBox robotBox;

    Predator pred;
    Prey prey;

    PeriodicTasks tasks;

    public Final(GetOpt gopt){
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
	    pred= new Predator(config,PRED_INIT);
	    prey = new Prey(PREY_INIT,PRED_INIT,config);
	    //p.slam.vw = vw;

	}catch(IOException ex){
	    System.out.println("ex:" + ex);
	    return;
	}

	robotBox = new SimBox(world);
	robotBox.sxyz = new double[]{.4,.2,.2};

	tasks = new PeriodicTasks();
	tasks.addFixedRate(new MoveTask(),MOVE_TASK_RATE);
	
	jfpred = new JFrame("Predator");
	jfpred.setLayout(new BorderLayout());
	jfpred.add(vcpred,BorderLayout.CENTER);
	
	jfpred.setSize(800,600);
	jfpred.setVisible(true);
	jfpred.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        VzGrid.addGrid(vwpred, new VzGrid(new VzLines.Style(Color.gray,1),
                                      new VzMesh.Style(Color.white)));
        vlpred.backgroundColor = Color.white;
	jfprey = new JFrame("Prey");
	jfprey.setLayout(new BorderLayout());
	jfprey.add(vcprey,BorderLayout.CENTER);
	
	jfprey.setSize(800,600);
	jfprey.setVisible(true);
	jfprey.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        VzGrid.addGrid(vwprey, new VzGrid(new VzLines.Style(Color.gray,1),
                                      new VzMesh.Style(Color.white)));
        vlprey.backgroundColor = Color.white;

	//VisWorld.Buffer vb = vw.getBuffer("SimWorld");
	//vb.addBack(new VisSimWorld());
	//	vb.swap();

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

	new Final(gopt);
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


    public class MoveTask implements PeriodicTasks.Task {
	
	double[] xytPred = LinAlg.copy(PRED_INIT);
	double[] xytPrey = LinAlg.copy(PREY_INIT);
	ArrayList<double[]> realTrajPred = new ArrayList<double[]>();
	ArrayList<double[]> realTrajPrey = new ArrayList<double[]>();

	Random r  =new Random(1);
	int predLaserCounter = LASER_COUNTER_MAX;
	int preyLaserCounter = LASER_COUNTER_MAX;
	boolean preyBackUp = false;

	public static final int LASER_COUNTER_MAX = 4;

	boolean first = true;

	public void run(double dt){
	    //Pred section
	    realTrajPred.add(LinAlg.resize(xytPred,2));
	    double[] desMove = pred.getDesiredMove(dt);
	    boolean noMove = false;
	    double[] NoisyMove = {desMove[0]*(1+r.nextGaussian()*PRED_NOISE_COV), desMove[1]*(1+r.nextGaussian()*PRED_NOISE_COV)};
	    double[] xytMove = {(NoisyMove[0] + NoisyMove[1])/2,0,(NoisyMove[1] - NoisyMove[0])/Predator.BASELINE};
	    ArrayList<double[]> laserPoints =null;
	    if(predLaserCounter == LASER_COUNTER_MAX){
		laserPoints =  new ArrayList<double[]>();
		predLaserCounter = -1;
	    }
	    if(desMove[0]!= 0.0 || desMove[1] != 0.0 || first){
		boolean validMove = validateMove(xytPred,xytMove,laserPoints);
		first = false;
		if(validMove){
		    xytPred = LinAlg.xytMultiply(xytPred,xytMove);
		    pred.move(desMove,laserPoints);
		    predLaserCounter++;
		}
	    }
	    //Need to determine what to show in this step for the Predator view		    
	    VisWorld.Buffer vb = vwpred.getBuffer("robot trajectory");
	    pred.slam.drawTrajectory(vb);
	    vb = vwpred.getBuffer("laser points");
	    pred.slam.drawCompiledLaserScan(vb);
	    vb = vwpred.getBuffer("map lines");
	    pred.slam.drawScanLines(vb,false);
	    
	    

	    //Prey section
	    realTrajPrey.add(LinAlg.resize(xytPrey,2));
	    prey.slam.getLines();
	    Vector<nodes> path = prey.ping_prey_status(prey.slam.getCurrentPosition(), 
						       xytPred,prey.slam.paddedLines,prey.slam.lines);
	    double distance = 0;
	    double turn = 0;
	    if(path == null || path.size()  <2){
		distance = PREY_SLAMMING_VELOCITY*dt;
		turn = r.nextGaussian()*Math.toRadians(5);
		if(preyBackUp){
		    distance = - distance;
		    turn  = Math.PI/4*3;
		}
	    } else{
		System.out.println("Goint to Point");
		nodes nextPoint = path.get(1);
		double dy = nextPoint.node_y - xytPrey[1];
		double dx =  nextPoint.node_x - xytPrey[0];
		distance = Math.sqrt(dy*dy + dx*dx);
		if(distance < PREY_DIST_THRESH && path.size() > 2){
		    nextPoint = path.get(2);
		    dy = nextPoint.node_y - xytPrey[1];
		    dx =  nextPoint.node_x - xytPrey[0];
		    distance = Math.sqrt(dy*dy + dx*dx);
		}
		turn = Math.atan2(dy,dx) - xytPrey[2];
		distance = Math.min(distance, PREY_RUNNING_VELOCITY*dt);
		//System.out.println("distance" + distance);
	    }
	    
	    double[] xytPreyMove = {distance,0,turn };

	    ArrayList<double[]> preyLaser = null;
	    if(preyLaserCounter == LASER_COUNTER_MAX){
		preyLaser = new ArrayList<double[]>();
		preyLaserCounter = -1;
	    }

	    if(validateMove(xytPrey,xytPreyMove,preyLaser)){
		preyLaserCounter++;
		xytPrey = LinAlg.xytMultiply(xytPrey,xytPreyMove);
		prey.move(new double[]{distance - turn*PREY_BASELINE/2,
				       distance + turn*PREY_BASELINE/2},preyLaser);
		preyBackUp = false;
	    }else{
		preyBackUp = true;
	    }
	    vb = vwprey.getBuffer("estimated Trajectory");
	    prey.slam.drawTrajectory(vb);
	    vb = vwprey.getBuffer("laser points");
	    prey.slam.drawCompiledLaserScan(vb);
	    vb = vwprey.getBuffer("map lines");
	    prey.slam.drawScanLines(vb,true);
	    vb = vwprey.getBuffer("actual trajectoy");
	    vb.addBack(new VzLines(new VisVertexData(realTrajPrey),VzLines.LINE_STRIP,new VzLines.Style(Color.blue,1)));
	    vb.swap();
	    
	    vb = vwpred.getBuffer("Prey");
	    if(predCanSeePrey()){
		System.out.println("I can See You!!");
		double[] curState = pred.slam.getCurrentPosition();
		double[] diff = LinAlg.xytInvMul31(xytPred,xytPrey);
		vb.addBack(new VisChain(LinAlg.xyzrpyToMatrix
					(new double[]{curState[0],curState[1], 
						      0,0,0,curState[2]}),
					LinAlg.xyzrpyToMatrix
					(new double[]{diff[0],diff[1],0,0,0,diff[2]}),
					new VzRobot()));
	    }
	    vb.swap();
	    
	}
    
    
	private boolean validateMove(double[] xyt, double[] move, ArrayList<double[]> laserScan){
	    double[] xytTmp = LinAlg.xytMultiply(xyt,move);
	    double[][] T = LinAlg.xyzrpyToMatrix(new double[]{xytTmp[0],xytTmp[1],0,0,0,xytTmp[2]});
	    synchronized(world){
		for (SimObject so : world.objects){
		    if(Collisions.collision(so.getShape(),so.getPose(),robotBox.getShape(),T)){
			return false;
		    }
		}
	    }
	    xyt = xytTmp;
	    double[] rpy = {0,0,xyt[2]};
	    double[][] R = LinAlg.rollPitchYawToMatrix(rpy);
	    R[0][3] = xyt[0];
	    R[1][3] = xyt[1];
	    ArrayList<double[]> laserPoints = null;
	    if(laserScan != null){
		double rad0 = -Math.PI/2;
		int nranges = 720;
		double radstep = Math.PI/nranges;
		double[] laserRange = Sensors.laser(world,new HashSet(),R,nranges,rad0,radstep,MAX_LASER_RANGE);
		double curTheta = rad0;
		for(int i = 0; i < nranges; i++){
		    if(laserRange[i] == MAX_LASER_RANGE){
			curTheta += radstep;
			continue;
		    }
		    laserRange[i] = laserRange[i] + r.nextGaussian()*LASER_RANGE_COV;
		    double[] pt = {laserRange[i] *Math.cos(curTheta),
				   laserRange[i]*Math.sin(curTheta)};
		    laserScan.add(pt);
		    curTheta += radstep;
		}
		
	    }
	    return true;
	}
	private boolean predCanSeePrey(){
	    double distance = LinAlg.distance(xytPred,xytPrey,2) - .5;
	    SimBox box = new SimBox(world);
	    box.sxyz = new double[]{distance,.001,1};
	    double cx = (xytPred[0] + xytPrey[0])/2;
	    double cy = (xytPred[1] + xytPrey[1])/2;
	    double pathAngle = Math.atan2((xytPrey[1]- xytPred[1]),(xytPrey[0] - xytPred[0]));
	    double[] xyzrpy = {cx,cy,0,0,0,pathAngle};
	    double[][] T = LinAlg.xyzrpyToMatrix(xyzrpy);
	    synchronized(world){
		for (SimObject so : world.objects){
		    if(Collisions.collision(so.getShape(),so.getPose(),box.getShape(),T)){
			return false;
		    }
		}
	    }
	    if(Math.abs(MathUtil.mod2pi(pathAngle-xytPred[2])) < Math.PI/2)
		return true;
	    return false;
	}

    }//end class MoveTask
    
}