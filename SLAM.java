package team;

import java.util.*;
import javax.swing.*;
import java.awt.*;

import april.laser.scanmatcher.*;
import april.laser.*;
import april.util.*;
import april.config.*;
import april.jmat.*;
import april.jmat.geom.*;
import april.graph.*;
import april.vis.*;

public class SLAM {
    
    public final static double GRID_SIZE_X = 15;
    public final static double GRID_SIZE_Y = 25;
    public final static double METERS_PER_PIXEL = .075;
    public final static double SEARCH_X = 5;
    public final static double SEARCH_Y = 5;
    public final static double SEARCH_THETA = Math.toRadians(15);
    public final static double SEARCH_THETA_RES = Math.toRadians(.25);
    public final static double SCAN_MATCH_RADIUS = 6;
    public final static double EDGE_DIST_THRESH = .3;
    public final static double EDGE_THETA_THRESH = Math.toRadians(5);
    public final static double PERP_DIST_THRESH = .5;
    public final static double THETA_LINE_JOIN_THRESH = Math.toRadians(4);
    public final static double RANGE_COVARIANCE = .01;
    public final static int NUM_ITERATIONS = 10;
    public final static int MAX_MATCHES = 7;
    public final static double DISTANCE_DIFF_THRESH = 3;
    public final static double MIN_DIST_THRESH = .5;
    public final static double PADDING_T = .75;
    public final static double LINE_SHORT = PADDING_T - 2*Predator.BASELINE;
    /*
    JFrame jf;

    VisWorld vw = new VisWorld();
    VisLayer vl = new VisLayer(vw);
    VisCanvas vc = new VisCanvas(vl);
    */

    public class Scan{
	GXYTNode node;
	int nodeNum;
	ArrayList<double[]> scan;
	ArrayList<ArrayList<double[]>> contours;
	ArrayList<LineFeature> lines;
	ArrayList<LineFeature> paddedLines;
    }

    Graph g;
    ArrayList<Scan> scanHistory;


    MultiResolutionScanMatcher matcher;
    ContourExtractor contourextractor;
    GridMap gm;
    LineFitter lineFitter = new LineFitter(null);

    double baseline;

    ArrayList<LineFeature> lines = null;
    ArrayList<LineFeature> paddedLines = null;
    boolean LinesNeedComputing = true;

    public SLAM(Config c, double[] initialState){

	g = new Graph();
	baseline = c.getDouble("robot.baseline",Predator.BASELINE);
	
	scanHistory = new ArrayList<Scan>();
	matcher = new MultiResolutionScanMatcher(c);
	contourextractor = new ContourExtractor(c);
	gm = GridMap.makeMeters(0,-GRID_SIZE_Y/2,
				GRID_SIZE_X,GRID_SIZE_Y,METERS_PER_PIXEL,0);
	GXYTNode gn = new GXYTNode();
	gn.state = LinAlg.copy(initialState);
	g.nodes.add(gn);
	CholeskySolver.verbose = false;


	/*	VzGrid.addGrid(vw, new VzGrid(new VzLines.Style(Color.gray,1),
                                      new VzMesh.Style(Color.white)));// = Color.white;//new java.awt.Color(255,255,255,0);
        vl.backgroundColor = Color.white;
	jf = new JFrame("SLAM Grid");
	jf.setLayout(new BorderLayout());
	jf.add(vc,BorderLayout.CENTER);
	
	jf.setSize(800,600);
	jf.setVisible(true);
	jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
	*/


    }


    public double[] getCurrentPosition(){
	return g.nodes.get(g.nodes.size()-1).state;
    }

    //This takes in a dl/dr and a laser scan from the new location. 
    //laserScan can be null, which means no scan from that location
    public void motion(double[] dldr, ArrayList<double[]> laserScan, boolean solve){
	//Motion model same as the Problem sets

	int numNodes = g.nodes.size();
	GXYTEdge ge = new GXYTEdge();
	ge.nodes = new int[]{numNodes-1,numNodes};
	ge.z = new double[]{(dldr[0] + dldr[1])/2,0,(dldr[1]-dldr[0])/baseline};
	ge.P = LinAlg.diag(new double[]{.1,.1,.01});//unprincipled noise
	GXYTNode gn = new GXYTNode();
	gn.state = LinAlg.xytMultiply(g.nodes.get(g.nodes.size()-1).state,ge.z);

	g.nodes.add(gn);
	g.edges.add(ge);

	//if laser scan, look for matches and add edges

	if(laserScan != null){
	    double minDist = Double.MAX_VALUE;
	    for(double[] pt : laserScan){
		double dist = Math.sqrt(pt[0]*pt[0] + pt[1]*pt[1]);
		minDist = Math.min(minDist,dist);
	    }
	    if(minDist >= MIN_DIST_THRESH){
		
	    
	    Scan scan = new Scan();
	    scan.node = gn;
	    scan.nodeNum = numNodes;
	    scan.scan = laserScan;
	    scan.contours = contourextractor.getContours(laserScan);
	    scan.lines = lineFitter.getLineFeatures(laserScan);
	    scan.paddedLines = new ArrayList<LineFeature>();
	    for(LineFeature lf : scan.lines){
		double[] p1 = lf.seg.p1;
		double[] p2 = lf.seg.p2;
		lf.seg.line.optimize();
		double slope_perp = -1.0/lf.seg.line.getM();
		double theta = Math.atan(slope_perp);
		double[] perp = new double[]{PADDING_T*Math.cos(theta), PADDING_T*Math.sin(theta)};
		//System.out.println("" + p1[0] + " " + p1[1]);
		double[] p1opt1 = new double[]{p1[0] + perp[0], p1[1] + perp[1]};
		double[] p1opt2 = new double[]{p1[0] - perp[0], p1[1] - perp[1]};
		if(LinAlg.normF(p1opt1) < LinAlg.normF(p1opt2)){
		    p1 = p1opt1;
		    p2 = new double[]{p2[0] + perp[0], p2[1] + perp[1]};
		}else{
		    p1 = p1opt2;
		    p2 = new double[]{p2[0] - perp[0], p2[1] - perp[1]};

		}
		//		System.out.println("" + p1[0] + " " + p1[1]);
		theta = Math.atan(lf.seg.line.getM());
		double[] vec = new double[]{LINE_SHORT*Math.cos(theta), LINE_SHORT*Math.sin(theta)};
		double[] p1a = new double[]{p1[0] + vec[0], p1[1] + vec[1] };
		double[] p1b = new double[]{p1[0] - vec[0], p1[1] - vec[1] };
		double[] p2a = new double[]{p2[0] - vec[0], p2[1] - vec[1] };
		double[] p2b = new double[]{p2[0] + vec[0], p2[1] + vec[1] };
		if(LinAlg.distance(p1a,p2a) < LinAlg.distance(p1b,p2b)){
		    p1 = p1a;
		    p2 = p2a;
		}else{
		    p1 = p1b;
		    p2 = p2b;
		}
		LineFeature paddedlf = new LineFeature();
		paddedlf.seg = new GLineSegment2D(p1,p2);
		paddedlf.npoints = lf.npoints;
		scan.paddedLines.add(paddedlf);
	    }

	    int numMatches = 0;
	    for(int j = scanHistory.size()-1; j >=0 ; j--){
		Scan s = scanHistory.get(j);
		//determine if possible overlap
		double[] curxyt = gn.state;
		double[] prevxyt = s.node.state;
		double[] difference = LinAlg.xytInvMul31(prevxyt,curxyt);
		double pathAngle = Math.atan2(difference[1],difference[0]);

		double dist = Math.sqrt(Math.pow(difference[0],2) + Math.pow(difference[1],2));

		//Only attempt match if poses are close enough
		if(dist > SCAN_MATCH_RADIUS || 
		   (Math.abs(difference[2]) > Math.PI/2 &&( difference[1] < 3 || Math.abs(pathAngle) > Math.PI/8))){
		   // || Math.abs(MathUtil.mod2pi(Math.PI - difference[2] + curxyt[2])) < Math.PI/2)){
		    continue;
		}
		//Fill the gridmap to represent the scan that we are trying to match to.
		//The origin will be the location of the robot taking the scan. 
		gm.fill(0);
		
		GridMap.LUT lut = gm.makeGaussianLUT(1.0,0,1.0/RANGE_COVARIANCE);
		ArrayList<double[]> contourPoints = new ArrayList<double[]>();
		if(true){
		    for(ArrayList<double[]> c: s.contours){
			contourPoints.addAll(c);
			for(int i = 0; i< c.size()-1; i++){
			    double[] p0 = c.get(i);
			    double[] p1 = c.get(i+1);

			    double length = LinAlg.distance(p0,p1);
			    double angle = Math.atan2(p1[1] - p0[1], p1[0] - p0[0]);
			    double cx = (p1[0] + p0[0])/2;
			    double cy = (p1[1] + p0[1])/2;
			    gm.drawRectangle(cx,cy, length,0,angle,lut);
			}
			//System.out.println("angle" + angle);
		    }
		} else {
		    for(LineFeature lf: s.lines){
			double[] p0 = lf.seg.p1;
			double[] p1 = lf.seg.p2;
			
			double length = LinAlg.distance(p0,p1);
			double angle = Math.atan2(p1[1] - p0[1], p1[0] - p0[0]);
			System.out.println("angle" + angle);
			double cx = (p1[0] + p0[0])/2;
			double cy = (p1[1] + p0[1])/2;
			gm.drawRectangle(cx,cy, length,0,angle,lut);
		    }
		}
		/*
		double[][] vertices = {{0,-25,-1},{50,-25,-1},{50,25,-1},{0,25,-1}};
		double[][] texcoords = {{1,1},{1,1},{1,1},{1,1}};
		//VzImage vi = new VzImage(new VisTexture(gm.makeBufferedImage()),vertices,texcoords,Color.white);
		VzImage vi = new VzImage(gm.makeBufferedImage());
		VisWorld.Buffer vb = vw.getBuffer("image");
		vb.addBack(vi);
		vb.swap();
		vb = vw.getBuffer("laser points");
		vb.addBack(new VisChain(LinAlg.translate(new double[]{0,0,1}),new VzPoints(new VisVertexData(contourPoints),new VzPoints.Style(Color.blue,2))));
		vb.swap();*/
		matcher.setModel(gm);
		//attempt match
		//The initial guess is the difference from the odometry
		MultiGaussian posterior = matcher.match(laserScan,
							difference,null,SEARCH_X,SEARCH_Y,SEARCH_THETA,
							SEARCH_THETA_RES);
		double[] xyt = posterior.getMean();
		/*vb = vw.getBuffer("matched data");
		double[] xyzrpy = {xyt[0],xyt[1],0,0,0,xyt[2]};
		vb.addBack(new VisChain(LinAlg.xyzrpyToMatrix(xyzrpy),new VzPoints(new VisVertexData(laserScan),new VzPoints.Style(Color.red,2))));
		vb.swap();
		System.out.println("difference: x " +difference[0] + " y "+ difference[1] + " theta " +difference[2]);
		System.out.println("xyt: x" + xyt[0] + " y " + xyt[1] + " theta " + xyt[2]);*/
		double distPost = Math.sqrt(xyt[0]*xyt[0] + xyt[1]*xyt[1]);
		double distXYTDiff = LinAlg.distance(xyt,difference,2);
		//create edge
		if(distPost > EDGE_DIST_THRESH && Math.abs(xyt[2])> EDGE_THETA_THRESH && distXYTDiff < DISTANCE_DIFF_THRESH){
		    GXYTEdge gels = new GXYTEdge();
		    gels.z = xyt;
		    //gels.P = posterior.getCovariance().copyArray();
		    gels.P = LinAlg.diag(new double[]{.1,.1,.01}); //unprincipled noise, but otherwise got an error
		    gels.nodes = new int[2];
		    gels.nodes[0] = s.nodeNum;
		    gels.nodes[1] = scan.nodeNum;
		    g.edges.add(gels);
		    //System.out.println("Edge Created " + g.edges.size());
		}
		j -= 10;
		numMatches++;
		if(numMatches >= MAX_MATCHES){
		    break;
		}
	    }
	    scanHistory.add(scan);
	}
	}//end if(laserScan != null)

	CholeskySolver solver = new CholeskySolver(g);
	for(int i = 0; i < NUM_ITERATIONS && solve; i++){
	    solver.iterate();
	}
	LinesNeedComputing = true;
    }

    public ArrayList<LineFeature> getLines(){
	if(!LinesNeedComputing){
	    return paddedLines;
	}
	ArrayList<LineFeature> allLines = new ArrayList<LineFeature>();
	ArrayList<LineFeature> allPaddedLines = new ArrayList<LineFeature>();
	for(Scan s : scanHistory){
	    ArrayList<LineFeature> rot = rotateAndMoveLineFeatures(s.lines,s.node.state);
	    allLines.addAll(rot);
	    rot = rotateAndMoveLineFeatures(s.paddedLines,s.node.state);
	    allPaddedLines.addAll(rot);
	}

	//	System.out.println(allPaddedLines.size() + " " + allLines.size());

	ArrayList<LineFeature> finalList = new ArrayList<LineFeature>();
	ArrayList<LineFeature> finalListPadded = new ArrayList<LineFeature>();

	while(allLines.size()> 0){
	    //	    assert(allLines.size() == allPaddedLines.size());
	    LineFeature cur = allLines.remove(allLines.size()-1);
	    LineFeature curPadded = allPaddedLines.remove(allPaddedLines.size()-1);
	    LineFeature bestMatch = null;
	    LineFeature bestPaddedMatch = null;
	    double thetaCur = Math.atan(cur.seg.line.getM());
	    double thetaBestMatch = 0;
	    for(int i = 0; i < finalList.size() && bestMatch == null; i++){
		LineFeature lf = finalList.get(i);
		double thetaLF = Math.atan(lf.seg.line.getM());
		double thetaDiff = Math.abs(MathUtil.mod2pi(thetaCur - thetaLF));
		if(thetaDiff < THETA_LINE_JOIN_THRESH){
		    double perpDist = cur.seg.line.perpendicularDistanceTo(lf.seg.p1);
		    if(perpDist < PERP_DIST_THRESH){
			double lineCoordcurp1 = lf.seg.line.getLineCoordinate(lf.seg.line.pointOnLineClosestTo(cur.seg.p1));
			double lineCoordcurp2 = lf.seg.line.getLineCoordinate(lf.seg.line.pointOnLineClosestTo(cur.seg.p2));
			double lineCoordlfp1 = cur.seg.line.getLineCoordinate(cur.seg.line.pointOnLineClosestTo(lf.seg.p1));
			double lineCoordlfp2 = cur.seg.line.getLineCoordinate(cur.seg.line.pointOnLineClosestTo(lf.seg.p2));
			double curCoordp1 = cur.seg.line.getLineCoordinate(cur.seg.p1);
			double curCoordp2 = cur.seg.line.getLineCoordinate(cur.seg.p2);
			double lfCoordp1 = lf.seg.line.getLineCoordinate(lf.seg.p1);
			double lfCoordp2 = lf.seg.line.getLineCoordinate(lf.seg.p2);
			if(inRange(lineCoordcurp1,lfCoordp1,lfCoordp2) ||
			   inRange(lineCoordcurp2,lfCoordp1,lfCoordp2) ||
			   inRange(lineCoordlfp1,curCoordp1,curCoordp2) ||
			   inRange(lineCoordlfp2,curCoordp1,curCoordp2)  ){
			    
			    thetaBestMatch = thetaLF;
			    bestMatch = lf;
			    bestPaddedMatch = finalListPadded.get(i);
			}
		    }
		}
	    } //end for(finalList)

	    if(bestMatch == null){
		finalList.add(cur);
		finalListPadded.add(curPadded);
	    } else{
		finalList.remove(bestMatch);
		finalListPadded.remove(bestPaddedMatch);
		double newTheta = (thetaBestMatch + thetaCur)/2;
		double cx = ((cur.seg.p1[0] + cur.seg.p2[0])*cur.npoints + 
			     (bestMatch.seg.p1[0] + bestMatch.seg.p2[0])*bestMatch.npoints)/2/(bestMatch.npoints+cur.npoints);
		double cy = ((cur.seg.p1[1] + cur.seg.p2[1])*cur.npoints
			     +( bestMatch.seg.p1[1] + bestMatch.seg.p2[1])*bestMatch.npoints)/2/(bestMatch.npoints+cur.npoints);
		double cxpadded = ((curPadded .seg.p1[0] + curPadded.seg.p2[0])*curPadded.npoints + 
			     (bestPaddedMatch.seg.p1[0] + bestPaddedMatch.seg.p2[0])*bestPaddedMatch.npoints)/2
		    /(bestPaddedMatch.npoints+curPadded.npoints);
		double cypadded = ((curPadded.seg.p1[1] + curPadded.seg.p2[1])*curPadded.npoints
			     +( bestPaddedMatch.seg.p1[1] + bestPaddedMatch.seg.p2[1])*bestPaddedMatch.npoints)/2/
		    (bestPaddedMatch.npoints+curPadded.npoints);
		GLine2D line = GLine2D.fromThetaPoint(newTheta, new double[]{cx,cy});
		double coordcp1 = line.getLineCoordinate(cur.seg.p1);
		double coordcp2 = line.getLineCoordinate(cur.seg.p2);
		double coordlfp1 = line.getLineCoordinate(bestMatch.seg.p1);
		double coordlfp2 = line.getLineCoordinate(bestMatch.seg.p2);
		double minCoord = Math.min(Math.min(coordcp1,coordcp2),Math.min(coordlfp1,coordlfp2));
		double maxCoord = Math.max(Math.max(coordcp1,coordcp2),Math.max(coordlfp1,coordlfp2));
		GLine2D linePadded = GLine2D.fromThetaPoint(newTheta, new double[]{cxpadded,cypadded});
		double coordcp1Padded = linePadded.getLineCoordinate(curPadded.seg.p1);
		double coordcp2Padded = linePadded.getLineCoordinate(curPadded.seg.p2);
		double coordlfp1Padded = linePadded.getLineCoordinate(bestPaddedMatch.seg.p1);
		double coordlfp2Padded = linePadded.getLineCoordinate(bestPaddedMatch.seg.p2);
		double minCoordPadded = Math.min(Math.min(coordcp1Padded,coordcp2Padded),Math.min(coordlfp1Padded,coordlfp2Padded));
		double maxCoordPadded = Math.max(Math.max(coordcp1Padded,coordcp2Padded),Math.max(coordlfp1Padded,coordlfp2Padded));
		LineFeature newlf = new LineFeature();
		newlf.seg = new GLineSegment2D(line.getPointOfCoordinate(minCoord),line.getPointOfCoordinate(maxCoord));
		newlf.npoints = bestMatch.npoints + cur.npoints;
		allLines.add(newlf);
		LineFeature newlfPadded = new LineFeature();
		newlfPadded.seg = new GLineSegment2D(linePadded.getPointOfCoordinate(minCoordPadded),linePadded.getPointOfCoordinate(maxCoordPadded));
		newlfPadded.npoints = bestPaddedMatch.npoints + curPadded.npoints;
		allPaddedLines.add(newlfPadded);
	    }



	}//end while()
	
	lines = finalList;
	paddedLines = finalListPadded;
	LinesNeedComputing = false;
	//	System.out.println("Finished making lines: " + lines);
	return finalListPadded;
    }

    private boolean inRange(double val,double ext1, double ext2){
	return (val > ext1 && val < ext2) ||(val > ext2 && val < ext1);
    }

    private ArrayList<LineFeature> rotateAndMoveLineFeatures(ArrayList<LineFeature> featureIn, double[] xyt){
	ArrayList<LineFeature> features = new ArrayList<LineFeature>();
	for(LineFeature l : featureIn){
	    LineFeature lf = new LineFeature();
	    lf.npoints = l.npoints;
	    lf.normal = l.normal;
	    double[] p0 = LinAlg.transform(xyt,l.seg.p1);
	    double[] p1 = LinAlg.transform(xyt,l.seg.p2);
	    lf.seg = new GLineSegment2D(p0,p1);
	    features.add(lf);
	}
	return features;
	
    }

    public void drawTrajectory(VisWorld.Buffer vb){
	ArrayList<double[]> points = new ArrayList<double[]>();
	for(GNode n : g.nodes){
	    points.add(LinAlg.resize(n.state,2));
	}
	VisVertexData vd = new VisVertexData(points);
	vb.addBack(new VzLines(vd,VzLines.LINE_STRIP,new VzLines.Style(new VisConstantColor(Color.red),1)));
	double[] xyt = getCurrentPosition();
	vb.addBack(new VisChain(LinAlg.xyzrpyToMatrix(new double[]{xyt[0],xyt[1],0,0,0,xyt[2]}),new VzRobot()));
	vb.swap();
    }

    public void drawCompiledLaserScan(VisWorld.Buffer vb){
	ArrayList<double[]> points = new ArrayList<double[]>();
	for(Scan s : scanHistory){
	    points.addAll(LinAlg.transform(s.node.state,s.scan));
	    //	    for(ArrayList<double[]> c :s.contours)
	    //	points.addAll(LinAlg.transform(s.node.state,c));
	}
	VisVertexData vd = new VisVertexData(points);
	vb.addBack(new VzPoints(vd, new VzPoints.Style(Color.red,1)));
	vb.swap();
    }

    public void drawScanLines(VisWorld.Buffer vb, boolean drawPadded){
	ArrayList<double[]> points = new ArrayList<double[]>();
	getLines();
	ArrayList<LineFeature> lines = this.lines;
	for(LineFeature lf: lines){
	    points.add(lf.seg.p1);
	    points.add(lf.seg.p2);
	}
	VisVertexData vd = new VisVertexData(points);
	vb.addBack(new VzLines(vd,VzLines.LINES,new VzLines.Style(new VisConstantColor(Color.blue),1)));
	if(drawPadded){
	    points = new ArrayList<double[]>();
	    for(LineFeature lf:paddedLines){
		points.add(lf.seg.p1);
		points.add(lf.seg.p2);
	    }
	    vd = new VisVertexData(points);
	    vb.addBack(new VzLines(vd,VzLines.LINES,new VzLines.Style(new VisConstantColor(Color.green),1)));
	}
	vb.swap();

    }

}