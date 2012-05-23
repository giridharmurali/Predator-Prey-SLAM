/*
 * Mobile Robotics Final Project
 * Prey Class
 */

// Path planning to new location if hiding spot not available. Else, path plan to same hiding spot.

// -5 - 25  -15  15

package team;

import java.io.*;
import java.util.*;
import java.awt.Polygon;
import java.awt.Point;
import java.awt.geom.*;
import java.lang.String;
import java.util.Vector;
import java.util.Collections;
import april.laser.*;
import april.config.*;

/**
 *
 * @author Giridhar Murali
 */

/* Main Prey Class */

public class Prey {

//    static ArrayList<Polygon> polygonsArr;
    Vector<nodes> node = new Vector<nodes>();
    Vector<nodes> path_to_hide = new Vector<nodes>(); 
    double[] prey_pos = new double[2];
    double[] pred_pos = new double[2];
    double[] hide_pos = new double[2];
    double[] run_to_point = new double[2];

    public SLAM slam;
   
    public Prey(double[] prey_pos1, double[] pred_pos1, Config c) { 
	slam = new SLAM(c, prey_pos1);
	prey_pos[0] = prey_pos1[0];
	prey_pos[1] = prey_pos1[1];
	pred_pos[0] = pred_pos1[0];
	pred_pos[1] = pred_pos1[1];
      
	System.out.println("Prey Created");
    }

 public void move(double[] dldr, ArrayList<double[]> laserscan){
     slam.motion(dldr,laserscan,false);
    }

public Vector<nodes> ping_prey_status(double[] prey_pos,double[] pred_pos,ArrayList<LineFeature> line, ArrayList<LineFeature> map)
{
    node.clear();
    path_to_hide.clear();
		   if(!ispreysafe(prey_pos, pred_pos, map)){ // If prey is safe, continue SLAMming
			System.out.println("Prey is vulnerable. Initiating Hiding Routine...");
		   initiate_hiding_routine(prey_pos, pred_pos,line,map);
         		}
		   return path_to_hide;	   	// Empty if prey is safe.
		    
}

public void run ( double[] run_to_point)
{

System.out.println("Running to ( " + run_to_point[0] + "," + run_to_point[1] + ")");
path_to_hide.clear();
nodes node = new nodes();
node.node_x = run_to_point[0];
node.node_y = run_to_point[1];
path_to_hide.add(node);
// make a new node with run_to_point and give that 
}
public void initiate_hiding_routine(double[] prey_pos, double[] pred_pos, ArrayList<LineFeature> line, ArrayList<LineFeature> map){
  
//   hide_pos = find_hiding_spot(prey_pos, pred_pos, line, map); // Find a hiding spot
//   if((hide_pos[0]!=prey_pos[0]) && (hide_pos[1]!=prey_pos[1]))
//      {
//	System.out.println("Hiding Spot Found. Searching for best path..");
	 path_to_hide = find_path(prey_pos, pred_pos, line,map);  // Find best path to the chosen hiding spot
//      }
   //          }while(path_to_hide.size() == 0); // Until a search solution is returned.
      // 	hide(path_to_hide); // From parent class (Locomotion)
}



/* Check if the prey is not in direct line of sight from the predator */
    
     boolean ispreysafe(double[] prey_pos,double[] pred_pos, ArrayList<LineFeature> line) {
        boolean result = false;
        int node_limit = 0;
        int i1, i2;
        Line2D.Double current_line = new Line2D.Double(prey_pos[0],prey_pos[1],pred_pos[0],pred_pos[1]);
        for (int i = 0; i < line.size(); i++) {
              result = current_line.intersectsLine(line.get(i).seg.p1[0], line.get(i).seg.p1[1], line.get(i).seg.p2[0],line.get(i).seg.p2[1]);
                if(result == false)
		 continue;
		else if (result == true)
		{
		  System.out.println("Prey is safe!");
                 return result;
		}
         }
	return result;
    }



/* Given predator and prey positions, search for hiding spot */

public double[] find_hiding_spot(double[] prey_pos, double[] pred_pos, ArrayList<LineFeature> line, ArrayList<LineFeature> map){

	//double x_diff = prey_pos[0]-pred_pos[0];
	//double y_diff = prey_pos[1]-pred_pos[1];
	//double Min = -2.5;
	//double Max =  2.5;
	System.out.println("Entered find_hiding_spot method.");
	hide_pos[0]=prey_pos[0];
	hide_pos[1]=prey_pos[1];
	double[] new_prey_pos = new double[2];
	double min_weight = 999999;
	int nearest_node_to_hide = -1;
	//double change_x, change_y;
	 /* x_diff = Math.abs(x_diff);
	y_diff = Math.abs(y_diff); */
/*
	if(x_diff >= y_diff){
	  while (!ispreysafe(new_prey_pos,pred_pos,line))
	   {
	     if(sign_count%2==0)
		sign = 1;
	     else 
		sign = -1;
             change = sign * (scale_count/2);
             sign_count++;
	     scale_count++;
	     new_prey_pos[0]=new_prey_pos[0]+change;		
        	};
		   }
	else{
	 while (!ispreysafe(new_prey_pos,pred_pos,line))
	   {
	     if(sign_count%2==0)
		sign = 1;
	     else 
		sign = -1;
	     change = sign * (scale_count/2);
             sign_count++;
	     scale_count++;
	     new_prey_pos[1]=new_prey_pos[1]+change;	
        	};
	 } */

/*int loop;
for(loop=0;loop< 20;loop++){
change_x = Min + (double)(Math.random() * ((Max - Min) + 0.4));
change_y = Min + (double)(Math.random() * ((Max - Min) + 0.4));
hide_pos_temp[0]=prey_pos[0] + change_x;
hide_pos_temp[1]=prey_pos[1] + change_y; 
if(ispreysafe(hide_pos,pred_pos,line) && hide_pos_temp[0]>-5 && hide_pos_temp[0]<25 && hide_pos_temp[1]>-15 && hide_pos_temp[1]<15 )
break;
}
if(loop == 50)
return hide_pos;
else
{
hide_pos[0]=hide_pos_temp[0];
hide_pos[1]=hide_pos_temp[1];
return hide_pos;
}*/

for(int i=0;i<node.size();i++)
{
new_prey_pos[0]=node.get(i).node_x;
new_prey_pos[1]=node.get(i).node_y;
nodes pn = new nodes();
pn.node_x=prey_pos[0];
pn.node_y=prey_pos[1];
System.out.println("Testing hiding spot : "+ new_prey_pos[0] + " " + new_prey_pos[1]);
if(ispreysafe(new_prey_pos,pred_pos,map))
{
hide_pos[0]=node.get(nearest_node_to_hide).node_x;
hide_pos[1] = node.get(nearest_node_to_hide).node_y;
return hide_pos;
}
}
/*if(ispreysafe(new_prey_pos,pred_pos,map) && min_weight>node.get(i).weight_between_nodes(pn,node.get(i)))
{
min_weight = node.get(i).weight_between_nodes(node.get(0),node.get(i));
System.out.println("Min weight is : " + min_weight); 
nearest_node_to_hide = i;
}
}*/
/*if(nearest_node_to_hide !=-1)
{
hide_pos[0]=node.get(nearest_node_to_hide).node_x;
hide_pos[1] = node.get(nearest_node_to_hide).node_y;
}*/
return hide_pos;
}

// Generate list of visible nodes from all other nodes.
public void inter_polygon_mapping() {
        System.out.println("Inter Polygon Mapping Initiated..");
        for (int iter = 0; iter < node.size() - 1; iter++) {
            for (int iter2 = iter + 1; iter2 < node.size(); iter2++) {
                       if (check_intersect(node.get(iter), node.get(iter2)) == false) {
			   //System.out.println("Connection Found");
                        double weights_temp;
                        node.get(iter).visible_nodes.add(node.get(iter2));
			//System.out.println("Step 1 done");
                        weights_temp = node.get(iter).weight_between_nodes(node.get(iter), node.get(iter2));
			//System.out.println("Weight calculated");
                        node.get(iter).weights.add(weights_temp);
			//System.out.println("Step 2 done");
                        node.get(iter2).visible_nodes.add(node.get(iter));
			//System.out.println("Step 3 done");
                        node.get(iter2).weights.add(weights_temp);
			//System.out.println("All Steps Done.");
		}
		       //		System.out.println(iter+ " , " + iter2+ " done.");
            }
        }
    }
/* Check if line between given nodes intersects with any of the lines in the map */

    public boolean check_intersect(nodes n1, nodes n2) { 
        boolean result = false;
        int node_limit = 0;
        int i1, i2;
        Line2D.Double current_line = new Line2D.Double(n1.node_x, n1.node_y, n2.node_x, n2.node_y);

        for (int i = 0; i < node.size(); i++) 
	{
	 for(int j = 0; j< node.size(); j++)
           {
	       if((n1==node.get(i)&&n2==node.get(j)) || (n1==node.get(j) && n2==node.get(i)))
		   continue;
		else
		{
		result = current_line.intersectsLine(node.get(i).node_x, node.get(i).node_y, node.get(j).node_x, node.get(j).node_y);
                  if (result == true) {
                    return result;
                   }
		}
           } 
        }
   return result;
    }

/* Do search and find path between two points */
    
public Vector<nodes> find_path(double[] prey_pos,double[] pred_pos, ArrayList<LineFeature> line, ArrayList<LineFeature> map) {    	
        int algo = 3; // A Star = 3 // Choose search Algorithm
	double start_x, start_y, end_x, end_y;  // Starting and Ending Points for Path Search
        //Prey p = new Prey();  // Create new prey object
	Vector<nodes> solution = new Vector<nodes>();
        Vector<Vector<nodes>> partial_path = new Vector<Vector<nodes>>();
    	Vector<Vector<nodes>> log_nodes = new Vector<Vector<nodes>>();
    	Vector<Integer> log_queue_size = new Vector<Integer>();
 
       try {
            int node_counter = -1;
            double x,y;
	    if(line.size()<4)
		{
		  System.out.println("Not enough edges for path planning");
		   return solution;
		 }
          else
 	    {
              for(int j=0; j<line.size(); j++) {  // Create nodes for all points
                    node_counter++;
                    x = line.get(j).seg.p1[0];
                    y = line.get(j).seg.p1[1];
                    node.add(new nodes());
                    node.get(node_counter).node_x = x;
                    node.get(node_counter).node_y = y;
                    node.get(node_counter).polygon_id = j;
                    node.get(node_counter).poly_size = 2;
		    node_counter++;
		    x = line.get(j).seg.p2[0];
	            y = line.get(j).seg.p2[1];
		    node.add(new nodes());
                    node.get(node_counter).node_x = x;
                    node.get(node_counter).node_y = y;
                    node.get(node_counter).polygon_id = j;
                    node.get(node_counter).poly_size = 2;	
	               }
                // Self-Connect Routine
                System.out.println("Self-Connect Routine Engaged with " + node_counter + " Nodes");
                double weight_temp = 0;
                for (int i = 0; i < node_counter; i++) {
                      if (i == node_counter - 1) {
                        //System.out.println("Entered If Condition");
                        node.get(i).visible_nodes.add(node.get(node_counter));
                        weight_temp = node.get(i).weight_between_nodes(node.get(i), node.get(node_counter));
                        node.get(i).weights.add(weight_temp);
                        node.get(node_counter).visible_nodes.add(node.get(i));
                        node.get(node_counter).weights.add(weight_temp);
                        //System.out.println("First-last Matched!");
                      }
                    //System.out.println("Starting Connections..");
                    node.get(i).visible_nodes.add(node.get(i + 1));
                    //System.out.println("1");
                    weight_temp = node.get(i).weight_between_nodes(node.get(i), node.get(i + 1));
                    //System.out.println("2");
                    node.get(i).weights.add(weight_temp);
                    //System.out.println("3");
                    node.get(i + 1).visible_nodes.add(node.get(i));
                    node.get(i + 1).weights.add(weight_temp);
		    //System.out.println("Printing nodes");
		    //System.out.println("Node " + i + ": " + node.get(i).node_x + " , " + node.get(i).node_y);
                    //System.out.println("Subsequent Matching Progress..");
                     }
                //------------------------------------------------------------------------
              
            //System.out.println("Self Connect Routine Successful!");
            //  self_connect_nodes();
                    inter_polygon_mapping();
                       System.out.println("Interpolygon Mapping Done!");
    
		       hide_pos=find_hiding_spot(prey_pos,pred_pos,line,map);
		  if((hide_pos[0]!=prey_pos[0]) && (hide_pos[1]!=prey_pos[1]))
		      {

            		// Printing Polygon Elements
            		start_x = prey_pos[0];
		            start_y = prey_pos[1];
			    end_x = hide_pos[0];
			    end_y = hide_pos[1];
		            int start_present = 0, end_present = 0;
		            nodes start_node = null, end_node = null;
		            System.out.println("Start:" + start_x + "," + start_y + "  End:" + end_x + "," + end_y);
		            node_counter++;
		            //Routine to check existence of points
		            for (int j = 0; j < node.size(); j++) {
                		if ((start_x == node.get(j).node_x) && (start_y == node.get(j).node_y)) {
               		     start_node = node.get(j);
                    		start_present = 1;
		                }
        		       }
            if (start_present == 1) {
                System.out.println("Start is already a Node.");
            } else {
                System.out.println("Start node added..");
                nodes n = new nodes();
                n.node_x = start_x;
                n.node_y = start_y;
                n.poly_size = 1;
                n.polygon_id = -1;
                node.add(n);
                start_node = n;
                //node.get(node_counter).polygon_id=polygon_counter;
                n.map_input_nodes(n,node);
                node_counter++;
                //p.addPoint(x,y);
            }

            for (int j = 0; j < node.size(); j++) {
                if ((end_x == node.get(j).node_x) && (end_y == node.get(j).node_y)) {
                    end_present = 1;
                    end_node = node.get(j);
                }
            }
            if (end_present == 1) {
                System.out.println("End is already a Node.");
            } else {
                System.out.println("End node added..");
                end_node = new nodes();
                end_node.node_x = end_x;
                end_node.node_y = end_y;
                //end_node.polygon_id=polygon_counter;
                end_node.poly_size = 1;
                node.add(end_node);
                end_node.map_input_nodes(end_node,node);
            }


            // Printing Visibility Graph
            for (int i = 0; i < node.size(); i++) {
                System.out.println("Node (" + node.get(i).node_x + "," + node.get(i).node_y + ")");
                System.out.println("Visible Nodes");
                for (int j = 0; j < node.get(i).visible_nodes.size(); j++) {
                    System.out.println(node.get(i).visible_nodes.get(j).node_x + "," + node.get(i).visible_nodes.get(j).node_y);
                }
            }
            
            boolean success = true;
            search_base s = new search_base();
            if (algo == 1) {
                s = new breadth_first();
            } else if (algo == 2) {
                s = new depth_first();
            } else if (algo == 3) {
                s = new a_star();
            }
           success = s.search(start_node, end_node, solution, partial_path, log_nodes, log_queue_size);
            System.out.println(success);
            if (success) {
		 System.out.println("Best path found!");               
		 return solution;
               }
	   else {
	     System.out.println("No Solution Exists. Failed to find best path.");	
	     return solution;
           }
}
else
{	            System.out.println("No hiding spot found. Preparing to run..");	
		 double x_diff = prey_pos[0]-pred_pos[0];
		   double y_diff = prey_pos[1]-pred_pos[1];
		   double abs_x_diff = Math.abs(x_diff);
		   double abs_y_diff = Math.abs(y_diff);
		double change_x = -2.5 + (double)(Math.random() * 12.5);
		double change_y = -2.5 + (double)(Math.random() * 12.5);
	
		//do{
		   if((abs_x_diff>abs_y_diff))
		      {
		        run_to_point[0]=prey_pos[0];
			if(y_diff >0)
			run_to_point[1]=prey_pos[1]+change_y;
			else
			run_to_point[1]=prey_pos[1]-change_y;
                 	}
		    else
			{
			  run_to_point[1]=prey_pos[1];
                          if(x_diff >0)
			  run_to_point[0]=prey_pos[0]+change_x;
			  else
			  run_to_point[0]=prey_pos[0]-change_x;
		          
				}
//}while((run_to_point[0]<-5)&&(run_to_point[0]>25)&&(run_to_point[1]<-15)&&(run_to_point[1]>15));
		run(run_to_point); 
   }	

}
       }catch (Exception e) {
            System.out.println("Catch Block Entered! Prepare to debug:" + e.toString());
	    e.printStackTrace();
	    return null;
        }
return null;
} 

}


