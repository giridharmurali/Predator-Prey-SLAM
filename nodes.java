package team;

import java.io.*;
import java.util.*;
import java.awt.Polygon;
import java.awt.Point;
import java.awt.geom.*;
import java.lang.String;
import java.util.Vector;
import java.util.Collections;

/* Node Class - Represents distinct nodes in the map to be searched for hiding spots */
public class nodes {  

    public double node_x, node_y;
    public int polygon_id, poly_size;
    public Vector<nodes> visible_nodes = new Vector<nodes>(); // Vector of visible nodes from a given point.
    public Vector weights = new Vector(); // Weight is just squared distance

/* Calculates distance between two nodes */

    public double weight_between_nodes(nodes n1, nodes n2) {
        double weight = ((n2.node_x - n1.node_x) * (n2.node_x - n1.node_x)) + ((n2.node_y - n1.node_y) * (n2.node_y - n1.node_y));
        return weight;
    }


/* Check if line between given nodes intersects with any of the lines in the map */

    public boolean check_intersect(nodes n1, nodes n2, Vector<nodes> node) { 
        boolean result = false;
        int node_limit = 0;
        int i1, i2;
        Line2D.Double current_line = new Line2D.Double(n1.node_x, n1.node_y, n2.node_x, n2.node_y);
        for (int i = 0; i < node.size(); i++) {
            if (node_limit < node.get(i).poly_size - 1) {
                i1 = i;
                i2 = i1 + 1;
            } else {
                i1 = i;
                i2 = i1 - node.get(i).poly_size + 1;
                node_limit = -1;
            }
            node_limit++;
            if ((n1 == node.get(i1)) || (n1 == node.get(i2)) || (n2 == node.get(i1)) || (n2 == node.get(i2))) {
                continue;
            } else {
                result = current_line.intersectsLine(node.get(i1).node_x, node.get(i1).node_y, node.get(i2).node_x, node.get(i2).node_y);
                if (result == true) {
                    return result;
                }
            }
        }

        return result;
    }



public void map_input_nodes(nodes n, Vector<nodes> node) {
        for (int iter = 0; iter < node.size(); iter++) {
            if (node.get(iter) == n) {
                continue;
            }
            if (check_intersect(n, node.get(iter),node) == false) {
                double weights_temp;
                n.visible_nodes.add(node.get(iter));
                weights_temp = node.get(iter).weight_between_nodes(n, node.get(iter));
                node.get(iter).weights.add(weights_temp);
                node.get(iter).visible_nodes.add(n);
                n.weights.add(weights_temp);
            }

        }
    }
}

