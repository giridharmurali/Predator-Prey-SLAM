/* Main Search Class - Specific search algorithms extend from this class */

/* Search Algorithm Options:
   ------------------------
1. Breadth First Search
2. Depth First Search
3. A star search

*/

package team;

import java.io.*;
import java.util.*;
import java.awt.Polygon;
import java.awt.Point;
import java.awt.geom.*;
import java.lang.String;
import java.util.Vector;
import java.util.Collections;

public class search_base {

    Vector<Vector<nodes>> queue = new Vector<Vector<nodes>>();
    Vector<Vector<nodes>> path_q = new Vector<Vector<nodes>>();

    protected void expand(Vector<Vector<nodes>> q, Vector<nodes> ex, Vector<nodes> p, Vector<Vector<nodes>> path_q, Vector<Vector<nodes>> log_nodes, Vector<Integer> log_queue_size) {
        nodes last_node = p.lastElement();
        Vector<nodes> temp_log = new Vector<nodes>();
        temp_log.add(last_node);

        for (int i = 0; i < last_node.visible_nodes.size(); i++) {
            temp_log.add(last_node.visible_nodes.get(i));
        }
        log_nodes.add(temp_log);
        log_queue_size.add(q.size());
        int unexplored_nodes = 0;
        if (last_node.visible_nodes.size() == 0) {
            path_q.add(p);
        } else {
            for (int loop = 0; loop < last_node.visible_nodes.size(); loop++) {
                boolean in_explored = ex.contains(last_node.visible_nodes.get(loop));
                if (in_explored == false) {
                    Vector<nodes> temp_p = new Vector<nodes>();
                    temp_p.addAll(p);
                    temp_p.add(last_node.visible_nodes.get(loop));
                    q.add(temp_p);
                    unexplored_nodes++;
                }
            }
            if (unexplored_nodes == last_node.visible_nodes.size()) {
                path_q.add(p);
            }
        }
    }

    protected boolean possibility(Vector<Vector<nodes>> queue) {
        return !queue.isEmpty();
    }

    protected Vector<nodes> get_next_path(Vector<Vector<nodes>> q, nodes end_node) {
        return q.remove(0);
    }

    public boolean search(nodes start_node, nodes end_node, Vector<nodes> solution, Vector<Vector<nodes>> partial_path, Vector<Vector<nodes>> log_nodes, Vector<Integer> log_queue_size) {
        Vector<nodes> path = new Vector<nodes>();
        Vector<nodes> explored = new Vector<nodes>();
        path.add(start_node); // Start node is always the first node in the path
        queue.add(path);
        while (possibility(queue)) {
            Vector<nodes> temp_p = get_next_path(queue, end_node);
            nodes temp_node = temp_p.lastElement();
            if (temp_node != end_node) {  
                explored.add(temp_node);
                expand(queue, explored, temp_p, path_q, log_nodes, log_queue_size);
            } else {   // End node is the last done in the path
                solution.addAll(temp_p);
                partial_path.addAll(path_q);
                System.out.println("Path Length=" + path_q.size());
                partial_path.addAll(queue);
                System.out.println("Queue Length=" + queue.size());
                //partial_path.remove(partial_path.size() - 2);
                return true;
            }
        }
        return false;
    }
}

class breadth_first extends search_base {

    protected Vector<nodes> get_next_path(Vector<Vector<nodes>> q, nodes end_node) {
        return q.remove(0);
    }
}

class depth_first extends search_base {

    protected Vector<nodes> get_next_path(Vector<Vector<nodes>> q, nodes end_node) {
        return q.remove(q.size() - 1);
    }
}

class a_star extends search_base {

    protected Vector<nodes> get_next_path(Vector<Vector<nodes>> q, nodes end_node) {
        double shortest = 9999999;
        int shortest_node = 0;
        double weight = 0;
        for (int i = 0; i < q.size(); i++) {
            Vector<nodes> p_arb = q.get(i);
            nodes n_arb = p_arb.get(p_arb.size() - 1);
            if (p_arb.size() > 1) {
                nodes n_arb2 = p_arb.get(p_arb.size() - 2);
                weight = (((n_arb2.node_x - n_arb.node_x) * (n_arb2.node_x - n_arb.node_x)) + ((n_arb2.node_y - n_arb.node_y) * (n_arb2.node_y - n_arb.node_y)));
            }
            double heuristic = (((end_node.node_x - n_arb.node_x) * (end_node.node_x - n_arb.node_x)) + ((end_node.node_y - n_arb.node_y) * (end_node.node_y - n_arb.node_y)));

            if (heuristic + weight < shortest) {
                shortest_node = i;
                shortest = heuristic + weight;
            }
        }
        return q.remove(shortest_node);
    }
}


//------------ End of search methods ----------- //


