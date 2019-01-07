package steiner.model;
import org.jgrapht.graph.DefaultWeightedEdge;

public class Edge extends DefaultWeightedEdge{      //methods to modify edge is defined in the graph class
    private double conductivity;
    public Edge(){
        conductivity = 1;
    }

    public double getConductivity() {
        return conductivity;
    }

    public void setConductivity(double conductivity) {
        this.conductivity= conductivity;
    }

    public String toString(){
        return "(vertex " + super.getSource() + " to " + "vertex " + super.getTarget() + ")";
    }
}
