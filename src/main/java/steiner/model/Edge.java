package steiner.model;
import org.jgrapht.graph.DefaultWeightedEdge;

public class Edge extends DefaultWeightedEdge{      //Methods for modifying edge is defined in the graph class
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
        return String.format("%.2f",this.getConductivity()) + " ("+super.getWeight() + ")" ;
    }

    public Object getSource(){
        return super.getSource();
    }

    public Object getTarget(){
        return super.getTarget();
    }

    public double getWeight(){
        return super.getWeight();
    }
}
