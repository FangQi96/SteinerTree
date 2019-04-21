package steiner.model;
import org.jgrapht.graph.DefaultWeightedEdge;

public class Edge extends DefaultWeightedEdge{
    private double conductivity;
    public Edge(){
        conductivity = 2;
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
