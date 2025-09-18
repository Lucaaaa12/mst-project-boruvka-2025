package datastructure.graph;

/**
 * Interface for the edge of a generic Graph data structure 
 * with vertexes containing <code>data</code> objects
 * @param <D> type of the data object
 */
public interface Edge<D> {

	/**
	 * Returns the source vertex of the edge
	 * @return the source vertex of the edge
	 */
	public Vertex<D> getSource();

	/**
	 * Returns the destination vertex of the edge
	 * @return the destination vertex of the edge
	 */
	public Vertex<D> getDest();
	
	/**
	 * Returns the weight of the edge
	 * @return the weight of the edge
	 */
	public double getWeight();

}
