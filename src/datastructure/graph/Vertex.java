package datastructure.graph;

/**
 * Interface for the vertex of a generic Graph data structure 
 * containing <code>data</code> objects
 * @param <D> type of the data object
 */
public interface Vertex<D> {
	
	/**
	 * Returns the vertex data
	 * @return the vertex data
	 */
	public D getData();
	
}
