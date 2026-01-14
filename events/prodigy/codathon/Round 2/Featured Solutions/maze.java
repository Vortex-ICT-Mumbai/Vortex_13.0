// This solution is developed by Sai Balkawade
// The sole creditor of this code is Sai Balkawade

import java.util.*;


public class SAI_BALKAWADE_Q3 {



    public static class Graph {
        public int V; // Number of vertices
        public List<List<Edge>> adj; // Adjacency list

        public Graph(int V) {
            this.V = V;
            adj = new ArrayList<>();
            for (int i = 0; i < V; i++) {
                adj.add(new ArrayList<>());
            }
        }

        public void addEdge(int u, int v, double weight) {
            adj.get(u).add(new Edge(v, weight));
        }

        public void addUndirectedEdge(int u, int v, double weight) {
            addEdge(u, v, weight);
            addEdge(v, u, weight);
        }
    }

    public static class Edge {
        public int dest;
        public double weight;

        public Edge(int dest, double weight) {
            this.dest = dest;
            this.weight = weight;
        }
    }

//bfs

    public static int[] bfs(Graph g, int start) {
        int[] dist = new int[g.V];
        Arrays.fill(dist, -1);
        dist[start] = 0;

        Queue<Integer> queue = new LinkedList<>();
        queue.offer(start);

        while (!queue.isEmpty()) {
            int u = queue.poll();
            for (Edge e : g.adj.get(u)) {
                if (dist[e.dest] == -1) {
                    dist[e.dest] = dist[u] + 1;
                    queue.offer(e.dest);
                }
            }
        }
        return dist;
    }

//dfs

    public static boolean[] dfs(Graph g, int start) {
        boolean[] visited = new boolean[g.V];
        dfsHelper(g, start, visited);
        return visited;
    }

    private static void dfsHelper(Graph g, int u, boolean[] visited) {
        visited[u] = true;
        for (Edge e : g.adj.get(u)) {
            if (!visited[e.dest]) {
                dfsHelper(g, e.dest, visited);
            }
        }
    }



    public static class DijkstraResult {
        public double[] dist;
        public int[] prev;

        public DijkstraResult(double[] dist, int[] prev) {
            this.dist = dist;
            this.prev = prev;
        }
    }

    public static DijkstraResult dijkstra(Graph g, int start) {
        double[] dist = new double[g.V];
        int[] prev = new int[g.V];
        Arrays.fill(dist, Double.POSITIVE_INFINITY);
        Arrays.fill(prev, -1);
        dist[start] = 0;

        PriorityQueue<int[]> pq = new PriorityQueue<>(Comparator.comparingDouble(a -> a[1]));
        pq.offer(new int[]{start, 0});

        while (!pq.isEmpty()) {
            int[] curr = pq.poll();
            int u = curr[0];
            double d = curr[1];

            if (d > dist[u]) continue;

            for (Edge e : g.adj.get(u)) {
                double newDist = dist[u] + e.weight;
                if (newDist < dist[e.dest]) {
                    dist[e.dest] = newDist;
                    prev[e.dest] = u;
                    pq.offer(new int[]{e.dest, (int)newDist});
                }
            }
        }

        return new DijkstraResult(dist, prev);
    }

    /**
     * Reconstruct path from Dijkstra result.
     */
    public static List<Integer> reconstructPath(int[] prev, int target) {
        List<Integer> path = new ArrayList<>();
        for (int at = target; at != -1; at = prev[at]) {
            path.add(at);
        }
        Collections.reverse(path);
        return path.isEmpty() || path.get(0) != prev[target] ? path : new ArrayList<>();
    }

    public interface Heuristic {
        double estimate(int node, int goal);
    }

    public static List<Integer> aStar(Graph g, int start, int goal, Heuristic h) {
        double[] gScore = new double[g.V];
        double[] fScore = new double[g.V];
        int[] prev = new int[g.V];

        Arrays.fill(gScore, Double.POSITIVE_INFINITY);
        Arrays.fill(fScore, Double.POSITIVE_INFINITY);
        Arrays.fill(prev, -1);

        gScore[start] = 0;
        fScore[start] = h.estimate(start, goal);

        PriorityQueue<int[]> openSet = new PriorityQueue<>(Comparator.comparingDouble(a -> a[1]));
        openSet.offer(new int[]{start, (int)fScore[start]});

        Set<Integer> closedSet = new HashSet<>();

        while (!openSet.isEmpty()) {
            int current = openSet.poll()[0];

            if (current == goal) {
                return reconstructPath(prev, goal);
            }

            closedSet.add(current);

            for (Edge e : g.adj.get(current)) {
                if (closedSet.contains(e.dest)) continue;

                double tentativeG = gScore[current] + e.weight;

                if (tentativeG < gScore[e.dest]) {
                    prev[e.dest] = current;
                    gScore[e.dest] = tentativeG;
                    fScore[e.dest] = gScore[e.dest] + h.estimate(e.dest, goal);
                    openSet.offer(new int[]{e.dest, (int)fScore[e.dest]});
                }
            }
        }

        return new ArrayList<>(); // No path found
    }

   
    public static long countPaths(Graph g, int start, int goal) {
        long[] memo = new long[g.V];
        Arrays.fill(memo, -1);
        return countPathsHelper(g, start, goal, memo, new boolean[g.V]);
    }

    private static long countPathsHelper(Graph g, int u, int goal, long[] memo, boolean[] visiting) {
        if (u == goal) return 1;
        if (memo[u] != -1) return memo[u];
        
        visiting[u] = true;
        long sum = 0;
        for (Edge e : g.adj.get(u)) {
            if (!visiting[e.dest]) {
                sum += countPathsHelper(g, e.dest, goal, memo, visiting);
            }
        }
        visiting[u] = false;
        return memo[u] = sum;
    }


    public static void main(String[] args) {

        int[][] matrix ={ {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
{1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1},

{1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1},

{1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1},

{1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1},

{1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1},

{1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1},

{1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1},

{1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1},

{1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1},

{1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1},

{1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1},

{1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1},

{1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1},

{1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1},

{1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1},

{1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1},

{1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1},

{1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1},

{1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1},

{1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1},

{1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1},

{1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1},

{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1},

{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}};
    

        System.out.println("=".repeat(60));
        System.out.println("PATH-FINDING ALGORITHMS");
        System.out.println("=".repeat(60));

        // Considering the input matrix as a graph
        Graph g = new Graph(625);
        for(int i =0; i<25; i++){
            for(int j = i; j<25; j++){
            g.addUndirectedEdge(0, 1, matrix[i][j]);
            }
        }
        // g.addUndirectedEdge(0, 1, 4);
        // g.addUndirectedEdge(0, 2, 2);
        // g.addUndirectedEdge(1, 2, 1);
        // g.addUndirectedEdge(1, 3, 5);
        // g.addUndirectedEdge(2, 3, 8);
        // g.addUndirectedEdge(2, 4, 10);
        // g.addUndirectedEdge(3, 4, 2);
        // g.addUndirectedEdge(3, 5, 6);
        // g.addUndirectedEdge(4, 5, 3);

        System.out.println("\nBFS from node 0:");
        int[] bfsDist = bfs(g, 0);
        for (int i = 0; i < bfsDist.length; i++) {
            System.out.printf("  Distance to %d: %d%n", i, bfsDist[i]);
        }

        System.out.println("\nDijkstra from node 0:");
        DijkstraResult result = dijkstra(g, 0);
        for (int i = 0; i < result.dist.length; i++) {
            System.out.printf("  Distance to %d: %.2f%n", i, result.dist[i]);
        }

        System.out.println("\nShortest path from 0 to 5:");
        List<Integer> path = reconstructPath(result.prev, 5);
        System.out.println("  Path: " + path);

    }
}
