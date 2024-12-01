// Online C compiler to run C program online
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#define MAX 100
void bfs(int adj[MAX][MAX], int V, int s) {
    // queue for BFS
    int q[MAX], front = 0, rear = 0;
    
    bool visited[MAX] = {false};
    
    visited[s] = true; // mark source as visited
    q[rear++] = s; // unqueue of source
    
    while (front < rear) {
        int curr = q[front++];
        printf(" %d", curr);
        
        // Get all adjacent vertices of the dequeued vertex
        // If an adjacent has not been visited, mark it visited and enqueue it
        for (int i = 0; i < V; i++) {
            if (adj[curr][i] == 1 && !visited[i]) {
                visited[i] = true;
                q[rear++] = i;
            }
        }
    }
}

void addEdge(int adj[MAX][MAX], int u, int v) {
    adj[u][v] = 1;
    adj[v][u] = 1;
}

int main() {
    int V = 5;
    int adj[MAX][MAX] = {0};
    
    addEdge(adj, 0, 1);
    addEdge(adj, 0, 2);
    addEdge(adj, 1, 3);
    //addEdge(adj, 1, 4);
    addEdge(adj, 2, 4);
    
    printf("BFS starting from 0:\n");
    bfs(adj, V, 0);

    return 0;
}
