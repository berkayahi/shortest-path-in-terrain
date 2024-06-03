#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "GRAPH_SEARCH.h"
#include "data_types.h"

int main()
{	
    initializePathCosts();
    initializeSLDMatrix(MATRIX_SIZE);
	printf("Terrain is defined as below:\n");
    // Dynamically generate and print state names for a terrain (grid).
    for (int i = 0; i < MATRIX_SIZE; i++) {
        for (int j = 0; j < MATRIX_SIZE; j++) {
            // Print state name based on grid position
            printf("%c%c ", 'A' + i, '1' + j);
        }
        printf("\n");  // Newline after each row
    }
    printf("\n");
    
    Node root, *goal;
    State *goal_state = NULL;
    enum METHODS method;
    int Max_Level, level;
	float alpha;
	
    // This part must be updated if a new algorithm is added. 
    printf("1 --> Breadth-First Search\n");
    printf("2 --> Uniform-Cost Search\n");
    printf("3 --> Depth-First Search\n");
    printf("4 --> Depth-Limited Search\n");
    printf("5 --> Iterative Deepening Search\n");
    printf("6 --> Greedy Search\n");
    printf("7 --> A* Search\n");
    printf("8 --> Generalized A* Search\n");
    printf("Select a method to solve the problem: ");
    scanf("%d", (int*)&method);
    if(method==DepthLimitedSearch){
	    printf("Enter maximum level for depth-limited search : ");                         
	    scanf("%d", &Max_Level);                  
	}  
	if(method==GeneralizedAStarSearch){
	    printf("Enter value of alpha for Generalized A* Search : ");                         
	    scanf("%f", &alpha);                  
	}   
    
    // Creating the root node ... 
    root.parent    = NULL;
    root.path_cost = 0;
    root.action    = NO_ACTION; // The program will not use this part. (NO_ACTION-->0)
    root.Number_of_Child = 0;
    	
    printf("======== SELECTION OF INITIAL STATE =============== \n");
    root.state     = *(Create_State());
    
    if(PREDETERMINED_GOAL_STATE)  // User will determine the goal state if it is true
    {
	    printf("======== SELECTION OF GOAL STATE =============== \n"); 
	    goal_state = Create_State();
    }
    
    clock_t start_time = clock(); // Start the timer to calculate program runtime
    
    if(method==GreedySearch || method==AStarSearch || method==GeneralizedAStarSearch){
        root.state.h_n  = Compute_Heuristic_Function(&(root.state), goal_state);
        if(PREDETERMINED_GOAL_STATE)
        	goal_state->h_n = 0;                 
	}  
    	
    switch(method) 
    {
        case BreadthFirstSearch: 
        case GreedySearch:               
            goal = First_GoalTest_Search_TREE(method, &root, goal_state);  break; 
		case DepthFirstSearch: 	
		case DepthLimitedSearch: 
			goal = DepthType_Search_TREE(method, &root, goal_state, Max_Level);  break;  
        case IterativeDeepeningSearch:
            for(level=0; TRUE ;level++){
            	goal = DepthType_Search_TREE(method, &root, goal_state, level);
            	if(goal!=FAILURE){
            		printf("The goal is found in level %d.\n", level); 
            		break;
				}		
			}
            break;     
        case UniformCostSearch: 
        case AStarSearch:   
		case GeneralizedAStarSearch:   
            goal = First_InsertFrontier_Search_TREE(method, &root, goal_state, alpha);  break;      
     
        default: 
            printf("ERROR: Unknown method.\n");  
            exit(-1);    
    }

    Show_Solution_Path(goal);
    printf("\n");
    // Calculate the runtime of the program.
    clock_t end_time = clock();
    double total_time = (double)(end_time - start_time) / CLOCKS_PER_SEC;
    printf("\nTotal processing duration of the program is: %f seconds\n\n", total_time);
    // Dynamically generate and print state names for a grid (terrain).
    for (int i = 0; i < MATRIX_SIZE; i++) {
        for (int j = 0; j < MATRIX_SIZE; j++) {
            printf("%c%c ", 'A' + i, '1' + j);
        }
        printf("\n");
    }
    printf("\nCode is completed.");
    return 0;
}

