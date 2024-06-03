/* 
    These functions are compulsory for search algorithms but they are specific
    to problems. More clearly, you must must update their blocks but do not change
    their input and output parameters.
    
    Also, you can add new functions at the end of file by declaring them in GRAPH_SEARCH.h
*/

#include "GRAPH_SEARCH.h"
#include "data_types.h"
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>  // For INT_MAX
#include <math.h>    // For fabs function

int PATH_COST[MATRIX_SIZE * MATRIX_SIZE][MATRIX_SIZE * MATRIX_SIZE];

//______________________________________________________________________________
State* Create_State()
{
	State *state = (State*)malloc(sizeof(State));
    if(state==NULL)
    	Warning_Memory_Allocation(); 
   
   	for(state->cell=0; state->cell<MATRIX_SIZE * MATRIX_SIZE; state->cell++){        
        printf("%d --> ", state->cell);
        Print_State(state);
        printf("\n");
    }
   
   	do{ 
    	printf("Enter the code of the state: ");
        scanf("%d", (int*)&state->cell);
   	}while(state->cell<0 || state->cell>=MATRIX_SIZE * MATRIX_SIZE);
	       
    return state;    
}

//______________________________________________________________________________
void Print_State(const State *const state) {
    int row = state->cell / MATRIX_SIZE;  // Use MATRIX_SIZE to adjust dynamically
    int col = state->cell % MATRIX_SIZE;

    char row_label = 'A' + row;  // Characters for rows
    int col_label = col + 1;     // Numeric columns starting from 1, i.e. A1,A2,A3,A4,A5,B1,B2,etc.

    printf("%c%d", row_label, col_label);
}

//______________________________________________________________________________
void Print_Action(const enum ACTIONS action) {
    switch(action){
        case Go_Up: printf("Go Up"); break;
        case Go_Down: printf("Go Down"); break;
        case Go_Left: printf("Go Left"); break;
        case Go_Right: printf("Go Right"); break;
    }
}

//______________________________________________________________________________
int Result(const State *const parent_state, const enum ACTIONS action, Transition_Model *const trans_model)
{
    int row = parent_state->cell / MATRIX_SIZE; // Use MATRIX_SIZE to adjust dynamically
    int col = parent_state->cell % MATRIX_SIZE; // Use MATRIX_SIZE to adjust dynamically
    int new_row = row, new_col = col;
            
    switch(action) {
        case Go_Up:
            if (row > 0) new_row = row - 1; // Move up if not in the first row
            break;
        case Go_Down:
            if (row < MATRIX_SIZE - 1) new_row = row + 1; // Move down if not in the last row
            break;
        case Go_Left:
            if (col > 0) new_col = col - 1; // Move left if not in the first column
            break;
        case Go_Right:
            if (col < MATRIX_SIZE - 1) new_col = col + 1; // Move right if not in the last column
            break;
    }
	 
    int new_cell = new_row * MATRIX_SIZE + new_col; // Calculate the new index
    if (parent_state->cell != new_cell && PATH_COST[parent_state->cell][new_cell] > 0) {
        // Check if there's a valid path cost and it's not the same cell
        State new_state = *parent_state;
        new_state.cell = new_cell;
        trans_model->new_state = new_state;
        trans_model->step_cost = PATH_COST[parent_state->cell][new_cell];
        return TRUE;
    }
    return FALSE;                                            
}

//______________________________________________________________________________
float Compute_Heuristic_Function(const State *const state, const State *const goal)
{
    float SLD[MATRIX_SIZE * MATRIX_SIZE][MATRIX_SIZE * MATRIX_SIZE];
    
    return SLD[state->cell][goal->cell];   
}

//_______________ Update if your goal state is not determined initially ___________________________________
int Goal_Test(const State *const state, const State *const goal_state)
{
	if(PREDETERMINED_GOAL_STATE)	
		return Compare_States(state, goal_state); 
	else
		return 1;
}

// ==================== WRITE YOUR OPTIONAL FUNCTIONS (IF REQUIRED) ==========================

// =============================== PATH_COST MATRIX GENERATOR =====================================
Terrain charToTerrain(char c) {
    switch (c) {
        case 'P': return Plain;
        case 'H': return Hill;
        case 'B': return Barrier;
        case 'S': return Sea;
        default:  return Barrier;  // Default to Barrier for safety.
    }
}

int areAdjacent(int start, int end, int size) {
    int startRow = start / size;
    int startCol = start % size;
    int endRow = end / size;
    int endCol = end % size;

    return (startRow == endRow && abs(startCol - endCol) == 1) ||
           (startCol == endCol && abs(startRow - endRow) == 1);
}

int calculateCost(Terrain** terrain, int start, int end, int size) {
    if (start == end) return 0;
    if (!areAdjacent(start, end, size)) return -1;

    Terrain startTerrain = terrain[start / size][start % size];
    Terrain endTerrain = terrain[end / size][end % size];

    if (startTerrain == Barrier || endTerrain == Barrier) return -99;

    switch (endTerrain) {
        case Plain: return 1;
        case Sea: return 3;
        case Hill:  return 5;
        default:    return -99;
    }
}

void initializePathCosts() {
    char terrainMap[MATRIX_SIZE][MATRIX_SIZE] = {
        {'P', 'S', 'B', 'P', 'P',},
        {'P', 'P', 'B', 'H', 'P'},
        {'B', 'P', 'H', 'S', 'P'},
        {'H', 'P', 'P', 'P', 'B'},
        {'P', 'S', 'P', 'P', 'P'}
    };

    Terrain** terrain = malloc(MATRIX_SIZE * sizeof(Terrain*));
    for (int i = 0; i < MATRIX_SIZE; i++) {
        terrain[i] = malloc(MATRIX_SIZE * sizeof(Terrain));
        for (int j = 0; j < MATRIX_SIZE; j++) {
            terrain[i][j] = charToTerrain(terrainMap[i][j]);
        }
    }

    for (int i = 0; i < MATRIX_SIZE * MATRIX_SIZE; i++) {
        for (int j = 0; j < MATRIX_SIZE * MATRIX_SIZE; j++) {
            PATH_COST[i][j] = calculateCost(terrain, i, j, MATRIX_SIZE);
        }
    }
}

// =============================== SLD HEURISTIC MATRIX GENERATOR =====================================
float calculateDistance(int start, int end, int size) {
    int startRow = start / size;
    int startCol = start % size;
    int endRow = end / size;
    int endCol = end % size;

    int dx = abs(startRow - endRow);
    int dy = abs(startCol - endCol);

    return sqrt(dx * dx + dy * dy);
}

// Initialize the heuristic matrix (SLD Matrix) on the GRAPH_SEARCH.c file
void initializeSLDMatrix(int size) {
    float** heuristicMatrix = (float**)malloc(size * size * sizeof(float*));
    for (int i = 0; i < size * size; i++) {
        heuristicMatrix[i] = (float*)malloc(size * size * sizeof(float));
        for (int j = 0; j < size * size; j++) {
            heuristicMatrix[i][j] = calculateDistance(i, j, size);
        }
    }
}
