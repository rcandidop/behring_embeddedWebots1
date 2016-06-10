#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <limits.h>

#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/led.h>
#include <webots/gps.h>

#define GROWING_STEPS 1
#define WAIT_UNTIL_CONNECTED 1
#define DO_NOT_RECONNECT_ONCE_DISC 1
#define TIME_OUT_IN_MS 2000

#define M_PI 3.14159265358979

#define WHEEL_RADIUS 0.205  //0.018750 // avg. wheel radius of the e-puck 1850.
#define WHEELBASE 0.053
#define ENCODER_RESOLUTION 159.23
#define REV_STEP 1000
#define STEP_TOLERANCE 6.0
#define RANGE (1024 / 2)
#define MIN_DIST 20.0f
#define MIN_SPEED 10.0f
#define NTOURNAMENTS 1 //5

#define CELL_SIZE 0.07

#define TIME_STEP 8
#define LEFT 0
#define RIGHT 1


double dMSpeed[2] = {0.0f, 0.0f}; // global buffer to store the current speed values.
double dPrevEncPos[2] = {0.0f, 0.0f}; // global buffer to save the previous encoder postions.
double *p_dOdometryData; // global buffer to save the current odometry data.
WbDeviceTag led[3]; // LEDs.

////rb_red_blk_//tree* //tree;

const double EPUCK_DIAMETER = 0.07f;


typedef enum {
  FREE_CELL,
  REAL_OBSTACLE_CELL,
  FAKE_OBSTACLE_CELL,
  INITIAL_POSITION_CELL,
  GOAL_POSITION_CELL
} BehringCells;

typedef struct point {
  float mX;
  float mY;
} Point;

typedef struct rect {
  Point mOrigin;
  Point mEnd;
} Rect;

typedef struct ePuck {
  WbDeviceTag mEPuckHandle;
  //int mLeftJoint;
  //int mRightJoint;
  WbDeviceTag mGPSHandle;
  Rect mRobotRect;
  double mRobotWidth;
  double mRobotHeight;
  float* mPosition;
  double mApproximateTheta;
} EPuck;

typedef struct floor {
  WbDeviceTag mFloorHandle;
  Rect mFloorRect;
  BehringCells** pmDiscretizedFloor;
  float mContinuousFloorWidth;
  float mContinuousFloorHeight;
  int mDiscretizedFloorWidth;
  int mDiscretizedFloorHeight;
} Floor;

typedef struct goal {
  WbDeviceTag mGoalHandle;
  int mGridColumn;
  int mGridLine;
} Goal;

typedef struct obstacle {
  WbDeviceTag mObstacleHandle;
  Rect mObstacleRect;
  float* mPosition;
} Obstacle;

typedef enum {
  NORTH,
  NORTHWEST,
  WEST,
  SOUTHWEST,
  SOUTH,
  SOUTHEAST,
  EAST,
  NORTHEAST
} RobotDirection;

typedef enum {
  EPUCK_NO_ERROR,
  EPUCK_ARGUMENTS_ERROR,
  EPUCK_CONNECTION_ERROR,
  EPUCK_HANDLE_NOT_FOUND_ERROR,
  EPUCK_POSITION_NOT_FOUND_ERROR,
  EPUCK_PARAMETER_NOT_FOUND_ERROR,
  EPUCK_JOINT_VELOCITY_ERROR
} error_code;

typedef enum {
  OBJECT_MIN_X_POSITION = 15,
  OBJECT_MIN_Y_POSITION,
  OBJECT_MIN_Z_POSITION,
  OBJECT_MAX_X_POSITION,
  OBJECT_MAX_Y_POSITION,
  OBJECT_MAX_Z_POSITION
} object_parameter;

typedef enum { FALSE, TRUE } t_bool;

void EndClient(EPuck* robot, error_code error);

void goto_position(float x, float y, float theta);
//static void print_position(void);

// motion control methods:
static void stop_robot();
void move_forward(double dSpeed, double dDist);
//static void turn_left(double dSpeed);
//static void turn_right(double dSpeed);
void turn_angle(double dAngle, double dSpeed, char rad);
static void set_motor_speed(double dSpeedL, double dSpeedR);
static double* get_encoder_positions();

// odometry:
static double* compute_odometry_data();

// LEDs:
//static void set_leds(int iActive);

//Connection related functions
error_code Connect(char* host, int port);
t_bool HasConnection();
void Disconnect();

//Message related functions
error_code GetObjectHandle(char* objectName, WbDeviceTag* handle);
error_code GetObjectPosition(WbDeviceTag gpsHandle, float* position);
error_code GetObjectOrientation(WbDeviceTag handle, float* orientation);
error_code GetObjectFloatParameters(WbDeviceTag handle, float* parameterValue, object_parameter parameter);
error_code GetObjectProximitySensor(WbDeviceTag handle, char* detectionState, float* detectedPointCoords, int* detectedObjectHandle);
error_code SetObjectPosition(WbDeviceTag handle, float* position);
error_code SetJointVelocity(int handle, float velocity);
error_code SetDifferentialWheelsVelocity(float leftSpeed, float rightSpeed);

error_code PauseSimulation(int timeInMs);

error_code InitController(char* host, int port, EPuck* robot);

error_code Control(EPuck* robot);
void OdometryMove(EPuck* robot, RobotDirection positionToGo);
void OdometryRotate(EPuck* robot, RobotDirection positionToGo);
error_code Move(EPuck* robot, Floor* floor, RobotDirection positionToGo);
error_code Rotate(EPuck* robot, RobotDirection positionToGo);


int GetDirectionList(int initialLine, int initialColumn, int gridWidth, int gridHeight, int** distance, int** approximation, RobotDirection** directions);

error_code BuildEPuck();
error_code BuildFloor(Floor* floor, EPuck* robot);
error_code BuildGoal(Floor* floor, Goal* goal);
error_code BuildInitialPosition(Floor* floor, EPuck* robot);
error_code BuildObstacles(Floor* floor, int obstacleQuantity, Obstacle** obstacles);

error_code GetBoundingRect(int handle, Rect* objectRect);

error_code CalculateGridPosition(Floor* pFloor, Point* pPoint, int* pGridColumn, int* pGridLine);

void DeallocateFloor(Floor* floor);
void DeallocateObstacles(Obstacle** obstacles, int obstaclesQuantity);

void ObstacleGrowth(BehringCells*** grid, int gridWidth, int gridHeight);
void DistanceCalculation(BehringCells** grid, int gridWidth, int gridHeight, int*** distance, int*** approximation);

void ObstacleGrowth(BehringCells*** grid, int gridWidth, int gridHeight)
{
  int i;
  int j;

  int nI;
  int nJ;

  int steps = GROWING_STEPS;

  BehringCells** newGrid = malloc(gridWidth * sizeof(int*));
  for(i=0; i<gridWidth; ++i)
    {
      newGrid[i] = malloc(gridHeight * sizeof(int));
      for(j=0; j<gridHeight; ++j)
	{
	  newGrid[i][j] = (*grid)[i][j];
	}
    }

  while(steps > 0)
    {
      for(i=0; i<gridWidth; ++i)
	{
	  for(j=0; j<gridHeight; ++j)
	    {
	      if((*grid)[i][j] == REAL_OBSTACLE_CELL || (*grid)[i][j] == FAKE_OBSTACLE_CELL)
		{
		  for(nI=-1; nI<=1; ++nI)
		    {
		      if(i+nI >= 0 && i+nI < gridWidth)
			{
			  for(nJ=-1; nJ<=1; ++nJ)
			    {
			      if(j+nJ >= 0 && j+nJ < gridHeight)
				{
				  if((*grid)[i+nI][j+nJ] == FREE_CELL)
				    {
				      //printf("GROW\n");
				      //if((abs(nI) == 1 && abs(nJ) != 1) || (abs(nI) != 1 && abs(nJ) == 1))
				      //	newGrid[i+nI][j+nJ] = REAL_OBSTACLE_CELL;
				      //else
				      newGrid[i+nI][j+nJ] = FAKE_OBSTACLE_CELL;
				    }
				}
			    }
			}
		    }
		}
	    }
	}

      for(i=0; i<gridWidth; ++i)
	{
	  for(j=0; j<gridHeight; ++j)
	    {
	      (*grid)[i][j] = newGrid[i][j];
	    }
	}

      --steps;
    }

  printf("GRID:\n");
  for(i=0; i<gridHeight; ++i)
    {
    for(j=0; j<gridWidth; ++j)
    {
    printf("%d-",(*grid)[j][i]);
    }
    printf("\n");
    }

  //Deallocate
  printf("Obstacle grown!\n");
  for(i=0; i<gridWidth; ++i)
    {
      free(newGrid[i]);
    }
  free(newGrid);
}

void DistanceCalculation(BehringCells** grid, int gridWidth, int gridHeight, int*** distance, int*** approximation)
{
  int** newDistance;
  int goalXPosition=0;
  int goalYPosition=0;

  int startXPosition=0;
  int startYPosition=0;
  //Memory allocation
  int i;
  int j;
  (*distance) = malloc(gridWidth * sizeof(int*));
  newDistance = malloc(gridWidth * sizeof(int*));
  (*approximation) = malloc(gridWidth * sizeof(int*));
  for(i=0; i<gridWidth; ++i)
    {
      (*distance)[i] = malloc(gridHeight * sizeof(int));
      newDistance[i] = malloc(gridHeight * sizeof(int));
      (*approximation)[i] = malloc(gridHeight * sizeof(int));
      for(j=0; j<gridHeight; ++j)
	{
	  if(grid[i][j] == GOAL_POSITION_CELL)
	    {
	      goalXPosition = i;
	      goalYPosition = j;
	    }
	  if(grid[i][j] == INITIAL_POSITION_CELL)
	    {
	      startXPosition = i;
	      startYPosition = j;
	    }
	  //(*distance)[i][j] = INT_MAX - GROWING_STEPS - 1;
	  //newDistance[i][j] = INT_MAX - GROWING_STEPS - 1;
	  (*distance)[i][j] = INT_MAX;
	  newDistance[i][j] = INT_MAX;
	  (*approximation)[i][j] = 0;
	}
    }

  //Only for count, the method don't have this variable
  int steps = 0;

  char initialPositionFound = 0;

  int nI;
  int nJ;
  int shortestNeighborhoodDistance;

  //Set the distance of the goal to 0
  (*distance)[goalXPosition][goalYPosition] = 0;
  newDistance[goalXPosition][goalYPosition] = 0;
  (*approximation)[goalXPosition][goalYPosition] = INT_MAX;

  while(!initialPositionFound)
    {
      for(i=0; i<gridWidth; ++i)
	{
	  for(j=0; j<gridHeight; ++j)
	    {
	      if(grid[i][j] == FREE_CELL || grid[i][j] == INITIAL_POSITION_CELL)
		{
		  //shortestNeighborhoodDistance = INT_MAX - GROWING_STEPS - 1;
		  shortestNeighborhoodDistance = INT_MAX;
		  for(nI=-1; nI<=1; ++nI)
		    {
		      if(i+nI >= 0 && i+nI < gridWidth)
			{
			  for(nJ=-1; nJ<=1; ++nJ)
			    {
			      //if(nJ != 0 || nI != 0)
			      //{
			      if(j+nJ >= 0 && j+nJ < gridHeight)
				{
				  //if(grid[i+nI][j+nJ] == FREE_CELL || grid[i+nI][j+nJ] == GOAL_POSITION_CELL)
				  //{
				  if((*distance)[i+nI][j+nJ] < shortestNeighborhoodDistance)
				    {
				      shortestNeighborhoodDistance = (*distance)[i+nI][j+nJ];
				    }
				  /*if(grid[i+nI][j+nJ] == INITIAL_POSITION_CELL)
				    {
				    if((*distance)[i][j] < INT_MAX)
				    {
				    initialPositionFound = 1;
				    }
				    }*/
				  //}
				}
			      //}
			    }
			}
		    }

		  if(shortestNeighborhoodDistance < (*distance)[i][j])
		    {
		      newDistance[i][j] = shortestNeighborhoodDistance + 1;
		    }
		}
	    }
	}

      char sameLattice = 1;
      for(i=0; i<gridWidth; ++i)
	{
	  for(j=0; j<gridHeight; ++j)
	    {
	      if((*distance)[i][j] != newDistance[i][j])
		{
		  (*distance)[i][j] = newDistance[i][j];
		  sameLattice = 0;
		}
	    }
	  //printf("\n");
	}

      ++steps;
      //printf("STEPS=%d\n", steps);

      //if((*distance)[startXPosition][startYPosition] < INT_MAX - GROWING_STEPS - 1)
      if((*distance)[startXPosition][startYPosition] < INT_MAX)
	initialPositionFound = 1;
      if(sameLattice)
	initialPositionFound = 1;
    }

  //printf("STEPS=%d", steps);

  int obstacleDistance;
  for(i=0; i<gridWidth; ++i)
    {
      for(j=0; j<gridHeight; ++j)
	{
	  if(grid[i][j] == REAL_OBSTACLE_CELL)
	    {
	      for(nI=-GROWING_STEPS; nI<=GROWING_STEPS; ++nI)
		{
		  if(i+nI >= 0 && i+nI < gridWidth)
		    {
		      for(nJ=-GROWING_STEPS; nJ<=GROWING_STEPS; ++nJ)
			{
			  if(j+nJ >= 0 && j+nJ < gridHeight)
			    {
			      if(grid[i+nI][j+nJ] == FAKE_OBSTACLE_CELL /*|| grid[i+nI][j+nJ] == INITIAL_POSITION_CELL*/)
				{
				  obstacleDistance = abs(nI) + abs(nJ);
				  if((*approximation)[i+nI][j+nJ] > 0)
				    {
				      if((*approximation)[i+nI][j+nJ] > obstacleDistance)
					{
					  (*approximation)[i+nI][j+nJ] = obstacleDistance;
					}
				    }
				  else
				    {
				      (*approximation)[i+nI][j+nJ] = obstacleDistance;
				    }
				}
			    }
			}
		    }
		}
	    }
	}
    }

  //Mudanca Gina
  for(i=0; i<gridWidth; ++i)
    {
      for(j=0; j<gridHeight; ++j)
	{
	  if(grid[i][j] == FREE_CELL)
	    {
	      if((*distance)[i][j] == INT_MAX)
		{
		  grid[i][j] = FAKE_OBSTACLE_CELL;
		}
	    }
	}
    }

  char obstacleGrowed = 0;
  while(!obstacleGrowed)
    {
      for(i=0; i<gridWidth; ++i)
	{
	  for(j=0; j<gridHeight; ++j)
	    {
	      if(grid[i][j] == FAKE_OBSTACLE_CELL || grid[i][j] == INITIAL_POSITION_CELL)
		{
		  shortestNeighborhoodDistance = INT_MAX;
		  for(nI=-1; nI<=1; ++nI)
		    {
		      if(i+nI >= 0 && i+nI < gridWidth)
			{
			  for(nJ=-1; nJ<=1; ++nJ)
			    {
			      if(j+nJ >= 0 && j+nJ < gridHeight)
				{
				  //Von Neumann neighborhood
				  //if(nI != nJ)
				  //{
				  if((*distance)[i+nI][j+nJ] < shortestNeighborhoodDistance)
				    {
				      shortestNeighborhoodDistance = (*distance)[i+nI][j+nJ];
				    }
				  //}
				}
			    }
			}
		    }

		  if(shortestNeighborhoodDistance < (INT_MAX/2))
		    {
		      newDistance[i][j] = (INT_MAX/2)+1;
		    }
		  else
		    {
		      if(shortestNeighborhoodDistance < (*distance)[i][j])
			newDistance[i][j] = shortestNeighborhoodDistance + 1;
		    }
		}
	    }
	}

      char sameObstacles = 1;
      for(i=0; i<gridWidth; ++i)
	{
	  for(j=0; j<gridHeight; ++j)
	    {
	      if((*distance)[i][j] != newDistance[i][j])
		{
		  (*distance)[i][j] = newDistance[i][j];
		  sameObstacles = 0;
		}
	    }
	}

      if(sameObstacles)
	obstacleGrowed = 1;
    }

  //printf("FINALDIST:%d\n", (*distance)[startXPosition][startYPosition]);

  for(i=0; i<gridWidth; ++i)
    {
      free(newDistance[i]);
    }
  free(newDistance);
}

error_code InitController(char* host, int port, EPuck* robot)
{
  if(Connect(host, port) == EPUCK_CONNECTION_ERROR)
    return EPUCK_CONNECTION_ERROR;

  if(BuildEPuck(robot) == EPUCK_HANDLE_NOT_FOUND_ERROR)
    return EPUCK_HANDLE_NOT_FOUND_ERROR;
  //:TODO: More logic to do
  return EPUCK_NO_ERROR;
}

error_code Control(EPuck* robot)
{
  //Build Floor
  Floor* floor = malloc(sizeof(Floor));
  floor->pmDiscretizedFloor = NULL;
  error_code error;
  error = BuildFloor(floor, robot);
  if(error != EPUCK_NO_ERROR)
    {
      printf("Error while building floor!\n");
      DeallocateFloor(floor);
      return error;
    }

  //Build Obstacles
  //:TODO: Dinamically gets obstacles quantity
  int obstaclesQuantity = 4;
  //int obstaclesQuantity = 1;
  Obstacle** obstaclesArray = malloc(obstaclesQuantity * sizeof(Obstacle*));
  error = BuildObstacles(floor, obstaclesQuantity, obstaclesArray);
  if(error != EPUCK_NO_ERROR)
    {
      printf("Error building obstacles!\n");
      DeallocateObstacles(obstaclesArray, obstaclesQuantity);
      DeallocateFloor(floor);
      return error;
    }

  //Build Goal
  Goal* goal = malloc(sizeof(Goal));
  error = BuildGoal(floor, goal);
  if(error != EPUCK_NO_ERROR)
    {
      printf("Error building goal!\n");
      free(goal);
      DeallocateObstacles(obstaclesArray, obstaclesQuantity);
      DeallocateFloor(floor);
      return error;
    }

  //Build initial position
  error = BuildInitialPosition(floor, robot);
  if(error != EPUCK_NO_ERROR)
    {
      printf("Error building initial position!");
      free(goal);
      DeallocateObstacles(obstaclesArray, obstaclesQuantity);
      DeallocateFloor(floor);
      return error;
    }


  /*int k;
    int l;
    int cObst = 0;
    for(k=0; k<floor->mDiscretizedFloorWidth; ++k)
    {
    for(l=0; l<floor->mDiscretizedFloorHeight; ++l)
    {
    if(floor->pmDiscretizedFloor[k][l] == OBSTACLE_CELL)
    ++cObst;
    }
    }
    printf("PREOBST=%d\n", cObst);*/

  //First CA phase
  ObstacleGrowth(&(floor->pmDiscretizedFloor), floor->mDiscretizedFloorWidth, floor->mDiscretizedFloorHeight);

  //printf("GROWTHED!\n");

  /*cObst = 0;
    for(k=0; k<floor->mDiscretizedFloorWidth; ++k)
    {
    for(l=0; l<floor->mDiscretizedFloorHeight; ++l)
    {
    if(floor->pmDiscretizedFloor[k][l] == OBSTACLE_CELL)
    ++cObst;
    }
    }
    printf("POSOBST=%d\n", cObst);*/

  //Get initial grid position
  Point robotPoint;
  robotPoint.mX = robot->mPosition[0];
  robotPoint.mY = robot->mPosition[1];
  int gridColumn;
  int gridLine;
  CalculateGridPosition(floor, &robotPoint, &(gridColumn), &(gridLine));

  //int stepsQuantity = 5;
  int step;
  int currentDirection = 0;
  //char goalFound = 0;

  //Second CA phase
  int** distanceGrid;
  int** approximationGrid;
  DistanceCalculation(floor->pmDiscretizedFloor, floor->mDiscretizedFloorWidth, floor->mDiscretizedFloorHeight, &(distanceGrid), &(approximationGrid));
  RobotDirection* directions;
  //Get plan
  int directionsSize =GetDirectionList(gridLine, gridColumn, floor->mDiscretizedFloorWidth, floor->mDiscretizedFloorHeight, distanceGrid, approximationGrid, &directions);
  //int directionsSize = 0;
  //printf("DIRECTIONSIZE=%d\n", directionsSize);

  int i;

  for(i=0; i<floor->mDiscretizedFloorWidth; ++i)
    {
      if(distanceGrid[i])
	free(distanceGrid[i]);
      if(approximationGrid[i])
	free(approximationGrid[i]);
    }
  free(distanceGrid);
  free(approximationGrid);

  /*for(i=0; i<directionsSize; ++i)
    printf("%d->",directions[i]);
    printf("\n");*/

  step = 0;
  while(step < directionsSize)
    {
      floor->pmDiscretizedFloor[gridColumn][gridLine] = FREE_CELL;

      OdometryMove(robot, directions[step]);

      /*error = Stay(robot);
	if(error != EPUCK_NO_ERROR)
	{
	free(directions);
	free(goal);
	//free(obstaclesArray);
	DeallocateObstacles(obstaclesArray, obstaclesQuantity);
	DeallocateFloor(floor);
	return error;
	}*/


      CalculateGridPosition(floor, &robotPoint, &(gridColumn), &(gridLine));

      if(floor->pmDiscretizedFloor[gridColumn][gridLine] != GOAL_POSITION_CELL)
	floor->pmDiscretizedFloor[gridColumn][gridLine] = INITIAL_POSITION_CELL;

      ++step;
    }

  //while(currentDirection < directionsSize)
  /*while(goalFound == 0)
    {
    robotPoint.mX = robot->mPosition[0];
    robotPoint.mY =	robot->mPosition[1];
    CalculateGridPosition(floor, &robotPoint, &(gridColumn), &(gridLine));

    if(floor->pmDiscretizedFloor[gridColumn][gridLine] == GOAL_POSITION_CELL)
    goalFound = 1;
    else
    {
    //Second CA phase
    int** distanceGrid;
    int** approximationGrid;
    DistanceCalculation(floor->pmDiscretizedFloor, floor->mDiscretizedFloorWidth, floor->mDiscretizedFloorHeight, &(distanceGrid), &(approximationGrid));

    RobotDirection* directions;
    //Get plan
    int directionsSize =GetDirectionList(gridLine, gridColumn, floor->mDiscretizedFloorWidth, floor->mDiscretizedFloorHeight, distanceGrid, approximationGrid, &directions);
    //int directionsSize = 0;
    printf("DIRECTIONSIZE=%d\n", directionsSize);

    int i;

    for(i=0; i<floor->mDiscretizedFloorWidth; ++i)
    {
    if(distanceGrid[i])
    free(distanceGrid[i]);
    if(approximationGrid[i])
    free(approximationGrid[i]);
    }
    free(distanceGrid);
    free(approximationGrid);

    for(i=0; i<directionsSize; ++i)
    printf("%d->",directions[i]);
    printf("\n");

    step = 0;
    currentDirection = 0;
    while(step < directionsSize)
    {
    if(step >= stepsQuantity)
    break;

    //if(currentDirection == directionsSize)
    //{
    //	goalFound = 1;
    //	break;
    //}

    floor->pmDiscretizedFloor[gridColumn][gridLine] = FREE_CELL;

    if (HasConnection() == TRUE)
    {
    printf("HASCONNEX\n");
    error = Move(robot, floor, directions[currentDirection]);
    if(error != EPUCK_NO_ERROR)
    {
    free(directions);
    free(goal);
    //free(obstaclesArray);
    DeallocateObstacles(obstaclesArray, obstaclesQuantity);
    DeallocateFloor(floor);
    return error;
    }

    printf("MOVED\n");

    error = Stay(robot);
    if(error != EPUCK_NO_ERROR)
    {
    free(directions);
    free(goal);
    //free(obstaclesArray);
    DeallocateObstacles(obstaclesArray, obstaclesQuantity);
    DeallocateFloor(floor);
    return error;
    }

    ++currentDirection;
    }
    else
    {
    break;
    }

    error = GetObjectPosition(robot->mEPuckHandle, robot->mPosition);
    if(error != EPUCK_NO_ERROR)
    {
    free(directions);
    free(goal);
    //free(obstaclesArray);
    DeallocateObstacles(obstaclesArray, obstaclesQuantity);
    DeallocateFloor(floor);
    return error;
    }

    //Point robotPoint;
    robotPoint.mX = robot->mPosition[0];
    robotPoint.mY =	robot->mPosition[1];
    CalculateGridPosition(floor, &robotPoint, &(gridColumn), &(gridLine));

    //printf("COL=%d LIN=%d\n", gridColumn, gridLine);

    if(floor->pmDiscretizedFloor[gridColumn][gridLine] != GOAL_POSITION_CELL)
    floor->pmDiscretizedFloor[gridColumn][gridLine] = INITIAL_POSITION_CELL;
    else
    {
    step = stepsQuantity;
    goalFound = 1;
    }

    ++step;
    }
    free(directions);
    }
    }*/

  //free(directions);
  free(goal);
  //free(obstaclesArray);
  DeallocateObstacles(obstaclesArray, obstaclesQuantity);
  DeallocateFloor(floor);
  return EPUCK_NO_ERROR;
}

void OdometryMove(EPuck* robot, RobotDirection positionToGo)
{
  //printf("POS: (%f %f) %f\n", robot->mPosition[0], robot->mPosition[1], robot->mApproximateTheta);

  double deltaS;
  OdometryRotate(robot, positionToGo);

  if(positionToGo == NORTH || positionToGo == SOUTH || positionToGo == WEST || positionToGo == EAST)
    {
      deltaS = robot->mRobotHeight;
    }
  else
    {
      deltaS = sqrt(pow(robot->mRobotHeight,2) + pow(robot->mRobotWidth, 2));
    }

  move_forward(200.0f, deltaS);

  /*switch(positionToGo)
    {
    case NORTH:
    {
    robot->mPosition[1] += EPUCK_DIAMETER;
    goto_position(robot->mPosition[0],robot->mPosition[1],robot->mApproximateTheta);
    break;
    }
    case NORTHWEST:
    {
    robot->mPosition[0] -= EPUCK_DIAMETER;
    robot->mPosition[1] += EPUCK_DIAMETER;
    goto_position(robot->mPosition[0],robot->mPosition[1],robot->mApproximateTheta);
    break;
    }
    case WEST:
    {
    robot->mPosition[0] -= EPUCK_DIAMETER;
    //robot->mPosition[1] += EPUCK_DIAMETER;
    goto_position(robot->mPosition[0],robot->mPosition[1],robot->mApproximateTheta);
    break;
    }
    case SOUTHWEST:
    {
    robot->mPosition[0] -= EPUCK_DIAMETER;
    robot->mPosition[1] -= EPUCK_DIAMETER;
    goto_position(robot->mPosition[0],robot->mPosition[1],robot->mApproximateTheta);
    break;
    }
    case SOUTH:
    {
    robot->mPosition[1] -= EPUCK_DIAMETER;
    goto_position(robot->mPosition[0],robot->mPosition[1],robot->mApproximateTheta);
    break;
    }
    case SOUTHEAST:
    {
    robot->mPosition[0] += EPUCK_DIAMETER;
    robot->mPosition[1] -= EPUCK_DIAMETER;
    goto_position(robot->mPosition[0],robot->mPosition[1],robot->mApproximateTheta);
    break;
    }
    case EAST:
    {
    robot->mPosition[0] += EPUCK_DIAMETER;
    goto_position(robot->mPosition[0],robot->mPosition[1],robot->mApproximateTheta);
    break;
    }
    case NORTHEAST:
    {
    robot->mPosition[0] += EPUCK_DIAMETER;
    robot->mPosition[1] += EPUCK_DIAMETER;
    goto_position(robot->mPosition[0],robot->mPosition[1],robot->mApproximateTheta);
    break;
    }
    };*/

  /*goto_position(0, 0.2, PI);
    stop_robot();
    goto_position(-0.2, 0.2, -PI/2);
    stop_robot();
    goto_position(-0.2, 0, 0);
    stop_robot();
    goto_position(0, 0, PI/2);
    stop_robot();*/

}

void OdometryRotate(EPuck* robot, RobotDirection positionToGo)
{
  double rotationAngle = 0.0f;

  switch(positionToGo)
    {
    case NORTH:
      {
	//rotationToAdd = 0.0;
	rotationAngle = 0.0 - robot->mApproximateTheta;

	//Update theta
	robot->mApproximateTheta = 0.0;

	break;
      }
    case NORTHEAST:
      {
	//rotationToAdd = (7*M_PI)/4;
	//rotationAngle = (7*M_PI)/4 - robot->mApproximateTheta;
	rotationAngle = M_PI/4 - robot->mApproximateTheta;

	//Update theta
	robot->mApproximateTheta = M_PI/4;

	break;
      }
    case EAST:
      {
	//rotationToAdd = (3*M_PI)/2;
	rotationAngle = M_PI/2 - robot->mApproximateTheta;

	//Update theta
	robot->mApproximateTheta = M_PI/2;

	break;
      }
    case SOUTHEAST:
      {
	//rotationToAdd = (5*M_PI)/4;
	//rotationAngle = (5*M_PI)/4 - robot->mApproximateTheta;
	rotationAngle = (3*M_PI)/4 - robot->mApproximateTheta;

	//Update theta
	robot->mApproximateTheta = (3*M_PI)/4;

	break;
      }
    case SOUTH:
      {
	//rotationToAdd = M_PI;
	rotationAngle = M_PI - robot->mApproximateTheta;

	//Update theta
	robot->mApproximateTheta = M_PI;

	break;
      }
    case SOUTHWEST:
      {
	//rotationToAdd = (3*M_PI)/4;
	rotationAngle = (5*M_PI)/4 - robot->mApproximateTheta;
	//horMovement = 1;

	//Update theta
	robot->mApproximateTheta = (5*M_PI)/4;

	break;
      }
    case WEST:
      {
	//rotationToAdd = M_PI/2;
	rotationAngle = (3*M_PI)/2 - robot->mApproximateTheta;
	//horMovement = 1;

	//Update theta
	robot->mApproximateTheta = (3*M_PI)/2;

	break;
      }
    case NORTHWEST:
      {
	//rotationToAdd = M_PI/4;
	rotationAngle = (7*M_PI)/4 - robot->mApproximateTheta;
	//horMovement = 1;

	//Update theta
	robot->mApproximateTheta = (7*M_PI)/4;

	break;
      }
    };

  if(rotationAngle > M_PI)
    {
      rotationAngle -= 2*M_PI;
    }
  else
    {
      if(rotationAngle < -M_PI)
	{
	  rotationAngle += 2*M_PI;
	}
    }


  //:TODO: Change rotation angle
  /*if (rotationAngle > M_PI)
    {
    rotationAngle = fabs(rotationAngle - (2 * M_PI));
    //horMovement = 0;
    }

    if ((int)(rotationAngle * 1000000) < 0)
    {
    rotationAngle = rotationAngle + (2 * M_PI);
    if(rotationAngle < M_PI)
    {
    //horMovement = 1;
    }
    else
    {
    //rotationAngle *= -1;
    rotationAngle = fabs(rotationAngle - (2 * M_PI));
    //horMovement = 0;
    }
    }*/

  if((int)(rotationAngle * 100) != 0)
    turn_angle(rotationAngle, 200.0f, 1);
}

error_code Move(EPuck* robot, Floor* floor, RobotDirection positionToGo)
{
  Rotate(robot, positionToGo);

  error_code error;

  float wheelRadius = 0.02;
  float deltaS;
  float deltaT = 1.0;
  float scalarVel;
  float angularVel;

  if(positionToGo == NORTH || positionToGo == SOUTH || positionToGo == WEST || positionToGo == EAST)
    {
      deltaS = robot->mRobotHeight;
    }
  else
    {
      deltaS = sqrt(pow(robot->mRobotHeight,2) + pow(robot->mRobotWidth, 2));
    }

  scalarVel = deltaS / deltaT;
  angularVel = scalarVel / wheelRadius;

  //errorRightJoint = SetJointVelocity(robot->mRightJoint, angularVel);
  //errorLeftJoint = SetJointVelocity(robot->mLeftJoint, angularVel);
  error = SetDifferentialWheelsVelocity(angularVel, angularVel);

  if(error != EPUCK_NO_ERROR)
    {
      return error;
    }

  //extApi_sleepMs(1000);
  PauseSimulation(1000);
  //printf("MOVE!\n");

  return EPUCK_NO_ERROR;
}

error_code Rotate(EPuck* robot, RobotDirection positionToGo)
{
  float rotationAngle = 0.0;
  char horMovement = 1;

  switch(positionToGo)
    {
    case NORTH:
      {
	//rotationToAdd = 0.0;
	rotationAngle = 0.0 - robot->mApproximateTheta;

	//Update theta
	robot->mApproximateTheta = 0.0;

	break;
      }
    case NORTHWEST:
      {
	//rotationToAdd = (7*M_PI)/4;
	//rotationAngle = (7*M_PI)/4 - robot->mApproximateTheta;
	rotationAngle = M_PI/4 - robot->mApproximateTheta;

	//Update theta
	robot->mApproximateTheta = M_PI/4;

	break;
      }
    case WEST:
      {
	//rotationToAdd = (3*M_PI)/2;
	rotationAngle = M_PI/2 - robot->mApproximateTheta;

	//Update theta
	robot->mApproximateTheta = M_PI/2;

	break;
      }
    case SOUTHWEST:
      {
	//rotationToAdd = (5*M_PI)/4;
	//rotationAngle = (5*M_PI)/4 - robot->mApproximateTheta;
	rotationAngle = (3*M_PI)/4 - robot->mApproximateTheta;

	//Update theta
	robot->mApproximateTheta = (3*M_PI)/4;

	break;
      }
    case SOUTH:
      {
	//rotationToAdd = M_PI;
	rotationAngle = M_PI - robot->mApproximateTheta;

	//Update theta
	robot->mApproximateTheta = M_PI;

	break;
      }
    case SOUTHEAST:
      {
	//rotationToAdd = (3*M_PI)/4;
	rotationAngle = (5*M_PI)/4 - robot->mApproximateTheta;
	//horMovement = 1;

	//Update theta
	robot->mApproximateTheta = (5*M_PI)/4;

	break;
      }
    case EAST:
      {
	//rotationToAdd = M_PI/2;
	rotationAngle = (3*M_PI)/2 - robot->mApproximateTheta;
	//horMovement = 1;

	//Update theta
	robot->mApproximateTheta = (3*M_PI)/2;

	break;
      }
    case NORTHEAST:
      {
	//rotationToAdd = M_PI/4;
	rotationAngle = (7*M_PI)/4 - robot->mApproximateTheta;
	//horMovement = 1;

	//Update theta
	robot->mApproximateTheta = (7*M_PI)/4;

	break;
      }
    };

  //printf("PREROT=%f\n", rotationAngle);

  if (rotationAngle > M_PI)
    {
      rotationAngle = fabs(rotationAngle - (2 * M_PI));
      horMovement = 0;
    }

  if ((int)(rotationAngle * 1000000) < 0)
    {
      rotationAngle = rotationAngle + (2 * M_PI);
      if(rotationAngle < M_PI)
	{
	  horMovement = 1;
	}
      else
	{
	  //rotationAngle *= -1;
	  rotationAngle = fabs(rotationAngle - (2 * M_PI));
	  horMovement = 0;
	}
    }

  //printf("POSROT=%f\n", rotationAngle);

  if((int)(rotationAngle * 1000000) > 0)
    {
      //error_code errorRightJoint;
      //error_code errorLeftJoint;
      error_code error;
      if(!horMovement)
	{
	  //errorRightJoint = SetJointVelocity(robot->mRightJoint, -(M_PI/8));
	  //errorLeftJoint = SetJointVelocity(robot->mLeftJoint, M_PI/8);
	  error = SetDifferentialWheelsVelocity(M_PI/8, -(M_PI/8));
	}
      else
	{
	  //errorRightJoint = SetJointVelocity(robot->mRightJoint, M_PI/8);
	  //errorLeftJoint = SetJointVelocity(robot->mLeftJoint, -(M_PI/8));
	  error = SetDifferentialWheelsVelocity(-(M_PI/8), M_PI/8);
	}

      if(error != EPUCK_NO_ERROR)
	{
	  return error;
	}

      //printf("ROTANGLE:%f\n", rotationAngle);
      //The correct is 1000
      //extApi_sleepMs(rotationAngle/(M_PI/8) * 1100);
      PauseSimulation(rotationAngle/(M_PI/8) * 1100);

      //:TODO: Get euler angles em convert to my angles
      //float* orientation = malloc(3*sizeof(float));
      //error_code errorTheta = GetObjectOrientation(robot->mEPuckHandle, orientation);
      //robot->mApproximateTheta = orientation[1];
      //free(orientation);
    }

  //printf("THETA=%f\n", robot->mApproximateTheta);

  return EPUCK_NO_ERROR;
}


int GetDirectionList(int initialLine, int initialColumn, int gridWidth, int gridHeight, int** distance, int** approximation, RobotDirection** directions)
{
  //Maybe this will cause errors
  (*directions) = malloc((gridWidth + gridHeight) * sizeof(RobotDirection));
  char end = 0;

  int currentI = initialColumn;
  int currentJ = initialLine;

  int i;
  int j;

  int bestI = currentI;
  int bestJ = currentJ;
  int currentDirection = 0;

  int bestIArray[9];
  int bestJArray[9];

  int arrayPos = 0;

  int goalIPosition = 0;
  int goalJPosition = 0;

  for(i=0; i<gridWidth; ++i)
    {
      for(j=0; j<gridHeight; ++j)
	{
	  if(distance[i][j] == 0)
	    {
	      goalIPosition = i;
	      goalJPosition = j;
	    }
	}
    }

  while(!end)
    {
      /*int nI;
	int nJ;
	printf("GETDIRECTIONLISTHERE:\n");
	for(nI=-1; nI<=1; ++nI)
	{
	if(currentI+nI >= 0 && currentI+nI < gridWidth)
	{
	for(nJ=-1; nJ<=1; ++nJ)
	{
	if(currentJ+nJ >= 0 && currentJ+nJ < gridHeight)
	{
	printf("%d - ",distance[currentI+nI][currentJ+nJ]);
	}
	}
	printf("\n");
	}
	}*/

      //printf("%d\n",currentDirection);
      arrayPos = 0;
      bestI = currentI;
      bestJ = currentJ;
      for(i=-1; i<=1; ++i)
	{
	  if(currentI + i >= 0 && currentI + i < gridWidth)
	    {
	      for(j=-1; j<=1; ++j)
		{
		  if(currentJ + j >= 0 && currentJ + j < gridHeight)
		    {
		      //printf("%d - ", distance[currentI + i][currentJ + j]);
		      //printf(distance[currentI + i][currentJ + j], distance[currentI][currentJ]);
		      //printf("%d %d\n", distance[currentI + i][currentJ + j], distance[currentI][currentJ]);
		      if(distance[currentI + i][currentJ + j] != INT_MAX)
			{
			  //Is on a free cell
			  //if(distance[currentI][currentJ] < INT_MAX/2)
			  //{
			  if(distance[currentI + i][currentJ + j] < distance[currentI][currentJ])
			    {
			      if(distance[currentI + i][currentJ + j] == distance[bestI][bestJ])
				{
				  bestIArray[arrayPos] = currentI + i;
				  bestJArray[arrayPos] = currentJ + j;

				  ++arrayPos;
				}
			      else
				{
				  if(distance[currentI + i][currentJ + j] < distance[bestI][bestJ])
				    {
				      bestI = currentI + i;
				      bestJ = currentJ + j;

				      arrayPos = 0;

				      bestIArray[arrayPos] = bestI;
				      bestJArray[arrayPos] = bestJ;

				      ++arrayPos;
				    }
				}
			      //printf("CURR: %d %d\n", currentI, currentJ);
			    }
			  //}
			  //else
			  //{
			  //if(i == 0 && j==0)
			  //{
			  //}
			  //else
			  //{
			  //if(distance[currentI + i][currentJ + j] <= distance[currentI][currentJ])
			  //{
			  /*if(distance[currentI + i][currentJ + j] < distance[bestI][bestJ])
			    {
			    arrayPos = 0;
			    bestI = currentI + i;
			    bestJ = currentJ + j;

			    bestIArray[arrayPos] = bestI;
			    bestJArray[arrayPos] = bestJ;

			    ++arrayPos;
			    }
			    else
			    {*/
			  //if(distance[currentI + i][currentJ + j] == distance[bestI][bestJ])
			  //{
			  //	bestIArray[arrayPos] = bestI;
			  //	bestJArray[arrayPos] = bestJ;

			  //	++arrayPos;
			  //}
			  //}
			  //}
			  //}
			  //}
			}
		    }
		}
	      //printf("\n");
	    }
	}

      //printf("\n");

      //printf("BESTDIST=%d\n", distance[bestIArray[0]][bestJArray[0]]);

      int appValue = 0;
      float bestEuclidianDistance = 10000000.0;
      float euclidianDistance;

      if(distance[currentI][currentJ] > (INT_MAX/2))
	{
	  for(i=0; i<arrayPos; ++i)
	    {
	      if(appValue < approximation[bestIArray[i]][bestJArray[i]])
		{
		  //printf("APP=%d\n",approximation[bestIArray[i]][bestJArray[i]]);
		  appValue = approximation[bestIArray[i]][bestJArray[i]];

		  bestI = bestIArray[i];
		  bestJ = bestJArray[i];

		  bestEuclidianDistance = (float)sqrt(pow(goalIPosition - bestIArray[i], 2) + pow(goalJPosition - bestJArray[i], 2));
		}
	      else
		{
		  if(appValue == approximation[bestIArray[i]][bestJArray[i]])
		    {
		      euclidianDistance = (float)sqrt(pow(goalIPosition - bestIArray[i], 2) + pow(goalJPosition - bestJArray[i], 2));
		      if(euclidianDistance < bestEuclidianDistance)
			{
			  bestI = bestIArray[i];
			  bestJ = bestJArray[i];

			  bestEuclidianDistance = euclidianDistance;
			}
		    }
		}
	    }
	}
      else
	{
	  /*float bestEuclidianDistance = 10000000.0;
	    float euclidianDistance;*/

	  for(i=0; i<arrayPos; ++i)
	    {
	      euclidianDistance = (float)sqrt(pow(goalIPosition - bestIArray[i], 2) + pow(goalJPosition - bestJArray[i], 2));
	      //printf("EUC:%d %d %d %d = %f\n",goalIPosition, bestIArray[i], goalJPosition, bestJArray[i], euclidianDistance);
	      if(euclidianDistance < bestEuclidianDistance)
		{
		  bestI = bestIArray[i];
		  bestJ = bestJArray[i];

		  bestEuclidianDistance = euclidianDistance;
		}
	    }
	}


      if(bestI - currentI == -1)
	{
	  if(bestJ - currentJ == -1)
	    {
	      (*directions)[currentDirection] = NORTHWEST;
	    }
	  else
	    {
	      if(bestJ - currentJ == 0)
		{
		  (*directions)[currentDirection] = WEST;
		}
	      else
		{
		  if(bestJ - currentJ == 1)
		    {
		      (*directions)[currentDirection] = SOUTHWEST;
		    }
		}
	    }
	}
      else
	{
	  if(bestI - currentI == 0)
	    {
	      if(bestJ - currentJ == -1)
		{
		  (*directions)[currentDirection] = NORTH;
		}
	      else
		{
		  if(bestJ - currentJ == 0)
		    {
		    }
		  else
		    {
		      if(bestJ - currentJ == 1)
			{
			  (*directions)[currentDirection] = SOUTH;
			}
		    }
		}
	    }
	  else
	    {
	      if(bestI - currentI == 1)
		{
		  if(bestJ - currentJ == -1)
		    {
		      (*directions)[currentDirection] = NORTHEAST;
		    }
		  else
		    {
		      if(bestJ - currentJ == 0)
			{
			  (*directions)[currentDirection] = EAST;
			}
		      else
			{
			  if(bestJ - currentJ == 1)
			    {
			      (*directions)[currentDirection] = SOUTHEAST;
			    }
			}
		    }
		}
	    }
	}

      //printf("(%d,%d)=%d\n",bestI, bestJ, distance[bestI][bestJ]);

      if(distance[bestI][bestJ] == 0)
	end = 1;

      currentI = bestI;
      currentJ = bestJ;
      ++currentDirection;

      //:TODO Erase this
      //if(currentDirection == 3)
      //end = 1;
    }

  //printf("CURRENTDIRECTION=%d ", currentDirection);
  return currentDirection;
  //return 0;
}

error_code BuildEPuck(EPuck* robot)
{
  //int rightJoint = 0;
  //int leftJoint = 0;
  
  //error_code errorCode = ObjectHandle("ePuck", &(robot->mEPuckHandle));
  //if(errorCode != EPUCK_NO_ERROR)
    //return errorCode;

  //errorCode = GetObjectHandle("GPS", &(robot->mGPSHandle));
  //if(errorCode != EPUCK_NO_ERROR)
  //return errorCode;

  /*errorCode = GetObjectHandle("ePuck_rightJoint", &rightJoint);
    if(errorCode != EPUCK_NO_ERROR)
    return errorCode;

    errorCode = GetObjectHandle("ePuck_leftJoint", &leftJoint);
    if(errorCode != EPUCK_NO_ERROR)
    return errorCode;*/

  //Get epuck bounding rect
  //GetBoundingRect(robot->mEPuckHandle, &(robot->mRobotRect));
  const double diameterOverTwo = EPUCK_DIAMETER/2;
  robot->mRobotRect.mOrigin.mX = -diameterOverTwo;
  robot->mRobotRect.mOrigin.mY = -diameterOverTwo;
  robot->mRobotRect.mEnd.mX = diameterOverTwo;
  robot->mRobotRect.mEnd.mY = diameterOverTwo;

  robot->mRobotWidth = fabs(robot->mRobotRect.mEnd.mX - robot->mRobotRect.mOrigin.mX);
  robot->mRobotHeight = fabs(robot->mRobotRect.mEnd.mY - robot->mRobotRect.mOrigin.mY);

  printf("%f %f\n", robot->mRobotWidth, robot->mRobotHeight);

  //	printf("W=%f, H=%f\n", robot->mRobotRect.mEnd.mX - robot->mRobotRect.mOrigin.mX, robot->mRobotRect.mEnd.mY - robot->mRobotRect.mOrigin.mY);

  robot->mPosition = malloc(3*sizeof(float));
  //errorCode = GetObjectPosition(robot->mEPuckHandle, robot->mPosition);
  robot->mPosition[0] = -0.455;
  robot->mPosition[1] = -0.455;

  /*if(errorCode != EPUCK_NO_ERROR)
    return errorCode;*/

  //robot->mRightJoint = rightJoint;
  //robot->mLeftJoint = leftJoint;

  robot->mApproximateTheta = 0;

  return EPUCK_NO_ERROR;
}

error_code BuildFloor(Floor* floor, EPuck* robot)
{
  //Get floor handle
  //error_code error = GetObjectHandle("CheckeredFloor", &(floor->mFloorHandle));
  //if(error == EPUCK_HANDLE_NOT_FOUND_ERROR)
  //return error;
  //Get floor bounding rect
  //GetBoundingRect(floor->mFloorHandle, &(floor->mFloorRect));
  floor->mFloorRect.mOrigin.mX = -0.49f;
  floor->mFloorRect.mOrigin.mY = -0.49f;
  floor->mFloorRect.mEnd.mX = 0.49f;
  floor->mFloorRect.mEnd.mY = 0.49f;

  floor->mContinuousFloorWidth = fabs(floor->mFloorRect.mEnd.mX - floor->mFloorRect.mOrigin.mX);
  floor->mContinuousFloorHeight = fabs(floor->mFloorRect.mEnd.mY - floor->mFloorRect.mOrigin.mY);
  /* floor->mContinuousFloorWidth = 0.98f; */
  /* floor->mContinuousFloorHeight = 0.98f; */
  /* printf("Floor width (cont): %.2f. Floor height (cont): %.2f\n", floor->mContinuousFloorWidth, floor->mContinuousFloorHeight); */
  

  /* Even if a float has .00 as decimal part, ceil is going to return the next closer integer.
     Ex: ceil(14.00) returns 15
     That's why the division is typedefed (below)
  */

  floor->mDiscretizedFloorWidth = ceil((int) (floor->mContinuousFloorWidth / robot->mRobotWidth));
  floor->mDiscretizedFloorHeight = ceil((int) (floor->mContinuousFloorHeight / robot->mRobotHeight));
  /* printf("Floor width (discrete): %d. Floor height (discrete): %d\n", floor->mDiscretizedFloorWidth, floor->mDiscretizedFloorHeight); */
  /* floor->mDiscretizedFloorWidth = 14; */
  /* floor->mDiscretizedFloorHeight = 14; */
  
  int i;
  int j;
  floor->pmDiscretizedFloor = malloc(floor->mDiscretizedFloorWidth * sizeof(BehringCells*));
  for(i=0; i<floor->mDiscretizedFloorWidth; ++i)
    {
      floor->pmDiscretizedFloor[i] = malloc(floor->mDiscretizedFloorHeight * sizeof(BehringCells));

      for(j=0; j<floor->mDiscretizedFloorHeight; ++j)
	{
	  floor->pmDiscretizedFloor[i][j] = FREE_CELL;
	  printf("%d", floor->pmDiscretizedFloor[i][j]);
	}
      printf("\n");
    }
  printf("Floor successfully built!\n");
  return EPUCK_NO_ERROR;
}

error_code BuildGoal(Floor* floor, Goal* goal)
{
  //Get goal handle
  //error_code error = GetObjectHandle("GoalPosition", &(goal->mGoalHandle));
  //if(error == EPUCK_HANDLE_NOT_FOUND_ERROR)
  //return error;

  //Get goal grid position
  float* goalPosition = malloc(3*sizeof(float));
  goalPosition[0] = 0.455;
  goalPosition[1] = 0.315;
  goalPosition[2] = 0.0;
  //error = GetObjectPosition(goal->mGoalHandle, goalPosition);
  //if(error != EPUCK_NO_ERROR)
  //{
  //	free(goalPosition);
  //	return error;
  //}

  Point goalPoint;
  goalPoint.mX = goalPosition[0];
  goalPoint.mY =	goalPosition[1];
  CalculateGridPosition(floor, &goalPoint, &(goal->mGridColumn), &(goal->mGridLine));

  floor->pmDiscretizedFloor[goal->mGridColumn][goal->mGridLine] = GOAL_POSITION_CELL;

  //printf("%f %f %f\n", goalPosition[0],goalPosition[1],goalPosition[2]);
  //printf("GOALGRID:%d %d\n", goal->mGridColumn, goal->mGridLine);

  free(goalPosition);
  printf("Goal successfully built!\n");
  return EPUCK_NO_ERROR;
}

error_code BuildInitialPosition(Floor* floor, EPuck* robot)
{
  Point robotPoint;
  robotPoint.mX = robot->mPosition[0];
  robotPoint.mY = robot->mPosition[1];
  int robotColumn;
  int robotLine;
  CalculateGridPosition(floor, &robotPoint, &(robotColumn), &(robotLine));

  floor->pmDiscretizedFloor[robotColumn][robotLine] = INITIAL_POSITION_CELL;

  printf("INITIALGRID:%d %d\n", robotColumn, robotLine);

  return EPUCK_NO_ERROR;
}

error_code BuildObstacles(Floor* floor, int obstacleQuantity, Obstacle** obstacles)
{
  int i;
  int obsI;
  int obsJ;
  Point originPoint;
  Point endPoint;
  int originColumn;
  int originLine;
  int endColumn;
  int endLine;

  for(i=0; i<obstacleQuantity; ++i)
    {
      obstacles[i] = malloc(sizeof(Obstacle));
      obstacles[i]->mPosition = malloc(3*sizeof(float));
      obstacles[i]->mObstacleRect.mOrigin.mX = -0.05f;
      obstacles[i]->mObstacleRect.mOrigin.mY = -0.05f;
      obstacles[i]->mObstacleRect.mEnd.mX = 0.05f;
      obstacles[i]->mObstacleRect.mEnd.mY = 0.05f;
    }

  /* ATTENTION: The axis convention related to the world behring.wbt is as follows 
                                |
				|
				|
				|
				|
				|
    -----------------------------------------------------------X
                                |
				|
				|
				|
				|
				|
				Y
  In Webots, our +Y is -X, and our +X is +Z			      
  */
  
  obstacles[0]->mPosition[0] = -0.27f;
  obstacles[0]->mPosition[1] = -0.27f;

  obstacles[1]->mPosition[0] = 0.3f;
  obstacles[1]->mPosition[1] = -0.07f;

  obstacles[2]->mPosition[0] = 0.0f;
  obstacles[2]->mPosition[1] = 0.0f;

  obstacles[3]->mPosition[0] = -0.15f;
  obstacles[3]->mPosition[1] = 0.35f;

  for(i = 0; i < obstacleQuantity; ++i)
    {
      originPoint.mX = obstacles[i]->mPosition[0] + (obstacles[i]->mObstacleRect.mOrigin.mX);
      originPoint.mY = obstacles[i]->mPosition[1] + (obstacles[i]->mObstacleRect.mOrigin.mY);

      //printf("OBSTP=(%f %f)\n", obstacles[i]->mPosition[0], obstacles[i]->mPosition[1]);


      CalculateGridPosition(floor, &originPoint, &originColumn, &originLine);


      endPoint.mX = obstacles[i]->mPosition[0] + (obstacles[i]->mObstacleRect.mEnd.mX);
      endPoint.mY = obstacles[i]->mPosition[1] + (obstacles[i]->mObstacleRect.mEnd.mY);

      printf("POINTS = originPoint:(%f %f) endPoint:(%f %f)\n", originPoint.mX, originPoint.mY, endPoint.mX, endPoint.mY);


      CalculateGridPosition(floor, &endPoint, &endColumn, &endLine);

      for(obsI=originColumn; obsI<=endColumn; ++obsI)
	{
	  for(obsJ=originLine; obsJ<=endLine; ++obsJ)
	    {
	      floor->pmDiscretizedFloor[obsI][obsJ] = REAL_OBSTACLE_CELL;
	      printf("GRIDOBS=%d %d\n", obsI, obsJ);
	    }
	}
    }


  /*int i;
    int j;

    int obsI;
    int obsJ;

    error_code error;
    char obstacleName[10] = "Obstacle";
    for(i=0; i<obstacleQuantity; ++i)
    {
    obstacles[i] = malloc(sizeof(Obstacle));
    obstacleName[8] = i + '0';
    obstacleName[9] = '\0';
    //Get obstacle handle
    error = GetObjectHandle(obstacleName, &(obstacles[i]->mObstacleHandle));
    if(error == EPUCK_HANDLE_NOT_FOUND_ERROR)
    {
    free(obstacles[i]);
    obstacles[i] = 0;
    for(j=(i-1); j>=0; --j)
    {
    free(obstacles[j]->mPosition);
    obstacles[j]->mPosition = 0;
    free(obstacles[j]);
    obstacles[j] = 0;
    }
    return error;
    }

    //Get obstacle bounding rect
    GetBoundingRect(obstacles[i]->mObstacleHandle, &(obstacles[i]->mObstacleRect));

    //Get obstacle position
    obstacles[i]->mPosition = malloc(3*sizeof(float));
    error = GetObjectPosition(obstacles[i]->mObstacleHandle, obstacles[i]->mPosition);
    if(error != EPUCK_NO_ERROR)
    {
    for(j=i; j>=0; --j)
    {
    free(obstacles[j]->mPosition);
    obstacles[j]->mPosition = 0;
    free(obstacles[j]);
    obstacles[j] = 0;
    }
    return error;
    }

    Point originPoint;
    originPoint.mX = obstacles[i]->mPosition[0] + (obstacles[i]->mObstacleRect.mOrigin.mX);
    originPoint.mY = obstacles[i]->mPosition[1] + (obstacles[i]->mObstacleRect.mOrigin.mY);

    //printf("OBSTP=(%f %f)\n", obstacles[i]->mPosition[0], obstacles[i]->mPosition[1]);

    int originColumn;
    int originLine;
    CalculateGridPosition(floor, &originPoint, &originColumn, &originLine);

    Point endPoint;
    endPoint.mX = obstacles[i]->mPosition[0] + (obstacles[i]->mObstacleRect.mEnd.mX);
    endPoint.mY = obstacles[i]->mPosition[1] + (obstacles[i]->mObstacleRect.mEnd.mY);

    //printf("POINTS=(%f %f) (%f %f)\n", originPoint.mX, originPoint.mY, endPoint.mX, endPoint.mY);

    int endColumn;
    int endLine;
    CalculateGridPosition(floor, &endPoint, &endColumn, &endLine);

    for(obsI=originColumn; obsI<=endColumn; ++obsI)
    {
    for(obsJ=originLine; obsJ<=endLine; ++obsJ)
    {
    floor->pmDiscretizedFloor[obsI][obsJ] = REAL_OBSTACLE_CELL;
    //printf("GRIDOBS=%d %d\n", obsI, obsJ);
    }
    }

    //printf("POS: (%f,%f) (%f,%f)\n", originPoint.mX, originPoint.mY, endPoint.mX, endPoint.mY);

    //printf("POS: %f %f %f\n", obstacles[i]->mPosition[0],obstacles[i]->mPosition[1],obstacles[i]->mPosition[2]);

    //printf("HANDLE=%d\n", obstacles[i]->mObstacleHandle);
    }*/
  printf("Obstacles sucessfully built!");
  return EPUCK_NO_ERROR;
}

/*error_code GetBoundingRect(int handle, Rect* objectRect)
{
  error_code error = GetObjectFloatParameters(handle, &(objectRect->mOrigin.mX), OBJECT_MIN_X_POSITION);
  if(error != EPUCK_NO_ERROR)
    return error;
  error = GetObjectFloatParameters(handle, &(objectRect->mOrigin.mY), OBJECT_MIN_Y_POSITION);
  if(error != EPUCK_NO_ERROR)
    return error;
  error = GetObjectFloatParameters(handle, &(objectRect->mEnd.mX), OBJECT_MAX_X_POSITION);
  if(error != EPUCK_NO_ERROR)
    return error;
  error = GetObjectFloatParameters(handle, &(objectRect->mEnd.mY), OBJECT_MAX_Y_POSITION);
  if(error != EPUCK_NO_ERROR)
    return error;

  //	printf("BOUNDING = %f %f - %f %f\n", objectRect->mOrigin.mX, objectRect->mOrigin.mY, objectRect->mEnd.mX, objectRect->mEnd.mY);

  return EPUCK_NO_ERROR;
}*/

error_code CalculateGridPosition(Floor* pFloor, Point* pPoint, int* pGridColumn, int* pGridLine)
{
  //This only works because the floor is on position (0,0,0).
  /* (*pGridColumn) = (fabs(pPoint->mX - pFloor->mFloorRect.mOrigin.mX) * pFloor->mDiscretizedFloorWidth) / pFloor->mContinuousFloorWidth; */
  /* (*pGridLine) = (fabs(pPoint->mY - pFloor->mFloorRect.mOrigin.mY) * pFloor->mDiscretizedFloorHeight) / pFloor->mContinuousFloorHeight; */
  *pGridColumn = fabs(pPoint->mX - pFloor->mFloorRect.mOrigin.mX) / CELL_SIZE;
  *pGridLine = fabs(pPoint->mY - pFloor->mFloorRect.mOrigin.mY) / CELL_SIZE;
  /* printf("line value: %.2f\n", (pPoint->mY - pFloor->mFloorRect.mOrigin.mY)); */
  
  return EPUCK_NO_ERROR;
}

void DeallocateFloor(Floor* floor)
{
  if(floor->pmDiscretizedFloor)
    {
      int i;
      for(i=0; i<floor->mDiscretizedFloorWidth; ++i)
	{
	  if(floor->pmDiscretizedFloor[i])
	    free(floor->pmDiscretizedFloor[i]);
	}
      free(floor->pmDiscretizedFloor);
    }
  free(floor);
}

void DeallocateObstacles(Obstacle** obstacles, int obstaclesQuantity)
{
  int i;
  if(obstaclesQuantity>0)
    {
      for(i=0; i<obstaclesQuantity; ++i)
	{
	  if(obstacles[i])
	    {
	      if(obstacles[i]->mPosition)
		{
		  free(obstacles[i]->mPosition);
		}
	      free(obstacles[i]);
	    }
	}

      free(obstacles);
    }


}

void WbDeviceTagDest(void* a) {
  free((WbDeviceTag*)a);
}

int WbDeviceTagComp(const void* a,const void* b) {
  if( *(WbDeviceTag*)a > *(WbDeviceTag*)b) return(1);
  if( *(WbDeviceTag*)a < *(WbDeviceTag*)b) return(-1);
  return(0);
}

void WbDeviceTagPrint(const void* a) {
  //printf("%i",*(WbDeviceTag*)a);
}

void InfoPrint(void* a) {
  ;
}

void InfoDest(void *a){
  ;
}

/*static void init(void)
  {
  //  init speed is 0
  speed[LEFT]=speed[RIGHT]=0;
  pspeed[LEFT]=pspeed[RIGHT]=0;
  wb_differential_wheels_set_speed(speed[LEFT],speed[RIGHT]);

  printf("Init OK\n");
  return;
  }*/

/*
 * Prints the robot's current position
 */
/*static void print_position(void)
  {
  printf("Current position : (%f,%f,%f)\n", ot.result.x, ot.result.y, ot.result.theta);

  return;
  }

  void set_speed(int l, int r)
  {
  pspeed[LEFT] = speed[LEFT];
  pspeed[RIGHT] = speed[RIGHT];
  speed[LEFT] = l;
  speed[RIGHT] = r;

  if (pspeed[LEFT] != speed[LEFT] || pspeed[RIGHT] != speed[RIGHT]) {
  wb_differential_wheels_set_speed(speed[LEFT], speed[RIGHT]);
  }
  }*/

/*
 * Send the robot to a relative position (x,y)
 * using the "odometry" and "odometry_goto" modules
 */
/*void goto_position(float x, float y, float theta)
  {
  //if (VE//////rbOSE > 0)
  printf("Going to (%f, %f, %f)\n",x,y,theta);

  // Set a target position
  odometry_goto_set_goal(&og, x, y, theta);

  // Move until the robot is close enough to the target position
  while (og.result.atgoal == 0) {
  if(!HasConnection())
  return;

  // Update position and calculate new speeds
  odometry_track_step(og.track);
  odometry_goto_step(&og);

  // Set the wheel speed
  //set_speed(og.result.speed_left, og.result.speed_right);
  printf("SETSPEED:(%d %d)\n", og.result.speed_left, og.result.speed_right);
  set_speed(og.result.speed_left, og.result.speed_right);

  print_position();

  print_position();
  }

  //if (VE//////rbOSE > 0)
  print_position();

  return;
  }*/

void stop_robot() {
  dMSpeed[LEFT] = 0.0f;
  dMSpeed[RIGHT] = 0.0f;
  wb_differential_wheels_set_speed(dMSpeed[LEFT], dMSpeed[RIGHT]);
}

void move_forward(double dSpeed, double dDist) {
  double dStepCount = 0.0f;
  double dStopPosLeft = 0.0f;
  double dStopPosRight = 0.0f;
  double *p_dEncPos;

  if( (dDist > 0.0f) && (dSpeed > 0.0f) ) {
    // calculate the number of step counts ...
    //dStepCount = (REV_STEP/(M_PI*WHEEL_RADIUS*2.0f))*dDist;
    dStepCount = (REV_STEP/(M_PI*WHEEL_RADIUS*2.0f))*dDist;

    // read the current encoder positions of both wheels and
    // calculate the encoder positions when the robot has to
    // stop at the given distance ...
    p_dEncPos = get_encoder_positions();

    dStopPosLeft = p_dEncPos[LEFT] + dStepCount;
    dStopPosRight = p_dEncPos[RIGHT] + dStepCount;

    // compute the current odometry data ...
    p_dOdometryData = compute_odometry_data();

    // set the speed of both wheels (move forward) ...
    set_motor_speed(dSpeed, dSpeed);

    // wait until the robot has reached the calculated positions of the encoders ...

    /*while(  (dStopPosLeft + STEP_TOLERANCE < p_dEncPos[0]) ||    // beyond the distance (*)...
      (dStopPosLeft - STEP_TOLERANCE > p_dEncPos[0]) ||    // short behind the distance (**)...
      (dStopPosRight + STEP_TOLERANCE < p_dEncPos[1]) ||  // (*)
      (dStopPosRight - STEP_TOLERANCE > p_dEncPos[1]) ) { // (**) */
    while(  (p_dEncPos[0] < dStopPosLeft) &&
            (p_dEncPos[1] < dStopPosRight) ) {
      // actualize the odometry data ...
      p_dOdometryData = compute_odometry_data();
      // actualize the wheel encoder position values ...
      p_dEncPos = get_encoder_positions();
      // slow down the speed if the robot is very close to the end (increases the precision)...
      if( fabs(dStopPosLeft - p_dEncPos[0]) <= MIN_DIST ) { set_motor_speed(MIN_SPEED, MIN_SPEED); }
    }
  }
  // the given distance is reached, so stop the robot ...
  stop_robot();

  // compute the odometry data ...
  p_dOdometryData = compute_odometry_data();
  // request the simulator to perform a simulation step [ms] ...
  wb_robot_step(TIME_STEP);
}

void turn_left(double dSpeed) {
  turn_angle(-90.0f, dSpeed, 0);
}

void turn_right(double dSpeed) {
  turn_angle(90.0f, dSpeed, 0);
}

void turn_angle(double dAngle, double dSpeed, char rad) {
  double dFactor = 0.0f;
  double dStepCount = 0.0f;
  double dStopPosLeft = 0.0f;
  double dStopPosRight = 0.0f;
  double *p_dEncPos;

  if(rad)
    dAngle = (180.0f/M_PI) * dAngle;

  //printf("ROT=%f\n",dAngle);

  if( (dAngle != 0.0f) && (dSpeed > 0.0f) ) {
    // calculate the turn factor ...
    dFactor = fabs(360.0f/dAngle);
    //printf("DFACTOR=%lf\n",dFactor);

    // calculate the number of step counts for the wheel rotations ...
    dStepCount = (REV_STEP*WHEELBASE)/(dFactor*WHEEL_RADIUS*2.0f); // e-puck
    //printf("DSTEPCOUNT=%lf\n",dStepCount);
    //dStepCount = ((WHEELBASE*M_PI*abs(angle))/360.0f)*STEP // Khepera

    // read the current encoder positions of the wheels ...
    p_dEncPos = get_encoder_positions();

    if(dAngle > 0) { // turn right ...
      // calculate the encoder positions ...
      dStopPosLeft = p_dEncPos[LEFT] + dStepCount;
      dStopPosRight = p_dEncPos[RIGHT] - dStepCount;

      // compute the current odometry data ...
      p_dOdometryData = compute_odometry_data();

      // turn the robot by setting the different wheel speeds ...
      set_motor_speed(dSpeed, -dSpeed);

      // turn right and wait until the robot has reached the
      // calculated positions of the encoders ...
      while(  (p_dEncPos[LEFT] < dStopPosLeft) &&
              (p_dEncPos[RIGHT] > dStopPosRight)  ) {
	// actualize the odometry data ...
	p_dOdometryData = compute_odometry_data();
	// actualize the wheel encoder position values ...
	p_dEncPos = get_encoder_positions();
	// if the robot is very close to the endpoinnt slow down the speed (increases the precision)...
	if( fabs(dStopPosLeft - p_dEncPos[0]) <= MIN_DIST ) { set_motor_speed(MIN_SPEED, -MIN_SPEED); }
      }
    } else { // turn left ...
      dStopPosLeft = p_dEncPos[LEFT] - dStepCount;
      dStopPosRight = p_dEncPos[RIGHT] + dStepCount;

      // compute the current odometry data ...
      p_dOdometryData = compute_odometry_data();

      // turn left the robot ...
      set_motor_speed(-dSpeed, dSpeed);

      while(  (p_dEncPos[LEFT] > dStopPosLeft) &&
              (p_dEncPos[RIGHT] < dStopPosRight)  ) {

	p_dOdometryData = compute_odometry_data();
	p_dEncPos = get_encoder_positions();
	if( fabs(dStopPosLeft - p_dEncPos[0]) <= MIN_DIST ) { set_motor_speed(-MIN_SPEED, MIN_SPEED); }
      }
    }
  }

  // the given angle is reached, so stop the robot ...
  stop_robot();

  // compute the odometry data ...
  p_dOdometryData = compute_odometry_data();
  // request the simulator to perform a simulation step [ms] ...
  wb_robot_step(TIME_STEP);
}

void set_motor_speed(double dSpeedL, double dSpeedR) {
  // actulalize the global speed variables ...
  dMSpeed[LEFT] = dSpeedL;
  dMSpeed[RIGHT] = dSpeedR;
  // set the speed values to both motors ...
  wb_differential_wheels_set_speed(dMSpeed[LEFT], dMSpeed[RIGHT]);
}

double* get_encoder_positions() {
  static double dEncPos[2];

  // read the encoder positions values of the wheels ...
  dEncPos[0] = wb_differential_wheels_get_left_encoder();
  dEncPos[1] = wb_differential_wheels_get_right_encoder();
  //if(iSim == 1) {
  wb_robot_step(TIME_STEP); // performing a simulation step in [ms] ...
  //}

  return dEncPos;
}

double* compute_odometry_data() {
  static double dOdometryData[3];
  double *p_dEncPos;

  double dNTicksLeft = 0.0f;
  double dNTicksRight = 0.0f;
  double dLeftDist = 0.0f;
  double dRightDist = 0.0f;
  double dDispCR = 0.0f;
  static double dTheta = 0.0f;
  static double dPosX = 0.0f;
  static double dPosY = 0.0f;

  // get the current encoder positions of both wheels ...
  p_dEncPos = get_encoder_positions();

  // calculate the linear distance (number of ticks across a
  // given span of time (sample)) of each wheel has traveled ...
  dNTicksLeft = p_dEncPos[LEFT] - dPrevEncPos[LEFT];
  dNTicksRight = p_dEncPos[RIGHT] - dPrevEncPos[RIGHT];
  // convert the number of the passed ticks (covered by each wheel) to meters ...
  dLeftDist = dNTicksLeft / ENCODER_RESOLUTION * WHEEL_RADIUS;
  dRightDist = dNTicksRight / ENCODER_RESOLUTION * WHEEL_RADIUS;

  // calculate the displacement (distance) from the center C
  // of the robot during the time span (sample) ...
  dDispCR = (dLeftDist + dRightDist)/2.0f;
  // calculate the orientation (rotation) theta ...
  dTheta += (dLeftDist - dRightDist)/(WHEELBASE);

  // convert/clip the orientation in degrees (+/-) ...
  //dTheta -= (double)((int)(dTheta/2.0f*M_PI))*(2.0f*M_PI); // ??

  // calculate the cartesian cooridnates (x, y) of the robot ...
  // sin and cos were swapped (G. Dante)
  dPosX += dDispCR * cos(dTheta);
  dPosY += dDispCR * sin(dTheta);

  // update the previous encoder positions ...
  dPrevEncPos[LEFT] = p_dEncPos[LEFT]; //dEncPosLeft;
  dPrevEncPos[RIGHT] = p_dEncPos[RIGHT]; //dEncPosRight;

  // write the calculated values into the buffer and return ...
  dOdometryData[0] = dPosX;
  dOdometryData[1] = dPosY;
  dOdometryData[2] = dTheta;

  //printf("(%f %f %f)\n", dPosX, dPosY, dTheta);

  return dOdometryData;
}

void set_leds(int iActive) {
  int i;
  for(i=0; i < 3; i++) {
    wb_led_set(led[i], iActive);
  }
}

error_code Connect(char* host, int port)
{
  // define variables ...
  //double dSpeed = 200.0f;
  //double dDistance = 0.20f;

  /* necessary to initialize webots stuff */
  wb_robot_init();

  // enables encoders and set encoders to {0,0}
  wb_differential_wheels_enable_encoders(TIME_STEP*4);
  wb_differential_wheels_set_encoders(0.0f,0.0f);

  led[0] = wb_robot_get_device("led0");
  led[1] = wb_robot_get_device("led2");
  led[2] = wb_robot_get_device("led6");

  // start the calibration method ...
  //UMBmark(dSpeed, dDistance);
  // deactivate the LEDs - show that the process is finished now ...
  //set_leds(0);

  // Initialize the modules
  /*odometry_track_init();
    odometry_goto_init();*/

  // Initializes tracking and goto structures
  /*odometry_track_start(&ot);
    odometry_goto_start(&og, &ot);

    ot.result.x = 0.0;
    ot.result.y = 0.0;
    ot.result.theta = 0.0;

    init();*/

  /* initialize red black //////tree*/
  //tree=////rb//treeCreate(WbDeviceTagComp,WbDeviceTagDest,InfoDest,WbDeviceTagPrint,InfoPrint);

  return EPUCK_NO_ERROR;
}

t_bool HasConnection()
{
  if(wb_robot_step(TIME_STEP) != -1)
    return TRUE;

  return FALSE;
}

void Disconnect()
{
  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();
}

//Getters
/*error_code GetObjectHandle(char* objectName, WbDeviceTag* handle)
{
  //if(simxGetObjectHandle(objectName, handle, simx_opmode_oneshot_wait)!=simx_error_noerror)
  //return EPUCK_HANDLE_NOT_FOUND_ERROR;
  (*handle) = wb_robot_get_device(objectName);
  return EPUCK_NO_ERROR;
}*/

/*error_code GetObjectPosition(WbDeviceTag gpsHandle, float* position)
{
  ////rb_red_blk_node* node;
  WbDeviceTag* newHandle;
  node = ////rbExactQuery(//tree,&gpsHandle);

  if(node) //(//node)
    {
      //printf("%f\n", ((float*)(//node->info))[0] + 0.1);
    }
  else
    {
      newHandle = malloc(sizeof(WbDeviceTag));
      position[0] = 0.0;
      position[1] = 0.0;
      position[2] = 0.0;
      ////rb//treeInsert(//tree,newHandle,position);
    }

  position = (float*) wb_gps_get_values(newHandle);

  //:TODO: Assynchronize call
  //if(simxGetObjectPosition(handle, -1, position, simx_opmode_oneshot_wait)!=simx_error_noerror)
  //	return EPUCK_POSITION_NOT_FOUND_ERROR;
  return EPUCK_NO_ERROR;
}*/

/*error_code GetObjectOrientation(WbDeviceTag handle, float* orientation)
{
  //:TODO: Assynchronize call
  //if(simxGetObjectOrientation(handle, -1, orientation, simx_opmode_oneshot_wait)!=simx_error_noerror)
  //	return EPUCK_POSITION_NOT_FOUND_ERROR;
  return EPUCK_NO_ERROR;
}*/

/*error_code GetObjectFloatParameters(WbDeviceTag handle, float* parameterValue, object_parameter parameter)
{
  //if(simxGetObjectFloatParameter(handle, parameter, parameterValue, simx_opmode_oneshot_wait) != simx_error_noerror)
  //	return EPUCK_PARAMETER_NOT_FOUND_ERROR;
  return EPUCK_NO_ERROR;
}*/

/*error_code GetObjectProximitySensor(WbDeviceTag handle, char* detectionState, float* detectedPointCoords, int* detectedObjectHandle)
{
  //:TODO: Change to streaming
  //if(simxReadProximitySensor(handle, detectionState, detectedPointCoords, detectedObjectHandle, NULL, simx_opmode_oneshot_wait) != simx_error_noerror)
  //	return EPUCK_PARAMETER_NOT_FOUND_ERROR;
  return EPUCK_NO_ERROR;
}*/

//Setters
/*error_code SetObjectPosition(WbDeviceTag handle, float* position)
{
  //simxSetObjectPosition(handle, -1, position, simx_opmode_oneshot);
  return EPUCK_NO_ERROR;
}*/

//error_code SetJointVelocity(int handle, float velocity)
//{
//simxSetJointTargetVelocity(handle, velocity, simx_opmode_oneshot_wait);
//simxSetJointTargetVelocity(handle, velocity, simx_opmode_oneshot);
//	return EPUCK_NO_ERROR;
//}

error_code SetDifferentialWheelsVelocity(float leftSpeed, float rightSpeed)
{
  wb_differential_wheels_set_speed(leftSpeed, rightSpeed);
  return EPUCK_NO_ERROR;
}


error_code PauseSimulation(int timeInMs)
{
  return EPUCK_NO_ERROR;
}

void EndClient(EPuck* robot, error_code error)
{
  switch(error)
    {
    case EPUCK_NO_ERROR:
      {
	//printf("Ai Sim!\n");
	break;
      }
    case EPUCK_ARGUMENTS_ERROR:
      {
	//printf("Invalid number of arguments.\n");
	break;
      }
    case EPUCK_CONNECTION_ERROR:
      {
	//printf("Could not connect to server.\n");
	break;
      }
    case EPUCK_HANDLE_NOT_FOUND_ERROR:
      {
	//printf("Could not create the robot.\n");
	break;
      }
    case EPUCK_POSITION_NOT_FOUND_ERROR:
      {
	//printf("Position not found.\n");
	break;
      }
    case EPUCK_PARAMETER_NOT_FOUND_ERROR:
      {
	//printf("Could not found a parameter.\n");
	break;
      }
    case EPUCK_JOINT_VELOCITY_ERROR:
      {
	//printf("Could not set joint velocity.\n");
	break;
      }
    default:
      {
	//printf("Controller ends with error.\n");
	break;
      }
    };

  free(robot->mPosition);
  free(robot);
}

int main(int argc, char **argv)
{
  //freopen ("stdout.txt","w",stdout);

  EPuck* robot = malloc(sizeof(EPuck));

  error_code errCode = InitController("127.0.0.1", 60000, robot);
  if(errCode != EPUCK_NO_ERROR)
    {
      EndClient(robot, errCode);
      return FALSE;
    }
  else
    {
      printf("Initializing control!\n");
      errCode = Control(robot);
      if(errCode != EPUCK_NO_ERROR)
	{
	  printf("Error controlling the robot!\n");
	  EndClient(robot, errCode);
	  return FALSE;
	}
    }
  printf("Ending client!\n");
  EndClient(robot, EPUCK_NO_ERROR);

  Disconnect();

  //fclose (stdout);

  return 0;


  /* necessary to initialize webots stuff */
  //wb_robot_init();

  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  //while (wb_robot_step(TIME_STEP) != -1) {

  /*
   * Read the sensors :
   * Enter here functions to read sensor data, like:
   *  double val = wb_distance_sensor_get_value(my_sensor);
   */

  /* Process sensor data here */

  /*
   * Enter here functions to send actuator commands, like:
   * wb_differential_wheels_set_speed(100.0,100.0);
   */
  //};

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  //wb_robot_cleanup();

  //return 0;
}
