#include "planner.h"

using namespace HybridAStar;
//###################################################
//                                        CONSTRUCTOR
//###################################################
Planner::Planner() {
  int thre = 99;
  n.getParam("/hybrid_astar/occ_thre", thre);
  configurationSpace.setOccThre(thre);

  // srand(323);
  // printf("%lf\n", 1.0 * rand() / RAND_MAX);
  // _____
  // TODOS
  //    initializeLookups();
  // Lookup::collisionLookup(collisionLookup);
  // ___________________
  // COLLISION DETECTION
  //    CollisionDetection configurationSpace;
  // _________________
  // TOPICS TO PUBLISH
  pubStart = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/start", 1);

  // ___________________
  // TOPICS TO SUBSCRIBE
  if (Constants::manual) {
    subMap = n.subscribe("/map", 1, &Planner::setMap, this);
  } else {
    subMap = n.subscribe("/occ_map", 1, &Planner::setMap, this);
  }

  subGoal = n.subscribe("/move_base_simple/goal", 1, &Planner::setGoal, this);
  subStart = n.subscribe("/initialpose", 1, &Planner::setStart, this);
};

//###################################################
//                                       LOOKUPTABLES
//###################################################
void Planner::initializeLookups() {
  if (Constants::dubinsLookup) {
    Lookup::dubinsLookup(dubinsLookup);
  }

  Lookup::collisionLookup(collisionLookup);
}

//###################################################
//                                                MAP
//###################################################
void Planner::setMap(const nav_msgs::OccupancyGrid::Ptr map) {
  validMap = true;
  std::cout << "I am seeing the map..." << std::endl;
  if (Constants::coutDEBUG) {
    std::cout << "I am seeing the map..." << std::endl;
  }

  grid = map;
  //update the configuration space with the current map
  configurationSpace.updateGrid(map);
  //create array for Voronoi diagram
//  ros::Time t0 = ros::Time::now();
  int height = map->info.height;
  int width = map->info.width;
  bool** binMap;
  binMap = new bool*[width];

  for (int x = 0; x < width; x++) { binMap[x] = new bool[height]; }

  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      binMap[x][y] = map->data[y * width + x] ? true : false;
    }
  }

  voronoiDiagram.initializeMap(width, height, binMap);
  voronoiDiagram.update();
  voronoiDiagram.visualize();
//  ros::Time t1 = ros::Time::now();
//  ros::Duration d(t1 - t0);
//  std::cout << "created Voronoi Diagram in ms: " << d * 1000 << std::endl;

  // plan if the switch is not set to manual and a transform is available
  if (!Constants::manual && listener.canTransform("/map", ros::Time(0), "/base_link", ros::Time(0), "/map", nullptr)) {

    listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);

    // assign the values to start from base_link
    start.pose.pose.position.x = transform.getOrigin().x();
    start.pose.pose.position.y = transform.getOrigin().y();
    tf::quaternionTFToMsg(transform.getRotation(), start.pose.pose.orientation);

    if (grid->info.height >= start.pose.pose.position.y && start.pose.pose.position.y >= 0 &&
        grid->info.width >= start.pose.pose.position.x && start.pose.pose.position.x >= 0) {
      // set the start as valid and plan
      validStart = true;
    } else  {
      validStart = false;
    }

    plan();
  }

  plan();
}

//###################################################
//                                   INITIALIZE START
//###################################################
void Planner::setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial) {
  float x = initial->pose.pose.position.x / Constants::cellSize;
  float y = initial->pose.pose.position.y / Constants::cellSize;
  float t = tf::getYaw(initial->pose.pose.orientation);
  // publish the start without covariance for rviz
  geometry_msgs::PoseStamped startN;
  startN.pose.position = initial->pose.pose.position;
  startN.pose.orientation = initial->pose.pose.orientation;
  startN.header.frame_id = "map";
  startN.header.stamp = ros::Time::now();

  std::cout << "I am seeing a new start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;

  // if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
  //   validStart = true;
  //   start = *initial;

  //   if (Constants::manual) { plan();}

  //   // publish start for RViz
  //   pubStart.publish(startN);
  // } else {
  //   std::cout << "invalid start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
  // }

  validStart = true;
  start = *initial;

  if (Constants::manual) { plan();}

  // publish start for RViz
  pubStart.publish(startN);
}

//###################################################
//                                    INITIALIZE GOAL
//###################################################
void Planner::setGoal(const geometry_msgs::PoseStamped::ConstPtr& end) {
  // retrieving goal position
  float x = end->pose.position.x / Constants::cellSize;
  float y = end->pose.position.y / Constants::cellSize;
  float t = tf::getYaw(end->pose.orientation);

  std::cout << "I am seeing a new goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;

  // if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
  //   validGoal = true;
  //   goal = *end;

  //   if (Constants::manual) { plan();}

  // } else {
  //   std::cout << "invalid goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
  // }

  validGoal = true;
  goal = *end;

  if (Constants::manual) { plan();}
}

//###################################################
//                                      PLAN THE PATH
//###################################################
void Planner::plan() {
  // if a start as well as goal are defined go ahead and plan
  if (validStart && validGoal && validMap) {
    printf("Process a start, a goal and a map!\n");

    // ___________________________
    // LISTS ALLOWCATED ROW MAJOR ORDER
    int width = grid->info.width;
    int height = grid->info.height;
    int depth = Constants::headings;
    int length = width * height * depth;
    // define list pointers and initialize lists
    Node3D* nodes3D = new Node3D[length]();
    Node2D* nodes2D = new Node2D[width * height]();

    // ________________________
    // retrieving goal position
    float x = goal.pose.position.x / Constants::cellSize;
    float y = goal.pose.position.y / Constants::cellSize;
    float t = tf::getYaw(goal.pose.orientation);
    // set theta to a value (0,2PI]
    t = Helper::normalizeHeadingRad(t);
    const Node3D nGoal(x, y, t, 0, 0, nullptr);
    // __________
    // DEBUG GOAL
    //    const Node3D nGoal(155.349, 36.1969, 0.7615936, 0, 0, nullptr);


    // _________________________
    // retrieving start position
    x = start.pose.pose.position.x / Constants::cellSize;
    y = start.pose.pose.position.y / Constants::cellSize;
    t = tf::getYaw(start.pose.pose.orientation);
    // set theta to a value (0,2PI]
    t = Helper::normalizeHeadingRad(t);
    Node3D nStart(x, y, t, 0, 0, nullptr);
    // ___________
    // DEBUG START
    //    Node3D nStart(108.291, 30.1081, 0, 0, 0, nullptr);


    // ___________________________
    // START AND TIME THE PLANNING
    ros::Time t0 = ros::Time::now();

    // CLEAR THE VISUALIZATION
    visualization.clear();
    // CLEAR THE PATH
    path.clear();
    smoothedPath.clear();
    smoothedVisuPath.clear();
    // GET THE PARAMETERS
    int cost_mode = 0;
    n.getParam("/hybrid_astar/cost_mode", cost_mode);

    double dis_wei = 0.5;
    n.getParam("/hybrid_astar/dis_wei", dis_wei);

    double occ_wei = 0.5;
    n.getParam("/hybrid_astar/occ_wei", occ_wei);
    // FIND THE PATH
    Node3D* nSolution = Algorithm::hybridAStar(nStart, nGoal, nodes3D, nodes2D, width, height,
      configurationSpace, dubinsLookup, visualization, cost_mode, dis_wei, occ_wei);
    // TRACE THE PATH
    smoother.tracePath(nSolution);
    // CREATE THE UPDATED PATH
    path.updatePath(smoother.getPath());
    // SMOOTH THE PATH
    // smoother.smoothPath(voronoiDiagram);
    // CREATE THE UPDATED PATH
    smoothedPath.updatePath(smoother.getPath());

    std::vector<Node3D> visuPath(smoother.getPath());
    if (!visuPath.empty()) {
      for (size_t i = 0; i < visuPath.size(); ++i) {
        x = visuPath[i].getX();
        y = visuPath[i].getY();
        visuPath[i].setX(0.02 * x - 2.57);
        visuPath[i].setY(0.02 * y - 2.57);
      }
    }
    smoothedVisuPath.updatePath(visuPath);

    ros::Time t1 = ros::Time::now();
    ros::Duration d(t1 - t0);
    std::cout << "TIME in ms: " << d * 1000 << std::endl;

    // _________________________________
    // PUBLISH THE RESULTS OF THE SEARCH
    path.publishPath();
    path.publishPathNodes();
    path.publishPathVehicles();

    smoothedPath.publishPath();
    smoothedPath.publishPathNodes();
    smoothedPath.publishPathVehicles();

    smoothedVisuPath.publishPath();
    smoothedVisuPath.publishPathNodes();
    smoothedVisuPath.publishPathVehicles();

    visualization.publishNode3DCosts(nodes3D, width, height, depth);
    visualization.publishNode2DCosts(nodes2D, width, height);

    // _________________________________
    // COMPREHENSIVE COST OF THE PATH
    float dx0 = 0.7068582;
    std::vector<Node3D> sPath = smoother.getPath();
    float compre_cost = 0.0;
    if (!sPath.empty()) {
      for (size_t i = 0; i < sPath.size()-1; ++i) {
        int X = (int)(sPath[i].getX() + 0.5);
        int Y = (int)(sPath[i].getY() + 0.5);
        int occ = (configurationSpace.getGrid())->data[Y * (configurationSpace.getGrid())->info.width + X]; // occ_value $\in$ [0, 100]
        double normal_occ = dx0 * (occ - 0.0) / (100.0 - 0.0);
        double wei_normal_occ = occ_wei * normal_occ;
        int prim = sPath[i].getPrim();
        // const Node3D* pred = sPath[i].getPred();
        const Node3D pred = sPath[i+1];
        // forward driving
        if (prim < 3) {
          // penalize turning
          if (pred.getPrim() != prim) {
            // penalize change of direction
            if (pred.getPrim() > 2) {
              compre_cost += dis_wei * dx0 * Constants::penaltyTurning * Constants::penaltyCOD + wei_normal_occ;
            } else {
              compre_cost += dis_wei * dx0 * Constants::penaltyTurning + wei_normal_occ;
            }
          } else {
            compre_cost += dis_wei * dx0 + wei_normal_occ;
          }
        }
        // reverse driving
        else {
          // penalize turning and reversing
          if (pred.getPrim() != prim) {
            // penalize change of direction
            if (pred.getPrim() < 3) {
              compre_cost += dis_wei * dx0 * Constants::penaltyTurning * Constants::penaltyReversing * Constants::penaltyCOD + wei_normal_occ;
            } else {
              compre_cost += dis_wei * dx0 * Constants::penaltyTurning * Constants::penaltyReversing + wei_normal_occ;
            }
          } else {
            compre_cost += dis_wei * dx0 * Constants::penaltyReversing + wei_normal_occ;
          }
        }
      }
    }
    printf("Comprehensive cost of the resultant path: %f\n", compre_cost);
    printf("sPath.size(): %lu\n", sPath.size());



    delete [] nodes3D;
    delete [] nodes2D;

    // GET THE PARAMETERS
    std::string s_path_name = "/sPath";
    if (!n.getParam("/hybrid_astar/s_path_name", s_path_name)) {
      s_path_name = "/sPath";
    }
    if (s_path_name != "/sPath") {
      ros::shutdown();
    }

  } else {
    std::cout << "missing goal or start" << std::endl;
  }
}
