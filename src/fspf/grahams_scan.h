#include <vector>
#include <math.h>
#include <eigen3/Eigen/Core>
#include <sstream>
#include "util.h"
#include "timer.h"
#include <float.h>

#ifndef GRAHAMS_SCAN_H
#define GRAHAMS_SCAN_H

class GrahamsScan{

public:
  struct point
  {
    Eigen::Vector2f loc;
    double angle;
    //POINTER TO NEXT NODE IN THE LIST
    point *next;
    //POINTER TO PREVIOUS NODE IN THE LIST
    point *prev;
  };

private:
  int NumPoints;
  point* firstPoint; //POINTER TO MIN POINT IN DOUBLELY LINKED LIST

public:
  GrahamsScan();
  ~GrahamsScan();
  std::vector<Eigen::Vector2f> run(std::vector<Eigen::Vector2f> points);
  //ACTUAL GRAHAM'S SCAN PROCEDURE
  void grahamScan(point* P);

private:
  //TEST POINT FOR CONVEXITY
  bool isConvexPoint(point *P);

  //ADDS POINT TO DOUBLELY LINKED LIST (USED DURING SORTING)
  void addPoint(GrahamsScan::point &Point);

  //FIND ANGLE GIVEN TWO POINTS
  double findAngle(Eigen::Vector2f &loc1, Eigen::Vector2f &loc2);

  void deleteAllPoints();
};

#endif //GRAHAMS_SCAN_H