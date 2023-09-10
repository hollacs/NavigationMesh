#pragma once

#include <vector>

class Vector;
class Navmesh;

void RegisterFunctions();

Navmesh* GetClosestNavmesh(const Vector& origin, float maxDistance);

bool FindShortestPath(const Vector& sp, const Vector& ep, Navmesh* pStart, Navmesh* pGoal, float bodysize, int maxSmoothPts, std::vector<Vector>& smoothPath);

bool IsDirectionWalkable(const Vector& sp, const Vector& dir, float bodysize, float distance, Vector& center);