#include "NavmeshApi.h"
#include "NavmeshManager.h"
#include "AStar.h"
#include "Maths.h"

#include "amxxmodule.h"

extern NavmeshManager g_meshManager;

void RegisterFunctions()
{
	MF_RegisterFunctionEx(&GetClosestNavmesh, "GetClosestNavmesh");
	MF_RegisterFunctionEx(&FindShortestPath, "FindShortestPath");
	MF_RegisterFunctionEx(&IsDirectionWalkable, "IsDirectionWalkable");
}

Navmesh* GetClosestNavmesh(const Vector& origin, float maxDistance)
{
	return g_meshManager.GetClosestMesh(origin, maxDistance);
}

bool FindShortestPath(const Vector& sp, const Vector& ep, Navmesh* pStart, Navmesh* pGoal, float bodysize, int maxSmoothPts, std::vector<Vector>& smoothPath)
{
	PathResult path;
	if (AStar::FindPath(sp, ep, pStart, pGoal, path))
	{
		// Make the start point inside the closest navmesh a little bit
		Vector sp2 = sp;
		if (!pStart->IsInside(sp2))
			sp2 = pStart->Clamp(sp, 5);

		smoothPath = path.Smooth(sp2, ep, maxSmoothPts, bodysize);
		return true;
	}

	return false;
}

bool IsDirectionWalkable(const Vector& sp, const Vector& dir, float bodysize, float distance, Vector &center)
{
	Vector ep = sp + dir * (bodysize + distance);

	Navmesh *pMesh = g_meshManager.GetClosestMesh(sp, 1000);

	if (pMesh != nullptr)
	{
		Vector sp2 = pMesh->Clamp(sp, 5);
		center = pMesh->GetOrigin();

		auto bridges = pMesh->GetBridges();
		for (auto it = bridges.begin(); it != bridges.end(); it++)
		{
			Vector left = it->GetLeftPos();
			Vector right = it->GetRightPos();

			Vector2D intersect;
			if (Geo_GetLineIntersection(sp2.Make2D(), ep.Make2D(), left.Make2D(), right.Make2D(), intersect, intersect) == 1)
				return true;
		}

		if (pMesh->IsInside(ep))
			return true;

		return false;
	}

	return true;
}