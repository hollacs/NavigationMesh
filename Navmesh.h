#pragma once

#include "extdll.h"

#include <vector>

class Navmesh;

class NavmeshBridge
{
public:
	NavmeshBridge() : m_pOwner(nullptr), m_pMesh(nullptr) {}
	NavmeshBridge(Navmesh* pOwner, Navmesh* pMesh, const Vector& junction, const Vector& left, const Vector& dir)
		: m_pOwner(pOwner), m_pMesh(pMesh), m_junction(junction), m_leftPos(left), m_dir(dir)
	{}

	Navmesh* GetMesh() const { return m_pMesh; }
	Navmesh* GetOwner() const { return m_pOwner; }
	Vector GetJunction() const { return m_junction; }
	Vector GetLeftPos() const { return m_leftPos; }
	Vector GetRightPos() const;
	Vector GetDirection() const { return m_dir; }

private:
	Navmesh* m_pOwner;
	Navmesh* m_pMesh;
	Vector m_junction;
	Vector m_leftPos;
	Vector m_dir;
};

class Navmesh
{
public:
	Navmesh() {}
	Navmesh(const Vector& a, const Vector& b, const Vector& c);

	Vector GetCorner(int n) const { return m_corners[n]; }
	Vector GetOrigin() const { return m_origin; }
	Vector GetNormal() const { return m_normal; }

	bool AddNeighbor(Navmesh* pMesh);
	bool RemoveNeighbor(Navmesh* pMesh);
	bool HasNeighbor(Navmesh* pMesh) const;

	Vector CalcZ(const Vector& orig, const Vector& norm, const Vector& pos) const;
	Vector CalcZ(const Vector& pos) { return CalcZ(m_origin, m_normal, pos); }
	Vector Clamp(const Vector& origin, float size=0) const;

	void SetCorner(int n, const Vector& origin)
	{
		m_corners[n] = origin;
	}

	const NavmeshBridge& GetBridge(Navmesh* pMesh) const;
	const std::vector<NavmeshBridge>& GetBridges() { return m_bridges; }

	float GetDistance(const Vector& origin) const;

	bool RayIntersect(const Vector& origin, const Vector& dir, Vector& intersect, float& dist);

	bool IsInside(const Vector& origin, float size=0) const;

private:
	Vector Along(int n, int ax) const;

	Vector m_corners[4];
	Vector m_origin;
	Vector m_normal;

	std::vector<NavmeshBridge> m_bridges;
};