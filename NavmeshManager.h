#pragma once

#include "Navmesh.h"

#include <vector>

class NavmeshManager
{
public:
	NavmeshManager() {}
	~NavmeshManager() { Clear(); }

	Navmesh* CreateMesh(const Vector& a, const Vector& b, const Vector& c);
	Navmesh* CreateMesh(Vector pts[3]);

	void RemoveMesh(Navmesh* pMesh);

	void Clear();

	int GetIndex(Navmesh* pMesh) const;
	Navmesh* At(size_t index) const;
	size_t Size() const { return m_navmeshes.size(); }

	Navmesh* GetClosestMesh(const Vector& orig, float maxRadius) const;
	Navmesh* GetAimMesh(Vector start, Vector end, Vector& intersect) const;

	std::vector<Navmesh*> CloneNavmeshes() { return m_navmeshes; }

	bool SaveFile(const char* pszPath);
	bool LoadFile(const char* pszPath);

	void SplitMesh(Navmesh* pMesh, const Vector& a, const Vector& b);

private:
	std::vector<Navmesh*> m_navmeshes;
};