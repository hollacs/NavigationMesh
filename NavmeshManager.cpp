#include "NavmeshManager.h"

#include <algorithm>
#include <sstream>

Navmesh* NavmeshManager::CreateMesh(const Vector& a, const Vector& b, const Vector& c)
{
	Navmesh* pMesh = new Navmesh(a, b, c);
	m_navmeshes.push_back(pMesh);

	return pMesh;
}

Navmesh* NavmeshManager::CreateMesh(Vector pts[3])
{
	return CreateMesh(pts[0], pts[1], pts[2]);
}

void NavmeshManager::RemoveMesh(Navmesh* pMesh)
{
	// Disconnect if it was connected to this mesh
	for (auto it = m_navmeshes.begin(); it != m_navmeshes.end(); ++it)
	{
		Navmesh* pmesh = *it;
		pmesh->RemoveNeighbor(pMesh);
	}

	m_navmeshes.erase(std::remove(m_navmeshes.begin(), m_navmeshes.end(), pMesh), m_navmeshes.end());
	delete pMesh; // release memory
}

void NavmeshManager::Clear()
{
	for (auto it = m_navmeshes.begin(); it != m_navmeshes.end(); ++it)
	{
		// release memory
		delete (*it);
	}

	m_navmeshes.clear();
}

int NavmeshManager::GetIndex(Navmesh* pMesh) const
{
	auto it = std::find(m_navmeshes.begin(), m_navmeshes.end(), pMesh);
	if (it == m_navmeshes.end())
		return -1;

	return std::distance(m_navmeshes.begin(), it);
}

Navmesh* NavmeshManager::At(size_t index) const
{
	if (index >= this->Size())
		return nullptr;

	return m_navmeshes.at(index);
}

Navmesh* NavmeshManager::GetClosestMesh(const Vector& orig, float maxRadius) const
{
	float minRadius = maxRadius;
	Navmesh* pResult = nullptr;

	for (auto it = m_navmeshes.cbegin(); it != m_navmeshes.cend(); ++it)
	{
		Navmesh* pMesh = *it;

		float radius = pMesh->GetDistance(orig);
		if (radius < minRadius)
		{
			minRadius = radius;
			pResult = pMesh;
		}
	}

	return pResult;
}

Navmesh* NavmeshManager::GetAimMesh(Vector start, Vector end, Vector& intersect) const
{
	Vector dir = (end - start).Normalize();

	float minDist = (start - end).Length();

	Navmesh* pResult = nullptr;

	for (auto it = m_navmeshes.begin(); it != m_navmeshes.end(); ++it)
	{
		Navmesh* pMesh = *it;

		Vector pos;
		float dist;

		if (pMesh->RayIntersect(start, dir, pos, dist))
		{
			if (dist < minDist)
			{
				minDist = dist;
				intersect = pos;
				pResult = pMesh;
			}
		}
	}

	return pResult;
}

bool NavmeshManager::SaveFile(const char* pszPath)
{
	FILE* pFile = fopen(pszPath, "w");
	if (pFile != NULL)
	{
		fputs("[data]\n", pFile);

		for (auto it = m_navmeshes.begin(); it != m_navmeshes.end(); it++)
		{
			Navmesh* pMesh = *it;

			Vector v[3] = { pMesh->GetCorner(0), pMesh->GetCorner(1), pMesh->GetCorner(2) };

			fprintf(pFile, "%f %f %f %f %f %f %f %f %f\n",
				v[0].x, v[0].y, v[0].z,
				v[1].x, v[1].y, v[1].z,
				v[2].x, v[2].y, v[2].z);
		}

		fputs("[bridges]\n", pFile);

		for (auto it = m_navmeshes.begin(); it != m_navmeshes.end(); it++)
		{
			Navmesh* pMesh = *it;

			auto bridges = pMesh->GetBridges();

			for (auto iter = bridges.begin(); iter != bridges.end(); iter++)
			{
				fprintf(pFile, "%d %d\n",
					this->GetIndex(iter->GetOwner()),
					this->GetIndex(iter->GetMesh()));
			}
		}

		fclose(pFile);

		return true;
	}

	return false;
}

bool NavmeshManager::LoadFile(const char* pszPath)
{
	this->Clear();

	FILE* pFile = fopen(pszPath, "r");
	if (pFile != NULL)
	{
		int mode = 1;
		char szLine[512];
		while (fgets(szLine, sizeof(szLine), pFile))
		{
			if (!szLine[0])
				continue;

			if (szLine[0] == '[')
			{
				if (szLine[1] == 'd')
				{
					mode = 1;
					continue;
				}
				else if (szLine[1] == 'b')
				{
					mode = 2;
					continue;
				}
			}

			if (mode == 1)
			{
				std::istringstream iss(szLine);

				Vector v[3];
				iss >> v[0].x >> v[0].y >> v[0].z >> v[1].x >> v[1].y >> v[1].z >> v[2].x >> v[2].y >> v[2].z;

				CreateMesh(v);
			}
			else if (mode == 2)
			{
				std::istringstream iss(szLine);

				int i, j;
				iss >> i >> j;

				auto pSrc = this->At(i);
				auto pDest = this->At(j);

				pSrc->AddNeighbor(pDest);
			}
		}

		fclose(pFile);

		return true;
	}

	return false;
}


void NavmeshManager::SplitMesh(Navmesh* pMesh, const Vector& a, const Vector& b)
{
	Vector min1, min2;
	Vector max1, max2;
	Vector c1, c2;

	min1 = pMesh->GetCorner(0);
	max2 = pMesh->GetCorner(2);

	int ax = (a.x == b.x) ? 0 : 1;
	int ay = (ax == 0) ? 1 : 0;

	if (a[ax] > min1[ax] && a[ay] > min1[ay])
	{
		c1 = b;
		c2 = a;
		max1 = a;
		min2 = b;
	}
	else
	{
		c1 = a;
		c2 = b;
		max1 = b;
		min2 = a;
	}

	max1[ax] -= 3;
	min2[ax] += 3;
	c1[ax] = max1[ax];
	c2[ax] = min2[ax];

	auto bridges = pMesh->GetBridges();

	std::vector<Navmesh*> neighbors;

	for (auto it = bridges.begin(); it != bridges.end(); it++)
	{
		neighbors.push_back(it->GetMesh());
	}

	RemoveMesh(pMesh);

	Navmesh* pa = CreateMesh(min1, max1, c1); // near min
	Navmesh* pb = CreateMesh(min2, max2, c2); // near max

	pa->AddNeighbor(pb);
	pb->AddNeighbor(pa);

	for (auto it = neighbors.begin(); it != neighbors.end(); it++)
	{
		Navmesh* pc = *it;
		Vector min = pc->GetCorner(0);
		Vector max = pc->GetCorner(2);

		if (min[ax] < max1[ax])
		{
			pa->AddNeighbor(pc);
			pc->AddNeighbor(pa);
		}

		if (max[ax] > min2[ax])
		{
			pb->AddNeighbor(pc);
			pc->AddNeighbor(pb);
		}
	}
}