#include "Navmesh.h"

#include "Maths.h"
#include "meta_api.h"

#include <array>
#include <algorithm>

Vector NavmeshBridge::GetRightPos() const
{
	return m_junction + (m_junction - m_leftPos).Normalize() * (m_junction - m_leftPos).Length();
}

Navmesh::Navmesh(const Vector& a, const Vector& b, const Vector& c)
{
	std::array<Vector, 3> arr = { a, b, c };

	auto x_extremes = std::minmax_element(arr.begin(), arr.end(),
		[](const Vector& lhs, const Vector& rhs) {
			return lhs.x < rhs.x;
		});

	auto y_extremes = std::minmax_element(arr.begin(), arr.end(),
		[](const Vector& lhs, const Vector& rhs) {
			return lhs.y < rhs.y;
		});

	m_corners[0] = Vector(x_extremes.first->x, y_extremes.first->y, 0);
	m_corners[1] = Vector(x_extremes.first->x, y_extremes.second->y, 0);
	m_corners[2] = Vector(x_extremes.second->x, y_extremes.second->y, 0);
	m_corners[3] = Vector(x_extremes.second->x, y_extremes.first->y, 0);

	m_normal = Geo_CalcNormal(arr[0], arr[1], arr[2]);

	Vector angle;
	VEC_TO_ANGLES(m_normal, angle);
	if (angle.x > 180)
	{
		m_normal = m_normal * -1;
	}

	for (int i = 0; i < 4; i++)
	{
		m_corners[i] = CalcZ(arr[0], m_normal, m_corners[i]);
	}

	m_origin = (m_corners[0] + m_corners[2]) / 2;
}

Vector Navmesh::Along(int n, int ax) const
{
	int c;

	if (ax == 0)
	{
		c = (n == 0) ? 1 : 3;
	}
	else
	{
		c = (n == 0) ? 3 : 1;
	}

	return m_corners[c];
}

Vector Navmesh::CalcZ(const Vector& orig, const Vector& norm, const Vector& pos) const
{
	Vector linePos = pos;
	linePos.z += 4096;

	Vector lineDir = (pos - linePos).Normalize();

	float t = (DotProduct(norm, orig) - DotProduct(norm, linePos)) / DotProduct(norm, lineDir);
	linePos = linePos + lineDir * t;

	return linePos;
}

Vector Navmesh::Clamp(const Vector& origin, float size) const
{
	Vector mins = m_corners[0];
	Vector maxs = m_corners[2];
	mins.x += size;
	mins.y += size;
	maxs.x -= size;
	maxs.y -= size;

	Vector v = origin;
	v.x = min(maxs.x, max(v.x, mins.x));
	v.y = min(maxs.y, max(v.y, mins.y));

	return v;
}

bool Navmesh::AddNeighbor(Navmesh* pMesh)
{
	if (this == pMesh || HasNeighbor(pMesh))
		return false;

	int ax;
	int c0, c1;
	float gap = FLT_MAX;

	struct
	{
		int ax, c0, c1;
	} closest;

	// Find out which sides are the closest to each other
	for (ax = 0; 2 > ax; ++ax) // Look at x axis and y axis
	{
		if (this->m_corners[0][ax] > pMesh->m_corners[2][ax])
			c0 = 0;
		else
			c0 = 2;

		c1 = c0 ? 0 : 2;

		float dist = fabs(this->m_corners[c0][ax] - pMesh->m_corners[c1][ax]);
		if (dist < gap)
		{
			gap = dist;

			closest.c0 = c0;
			closest.c1 = c1;
			closest.ax = ax;
		}
	}

	if (gap == FLT_MAX)
		return false;

	// Make a list of the 4 points along the common side
	std::array<Vector, 4> list = {
		this->m_corners[closest.c0],
		this->Along(closest.c0, closest.ax),
		pMesh->m_corners[closest.c1],
		pMesh->Along(closest.c1, closest.ax)
	};

	ax = closest.ax ? 0 : 1;

	std::sort(list.begin(), list.end(),
		[ax](const Vector& a, const Vector& b) -> bool {
			return a[ax] > b[ax];
		});

	Vector a = this->Clamp(list[1]);
	Vector b = this->Clamp(list[2]);
	a = this->CalcZ(a);
	b = this->CalcZ(b);

	Vector junction = (a + b) / 2;
	Vector left = a - junction;

	Vector c = pMesh->Clamp(junction);
	c = pMesh->CalcZ(c);

	Vector dir = (c - junction).Normalize();

	float dot = (dir.x * -left.y + dir.y * left.x);
	left = (dot < 0) ? a : b;

	NavmeshBridge bridge = NavmeshBridge(this, pMesh, junction, left, dir);
	m_bridges.push_back(bridge);

	return true;
}

bool Navmesh::RemoveNeighbor(Navmesh* pMesh)
{
	auto iter = std::find_if(m_bridges.begin(), m_bridges.end(),
		[pMesh](const NavmeshBridge& bridge) {
			return bridge.GetMesh() == pMesh;
		});

	if (iter == m_bridges.end())
		return false;

	m_bridges.erase(iter);
	return true;
}

bool Navmesh::HasNeighbor(Navmesh* pMesh) const
{
	auto iter = std::find_if(m_bridges.begin(), m_bridges.end(),
		[pMesh](const NavmeshBridge& bridge) {
			return bridge.GetMesh() == pMesh;
		});

	return (iter != m_bridges.end());
}

const NavmeshBridge& Navmesh::GetBridge(Navmesh* pMesh) const
{
	auto iter = std::find_if(m_bridges.begin(), m_bridges.end(),
		[pMesh](const NavmeshBridge& bridge) {
			return bridge.GetMesh() == pMesh;
		});

	return *iter;
}

float Navmesh::GetDistance(const Vector& origin) const
{
	Vector pos = Clamp(origin);
	pos = CalcZ(m_origin, m_normal, pos);

	return (pos - origin).Length();
}

bool Navmesh::RayIntersect(const Vector& origin, const Vector& dir, Vector& intersect, float& dist)
{
	float denom = DotProduct(m_normal, dir);
	if (fabs(denom) > 0.0001f)
	{
		dist = DotProduct(m_origin - origin, m_normal) / denom;

		if (dist >= 0)
		{
			intersect = origin + dir * dist;
			if (IsInside(intersect))
			{
				return true;
			}
		}
	}

	return false;
}

bool Navmesh::IsInside(const Vector& origin, float size) const
{
	Vector mins = m_corners[0];
	Vector maxs = m_corners[2];

	if (origin.x >= mins.x && origin.y >= mins.y && origin.x <= maxs.x && origin.y <= maxs.y)
		return true;

	return false;
}