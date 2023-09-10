#pragma once

#include "NavmeshManager.h"

#include "extdll.h"
#include "usercmd.h"

#include <string>

class NavmeshEditor
{
public:
	NavmeshEditor() : 
		m_pEditor(nullptr), m_pAimMesh(nullptr), m_pDragMesh(nullptr), m_pMeshManager(nullptr), 
		m_doublePressTime(0), m_doublePressed(false), m_tick(0), m_tick2(0)
	{}

	void AssignMesh(NavmeshManager* pManager) { m_pMeshManager = pManager; }

	void Update(edict_t* pEntity);
	void CmdStart(const edict_t* pPlayer, const usercmd_t* cmd, unsigned int random_seed);
	void HandleCmd(edict_t* pEntity, const std::vector<std::string>& arguments);

	void MarkPoint(edict_t* pEntity);

	void Reset();

	void SplitMesh(edict_t* pEntity);

private:

	edict_t* m_pEditor;
	Navmesh *m_pAimMesh;
	Navmesh* m_pDragMesh;
	Navmesh* m_pStart;
	Navmesh* m_pGoal;
	Navmesh m_navmesh;
	NavmeshManager* m_pMeshManager;
	std::vector<Vector> m_pts;
	Vector m_dragPoint;
	Vector m_splitPoints[2];
	float m_doublePressTime;
	bool m_doublePressed;
	float m_tick, m_tick2;
};