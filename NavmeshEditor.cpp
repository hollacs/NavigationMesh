#include "NavmeshEditor.h"
#include "AStar.h"
#include "Utils.h"

#include "meta_api.h"

extern short g_sprite;
extern char g_pszFilePath[100];

void NavmeshEditor::Update(edict_t* pEntity)
{
	if (m_pEditor != pEntity)
		return;

	if (gpGlobals->time >= m_tick2 + 0.1)
	{
		if (m_pDragMesh != nullptr)
		{
			Vector aimpos = UTIL_GetAimPos(pEntity);
			aimpos.z += 5;

			if (pEntity->v.button & IN_USE)
				UTIL_BeamPoints(pEntity, m_dragPoint, aimpos, g_sprite, 0, 0, 1, 15, 0, 200, 100, 0, 255, 0);
			else
				UTIL_BeamPoints(pEntity, m_dragPoint, aimpos, g_sprite, 0, 0, 1, 15, 0, 255, 0, 0, 255, 0);
		}
		else if (m_pAimMesh != nullptr)
		{
			int ax = 0;

			float angle_y = pEntity->v.v_angle.y + 180;
			if (angle_y > 50 && angle_y < 130)
			{
				angle_y = 90;
				ax = 1;
			}
			else if (angle_y > 140 && angle_y < 220)
			{
				angle_y = 180;
				ax = 0;
			}
			else if (angle_y > 230 && angle_y < 310)
			{
				angle_y = 270;
				ax = 1;
			}
			else if (angle_y > 320 || angle_y < 40)
			{
				angle_y = 0;
				ax = 0;
			}
			else
			{
				angle_y = 999;
			}

			//UTIL_ClientPrintAll(HUD_PRINTTALK, UTIL_VarArgs("y = %.2f\n", pEntity->v.v_angle.x));

			if (angle_y != 999)
			{
				MAKE_VECTORS(pEntity->v.v_angle);

				float dist;
				Vector intersect;
				m_pAimMesh->RayIntersect(pEntity->v.origin + pEntity->v.view_ofs, gpGlobals->v_forward, intersect, dist);

				Vector a, b;
				a = b = intersect;
				a[ax] = m_pAimMesh->GetCorner(0)[ax];
				b[ax] = m_pAimMesh->GetCorner(2)[ax];
				a = m_pAimMesh->Clamp(a);
				b = m_pAimMesh->Clamp(b);
				a = m_pAimMesh->CalcZ(a);
				b = m_pAimMesh->CalcZ(b);
				m_splitPoints[0] = a;
				m_splitPoints[1] = b;

				UTIL_BeamPoints(pEntity, a, b, g_sprite, 0, 0, 1, 15, 0, 255, 0, 0, 255, 0);
			}
		}

		m_tick2 = gpGlobals->time;
	}

	if (gpGlobals->time >= m_tick + 0.5)
	{
		for (size_t i = 0; i < m_pts.size(); i++)
		{
			Vector v = m_pts[i];
			UTIL_BeamPoints(pEntity, v, Vector(v.x, v.y, v.z + 16), g_sprite, 0, 0, 4, 10, 0, 0, 100, 200, 200, 0);
		}

		if (m_pts.size() >= 3)
		{
			for (int i = 0, j = 3; i < 4; j = i++)
			{
				Vector a = m_navmesh.GetCorner(i);
				Vector b = m_navmesh.GetCorner(j);

				UTIL_BeamPoints(pEntity, a, b, g_sprite, 0, 0, 4, 10, 0, 0, 200, 200, 255, 0);
				UTIL_BeamPoints(pEntity, m_navmesh.GetOrigin(), m_navmesh.GetOrigin() + m_navmesh.GetNormal() * 20, 
					g_sprite, 0, 0, 5, 10, 0, 0, 200, 200, 255, 0);
			}
		}

		if (m_pMeshManager->Size() > 0)
		{
			Vector start = pEntity->v.origin + pEntity->v.view_ofs;
			MAKE_VECTORS(pEntity->v.v_angle);
			Vector end = start + gpGlobals->v_forward * 4096;

			Vector intersect;
			m_pAimMesh = m_pMeshManager->GetAimMesh(start, end, intersect);

			auto navmeshes = m_pMeshManager->CloneNavmeshes();
			int size = min(navmeshes.size(), 10);

			for (int i = 0; i < size; i++)
			{
				Navmesh* pMin = nullptr;
				std::vector<Navmesh*>::iterator iter;
				float minDist = FLT_MAX;

				for (auto it = navmeshes.begin(); it != navmeshes.end(); it++)
				{
					float dist = (*it)->GetDistance(pEntity->v.origin);
					if (dist < minDist)
					{
						minDist = dist;
						iter = it;
						pMin = *it;
					}
				}

				if (pMin == nullptr)
					break;

				navmeshes.erase(iter);

				if (m_pAimMesh == pMin)
				{
					for (int i = 0, j = 3; i < 4; j = i++)
					{
						UTIL_BeamPoints(pEntity, pMin->GetCorner(i), pMin->GetCorner(j), g_sprite, 0, 0, 5, 10, 0, 200, 200, 0, 255, 0);
						UTIL_BeamPoints(pEntity, pMin->GetOrigin(), pMin->GetOrigin() + pMin->GetNormal() * 16, g_sprite, 0, 0, 5, 10, 0, 200, 200, 0, 255, 0);
					}

					auto& bridges = pMin->GetBridges();

					for (auto it = bridges.begin(); it != bridges.end(); it++)
					{
						Vector pos = it->GetJunction();
						Vector dir = it->GetDirection();
						Vector a = pos - dir * 16;
						Vector b = pos + dir * 16;
						a.z += 5;
						b.z += 5;
						a = it->GetOwner()->CalcZ(a);
						b = it->GetMesh()->CalcZ(b);
						a.z += 5;
						b.z += 5;

						UTIL_BeamPoints(pEntity, a, b, g_sprite, 0, 0, 5, 10, 0, 200, 100, 0, 255, 0);
					}
				}
				else
				{
					for (int i = 0, j = 3; i < 4; j = i++)
					{
						UTIL_BeamPoints(pEntity, pMin->GetCorner(i), pMin->GetCorner(j), g_sprite, 0, 0, 5, 10, 0, 0, 255, 0, 255, 0);
					}
				}
			}
		}

		m_tick = gpGlobals->time;
	}
}

void NavmeshEditor::CmdStart(const edict_t* pPlayer, const usercmd_t* cmd, unsigned int random_seed)
{
	if (m_pEditor != pPlayer)
		RETURN_META(MRES_IGNORED);

	if (cmd->impulse == 201)
	{
		if (m_pts.size() < 3)
		{
			UTIL_ClientPrintAll(HUD_PRINTTALK, "[NAV] You have to mark down 3 points first\n");
			return;
		}

		m_pMeshManager->CreateMesh(m_pts.data());
		m_pts.clear();
	}

	// both way connection
	if ((cmd->buttons & IN_USE) && (~pPlayer->v.oldbuttons & IN_USE))
	{
		if (gpGlobals->time < m_doublePressTime + 0.3)
		{
			UTIL_ClientPrintAll(HUD_PRINTTALK, "[NAV] One-way\n");
			m_doublePressed = true;
		}

		Vector start = pPlayer->v.origin + pPlayer->v.view_ofs;
		MAKE_VECTORS(pPlayer->v.v_angle);
		Vector end = start + gpGlobals->v_forward * 4096;

		m_pDragMesh = m_pMeshManager->GetAimMesh(start, end, m_dragPoint);
		m_dragPoint.x += 5;

		m_doublePressTime = gpGlobals->time;
	}
	else if ((pPlayer->v.oldbuttons & IN_USE) && (~cmd->buttons & IN_USE))
	{
		if (m_pDragMesh != nullptr)
		{
			Vector start = pPlayer->v.origin + pPlayer->v.view_ofs;
			MAKE_VECTORS(pPlayer->v.v_angle);
			Vector end = start + gpGlobals->v_forward * 4096;

			Vector aim;
			Navmesh* pAim = m_pMeshManager->GetAimMesh(start, end, aim);

			if (pAim != nullptr && pAim != m_pDragMesh)
			{
				m_pDragMesh->AddNeighbor(pAim);

				UTIL_ClientPrintAll(HUD_PRINTTALK, UTIL_VarArgs("[NAV] Navmesh #%d add neighbor (#%d)\n", 
					m_pMeshManager->GetIndex(m_pDragMesh), m_pMeshManager->GetIndex(pAim)));

				if (!m_doublePressed)
				{
					pAim->AddNeighbor(m_pDragMesh);
					UTIL_ClientPrintAll(HUD_PRINTTALK, UTIL_VarArgs("[NAV] Navmesh #%d add neighbor (#%d)\n", 
						m_pMeshManager->GetIndex(pAim), m_pMeshManager->GetIndex(m_pDragMesh)));
				}
			}

			m_pDragMesh = nullptr;
			m_doublePressed = false;
		}
	}

	// remove connection
	if ((cmd->buttons & IN_RELOAD) && (~pPlayer->v.oldbuttons & IN_RELOAD))
	{
		Vector start = pPlayer->v.origin + pPlayer->v.view_ofs;
		MAKE_VECTORS(pPlayer->v.v_angle);
		Vector end = start + gpGlobals->v_forward * 4096;

		m_pDragMesh = m_pMeshManager->GetAimMesh(start, end, m_dragPoint);
		m_dragPoint.x += 5;
	}
	else if ((pPlayer->v.oldbuttons & IN_RELOAD) && (~cmd->buttons & IN_RELOAD))
	{
		if (m_pDragMesh != nullptr)
		{
			Vector start = pPlayer->v.origin + pPlayer->v.view_ofs;
			MAKE_VECTORS(pPlayer->v.v_angle);
			Vector end = start + gpGlobals->v_forward * 4096;

			Vector aim;
			Navmesh* pAim = m_pMeshManager->GetAimMesh(start, end, aim);

			if (pAim != nullptr && pAim != m_pDragMesh)
			{
				m_pDragMesh->RemoveNeighbor(pAim);
				pAim->RemoveNeighbor(m_pDragMesh);

				UTIL_ClientPrintAll(HUD_PRINTTALK, UTIL_VarArgs("[NAV] Navmesh #%d remove neighbor (#%d)\n", 
					m_pMeshManager->GetIndex(m_pDragMesh), m_pMeshManager->GetIndex(pAim)));
				UTIL_ClientPrintAll(HUD_PRINTTALK, UTIL_VarArgs("[NAV] Navmesh #%d remove neighbor (#%d)\n", 
					m_pMeshManager->GetIndex(pAim), m_pMeshManager->GetIndex(m_pDragMesh)));
			}

			m_pDragMesh = nullptr;
		}
	}

	RETURN_META(MRES_IGNORED);
}

void NavmeshEditor::HandleCmd(edict_t* pEntity, const std::vector<std::string>& arguments)
{
	if (arguments.size() < 1)
		return;

	if (arguments[0] == "edit")
	{
		if (arguments.size() >= 2)
		{
			int num = std::stoi(arguments[1]);

			if (num == 0)
			{
				m_pEditor = NULL;
				UTIL_ClientPrintAll(HUD_PRINTTALK, "[NAV] Navmesh Edit Mode: Off\n");
			}
			else
			{
				m_pEditor = pEntity;
				UTIL_ClientPrintAll(HUD_PRINTTALK, "[NAV] Navmesh Edit Mode: On\n");
			}
		}

		return;
	}

	if (m_pEditor != pEntity)
	{
		UTIL_ClientPrintAll(HUD_PRINTTALK, "[NAV] Only the editor can use this command\n");
		return;
	}

	if (arguments[0] == "nomark")
	{
		m_pts.clear();
	}
	else if (arguments[0] == "start")
	{
		m_pStart = m_pAimMesh;
		UTIL_ClientPrintAll(HUD_PRINTTALK, "[NAV] Start Set");
	}
	else if (arguments[0] == "goal")
	{
		m_pGoal = m_pAimMesh;
		UTIL_ClientPrintAll(HUD_PRINTTALK, "[NAV] Goal Set");
	}
	else if (arguments[0] == "astar")
	{
		if (m_pStart == nullptr || m_pGoal == nullptr)
			return;

		PathResult path;
		if (AStar::FindPath(m_pStart->GetOrigin(), m_pGoal->GetOrigin(), m_pStart, m_pGoal, path))
		{
			std::vector<Vector> smoothPath = path.Smooth(m_pStart->GetOrigin(), m_pGoal->GetOrigin());

			for (size_t i = 1; i < smoothPath.size(); i++)
			{
				auto p1 = smoothPath[i - 1];
				auto p2 = smoothPath[i];

				UTIL_BeamPoints(pEntity, p1, p2, g_sprite, 0, 0, 100, 15, 0, 0, 100, 200, 255, 0);
			}

			UTIL_ClientPrintAll(HUD_PRINTTALK, UTIL_VarArgs("[NAV] Path found (%d)\n", path.Size()));
		}
		else
		{
			UTIL_ClientPrintAll(HUD_PRINTTALK, "[NAV] Path Not found...\n");
		}
	}
	else if (arguments[0] == "remove")
	{
		if (m_pAimMesh != nullptr)
		{
			UTIL_ClientPrintAll(HUD_PRINTTALK, UTIL_VarArgs("[NAV] Navmesh #%d removed\n", 
				m_pMeshManager->GetIndex(m_pAimMesh)));

			m_pMeshManager->RemoveMesh(m_pAimMesh);
		}
	}
	else if (arguments[0] == "save")
	{
		if (m_pMeshManager->SaveFile(g_pszFilePath))
		{
			UTIL_ClientPrintAll(HUD_PRINTTALK, "[NAV] Saved navmeshes to file successfully\n");
		}
		else
		{
			UTIL_ClientPrintAll(HUD_PRINTTALK, "[NAV] Failed to save navmeshe\n");
		}
	}
	else
	{
		UTIL_ClientPrintAll(HUD_PRINTTALK, "[NAV] Unknown Command\n");
	}
}

void NavmeshEditor::MarkPoint(edict_t* pEntity)
{
	if (pEntity == m_pEditor)
	{
		Vector origin;
		origin = UTIL_GetAimPos(pEntity);
		origin.z += 2;

		if (m_pts.size() >= 3)
			m_pts.erase(m_pts.begin());

		m_pts.push_back(origin);

		if (m_pts.size() >= 3)
			m_navmesh = Navmesh(m_pts[0], m_pts[1], m_pts[2]);
	}
}

void NavmeshEditor::Reset()
{
	m_pEditor = nullptr;
	m_pAimMesh = nullptr;
	m_pDragMesh = nullptr;
	m_pMeshManager = nullptr;
	m_pts.clear();
	m_doublePressed = false;
	m_doublePressTime = 0;
	m_tick = 0;
	m_tick2 = 0;
}

void NavmeshEditor::SplitMesh(edict_t* pEntity)
{
	if (pEntity == m_pEditor)
	{
		if (m_pAimMesh != nullptr)
		{
			m_pMeshManager->SplitMesh(m_pAimMesh, m_splitPoints[0], m_splitPoints[1]);
			UTIL_ClientPrintAll(HUD_PRINTTALK, "SPLIT !!");
		}
	}
}