#include "amxxmodule.h"
#include "Cmd.h"
#include "NavmeshEditor.h"
#include "NavmeshApi.h"

short g_sprite;

void HandleCmd_Nav(edict_t* pEntity, const std::vector<std::string>& arguments);
//void HandleCmd_Npc(edict_t* pEntity, const std::vector<std::string>& arguments);
void HandleCmd_Drop(edict_t* pEntity, const std::vector<std::string>& arguments);
void HandleCmd_BuyEquip(edict_t* pEntity, const std::vector<std::string>& arguments);

static Cmd cmd_buyequip("buyequip", HandleCmd_BuyEquip);
static Cmd cmd_nav("nav", HandleCmd_Nav);
static Cmd cmd_drop("drop", HandleCmd_Drop);

NavmeshManager g_meshManager;
NavmeshEditor g_meshEditor;
char g_pszFilePath[100];

void OnPluginsLoaded()
{
	g_sprite = PRECACHE_MODEL("sprites/zbeam4.spr");

	g_meshEditor.AssignMesh(&g_meshManager);

	MF_BuildPathnameR(g_pszFilePath, sizeof(g_pszFilePath), "%s/navmesh/%s.txt", MF_GetLocalInfo("amxx_configsdir", "addons/amxmodx/configs"), STRING(gpGlobals->mapname));
	
	g_meshManager.LoadFile(g_pszFilePath);

	RegisterFunctions();
}

void OnPluginsUnloaded()
{
	g_meshManager.Clear();
	g_meshEditor.Reset();
}

void ClientCommand(edict_t* pEntity)
{
	HandleCmds(pEntity);
	RETURN_META(MRES_IGNORED);
}

void PlayerPreThink(edict_t* pEntity)
{
	g_meshEditor.Update(pEntity);
	RETURN_META(MRES_IGNORED);
}

void CmdStart(const edict_t* pPlayer, const struct usercmd_s* pCmd, unsigned int random_seed)
{
	g_meshEditor.CmdStart(pPlayer, pCmd, random_seed);
	RETURN_META(MRES_IGNORED);
}

void HandleCmd_Nav(edict_t* pEntity, const std::vector<std::string>& arguments)
{
	g_meshEditor.HandleCmd(pEntity, arguments);
}

void HandleCmd_Drop(edict_t* pEntity, const std::vector<std::string>& arguments)
{
	g_meshEditor.MarkPoint(pEntity);
}

void HandleCmd_BuyEquip(edict_t* pEntity, const std::vector<std::string>& arguments)
{
	g_meshEditor.SplitMesh(pEntity);
}