#include "Cmd.h"
#include "meta_api.h"

static std::vector<Cmd*> s_Commands;

Cmd::Cmd(const char* pszName, pfnCmdCallback_t callback)
{
	s_Commands.push_back(this);

	m_pszName = pszName;
	m_pfnCallback = callback;
}

void Cmd::MakeCallback(edict_t* pEntity, const std::vector<std::string>& arguments) const
{
	m_pfnCallback(pEntity, arguments);
}

void HandleCmds(edict_t* pEntity)
{
	const char* pszCmdName = CMD_ARGV(0);
	for (Cmd* pCommand : s_Commands)
	{
		if (strcmp(pCommand->GetName(), pszCmdName) != 0)
			continue;

		int arguments_count = CMD_ARGC();
		std::vector<std::string> arguments;
		for (int i = 1; i < arguments_count; i++)
		{
			arguments.push_back(std::string(CMD_ARGV(i)));
		}

		pCommand->MakeCallback(pEntity, arguments);

		RETURN_META(MRES_SUPERCEDE);
		break;
	}

	RETURN_META(MRES_IGNORED);
}