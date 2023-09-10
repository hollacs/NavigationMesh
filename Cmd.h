#pragma once

#include "extdll.h"

#include <string>
#include <vector>

typedef void (*pfnCmdCallback_t)(edict_t* pEntity, const std::vector<std::string>&);

class Cmd
{
public:
	Cmd(const char* pszName, pfnCmdCallback_t callback);

	const char* GetName(void) const { return m_pszName; }

	void MakeCallback(edict_t* pEntity, const std::vector<std::string>& arguments) const;

private:
	const char* m_pszName;
	pfnCmdCallback_t m_pfnCallback;
};

void HandleCmds(edict_t* pEntity);