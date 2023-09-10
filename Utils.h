#pragma once

#include "extdll.h"

void UTIL_ClientPrintAll(int msg_dest, const char* msg_name, const char* param1, const char* param2, const char* param3, const char* param4);

char* UTIL_VarArgs(const char* format, ...);

void UTIL_BeamPoints(edict_t* pEntity, const Vector& pos1, const Vector& pos2, short sprite, int startFrame, int frameRate, int life, int width, int noise, int r, int g, int b, int brightness, int speed);

Vector UTIL_GetAimPos(edict_t* pEntity);

edict_t* UTIL_FindEntityByClassname(edict_t* pStartEntity, const char* szName);

edict_t* UTIL_FindEntityByString(edict_t* pentStart, const char* szKeyword, const char* szValue);

edict_t* UTIL_FindEntityInSphere(edict_t* pStart, const Vector& vecCenter, float flRadius);