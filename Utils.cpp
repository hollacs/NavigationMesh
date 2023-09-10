#include "Utils.h"

#include "meta_api.h"

void UTIL_ClientPrintAll(int msg_dest, const char* msg_name, const char* param1, const char* param2, const char* param3, const char* param4)
{
	static int msgTextMsg = 0;

	if (msgTextMsg == 0)
	{
		msgTextMsg = GET_USER_MSG_ID(PLID, "TextMsg", NULL);

		if (!msgTextMsg)
			return;
	}

	MESSAGE_BEGIN(MSG_ALL, msgTextMsg);
	WRITE_BYTE(msg_dest);
	WRITE_STRING(msg_name);
	if (param1)
		WRITE_STRING(param1);
	if (param2)
		WRITE_STRING(param2);
	if (param3)
		WRITE_STRING(param3);
	if (param4)
		WRITE_STRING(param4);
	MESSAGE_END();
}

char* UTIL_VarArgs(const char* format, ...)
{
	va_list argptr;
	static char string[1024];

	va_start(argptr, format);
	vsprintf(string, format, argptr);
	va_end(argptr);

	return string;
}

void UTIL_BeamPoints(edict_t* pEntity, const Vector& pos1, const Vector& pos2, short sprite, int startFrame, int frameRate, int life, int width, int noise, int r, int g, int b, int brightness, int speed)
{
	MESSAGE_BEGIN(pEntity == nullptr ? MSG_BROADCAST : MSG_ONE_UNRELIABLE, SVC_TEMPENTITY, NULL, pEntity);
	WRITE_BYTE(TE_BEAMPOINTS);
	WRITE_COORD(pos1.x);
	WRITE_COORD(pos1.y);
	WRITE_COORD(pos1.z);
	WRITE_COORD(pos2.x);
	WRITE_COORD(pos2.y);
	WRITE_COORD(pos2.z);
	WRITE_SHORT(sprite);
	WRITE_BYTE(startFrame);		// startframe
	WRITE_BYTE(frameRate);		// framerate
	WRITE_BYTE(life);		// life
	WRITE_BYTE(width);		// width
	WRITE_BYTE(noise);		// noise
	WRITE_BYTE(r);	// r
	WRITE_BYTE(g);		// g
	WRITE_BYTE(b);		// b
	WRITE_BYTE(brightness);	// brightness
	WRITE_BYTE(speed);		// speed
	MESSAGE_END();
}

Vector UTIL_GetAimPos(edict_t *pEntity)
{
	Vector start, end;
	start = pEntity->v.origin + pEntity->v.view_ofs;
	MAKE_VECTORS(pEntity->v.v_angle);
	end = start + gpGlobals->v_forward * 4096;

	TraceResult ptr;
	TRACE_LINE(start, end, ignore_monsters, pEntity, &ptr);

	return ptr.vecEndPos;
}

edict_t* UTIL_FindEntityByClassname(edict_t* pStartEntity, const char* szName)
{
	return UTIL_FindEntityByString(pStartEntity, "classname", szName);
}

edict_t* UTIL_FindEntityByString(edict_t* pentStart, const char* szKeyword, const char* szValue)
{
	edict_t* pentEntity;

	pentEntity = FIND_ENTITY_BY_STRING(pentStart, szKeyword, szValue);

	if (!FNullEnt(pentEntity))
		return pentEntity;

	return NULL;
}

edict_t* UTIL_FindEntityInSphere(edict_t* pStart, const Vector& vecCenter, float flRadius)
{
	if (!pStart) pStart = NULL;

	pStart = FIND_ENTITY_IN_SPHERE(pStart, vecCenter, flRadius);

	if (!FNullEnt(pStart)) return pStart;
	return NULL;
}

bool UTIL_IsInViewCone(edict_t* pEntity, const Vector& origin)
{
	Vector vecLOS, vecForward;
	float flDot;

	MAKE_VECTORS(pEntity->v.v_angle);
	vecForward = gpGlobals->v_forward;
	vecLOS = origin - (pEntity->v.origin + pEntity->v.view_ofs);

	vecLOS = vecLOS.Normalize();
	flDot = DotProduct(vecLOS, vecForward);

	if (flDot >= cos(pEntity->v.fov * (M_PI / 360)))
		return true;

	return false;
}
