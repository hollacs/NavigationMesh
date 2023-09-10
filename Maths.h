#pragma once

#include "extdll.h"

#define PERP(u,v)  ((u).x * (v).y - (u).y * (v).x)
#define SMALL_NUM   0.00000001

Vector Geo_CalcNormal(const Vector& a, const Vector& b, const Vector& c);

int Geo_GetSide(const Vector& a, const Vector& b, const Vector& c);

int Geo_GetLineIntersection(const Vector2D& s1p0, const Vector2D& s1p1, const Vector2D& s2p0, const Vector2D& s2p1, Vector2D& i0, Vector2D& i1);

int InSegment(const Vector2D& p, const Vector2D& sp0, const Vector2D& sp1);