#include "Maths.h"

Vector Geo_CalcNormal(const Vector& a, const Vector& b, const Vector& c)
{
	Vector dir = CrossProduct(b - a, c - a);
	return dir.Normalize();
}

int Geo_GetSide(const Vector& a, const Vector& b, const Vector& c)
{
	float sum = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
	if (sum > 0)
		return 1;
	if (sum < 0)
		return -1;

	return 0;
}

int Geo_GetLineIntersection(const Vector2D &s1p0, const Vector2D& s1p1, const Vector2D& s2p0, const Vector2D& s2p1, Vector2D& i0, Vector2D& i1)
{
    Vector2D    u = s1p1 - s1p0;
    Vector2D    v = s2p1 - s2p0;
    Vector2D    w = s1p0 - s2p0;
    float     D = PERP(u, v);

    // test if  they are parallel (includes either being a point)
    if (fabs(D) < SMALL_NUM) {           // S1 and S2 are parallel
        if (PERP(u, w) != 0 || PERP(v, w) != 0) {
            return 0;                    // they are NOT collinear
        }
        // they are collinear or degenerate
        // check if they are degenerate  points
        float du = DotProduct(u, u);
        float dv = DotProduct(v, v);
        if (du == 0 && dv == 0) {            // both segments are points
            if (s1p0.x != s2p0.x && s1p0.y != s2p0.y)         // they are distinct  points
                return 0;
            i0 = s1p0;                 // they are the same point
            return 1;
        }
        if (du == 0) {                     // S1 is a single point
            if (InSegment(s1p0, s2p0, s2p1) == 0)  // but is not in S2
                return 0;
            i0 = s1p0;
            return 1;
        }
        if (dv == 0) {                     // S2 a single point
            if (InSegment(s2p0, s1p0, s1p1) == 0)  // but is not in S1
                return 0;
            i0 = s2p0;
            return 1;
        }
        // they are collinear segments - get  overlap (or not)
        float t0, t1;                    // endpoints of S1 in eqn for S2
        Vector2D w2 = s1p1 - s2p0;
        if (v.x != 0) {
            t0 = w.x / v.x;
            t1 = w2.x / v.x;
        }
        else {
            t0 = w.y / v.y;
            t1 = w2.y / v.y;
        }
        if (t0 > t1) {                   // must have t0 smaller than t1
            float t = t0; t0 = t1; t1 = t;    // swap if not
        }
        if (t0 > 1 || t1 < 0) {
            return 0;      // NO overlap
        }
        t0 = t0 < 0 ? 0 : t0;               // clip to min 0
        t1 = t1 > 1 ? 1 : t1;               // clip to max 1
        if (t0 == t1) {                  // intersect is a point
            i0 = s2p0 + t0 * v;
            return 1;
        }

        // they overlap in a valid subsegment
        i0 = s2p0 + t0 * v;
        i1 = s2p0 + t1 * v;
        return 2;
    }

    // the segments are skew and may intersect in a point
    // get the intersect parameter for S1
    float     sI = PERP(v, w) / D;
    if (sI < 0 || sI > 1)                // no intersect with S1
        return 0;

    // get the intersect parameter for S2
    float     tI = PERP(u, w) / D;
    if (tI < 0 || tI > 1)                // no intersect with S2
        return 0;

    i0 = s1p0 + sI * u;                // compute S1 intersect point
    return 1;
}

int InSegment(const Vector2D& p, const Vector2D& sp0, const Vector2D &sp1)
{
    if (sp0.x != sp1.x) {    // S is not  vertical
        if (sp0.x <= p.x && p.x <= sp1.x)
            return 1;
        if (sp0.x >= p.x && p.x >= sp1.x)
            return 1;
    }
    else {    // S is vertical, so test y  coordinate
        if (sp0.y <= p.y && p.y <= sp1.y)
            return 1;
        if (sp0.y >= p.y && p.y >= sp1.y)
            return 1;
    }
    return 0;
}