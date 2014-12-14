//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//
{$POINTERMATH ON}

unit RN_DetourCommon;
interface


  procedure dtSwap(var a,b: Single); overload;
  procedure dtSwap(var a,b: Word); overload;
  procedure dtSwap(var a,b: Integer); overload;
  procedure dtSwap(var a,b: Pointer); overload;
  procedure dtSwap(var a,b: PSingle); overload;
  function dtMin(a,b: Single): Single; overload;
  function dtMin(a,b: Integer): Integer; overload;
  function dtMin(a,b: Cardinal): Cardinal; overload;
  function dtMax(a,b: Single): Single; overload;
  function dtMax(a,b: Integer): Integer; overload;
  function dtMax(a,b: Cardinal): Cardinal; overload;
  function dtClamp(v, mn, mx: Single): Single; overload;
  function dtClamp(v, mn, mx: Integer): Integer; overload;
  procedure dtVcross(dest: PSingle; v1, v2: PSingle);
  function dtVdot(v1, v2: PSingle): Single;
  procedure dtVmad(dest: PSingle; v1, v2: PSingle; s: Single);
  procedure dtVlerp(dest: PSingle; v1, v2: PSingle; t: Single);
  procedure dtVadd(dest: PSingle; v1, v2: PSingle);
  procedure dtVsub(dest: PSingle; v1, v2: PSingle);
  procedure dtVmin(mn: PSingle; v: PSingle);
  procedure dtVmax(mx: PSingle; v: PSingle);
  procedure dtVscale(dest: PSingle; v: PSingle; t: Single);
  procedure dtVset(dest: PSingle; x,y,z: Single);
  procedure dtVcopy(dest: PSingle; a: PSingle);
  function dtVlen(v: PSingle): Single;
  function dtVlenSqr(v: PSingle): Single;
  function dtVdist(v1, v2: PSingle): Single;
  function dtVdistSqr(v1, v2: PSingle): Single;
  function dtVdist2D(v1, v2: PSingle): Single;
  function dtVdist2DSqr(v1, v2: PSingle): Single;
  procedure dtVnormalize(v: PSingle);
  function dtVequal(p0, p1: PSingle): Boolean;
  function dtVdot2D(u, v: PSingle): Single;
  function dtVperp2D(u, v: PSingle): Single;
  function dtTriArea2D(a, b, c: PSingle): Single;
  function dtOverlapQuantBounds(amin, amax, bmin, bmax: PWord): Boolean;
  function dtOverlapBounds(amin, amax, bmin, bmax: PSingle): Boolean;
  procedure dtClosestPtPointTriangle(closest, p, a, b, c: PSingle);
  function dtClosestHeightPointTriangle(p, a, b, c: PSingle; h: PSingle): Boolean;
  function dtIntersectSegmentPoly2D(p0, p1: PSingle; verts: PSingle; nverts: Integer; tmin, tmax: PSingle; segMin, segMax: PInteger): Boolean;
  function dtIntersectSegSeg2D(ap, aq, bp, bq: PSingle; s, t: PSingle): Boolean;
  function dtPointInPolygon(pt, verts: PSingle; nverts: Integer): Boolean;
  function dtDistancePtPolyEdgesSqr(pt, verts: PSingle; nverts: Integer; ed, et: PSingle): Boolean;
  function dtDistancePtSegSqr2D(pt, p, q: PSingle; t: PSingle): Single;
  procedure dtCalcPolyCenter(tc: PSingle; const idx: PWord; nidx: Integer; verts: PSingle);
  function dtOverlapPolyPoly2D(polya: PSingle; npolya: Integer; polyb: PSingle; npolyb: Integer): Boolean;
  function dtNextPow2(v: Cardinal): Cardinal;
  function dtIlog2(v: Cardinal): Cardinal;
  function dtAlign4(x: Integer): Integer;
  function dtOppositeTile(side: Integer): Integer;
  procedure dtSwapByte(a, b: PByte);
  procedure dtSwapEndian(v: PWord); overload;
  procedure dtSwapEndian(v: PShortInt); overload;
  procedure dtSwapEndian(v: PCardinal); overload;
  procedure dtSwapEndian(v: PInteger); overload;
  procedure dtSwapEndian(v: PSingle); overload;
  procedure dtRandomPointInConvexPoly(pts: PSingle; npts: Integer; areas: PSingle; s, t: Single; &out: PSingle);


implementation
uses Math, SysUtils;

(*
@defgroup detour Detour

Members in this module are used to create, manipulate, and query navigation
meshes.

@note This is a summary list of members.  Use the index or search
feature to find minor members.
*)

/// @name General helper functions
/// @{

/// Used to ignore a function parameter.  VS complains about unused parameters
/// and this silences the warning.
///  @param [in] _ Unused parameter
//template<class T> void dtIgnoreUnused(const T&) { }

/// Swaps the values of the two parameters.
///  @param[in,out]  a  Value A
///  @param[in,out]  b  Value B
procedure dtSwap(var a,b: Single);
var T: Single;
begin
  T := a; a := b; b := T;
end;

procedure dtSwap(var a,b: Word);
var T: Word;
begin
  T := a; a := b; b := T;
end;

procedure dtSwap(var a,b: Integer);
var T: Integer;
begin
  T := a; a := b; b := T;
end;

procedure dtSwap(var a,b: Pointer);
var T: Pointer;
begin
  T := a; a := b; b := T;
end;

procedure dtSwap(var a,b: PSingle);
var T: PSingle;
begin
  T := a; a := b; b := T;
end;

/// Returns the minimum of two values.
///  @param[in]    a  Value A
///  @param[in]    b  Value B
///  @return The minimum of the two values.
function dtMin(a,b: Single): Single;
begin
  Result := Min(a,b);
end;

function dtMin(a,b: Integer): Integer;
begin
  Result := Min(a,b);
end;

function dtMin(a,b: Cardinal): Cardinal;
begin
  Result := Min(a,b);
end;

/// Returns the maximum of two values.
///  @param[in]    a  Value A
///  @param[in]    b  Value B
///  @return The maximum of the two values.
function dtMax(a,b: Single): Single;
begin
  Result := Max(a,b);
end;

function dtMax(a,b: Integer): Integer;
begin
  Result := Max(a,b);
end;

function dtMax(a,b: Cardinal): Cardinal;
begin
  Result := Max(a,b);
end;

/// Returns the absolute value.
///  @param[in]    a  The value.
///  @return The absolute value of the specified value.
//template<class T> inline T dtAbs(T a) { return a < 0 ? -a : a; }

/// Returns the square of the value.
///  @param[in]    a  The value.
///  @return The square of the value.
function dtSqr(a: Single): Single;
begin
  Result := a*a;
end;

/// Clamps the value to the specified range.
///  @param[in]    v  The value to clamp.
///  @param[in]    mn  The minimum permitted return value.
///  @param[in]    mx  The maximum permitted return value.
///  @return The value, clamped to the specified range.
function dtClamp(v, mn, mx: Single): Single;
begin
  Result := EnsureRange(v, mn, mx);
end;
function dtClamp(v, mn, mx: Integer): Integer;
begin
  Result := EnsureRange(v, mn, mx);
end;

/// Returns the square root of the value.
///  @param[in]    x  The value.
///  @return The square root of the vlaue.
//float dtSqrt(float x);

/// @}
/// @name Vector helper functions.
/// @{

/// Derives the cross product of two vectors. (@p v1 x @p v2)
///  @param[out]  dest  The cross product. [(x, y, z)]
///  @param[in]    v1    A Vector [(x, y, z)]
///  @param[in]    v2    A vector [(x, y, z)]
procedure dtVcross(dest: PSingle; v1, v2: PSingle);
begin
  dest[0] := v1[1]*v2[2] - v1[2]*v2[1];
  dest[1] := v1[2]*v2[0] - v1[0]*v2[2];
  dest[2] := v1[0]*v2[1] - v1[1]*v2[0];
end;

/// Derives the dot product of two vectors. (@p v1 . @p v2)
///  @param[in]    v1  A Vector [(x, y, z)]
///  @param[in]    v2  A vector [(x, y, z)]
/// @return The dot product.
function dtVdot(v1, v2: PSingle): Single;
begin
  Result := v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
end;

/// Performs a scaled vector addition. (@p v1 + (@p v2 * @p s))
///  @param[out]  dest  The result vector. [(x, y, z)]
///  @param[in]    v1    The base vector. [(x, y, z)]
///  @param[in]    v2    The vector to scale and add to @p v1. [(x, y, z)]
///  @param[in]    s    The amount to scale @p v2 by before adding to @p v1.
procedure dtVmad(dest: PSingle; v1, v2: PSingle; s: Single);
begin
  dest[0] := v1[0]+v2[0]*s;
  dest[1] := v1[1]+v2[1]*s;
  dest[2] := v1[2]+v2[2]*s;
end;

/// Performs a linear interpolation between two vectors. (@p v1 toward @p v2)
///  @param[out]  dest  The result vector. [(x, y, x)]
///  @param[in]    v1    The starting vector.
///  @param[in]    v2    The destination vector.
///   @param[in]    t    The interpolation factor. [Limits: 0 <= value <= 1.0]
procedure dtVlerp(dest: PSingle; v1, v2: PSingle; t: Single);
begin
  dest[0] := v1[0]+(v2[0]-v1[0])*t;
  dest[1] := v1[1]+(v2[1]-v1[1])*t;
  dest[2] := v1[2]+(v2[2]-v1[2])*t;
end;

/// Performs a vector addition. (@p v1 + @p v2)
///  @param[out]  dest  The result vector. [(x, y, z)]
///  @param[in]    v1    The base vector. [(x, y, z)]
///  @param[in]    v2    The vector to add to @p v1. [(x, y, z)]
procedure dtVadd(dest: PSingle; v1, v2: PSingle);
begin
  dest[0] := v1[0]+v2[0];
  dest[1] := v1[1]+v2[1];
  dest[2] := v1[2]+v2[2];
end;

/// Performs a vector subtraction. (@p v1 - @p v2)
///  @param[out]  dest  The result vector. [(x, y, z)]
///  @param[in]    v1    The base vector. [(x, y, z)]
///  @param[in]    v2    The vector to subtract from @p v1. [(x, y, z)]
procedure dtVsub(dest: PSingle; v1, v2: PSingle);
begin
  dest[0] := v1[0]-v2[0];
  dest[1] := v1[1]-v2[1];
  dest[2] := v1[2]-v2[2];
end;

/// Selects the minimum value of each element from the specified vectors.
///  @param[in,out]  mn  A vector.  (Will be updated with the result.) [(x, y, z)]
///  @param[in]    v  A vector. [(x, y, z)]
procedure dtVmin(mn: PSingle; v: PSingle);
begin
  mn[0] := dtMin(mn[0], v[0]);
  mn[1] := dtMin(mn[1], v[1]);
  mn[2] := dtMin(mn[2], v[2]);
end;

/// Selects the maximum value of each element from the specified vectors.
///  @param[in,out]  mx  A vector.  (Will be updated with the result.) [(x, y, z)]
///  @param[in]    v  A vector. [(x, y, z)]
procedure dtVmax(mx: PSingle; v: PSingle);
begin
  mx[0] := dtMax(mx[0], v[0]);
  mx[1] := dtMax(mx[1], v[1]);
  mx[2] := dtMax(mx[2], v[2]);
end;

/// Scales the vector by the specified value. (@p v * @p t)
///  @param[out]  dest  The result vector. [(x, y, z)]
///  @param[in]    v    The vector to scale. [(x, y, z)]
///  @param[in]    t    The scaling factor.
procedure dtVscale(dest: PSingle; v: PSingle; t: Single);
begin
  dest[0] := v[0]*t;
  dest[1] := v[1]*t;
  dest[2] := v[2]*t;
end;

/// Sets the vector elements to the specified values.
///  @param[out]  dest  The result vector. [(x, y, z)]
///  @param[in]    x    The x-value of the vector.
///  @param[in]    y    The y-value of the vector.
///  @param[in]    z    The z-value of the vector.
procedure dtVset(dest: PSingle; x,y,z: Single);
begin
  dest[0] := x; dest[1] := y; dest[2] := z;
end;

/// Performs a vector copy.
///  @param[out]  dest  The result. [(x, y, z)]
///  @param[in]    a    The vector to copy. [(x, y, z)]
procedure dtVcopy(dest: PSingle; a: PSingle);
begin
  dest[0] := a[0];
  dest[1] := a[1];
  dest[2] := a[2];
end;

/// Derives the scalar length of the vector.
///  @param[in]    v The vector. [(x, y, z)]
/// @return The scalar length of the vector.
function dtVlen(v: PSingle): Single;
begin
  Result := Sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
end;

/// Derives the square of the scalar length of the vector. (len * len)
///  @param[in]    v The vector. [(x, y, z)]
/// @return The square of the scalar length of the vector.
function dtVlenSqr(v: PSingle): Single;
begin
  Result := v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
end;

/// Returns the distance between two points.
///  @param[in]    v1  A point. [(x, y, z)]
///  @param[in]    v2  A point. [(x, y, z)]
/// @return The distance between the two points.
function dtVdist(v1, v2: PSingle): Single;
var dx,dy,dz: Single;
begin
  dx := v2[0] - v1[0];
  dy := v2[1] - v1[1];
  dz := v2[2] - v1[2];
  Result := Sqrt(dx*dx + dy*dy + dz*dz);
end;

/// Returns the square of the distance between two points.
///  @param[in]    v1  A point. [(x, y, z)]
///  @param[in]    v2  A point. [(x, y, z)]
/// @return The square of the distance between the two points.
function dtVdistSqr(v1, v2: PSingle): Single;
var dx,dy,dz: Single;
begin
  dx := v2[0] - v1[0];
  dy := v2[1] - v1[1];
  dz := v2[2] - v1[2];
  Result := dx*dx + dy*dy + dz*dz;
end;

/// Derives the distance between the specified points on the xz-plane.
///  @param[in]    v1  A point. [(x, y, z)]
///  @param[in]    v2  A point. [(x, y, z)]
/// @return The distance between the point on the xz-plane.
///
/// The vectors are projected onto the xz-plane, so the y-values are ignored.
function dtVdist2D(v1, v2: PSingle): Single;
var dx,dz: Single;
begin
  dx := v2[0] - v1[0];
  dz := v2[2] - v1[2];
  Result := Sqrt(dx*dx + dz*dz);
end;

/// Derives the square of the distance between the specified points on the xz-plane.
///  @param[in]    v1  A point. [(x, y, z)]
///  @param[in]    v2  A point. [(x, y, z)]
/// @return The square of the distance between the point on the xz-plane.
function dtVdist2DSqr(v1, v2: PSingle): Single;
var dx,dz: Single;
begin
  dx := v2[0] - v1[0];
  dz := v2[2] - v1[2];
  Result := dx*dx + dz*dz;
end;

/// Normalizes the vector.
///  @param[in,out]  v  The vector to normalize. [(x, y, z)]
procedure dtVnormalize(v: PSingle);
var d: Single;
begin
  d := 1.0 / Sqrt(Sqr(v[0]) + Sqr(v[1]) + Sqr(v[2]));
  v[0] := v[0] * d;
  v[1] := v[1] * d;
  v[2] := v[2] * d;
end;

/// Performs a 'sloppy' colocation check of the specified points.
///  @param[in]    p0  A point. [(x, y, z)]
///  @param[in]    p1  A point. [(x, y, z)]
/// @return True if the points are considered to be at the same location.
///
/// Basically, this function will return true if the specified points are
/// close enough to eachother to be considered colocated.
function dtVequal(p0, p1: PSingle): Boolean;
var thr,d: Single;
begin
  thr := dtSqr(1.0/16384.0);
  d := dtVdistSqr(p0, p1);
  Result := d < thr;
end;

/// Derives the dot product of two vectors on the xz-plane. (@p u . @p v)
///  @param[in]    u    A vector [(x, y, z)]
///  @param[in]    v    A vector [(x, y, z)]
/// @return The dot product on the xz-plane.
///
/// The vectors are projected onto the xz-plane, so the y-values are ignored.
function dtVdot2D(u, v: PSingle): Single;
begin
  Result := u[0]*v[0] + u[2]*v[2];
end;

/// Derives the xz-plane 2D perp product of the two vectors. (uz*vx - ux*vz)
///  @param[in]    u    The LHV vector [(x, y, z)]
///  @param[in]    v    The RHV vector [(x, y, z)]
/// @return The dot product on the xz-plane.
///
/// The vectors are projected onto the xz-plane, so the y-values are ignored.
function dtVperp2D(u, v: PSingle): Single;
begin
  Result := u[2]*v[0] - u[0]*v[2];
end;

/// @}
/// @name Computational geometry helper functions.
/// @{

/// Derives the signed xz-plane area of the triangle ABC, or the relationship of line AB to point C.
///  @param[in]    a    Vertex A. [(x, y, z)]
///  @param[in]    b    Vertex B. [(x, y, z)]
///  @param[in]    c    Vertex C. [(x, y, z)]
/// @return The signed xz-plane area of the triangle.
function dtTriArea2D(a, b, c: PSingle): Single;
var abx, abz, acx, acz: Single;
begin
  abx := b[0] - a[0];
  abz := b[2] - a[2];
  acx := c[0] - a[0];
  acz := c[2] - a[2];
  Result := acx*abz - abx*acz;
end;

/// Determines if two axis-aligned bounding boxes overlap.
///  @param[in]    amin  Minimum bounds of box A. [(x, y, z)]
///  @param[in]    amax  Maximum bounds of box A. [(x, y, z)]
///  @param[in]    bmin  Minimum bounds of box B. [(x, y, z)]
///  @param[in]    bmax  Maximum bounds of box B. [(x, y, z)]
/// @return True if the two AABB's overlap.
/// @see dtOverlapBounds
function dtOverlapQuantBounds(amin, amax, bmin, bmax: PWord): Boolean;
var overlap: Boolean;
begin
  overlap := true;
  overlap := overlap and not ((amin[0] > bmax[0]) or (amax[0] < bmin[0]));
  overlap := overlap and not ((amin[1] > bmax[1]) or (amax[1] < bmin[1]));
  overlap := overlap and not ((amin[2] > bmax[2]) or (amax[2] < bmin[2]));
  Result := overlap;
end;

/// Determines if two axis-aligned bounding boxes overlap.
///  @param[in]    amin  Minimum bounds of box A. [(x, y, z)]
///  @param[in]    amax  Maximum bounds of box A. [(x, y, z)]
///  @param[in]    bmin  Minimum bounds of box B. [(x, y, z)]
///  @param[in]    bmax  Maximum bounds of box B. [(x, y, z)]
/// @return True if the two AABB's overlap.
/// @see dtOverlapQuantBounds
function dtOverlapBounds(amin, amax, bmin, bmax: PSingle): Boolean;
var overlap: Boolean;
begin
  overlap := true;
  overlap := overlap and not ((amin[0] > bmax[0]) or (amax[0] < bmin[0]));
  overlap := overlap and not ((amin[1] > bmax[1]) or (amax[1] < bmin[1]));
  overlap := overlap and not ((amin[2] > bmax[2]) or (amax[2] < bmin[2]));
  Result := overlap;
end;

/// Derives the closest point on a triangle from the specified reference point.
///  @param[out]  closest  The closest point on the triangle.
///  @param[in]    p    The reference point from which to test. [(x, y, z)]
///  @param[in]    a    Vertex A of triangle ABC. [(x, y, z)]
///  @param[in]    b    Vertex B of triangle ABC. [(x, y, z)]
///  @param[in]    c    Vertex C of triangle ABC. [(x, y, z)]
//procedure dtClosestPtPointTriangle(closest, p, a, b, c: PSingle);

/// Derives the y-axis height of the closest point on the triangle from the specified reference point.
///  @param[in]    p    The reference point from which to test. [(x, y, z)]
///  @param[in]    a    Vertex A of triangle ABC. [(x, y, z)]
///  @param[in]    b    Vertex B of triangle ABC. [(x, y, z)]
///  @param[in]    c    Vertex C of triangle ABC. [(x, y, z)]
///  @param[out]  h    The resulting height.
//function dtClosestHeightPointTriangle(p, a, b, c: PSingle; h: PSingle): Boolean;

//function dtIntersectSegmentPoly2D(p0, p1: PSingle; verts: PSingle; nverts: Integer; tmin, tmax: PSingle; segMin, segMax: PInteger): Boolean;

//function dtIntersectSegSeg2D(ap, aq, bp, bq: PSingle; s, t: PSingle): Boolean;

/// Determines if the specified point is inside the convex polygon on the xz-plane.
///  @param[in]    pt    The point to check. [(x, y, z)]
///  @param[in]    verts  The polygon vertices. [(x, y, z) * @p nverts]
///  @param[in]    nverts  The number of vertices. [Limit: >= 3]
/// @return True if the point is inside the polygon.
//function dtPointInPolygon(pt, verts: PSingle; nverts: Integer): Boolean;

//function dtDistancePtPolyEdgesSqr(pt, verts: PSingle; nverts: Integer; ed, et: PSingle): Boolean;

//function dtDistancePtSegSqr2D(pt, p, q: PSingle; t: PSingle): Single;

/// Derives the centroid of a convex polygon.
///  @param[out]  tc    The centroid of the polgyon. [(x, y, z)]
///  @param[in]    idx    The polygon indices. [(vertIndex) * @p nidx]
///  @param[in]    nidx  The number of indices in the polygon. [Limit: >= 3]
///  @param[in]    verts  The polygon vertices. [(x, y, z) * vertCount]
//procedure dtCalcPolyCenter(tc: PSingle; const idx: PWord; nidx: Integer; verts: PSingle);

/// Determines if the two convex polygons overlap on the xz-plane.
///  @param[in]    polya    Polygon A vertices.  [(x, y, z) * @p npolya]
///  @param[in]    npolya    The number of vertices in polygon A.
///  @param[in]    polyb    Polygon B vertices.  [(x, y, z) * @p npolyb]
///  @param[in]    npolyb    The number of vertices in polygon B.
/// @return True if the two polygons overlap.
//function dtOverlapPolyPoly2D(polya: PSingle; npolya: Integer; polyb: PSingle; npolyb: Integer): Boolean;

/// @}
/// @name Miscellanious functions.
/// @{

function dtNextPow2(v: Cardinal): Cardinal;
begin
  if v > 0 then
  begin
    Dec(v);
    v := v or (v shr 1);
    v := v or (v shr 2);
    v := v or (v shr 4);
    v := v or (v shr 8);
    v := v or (v shr 16);
    Inc(v);
  end;
  Result := v;
end;

{
  unsigned int r;
  unsigned int shift;
  r = (v > 0xffff) << 4; v >>= r;
  shift = (v > 0xff) << 3; v >>= shift; r |= shift;
  shift = (v > 0xf) << 2; v >>= shift; r |= shift;
  shift = (v > 0x3) << 1; v >>= shift; r |= shift;
  r |= (v >> 1);
  return r;
}
function dtIlog2(v: Cardinal): Cardinal;
var r, shift: Cardinal;
begin
  r := Byte(v > $ffff) shl 4; v := v shr r;
  shift := Byte(v > $ff) shl 3; v := v shr shift; r := r or shift;
  shift := Byte(v > $f) shl 2; v := v shr shift; r := r or shift;
  shift := Byte(v > $3) shl 1; v := v shr shift; r := r or shift;
  r := r or Byte(v shr 1);
  Result := r;
end;

function dtAlign4(x: Integer): Integer; begin Result := (x+3) and not 3; end;

function dtOppositeTile(side: Integer): Integer; begin Result := (side+4) and $7; end;

procedure dtSwapByte(a, b: PByte);
var tmp: Byte;
begin
  tmp := a^;
  a^ := b^;
  b^ := tmp;
end;

procedure dtSwapEndian(v: PWord);
var x: PByte;
begin
  x := PByte(v);
  dtSwapByte(x+0, x+1);
end;

procedure dtSwapEndian(v: PShortInt);
var x: PByte;
begin
  x := PByte(v);
  dtSwapByte(x+0, x+1);
end;

procedure dtSwapEndian(v: PCardinal);
var x: PByte;
begin
  x := PByte(v);
  dtSwapByte(x+0, x+3); dtSwapByte(x+1, x+2);
end;

procedure dtSwapEndian(v: PInteger);
var x: PByte;
begin
  x := PByte(v);
  dtSwapByte(x+0, x+3); dtSwapByte(x+1, x+2);
end;

procedure dtSwapEndian(v: PSingle);
var x: PByte;
begin
  x := PByte(v);
  dtSwapByte(x+0, x+3); dtSwapByte(x+1, x+2);
end;

/// @}

//#endif // DETOURCOMMON_H

///////////////////////////////////////////////////////////////////////////

// This section contains detailed documentation for members that don't have
// a source file. It reduces clutter in the main section of the header.

(**

@fn float dtTriArea2D(const float* a, const float* b, const float* c)
@par

The vertices are projected onto the xz-plane, so the y-values are ignored.

This is a low cost function than can be used for various purposes.  Its main purpose
is for point/line relationship testing.

In all cases: A value of zero indicates that all vertices are collinear or represent the same point.
(On the xz-plane.)

When used for point/line relationship tests, AB usually represents a line against which
the C point is to be tested.  In this case:

A positive value indicates that point C is to the left of line AB, looking from A toward B.<br/>
A negative value indicates that point C is to the right of lineAB, looking from A toward B.

When used for evaluating a triangle:

The absolute value of the return value is two times the area of the triangle when it is
projected onto the xz-plane.

A positive return value indicates:

<ul>
<li>The vertices are wrapped in the normal Detour wrap direction.</li>
<li>The triangle's 3D face normal is in the general up direction.</li>
</ul>

A negative return value indicates:

<ul>
<li>The vertices are reverse wrapped. (Wrapped opposite the normal Detour wrap direction.)</li>
<li>The triangle's 3D face normal is in the general down direction.</li>
</ul>

*)

//////////////////////////////////////////////////////////////////////////////////////////

procedure dtClosestPtPointTriangle(closest, p, a, b, c: PSingle);
var ab,ac,ap,bp,cp: array [0..2] of Single; d1,d2,d3,d4,vc,v,d5,d6,vb,w,va,denom: Single;
begin
  // Check if P in vertex region outside A
  dtVsub(@ab, b, a);
  dtVsub(@ac, c, a);
  dtVsub(@ap, p, a);
  d1 := dtVdot(@ab, @ap);
  d2 := dtVdot(@ac, @ap);
  if (d1 <= 0.0) and (d2 <= 0.0) then
  begin
    // barycentric coordinates (1,0,0)
    dtVcopy(closest, a);
    Exit;
  end;

  // Check if P in vertex region outside B
  dtVsub(@bp, p, b);
  d3 := dtVdot(@ab, @bp);
  d4 := dtVdot(@ac, @bp);
  if (d3 >= 0.0) and (d4 <= d3) then
  begin
    // barycentric coordinates (0,1,0)
    dtVcopy(closest, b);
    Exit;
  end;

  // Check if P in edge region of AB, if so return projection of P onto AB
  vc := d1*d4 - d3*d2;
  if (vc <= 0.0) and (d1 >= 0.0) and (d3 <= 0.0) then
  begin
    // barycentric coordinates (1-v,v,0)
    v := d1 / (d1 - d3);
    closest[0] := a[0] + v * ab[0];
    closest[1] := a[1] + v * ab[1];
    closest[2] := a[2] + v * ab[2];
    Exit;
  end;

  // Check if P in vertex region outside C
  dtVsub(@cp, p, c);
  d5 := dtVdot(@ab, @cp);
  d6 := dtVdot(@ac, @cp);
  if (d6 >= 0.0) and (d5 <= d6) then
  begin
    // barycentric coordinates (0,0,1)
    dtVcopy(closest, c);
    Exit;
  end;

  // Check if P in edge region of AC, if so return projection of P onto AC
  vb := d5*d2 - d1*d6;
  if (vb <= 0.0) and (d2 >= 0.0) and (d6 <= 0.0) then
  begin
    // barycentric coordinates (1-w,0,w)
    w := d2 / (d2 - d6);
    closest[0] := a[0] + w * ac[0];
    closest[1] := a[1] + w * ac[1];
    closest[2] := a[2] + w * ac[2];
    Exit;
  end;

  // Check if P in edge region of BC, if so return projection of P onto BC
  va := d3*d6 - d5*d4;
  if (va <= 0.0) and ((d4 - d3) >= 0.0) and ((d5 - d6) >= 0.0) then
  begin
    // barycentric coordinates (0,1-w,w)
    w := (d4 - d3) / ((d4 - d3) + (d5 - d6));
    closest[0] := b[0] + w * (c[0] - b[0]);
    closest[1] := b[1] + w * (c[1] - b[1]);
    closest[2] := b[2] + w * (c[2] - b[2]);
    Exit;
  end;

  // P inside face region. Compute Q through its barycentric coordinates (u,v,w)
  denom := 1.0 / (va + vb + vc);
  v := vb * denom;
  w := vc * denom;
  closest[0] := a[0] + ab[0] * v + ac[0] * w;
  closest[1] := a[1] + ab[1] * v + ac[1] * w;
  closest[2] := a[2] + ab[2] * v + ac[2] * w;
end;

function dtIntersectSegmentPoly2D(p0, p1: PSingle; verts: PSingle; nverts: Integer; tmin, tmax: PSingle; segMin, segMax: PInteger): Boolean;
const EPS = 0.00000001;
var dir,edge,diff: array [0..2] of Single; i,j: Integer; n,d,t: Single;
begin
  tmin^ := 0;
  tmax^ := 1;
  segMin^ := -1;
  segMax^ := -1;

  dtVsub(@dir, p1, p0);

  i := 0; j := nverts-1;
  while (i < nverts) do
  begin
    dtVsub(@edge, @verts[i*3], @verts[j*3]);
    dtVsub(@diff, p0, @verts[j*3]);
    n := dtVperp2D(@edge, @diff);
    d := dtVperp2D(@dir, @edge);
    if (Abs(d) < EPS) then
    begin
      // S is nearly parallel to this edge
      if (n < 0) then
        Exit(false)
      else
        begin j:=i; Inc(i); continue; end;
    end;
    t := n / d;
    if (d < 0) then
    begin
      // segment S is entering across this edge
      if (t > tmin^) then
      begin
        tmin^ := t;
        segMin^ := j;
        // S enters after leaving polygon
        if (tmin^ > tmax^) then
          Exit(false);
      end;
    end
    else
    begin
      // segment S is leaving across this edge
      if (t < tmax^) then
      begin
        tmax^ := t;
        segMax^ := j;
        // S leaves before entering polygon
        if (tmax^ < tmin^) then
          Exit(false);
      end;
    end;

    j := i;
    Inc(i);
  end;

  Result := true;
end;

function dtDistancePtSegSqr2D(pt, p, q: PSingle; t: PSingle): Single;
var pqx,pqz,dx,dz,d: Single;
begin
  pqx := q[0] - p[0];
  pqz := q[2] - p[2];
  dx := pt[0] - p[0];
  dz := pt[2] - p[2];
  d := pqx*pqx + pqz*pqz;
  t^ := pqx*dx + pqz*dz;
  if (d > 0) then t^ := t^ / d;
  if (t^ < 0) then t^ := 0
  else if (t^ > 1) then t^ := 1;
  dx := p[0] + t^*pqx - pt[0];
  dz := p[2] + t^*pqz - pt[2];
  Result := dx*dx + dz*dz;
end;

procedure dtCalcPolyCenter(tc: PSingle; const idx: PWord; nidx: Integer; verts: PSingle);
var j: Integer; v: PSingle; s: Single;
begin
  tc[0] := 0.0;
  tc[1] := 0.0;
  tc[2] := 0.0;
  for j := 0 to nidx - 1 do
  begin
    v := @verts[idx[j]*3];
    tc[0] := tc[0] + v[0];
    tc[1] := tc[1] + v[1];
    tc[2] := tc[2] + v[2];
  end;
  s := 1.0 / nidx;
  tc[0] := tc[0] * s;
  tc[1] := tc[1] * s;
  tc[2] := tc[2] * s;
end;

function dtClosestHeightPointTriangle(p, a, b, c: PSingle; h: PSingle): Boolean;
const EPS = 0.0001;
var v0,v1,v2: array [0..2] of Single; dot00,dot01,dot02,dot11,dot12: Single; invDenom,u,v: Single;
begin
  dtVsub(@v0, c,a);
  dtVsub(@v1, b,a);
  dtVsub(@v2, p,a);

  dot00 := dtVdot2D(@v0, @v0);
  dot01 := dtVdot2D(@v0, @v1);
  dot02 := dtVdot2D(@v0, @v2);
  dot11 := dtVdot2D(@v1, @v1);
  dot12 := dtVdot2D(@v1, @v2);

  // Compute barycentric coordinates
  invDenom := 1.0 / (dot00 * dot11 - dot01 * dot01);
  u := (dot11 * dot02 - dot01 * dot12) * invDenom;
  v := (dot00 * dot12 - dot01 * dot02) * invDenom;

  // The (sloppy) epsilon is needed to allow to get height of points which
  // are interpolated along the edges of the triangles.
  //static const float EPS := 1e-4f;

  // If point lies inside the triangle, return interpolated ycoord.
  if (u >= -EPS) and (v >= -EPS) and ((u+v) <= 1+EPS) then
  begin
    h^ := a[1] + v0[1]*u + v1[1]*v;
    Exit(true);
  end;

  Result := false;
end;

/// @par
///
/// All points are projected onto the xz-plane, so the y-values are ignored.
function dtPointInPolygon(pt, verts: PSingle; nverts: Integer): Boolean;
var i,j: Integer; c: Boolean; vi,vj: PSingle;
begin
  // TODO: Replace pnpoly with triArea2D tests?
  c := false;
  i := 0; j := nverts-1;
  while (i < nverts) do
  begin
    vi := @verts[i*3];
    vj := @verts[j*3];
    if (((vi[2] > pt[2]) <> (vj[2] > pt[2])) and
      (pt[0] < (vj[0]-vi[0]) * (pt[2]-vi[2]) / (vj[2]-vi[2]) + vi[0]) ) then
      c := not c;

    j := i;
    Inc(i);
  end;
  Result := c;
end;

function dtDistancePtPolyEdgesSqr(pt, verts: PSingle; nverts: Integer; ed, et: PSingle): Boolean;
var i,j: Integer; c: Boolean; vi,vj: PSingle;
begin
  // TODO: Replace pnpoly with triArea2D tests?
  c := false;
  i := 0; j := nverts-1;
  while (i < nverts) do
  begin
    vi := @verts[i*3];
    vj := @verts[j*3];
    if (((vi[2] > pt[2]) <> (vj[2] > pt[2])) and
      (pt[0] < (vj[0]-vi[0]) * (pt[2]-vi[2]) / (vj[2]-vi[2]) + vi[0]) ) then
      c := not c;
    ed[j] := dtDistancePtSegSqr2D(pt, vj, vi, @et[j]);

    j := i;
    Inc(i);
  end;
  Result := c;
end;

procedure projectPoly(axis, poly: PSingle; npoly: Integer; rmin, rmax: PSingle);
var i: Integer; d: Single;
begin
  rmin^ := dtVdot2D(axis, @poly[0]);
  rmax^ := dtVdot2D(axis, @poly[0]);
  for i := 1 to npoly - 1 do
  begin
    d := dtVdot2D(axis, @poly[i*3]);
    rmin^ := dtMin(rmin^, d);
    rmax^ := dtMax(rmax^, d);
  end;
end;

function overlapRange(amin, amax, bmin, bmax, eps: Single): Boolean;
begin
  Result := not (((amin+eps) > bmax) or ((amax-eps) < bmin));
end;

/// @par
///
/// All vertices are projected onto the xz-plane, so the y-values are ignored.
function dtOverlapPolyPoly2D(polya: PSingle; npolya: Integer; polyb: PSingle; npolyb: Integer): Boolean;
const EPS = 0.0001;
var i,j: Integer; va,vb: PSingle; n: array [0..2] of Single; amin,amax,bmin,bmax: Single;
begin
  i := 0; j := npolya-1;
  while i < npolya do
  begin
    va := @polya[j*3];
    vb := @polya[i*3];
    n[0] := vb[2]-va[2]; n[1] := 0; n[2] := -(vb[0]-va[0]);

    projectPoly(@n, polya, npolya, @amin, @amax);
    projectPoly(@n, polyb, npolyb, @bmin, @bmax);
    if (not overlapRange(amin, amax, bmin, bmax, eps)) then
    begin
      // Found separating axis
      Exit(false);
    end;

    j := i;
    Inc(i);
  end;

  i := 0; j := npolyb-1;
  while i < npolyb do
  begin
    va := @polyb[j*3];
    vb := @polyb[i*3];
    n[0] := vb[2]-va[2]; n[1] := 0; n[2] := -(vb[0]-va[0]);

    projectPoly(@n, polya, npolya, @amin, @amax);
    projectPoly(@n, polyb, npolyb, @bmin, @bmax);
    if (not overlapRange(amin, amax, bmin, bmax, eps)) then
    begin
      // Found separating axis
      Exit(false);
    end;

    j := i;
    Inc(i);
  end;

  Result := true;
end;

// Returns a random point in a convex polygon.
// Adapted from Graphics Gems article.
procedure dtRandomPointInConvexPoly(pts: PSingle; npts: Integer; areas: PSingle; s, t: Single; &out: PSingle);
var areasum,thr,acc,u,dacc,v,a,b,c: Single; i,tri: Integer; pa,pb,pc: PSingle;
begin
  // Calc triangle araes
  areasum := 0.0;
  for i := 2 to npts - 1 do
  begin
    areas[i] := dtTriArea2D(@pts[0], @pts[(i-1)*3], @pts[i*3]);
    areasum := areasum + dtMax(0.001, areas[i]);
  end;
  // Find sub triangle weighted by area.
  thr := s*areasum;
  acc := 0.0;
  u := 0.0;
  tri := 0;
  for i := 2 to npts - 1 do
  begin
    dacc := areas[i];
    if (thr >= acc) and (thr < (acc+dacc)) then
    begin
      u := (thr - acc) / dacc;
      tri := i;
      break;
    end;
    acc := acc + dacc;
  end;

  v := Sqrt(t);

  a := 1 - v;
  b := (1 - u) * v;
  c := u * v;
  pa := @pts[0];
  pb := @pts[(tri-1)*3];
  pc := @pts[tri*3];

  &out[0] := a*pa[0] + b*pb[0] + c*pc[0];
  &out[1] := a*pa[1] + b*pb[1] + c*pc[1];
  &out[2] := a*pa[2] + b*pb[2] + c*pc[2];
end;

function vperpXZ(a, b: PSingle): Single; begin Result := a[0]*b[2] - a[2]*b[0]; end;

function dtIntersectSegSeg2D(ap, aq, bp, bq: PSingle; s, t: PSingle): Boolean;
var u,v,w: array [0..2] of Single; d: Single;
begin
  dtVsub(@u,aq,ap);
  dtVsub(@v,bq,bp);
  dtVsub(@w,ap,bp);
  d := vperpXZ(@u,@v);
  if (Abs(d) < 0.000001) then Exit(false);
  s^ := vperpXZ(@v,@w) / d;
  t^ := vperpXZ(@u,@w) / d;
  Result := true;
end;

end.
