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

unit RN_RecastHelper;
interface
uses
  RN_Helper, RN_Recast;

procedure rcSwap(var a,b: Single); overload;
procedure rcSwap(var a,b: Word); overload;
procedure rcSwap(var a,b: Integer); overload;
procedure rcSwap(var a,b: Pointer); overload;
procedure rcSwap(var a,b: PSingle); overload;
function rcMin(a,b: Single): Single; overload;
function rcMin(a,b: Integer): Integer; overload;
function rcMax(a,b: Single): Single; overload;
function rcMax(a,b: Integer): Integer; overload;
function rcMax(a,b: Cardinal): Cardinal; overload;
function rcClamp(v, mn, mx: Single): Single; overload;
function rcClamp(v, mn, mx: Integer): Integer; overload;
procedure rcVcross(dest: PSingle; v1, v2: PSingle);
function rcVdot(v1, v2: PSingle): Single;
procedure rcVmad(dest: PSingle; v1, v2: PSingle; s: Single);
procedure rcVadd(dest: PSingle; v1, v2: PSingle);
procedure rcVsub(dest: PSingle; v1, v2: PSingle);
procedure rcVmin(mn: PSingle; v: PSingle);
procedure rcVmax(mx: PSingle; v: PSingle);
procedure rcVcopy(dest: PSingle; v: PSingle);
function rcVdist(v1, v2: PSingle): Single;
function rcVdistSqr(v1, v2: PSingle): Single;
procedure rcVnormalize(v: PSingle);
procedure rcSetCon(s: PrcCompactSpan; dir, i: Integer);
function rcGetCon(s: PrcCompactSpan; dir: Integer): Integer;
function rcGetDirOffsetX(dir: Integer): Integer;
function rcGetDirOffsetY(dir: Integer): Integer;

implementation
uses
  Math;

/// Swaps the values of the two parameters.
///  @param[in,out]  a  Value A
///  @param[in,out]  b  Value B
procedure rcSwap(var a,b: Single);
var T: Single;
begin
  T := a; a := b; b := T;
end;

procedure rcSwap(var a,b: Word);
var T: Word;
begin
  T := a; a := b; b := T;
end;

procedure rcSwap(var a,b: Integer);
var T: Integer;
begin
  T := a; a := b; b := T;
end;

procedure rcSwap(var a,b: Pointer);
var T: Pointer;
begin
  T := a; a := b; b := T;
end;

procedure rcSwap(var a,b: PSingle);
var T: PSingle;
begin
  T := a; a := b; b := T;
end;

/// Returns the minimum of two values.
///  @param[in]    a  Value A
///  @param[in]    b  Value B
///  @return The minimum of the two values.
//template<class T> inline T rcMin(T a, T b) { return a < b ? a : b; }
function rcMin(a,b: Single): Single;
begin
  Result := Min(a,b);
end;

function rcMin(a,b: Integer): Integer;
begin
  Result := Min(a,b);
end;

/// Returns the maximum of two values.
///  @param[in]    a  Value A
///  @param[in]    b  Value B
///  @return The maximum of the two values.
//template<class T> inline T rcMax(T a, T b) { return a > b ? a : b; }
function rcMax(a,b: Single): Single;
begin
  Result := Max(a,b);
end;

function rcMax(a,b: Integer): Integer;
begin
  Result := Max(a,b);
end;

function rcMax(a,b: Cardinal): Cardinal;
begin
  Result := Max(a,b);
end;

/// Returns the absolute value.
///  @param[in]    a  The value.
///  @return The absolute value of the specified value.
//template<class T> inline T rcAbs(T a) { return a < 0 ? -a : a; }

/// Returns the square of the value.
///  @param[in]    a  The value.
///  @return The square of the value.
//template<class T> inline T rcSqr(T a) { return a*a; }

/// Clamps the value to the specified range.
///  @param[in]    v  The value to clamp.
///  @param[in]    mn  The minimum permitted return value.
///  @param[in]    mx  The maximum permitted return value.
///  @return The value, clamped to the specified range.
//template<class T> inline T rcClamp(T v, T mn, T mx) { return v < mn ? mn : (v > mx ? mx : v); }
function rcClamp(v, mn, mx: Single): Single;
begin
  Result := EnsureRange(v, mn, mx);
end;
function rcClamp(v, mn, mx: Integer): Integer;
begin
  Result := EnsureRange(v, mn, mx);
end;

/// Returns the square root of the value.
///  @param[in]    x  The value.
///  @return The square root of the vlaue.
//float rcSqrt(float x);

/// @}
/// @name Vector helper functions.
/// @{

/// Derives the cross product of two vectors. (@p v1 x @p v2)
///  @param[out]  dest  The cross product. [(x, y, z)]
///  @param[in]    v1    A Vector [(x, y, z)]
///  @param[in]    v2    A vector [(x, y, z)]
procedure rcVcross(dest: PSingle; v1, v2: PSingle);
begin
  dest[0] := v1[1]*v2[2] - v1[2]*v2[1];
  dest[1] := v1[2]*v2[0] - v1[0]*v2[2];
  dest[2] := v1[0]*v2[1] - v1[1]*v2[0];
end;

/// Derives the dot product of two vectors. (@p v1 . @p v2)
///  @param[in]    v1  A Vector [(x, y, z)]
///  @param[in]    v2  A vector [(x, y, z)]
/// @return The dot product.
function rcVdot(v1, v2: PSingle): Single;
begin
  Result := v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
end;

/// Performs a scaled vector addition. (@p v1 + (@p v2 * @p s))
///  @param[out]  dest  The result vector. [(x, y, z)]
///  @param[in]    v1    The base vector. [(x, y, z)]
///  @param[in]    v2    The vector to scale and add to @p v1. [(x, y, z)]
///  @param[in]    s    The amount to scale @p v2 by before adding to @p v1.
procedure rcVmad(dest: PSingle; v1, v2: PSingle; s: Single);
begin
  dest[0] := v1[0]+v2[0]*s;
  dest[1] := v1[1]+v2[1]*s;
  dest[2] := v1[2]+v2[2]*s;
end;

/// Performs a vector addition. (@p v1 + @p v2)
///  @param[out]  dest  The result vector. [(x, y, z)]
///  @param[in]    v1    The base vector. [(x, y, z)]
///  @param[in]    v2    The vector to add to @p v1. [(x, y, z)]
procedure rcVadd(dest: PSingle; v1, v2: PSingle);
begin
  dest[0] := v1[0]+v2[0];
  dest[1] := v1[1]+v2[1];
  dest[2] := v1[2]+v2[2];
end;

/// Performs a vector subtraction. (@p v1 - @p v2)
///  @param[out]  dest  The result vector. [(x, y, z)]
///  @param[in]    v1    The base vector. [(x, y, z)]
///  @param[in]    v2    The vector to subtract from @p v1. [(x, y, z)]
procedure rcVsub(dest: PSingle; v1, v2: PSingle);
begin
  dest[0] := v1[0]-v2[0];
  dest[1] := v1[1]-v2[1];
  dest[2] := v1[2]-v2[2];
end;

/// Selects the minimum value of each element from the specified vectors.
///  @param[in,out]  mn  A vector.  (Will be updated with the result.) [(x, y, z)]
///  @param[in]    v  A vector. [(x, y, z)]
procedure rcVmin(mn: PSingle; v: PSingle);
begin
  mn[0] := rcMin(mn[0], v[0]);
  mn[1] := rcMin(mn[1], v[1]);
  mn[2] := rcMin(mn[2], v[2]);
end;

/// Selects the maximum value of each element from the specified vectors.
///  @param[in,out]  mx  A vector.  (Will be updated with the result.) [(x, y, z)]
///  @param[in]    v  A vector. [(x, y, z)]
procedure rcVmax(mx: PSingle; v: PSingle);
begin
  mx[0] := rcMax(mx[0], v[0]);
  mx[1] := rcMax(mx[1], v[1]);
  mx[2] := rcMax(mx[2], v[2]);
end;

/// Performs a vector copy.
///  @param[out]  dest  The result. [(x, y, z)]
///  @param[in]    v    The vector to copy. [(x, y, z)]
procedure rcVcopy(dest: PSingle; v: PSingle);
begin
  dest[0] := v[0];
  dest[1] := v[1];
  dest[2] := v[2];
end;

/// Returns the distance between two points.
///  @param[in]    v1  A point. [(x, y, z)]
///  @param[in]    v2  A point. [(x, y, z)]
/// @return The distance between the two points.
function rcVdist(v1, v2: PSingle): Single;
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
function rcVdistSqr(v1, v2: PSingle): Single;
var dx,dy,dz: Single;
begin
  dx := v2[0] - v1[0];
  dy := v2[1] - v1[1];
  dz := v2[2] - v1[2];
  Result := dx*dx + dy*dy + dz*dz;
end;

/// Normalizes the vector.
///  @param[in,out]  v  The vector to normalize. [(x, y, z)]
procedure rcVnormalize(v: PSingle);
var d: Single;
begin
  d := 1.0 / Sqrt(Sqr(v[0]) + Sqr(v[1]) + Sqr(v[2]));
  v[0] := v[0] * d;
  v[1] := v[1] * d;
  v[2] := v[2] * d;
end;

/// Sets the neighbor connection data for the specified direction.
///  @param[in]    s    The span to update.
///  @param[in]    dir    The direction to set. [Limits: 0 <= value < 4]
///  @param[in]    i    The index of the neighbor span.
procedure rcSetCon(s: PrcCompactSpan; dir, i: Integer);
var shift: Cardinal; con: Integer;
begin
  shift := Cardinal(dir)*6;
  con := s.con;
  s.con := (con and not ($3f shl shift)) or ((Cardinal(i) and $3f) shl shift);
end;

/// Gets neighbor connection data for the specified direction.
///  @param[in]    s    The span to check.
///  @param[in]    dir    The direction to check. [Limits: 0 <= value < 4]
///  @return The neighbor connection data for the specified direction,
///    or #RC_NOT_CONNECTED if there is no connection.
function rcGetCon(s: PrcCompactSpan; dir: Integer): Integer;
var shift: Cardinal;
begin
  shift := Cardinal(dir)*6;
  Result := (s.con shr shift) and $3f;
end;

/// Gets the standard width (x-axis) offset for the specified direction.
///  @param[in]    dir    The direction. [Limits: 0 <= value < 4]
///  @return The width offset to apply to the current cell position to move
///    in the direction.
function rcGetDirOffsetX(dir: Integer): Integer;
const offset: array [0..3] of Integer = (-1, 0, 1, 0);
begin
  Result := offset[dir and $03];
end;

/// Gets the standard height (z-axis) offset for the specified direction.
///  @param[in]    dir    The direction. [Limits: 0 <= value < 4]
///  @return The height offset to apply to the current cell position to move
///    in the direction.
function rcGetDirOffsetY(dir: Integer): Integer;
const offset: array [0..3] of Integer = (0, 1, 0, -1);
begin
  Result := offset[dir and $03];
end;

end.
