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

unit RN_DebugDraw;
interface
uses Math, RN_Helper;

type
  TduDebugDrawPrimitives =
  (
    DU_DRAW_POINTS,
    DU_DRAW_LINES,
    DU_DRAW_TRIS,
    DU_DRAW_QUADS
  );

  /// Abstract debug draw interface.
  TduDebugDraw = class
  public
    procedure depthMask(state: Boolean); virtual; abstract;

    procedure texture(state: Boolean); virtual; abstract;

    /// Begin drawing primitives.
    ///  @param prim [in] primitive type to draw, one of rcDebugDrawPrimitives.
    ///  @param size [in] size of a primitive, applies to point size and line width only.
    procedure &begin(prim: TduDebugDrawPrimitives; size: Single = 1.0); virtual; abstract;

    /// Submit a vertex
    ///  @param pos [in] position of the verts.
    ///  @param color [in] color of the verts.
    procedure vertex(const pos: PSingle; color: Cardinal); overload; virtual; abstract;

    /// Submit a vertex
    ///  @param x,y,z [in] position of the verts.
    ///  @param color [in] color of the verts.
    procedure vertex(const x,y,z: Single; color: Cardinal); overload; virtual; abstract;

    /// Submit a vertex
    ///  @param pos [in] position of the verts.
    ///  @param color [in] color of the verts.
    procedure vertex(const pos: PSingle; color: Cardinal; uv: PSingle); overload; virtual; abstract;

    /// Submit a vertex
    ///  @param x,y,z [in] position of the verts.
    ///  @param color [in] color of the verts.
    procedure vertex(const x,y,z: Single; color: Cardinal; u,v: Single); overload; virtual; abstract;

    /// End drawing primitives.
    procedure &end; virtual; abstract;
  end;

function duIntToCol(i, a: Integer): Cardinal; overload;
procedure duIntToCol(i: Integer; col: PSingle); overload;

procedure duCalcBoxColors(var colors: array of Cardinal; colTop, colSide: Cardinal);

procedure duDebugDrawCylinderWire(dd: TduDebugDraw; minx, miny, minz: Single;
               maxx, maxy, maxz: Single; col: Cardinal; const lineWidth: Single);

procedure duDebugDrawBoxWire(dd: TduDebugDraw; minx, miny, minz: Single;
            maxx, maxy, maxz: Single; col: Cardinal; const lineWidth: Single);

procedure duDebugDrawArc(dd: TduDebugDraw; x0, y0, z0: Single;
          x1, y1, z1, h: Single;
          as0, as1: Single; col: Cardinal; const lineWidth: Single);

procedure duDebugDrawArrow(dd: TduDebugDraw; const x0, y0, z0, x1, y1, z1, as0, as1: Single; col: Cardinal; const lineWidth: Single);

procedure duDebugDrawCircle(dd: TduDebugDraw; x,y,z: Single;
             r: Single; col: Cardinal; const lineWidth: Single);

procedure duDebugDrawCross(dd: TduDebugDraw; x,y,z: Single;
            size: Single; col: Cardinal; const lineWidth: Single);

procedure duDebugDrawBox(dd: TduDebugDraw; minx, miny, minz: Single;
          maxx, maxy, maxz: Single; const fcol: PCardinal);

procedure duDebugDrawCylinder(dd: TduDebugDraw; minx, miny, minz: Single;
             maxx, maxy, maxz: Single; col: Cardinal);

{procedure duDebugDrawGridXZ(dd: TduDebugDraw; const ox, oy, oz: Single;
             const int w, const int h, const float size,
             const col: Cardinal; const lineWidth: Single);
}

// Versions without begin/end, can be used to draw multiple primitives.
procedure duAppendCylinderWire(dd: TduDebugDraw; minx, miny, minz: Single;
              maxx, maxy, maxz: Single; col: Cardinal);

procedure duAppendBoxWire(dd: TduDebugDraw; minx, miny, minz: Single;
           maxx, maxy, maxz: Single; col: Cardinal);

procedure duAppendBoxPoints(dd: TduDebugDraw; minx, miny, minz: Single;
             maxx, maxy, maxz: Single; col: Cardinal);


procedure duAppendArc(dd: TduDebugDraw; const x0, y0, z0, x1, y1, z1, h, as0, as1: Single; col: Cardinal);

procedure duAppendArrow(dd: TduDebugDraw; const x0, y0, z0, x1, y1, z1, as0, as1: Single; col: Cardinal);

procedure duAppendCircle(dd: TduDebugDraw; x,y,z,r: Single; col: Cardinal);

procedure duAppendCross(dd: TduDebugDraw; x,y,z: Single;
            s: Single; col: Cardinal);

procedure duAppendBox(dd: TduDebugDraw; minx, miny, minz: Single;
         maxx, maxy, maxz: Single; const col: PCardinal);

procedure duAppendCylinder(dd: TduDebugDraw; minx, miny, minz: Single;
            maxx, maxy, maxz: Single; col: Cardinal);

type
  TduDisplayList = class(TduDebugDraw)
  private
    m_pos: PSingle;
    m_color: PCardinal;
    m_size: Integer;
    m_cap: Integer;

    m_depthMask: Boolean;
    m_prim: TduDebugDrawPrimitives;
    m_primSize: Single;

    procedure resize(cap: Integer);
  public
    constructor Create(cap: Integer = 512);
    procedure depthMask(state: Boolean); override;
    procedure &begin(prim: TduDebugDrawPrimitives; size: Single = 1.0); override;
    procedure vertex(const x,y,z: Single; color: Cardinal); override;
    procedure vertex(const pos: PSingle; color: Cardinal); override;
    procedure &end(); override;
    procedure clear();
    procedure draw(dd: TduDebugDraw);
  end;

function duRGBA(r,g,b,a: Byte): Cardinal;
function duRGBAf(fr,fg,fb,fa: Single): Cardinal;
function duMultCol(const col: Cardinal; const d: Cardinal): Cardinal;
function duDarkenCol(col: Cardinal): Cardinal;
function duLerpCol(ca,cb,u: Cardinal): Cardinal;

implementation

function duRGBA(r,g,b,a: Byte): Cardinal;
begin
  Result := r or (g shl 8) or (b shl 16) or (a shl 24);
end;

function duRGBAf(fr,fg,fb,fa: Single): Cardinal;
var r,g,b,a: Byte;
begin
  r := Trunc(fr*255.0);
  g := Trunc(fg*255.0);
  b := Trunc(fb*255.0);
  a := Trunc(fa*255.0);
  Result := duRGBA(r,g,b,a);
end;

function duMultCol(const col: Cardinal; const d: Cardinal): Cardinal;
var r,g,b,a: Cardinal;
begin
  r := col and $ff;
  g := (col shr 8) and $ff;
  b := (col shr 16) and $ff;
  a := (col shr 24) and $ff;
  Result := duRGBA((r*d) shr 8, (g*d) shr 8, (b*d) shr 8, a);
end;

function duDarkenCol(col: Cardinal): Cardinal;
begin
  Result := ((col shr 1) and $007f7f7f) or (col and $ff000000);
end;

function duLerpCol(ca,cb,u: Cardinal): Cardinal;
var ra,ga,ba,aa,rb,gb,bb,ab,r,g,b,a: Cardinal;
begin
  ra := ca and $ff;
  ga := (ca shr 8) and $ff;
  ba := (ca shr 16) and $ff;
  aa := (ca shr 24) and $ff;
  rb := cb and $ff;
  gb := (cb shr 8) and $ff;
  bb := (cb shr 16) and $ff;
  ab := (cb shr 24) and $ff;

  r := (ra*(255-u) + rb*u) div 255;
  g := (ga*(255-u) + gb*u) div 255;
  b := (ba*(255-u) + bb*u) div 255;
  a := (aa*(255-u) + ab*u) div 255;
  Result := duRGBA(r,g,b,a);
end;

function duTransCol(c,a: Cardinal): Cardinal;
begin
  Result := (a shl 24) or (c and $00ffffff);
end;

function bit(a, b: Integer): Integer;
begin
  Result := (a and (1 shl b)) shr b;
end;

function duIntToCol(i,a: Integer): Cardinal;
var r,g,b: Integer;
begin
  r := bit(i, 1) + bit(i, 3) * 2 + 1;
  g := bit(i, 2) + bit(i, 4) * 2 + 1;
  b := bit(i, 0) + bit(i, 5) * 2 + 1;
  Result := duRGBA(r*63,g*63,b*63,a);
end;

procedure duIntToCol(i: Integer; col: PSingle);
var r,g,b: Integer;
begin
  r := bit(i, 0) + bit(i, 3) * 2 + 1;
  g := bit(i, 1) + bit(i, 4) * 2 + 1;
  b := bit(i, 2) + bit(i, 5) * 2 + 1;
  col[0] := 1 - r*63.0/255.0;
  col[1] := 1 - g*63.0/255.0;
  col[2] := 1 - b*63.0/255.0;
end;

procedure duCalcBoxColors(var colors: array of Cardinal; colTop, colSide: Cardinal);
begin
  //if (!colors) Result :=;

  colors[0] := duMultCol(colTop, 250);
  colors[1] := duMultCol(colSide, 140);
  colors[2] := duMultCol(colSide, 165);
  colors[3] := duMultCol(colSide, 217);
  colors[4] := duMultCol(colSide, 165);
  colors[5] := duMultCol(colSide, 217);
end;

procedure duDebugDrawCylinderWire(dd: TduDebugDraw; minx, miny, minz: Single;
               maxx, maxy, maxz: Single; col: Cardinal; const lineWidth: Single);
begin
  if (dd = nil) then Exit;

  dd.&begin(DU_DRAW_LINES, lineWidth);
  duAppendCylinderWire(dd, minx,miny,minz, maxx,maxy,maxz, col);
  dd.&end();
end;

procedure duDebugDrawBoxWire(dd: TduDebugDraw; minx, miny, minz: Single;
            maxx, maxy, maxz: Single; col: Cardinal; const lineWidth: Single);
begin
  if (dd = nil) then Exit;

  dd.&begin(DU_DRAW_LINES, lineWidth);
  duAppendBoxWire(dd, minx,miny,minz, maxx,maxy,maxz, col);
  dd.&end();
end;

procedure duDebugDrawArc(dd: TduDebugDraw; x0, y0, z0: Single;
          x1, y1, z1, h: Single;
          as0, as1: Single; col: Cardinal; const lineWidth: Single);
begin
  if (dd = nil) then Exit;

  dd.&begin(DU_DRAW_LINES, lineWidth);
  duAppendArc(dd, x0,y0,z0, x1,y1,z1, h, as0, as1, col);
  dd.&end();
end;

procedure duDebugDrawArrow(dd: TduDebugDraw; const x0, y0, z0, x1, y1, z1, as0, as1: Single; col: Cardinal; const lineWidth: Single);
begin
  if (dd = nil) then Exit;

  dd.&begin(DU_DRAW_LINES, lineWidth);
  duAppendArrow(dd, x0,y0,z0, x1,y1,z1, as0, as1, col);
  dd.&end();
end;

procedure duDebugDrawCircle(dd: TduDebugDraw; x,y,z: Single;
             r: Single; col: Cardinal; const lineWidth: Single);
begin
  if (dd = nil) then Exit;

  dd.&begin(DU_DRAW_LINES, lineWidth);
  duAppendCircle(dd, x,y,z, r, col);
  dd.&end();
end;

procedure duDebugDrawCross(dd: TduDebugDraw; x,y,z: Single;
            size: Single; col: Cardinal; const lineWidth: Single);
begin
  if (dd = nil) then Exit;

  dd.&begin(DU_DRAW_LINES, lineWidth);
  duAppendCross(dd, x,y,z, size, col);
  dd.&end();
end;

procedure duDebugDrawBox(dd: TduDebugDraw; minx, miny, minz: Single;
          maxx, maxy, maxz: Single; const fcol: PCardinal);
begin
  if (dd = nil) then Exit;

  dd.&begin(DU_DRAW_QUADS);
  duAppendBox(dd, minx,miny,minz, maxx,maxy,maxz, fcol);
  dd.&end();
end;

procedure duDebugDrawCylinder(dd: TduDebugDraw; minx, miny, minz: Single;
             maxx, maxy, maxz: Single; col: Cardinal);
begin
  if (dd = nil) then Exit;

  dd.&begin(DU_DRAW_TRIS);
  duAppendCylinder(dd, minx,miny,minz, maxx,maxy,maxz, col);
  dd.&end();
end;

{procedure duDebugDrawGridXZ(dd: TduDebugDraw; const float ox, const float oy, const float oz,
             const int w, const int h, const float size,
             const col: Cardinal; const lineWidth: Single)
begin
  if (dd = nil) then Exit;

  dd.&begin(DU_DRAW_LINES, lineWidth);
  for (int i := 0; i <= h; ++i)
  begin
    dd.vertex(ox,oy,oz+i*size, col);
    dd.vertex(ox+w*size,oy,oz+i*size, col);
  end;
  for (int i := 0; i <= w; ++i)
  begin
    dd.vertex(ox+i*size,oy,oz, col);
    dd.vertex(ox+i*size,oy,oz+h*size, col);
  end;
  dd.&end();
end;}


procedure duAppendCylinderWire(dd: TduDebugDraw; minx, miny, minz: Single;
              maxx, maxy, maxz: Single; col: Cardinal);
const NUM_SEG = 16;
var dir: array [0..NUM_SEG*2-1] of Single; init: Boolean; a,cx,cz,rx,rz: Single; i,j: Integer;
begin
  if (dd = nil) then Exit;

  init := false;
  if (not init) then
  begin
    init := true;
    for i := 0 to NUM_SEG - 1 do
    begin
      a := i/NUM_SEG*PI*2;
      dir[i*2] := Cos(a);
      dir[i*2+1] := Sin(a);
    end;
  end;

  cx := (maxx + minx)/2;
  cz := (maxz + minz)/2;
  rx := (maxx - minx)/2;
  rz := (maxz - minz)/2;

  i := 0; j := NUM_SEG-1;
  while (i < NUM_SEG) do
  begin
    dd.vertex(cx+dir[j*2+0]*rx, miny, cz+dir[j*2+1]*rz, col);
    dd.vertex(cx+dir[i*2+0]*rx, miny, cz+dir[i*2+1]*rz, col);
    dd.vertex(cx+dir[j*2+0]*rx, maxy, cz+dir[j*2+1]*rz, col);
    dd.vertex(cx+dir[i*2+0]*rx, maxy, cz+dir[i*2+1]*rz, col);
    j := i;
    Inc(i);
  end;
  i := 0;
  while (i < NUM_SEG) do
  begin
    dd.vertex(cx+dir[i*2+0]*rx, miny, cz+dir[i*2+1]*rz, col);
    dd.vertex(cx+dir[i*2+0]*rx, maxy, cz+dir[i*2+1]*rz, col);
    Inc(i, NUM_SEG div 4);
  end;
end;

procedure duAppendBoxWire(dd: TduDebugDraw; minx, miny, minz: Single;
           maxx, maxy, maxz: Single; col: Cardinal);
begin
  if (dd = nil) then Exit;
  // Top
  dd.vertex(minx, miny, minz, col);
  dd.vertex(maxx, miny, minz, col);
  dd.vertex(maxx, miny, minz, col);
  dd.vertex(maxx, miny, maxz, col);
  dd.vertex(maxx, miny, maxz, col);
  dd.vertex(minx, miny, maxz, col);
  dd.vertex(minx, miny, maxz, col);
  dd.vertex(minx, miny, minz, col);

  // bottom
  dd.vertex(minx, maxy, minz, col);
  dd.vertex(maxx, maxy, minz, col);
  dd.vertex(maxx, maxy, minz, col);
  dd.vertex(maxx, maxy, maxz, col);
  dd.vertex(maxx, maxy, maxz, col);
  dd.vertex(minx, maxy, maxz, col);
  dd.vertex(minx, maxy, maxz, col);
  dd.vertex(minx, maxy, minz, col);

  // Sides
  dd.vertex(minx, miny, minz, col);
  dd.vertex(minx, maxy, minz, col);
  dd.vertex(maxx, miny, minz, col);
  dd.vertex(maxx, maxy, minz, col);
  dd.vertex(maxx, miny, maxz, col);
  dd.vertex(maxx, maxy, maxz, col);
  dd.vertex(minx, miny, maxz, col);
  dd.vertex(minx, maxy, maxz, col);
end;

procedure duAppendBoxPoints(dd: TduDebugDraw; minx, miny, minz: Single;
             maxx, maxy, maxz: Single; col: Cardinal);
begin
  if (dd = nil) then Exit;
  // Top
  dd.vertex(minx, miny, minz, col);
  dd.vertex(maxx, miny, minz, col);
  dd.vertex(maxx, miny, minz, col);
  dd.vertex(maxx, miny, maxz, col);
  dd.vertex(maxx, miny, maxz, col);
  dd.vertex(minx, miny, maxz, col);
  dd.vertex(minx, miny, maxz, col);
  dd.vertex(minx, miny, minz, col);

  // bottom
  dd.vertex(minx, maxy, minz, col);
  dd.vertex(maxx, maxy, minz, col);
  dd.vertex(maxx, maxy, minz, col);
  dd.vertex(maxx, maxy, maxz, col);
  dd.vertex(maxx, maxy, maxz, col);
  dd.vertex(minx, maxy, maxz, col);
  dd.vertex(minx, maxy, maxz, col);
  dd.vertex(minx, maxy, minz, col);
end;

procedure duAppendBox(dd: TduDebugDraw; minx, miny, minz: Single;
         maxx, maxy, maxz: Single; const col: PCardinal);
const inds: array [0..6*4-1] of Byte =
  (
    7, 6, 5, 4,
    0, 1, 2, 3,
    1, 5, 6, 2,
    3, 7, 4, 0,
    2, 6, 7, 3,
    0, 4, 5, 1
  );
var verts: array [0..8*3-1] of Single; i: Integer;
begin
  if (dd = nil) then Exit;
  {
    minx, miny, minz,
    maxx, miny, minz,
    maxx, miny, maxz,
    minx, miny, maxz,
    minx, maxy, minz,
    maxx, maxy, minz,
    maxx, maxy, maxz,
    minx, maxy, maxz,
  };
  verts[0*3+0] := minx; verts[0*3+1] := miny; verts[0*3+2] := minz;
  verts[1*3+0] := maxx; verts[1*3+1] := miny; verts[1*3+2] := minz;
  verts[2*3+0] := maxx; verts[2*3+1] := miny; verts[2*3+2] := maxz;
  verts[3*3+0] := minx; verts[3*3+1] := miny; verts[3*3+2] := maxz;
  verts[4*3+0] := minx; verts[4*3+1] := maxy; verts[4*3+2] := minz;
  verts[5*3+0] := maxx; verts[5*3+1] := maxy; verts[5*3+2] := minz;
  verts[6*3+0] := maxx; verts[6*3+1] := maxy; verts[6*3+2] := maxz;
  verts[7*3+0] := minx; verts[7*3+1] := maxy; verts[7*3+2] := maxz;

  for i := 0 to 5 do
  begin
    dd.vertex(@verts[inds[i*4+0]*3], col[i]);
    dd.vertex(@verts[inds[i*4+1]*3], col[i]);
    dd.vertex(@verts[inds[i*4+2]*3], col[i]);
    dd.vertex(@verts[inds[i*4+3]*3], col[i]);
  end;
end;

procedure duAppendCylinder(dd: TduDebugDraw; minx, miny, minz: Single;
            maxx, maxy, maxz: Single; col: Cardinal);
const NUM_SEG = 16;
var dir: array [0..NUM_SEG*2-1] of Single; init: Boolean; i, j, a, b, c: Integer; ang, cx, cz, rx, rz: Single; col2: Cardinal;
begin
  if (dd = nil) then Exit;

  init := false;
  if (not init) then
  begin
    init := true;
    for i := 0 to NUM_SEG - 1 do
    begin
      ang := i/NUM_SEG*PI*2;
      dir[i*2] := cos(ang);
      dir[i*2+1] := sin(ang);
    end;
  end;

  col2 := duMultCol(col, 160);

  cx := (maxx + minx)/2;
  cz := (maxz + minz)/2;
  rx := (maxx - minx)/2;
  rz := (maxz - minz)/2;

  for i := 2 to NUM_SEG - 1 do
  begin
    a := 0; b := i-1; c := i;
    dd.vertex(cx+dir[a*2+0]*rx, miny, cz+dir[a*2+1]*rz, col2);
    dd.vertex(cx+dir[b*2+0]*rx, miny, cz+dir[b*2+1]*rz, col2);
    dd.vertex(cx+dir[c*2+0]*rx, miny, cz+dir[c*2+1]*rz, col2);
  end;
  for i := 2 to NUM_SEG - 1 do
  begin
    a := 0; b := i; c := i-1;
    dd.vertex(cx+dir[a*2+0]*rx, maxy, cz+dir[a*2+1]*rz, col);
    dd.vertex(cx+dir[b*2+0]*rx, maxy, cz+dir[b*2+1]*rz, col);
    dd.vertex(cx+dir[c*2+0]*rx, maxy, cz+dir[c*2+1]*rz, col);
  end;
  i := 0;
  j := NUM_SEG-1;
  while (i < NUM_SEG) do
  begin
    dd.vertex(cx+dir[i*2+0]*rx, miny, cz+dir[i*2+1]*rz, col2);
    dd.vertex(cx+dir[j*2+0]*rx, miny, cz+dir[j*2+1]*rz, col2);
    dd.vertex(cx+dir[j*2+0]*rx, maxy, cz+dir[j*2+1]*rz, col);

    dd.vertex(cx+dir[i*2+0]*rx, miny, cz+dir[i*2+1]*rz, col2);
    dd.vertex(cx+dir[j*2+0]*rx, maxy, cz+dir[j*2+1]*rz, col);
    dd.vertex(cx+dir[i*2+0]*rx, maxy, cz+dir[i*2+1]*rz, col);

    j := i;
    Inc(i);
  end;
end;


procedure evalArc(const x0, y0, z0, dx, dy, dz, h, u: Single; res: PSingle);
begin
  res[0] := x0 + dx * u;
  res[1] := y0 + dy * u + h * (1-(u*2-1)*(u*2-1));
  res[2] := z0 + dz * u;
end;


procedure vcross(dest: PSingle; const v1,v2: PSingle);
begin
  dest[0] := v1[1]*v2[2] - v1[2]*v2[1];
  dest[1] := v1[2]*v2[0] - v1[0]*v2[2];
  dest[2] := v1[0]*v2[1] - v1[1]*v2[0];
end;

procedure vnormalize(v: PSingle);
var d: Single;
begin
  d := 1.0 / sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
  v[0] := v[0] * d;
  v[1] := v[1] * d;
  v[2] := v[2] * d;
end;

procedure vsub(dest: PSingle; const v1,v2: PSingle);
begin
  dest[0] := v1[0]-v2[0];
  dest[1] := v1[1]-v2[1];
  dest[2] := v1[2]-v2[2];
end;

function vdistSqr(const v1,v2: PSingle): Single;
var x,y,z: Single;
begin
  x := v1[0]-v2[0];
  y := v1[1]-v2[1];
  z := v1[2]-v2[2];
  Result := x*x + y*y + z*z;
end;


procedure appendArrowHead(dd: TduDebugDraw; const p, q: PSingle;
           const s: Single; col: Cardinal);
const eps = 0.001;
var ax,ay,az: array [0..2] of Single;
begin
  //if (!dd) Result :=;
  if (vdistSqr(p,q) < eps*eps) then Exit;
  ay[0] := 0; ay[1] := 1; ay[2] := 0;
  vsub(@az[0], q, p);
  vnormalize(@az[0]);
  vcross(@ax[0], @ay[0], @az[0]);
  vcross(@ay[0], @az[0], @ax[0]);
  vnormalize(@ay[0]);

  dd.vertex(p, col);
//  dd.vertex(p[0]+az[0]*s+ay[0]*s/2, p[1]+az[1]*s+ay[1]*s/2, p[2]+az[2]*s+ay[2]*s/2, col);
  dd.vertex(p[0]+az[0]*s+ax[0]*s/3, p[1]+az[1]*s+ax[1]*s/3, p[2]+az[2]*s+ax[2]*s/3, col);

  dd.vertex(p, col);
//  dd.vertex(p[0]+az[0]*s-ay[0]*s/2, p[1]+az[1]*s-ay[1]*s/2, p[2]+az[2]*s-ay[2]*s/2, col);
  dd.vertex(p[0]+az[0]*s-ax[0]*s/3, p[1]+az[1]*s-ax[1]*s/3, p[2]+az[2]*s-ax[2]*s/3, col);

end;

procedure duAppendArc(dd: TduDebugDraw; const x0, y0, z0, x1, y1, z1, h, as0, as1: Single; col: Cardinal);
const NUM_ARC_PTS = 8;
const PAD = 0.05;
const ARC_PTS_SCALE = (1.0-PAD*2) / NUM_ARC_PTS;
var dx,dy,dz,len,u: Single; prev,pt,p,q: array [0..2] of Single; i: Integer;
begin
  //if (!dd) Result :=;

  dx := x1 - x0;
  dy := y1 - y0;
  dz := z1 - z0;
  len := sqrt(dx*dx + dy*dy + dz*dz);
  //float prev[3];
  evalArc(x0,y0,z0, dx,dy,dz, len*h, PAD, @prev[0]);
  for i := 1 to NUM_ARC_PTS do
  begin
    u := PAD + i * ARC_PTS_SCALE;
    //float pt[3];
    evalArc(x0,y0,z0, dx,dy,dz, len*h, u, @pt[0]);
    dd.vertex(prev[0],prev[1],prev[2], col);
    dd.vertex(pt[0],pt[1],pt[2], col);
    prev[0] := pt[0]; prev[1] := pt[1]; prev[2] := pt[2];
  end;

  // End arrows
  if (as0 > 0.001) then
  begin
    //float p[3], q[3];
    evalArc(x0,y0,z0, dx,dy,dz, len*h, PAD, @p[0]);
    evalArc(x0,y0,z0, dx,dy,dz, len*h, PAD+0.05, @q[0]);
    appendArrowHead(dd, @p[0], @q[0], as0, col);
  end;

  if (as1 > 0.001) then
  begin
    //float p[3], q[3];
    evalArc(x0,y0,z0, dx,dy,dz, len*h, 1-PAD, @p[0]);
    evalArc(x0,y0,z0, dx,dy,dz, len*h, 1-(PAD+0.05), @q[0]);
    appendArrowHead(dd, @p[0], @q[0], as1, col);
  end;
end;

procedure duAppendArrow(dd: TduDebugDraw; const x0, y0, z0, x1, y1, z1, as0, as1: Single; col: Cardinal);
var p,q: array [0..2] of Single;
begin
  if (dd = nil) then Exit;

  dd.vertex(x0,y0,z0, col);
  dd.vertex(x1,y1,z1, col);

  // End arrows
  p[0] := x0; p[1] := y0; p[2] := z0; q[0] := x1; q[1] := y1; q[2] := z1;
  if (as0 > 0.001) then
    appendArrowHead(dd, @p[0], @q[0], as0, col);
  if (as1 > 0.001) then
    appendArrowHead(dd, @q[0], @p[0], as1, col);
end;

procedure duAppendCircle(dd: TduDebugDraw; x,y,z,r: Single; col: Cardinal);
const NUM_SEG = 40;
var dir: array [0..40*2-1] of Single; init: Boolean; i,j: Integer; a: Single;
begin
  if (dd = nil) then Exit;

  init := false;
  if (not init) then
  begin
    init := true;
    for i := 0 to NUM_SEG - 1 do
    begin
      a := i/NUM_SEG*Pi*2;
      dir[i*2] := cos(a);
      dir[i*2+1] := sin(a);
    end;
  end;

  i := 0; j := NUM_SEG-1;
  while (i < NUM_SEG) do
  begin
    dd.vertex(x+dir[j*2+0]*r, y, z+dir[j*2+1]*r, col);
    dd.vertex(x+dir[i*2+0]*r, y, z+dir[i*2+1]*r, col);

    j := i;
    Inc(i);
  end;
end;

procedure duAppendCross(dd: TduDebugDraw; x,y,z: Single;
            s: Single; col: Cardinal);
begin
  if (dd = nil) then Exit;
  dd.vertex(x-s,y,z, col);
  dd.vertex(x+s,y,z, col);
  dd.vertex(x,y-s,z, col);
  dd.vertex(x,y+s,z, col);
  dd.vertex(x,y,z-s, col);
  dd.vertex(x,y,z+s, col);
end;

constructor TduDisplayList.Create(cap: Integer = 512);
begin
  inherited Create;

  m_cap := cap;
  m_depthMask := true;
  m_prim := DU_DRAW_LINES;
  m_primSize := 1.0;

  if (cap < 8) then
    cap := 8;
  resize(cap);
end;

procedure TduDisplayList.resize(cap: Integer);
var newPos: PSingle; newColor: PCardinal;
begin
  GetMem(newPos, cap*3*sizeof(Single));
  if (m_size <> 0)then
    Move(m_pos^, newPos^, sizeof(Single)*3*m_size);
  FreeMem(m_pos);
  m_pos := newPos;

  GetMem(newColor, cap*3*sizeof(Cardinal));
  if (m_size <> 0) then
    Move(m_color^, newColor^, sizeof(Cardinal)*3*m_size);
  FreeMem(m_color);
  m_color := newColor;

  m_cap := cap;
end;

procedure TduDisplayList.clear();
begin
  m_size := 0;
end;

procedure TduDisplayList.depthMask(state: Boolean);
begin
  m_depthMask := state;
end;

procedure TduDisplayList.&begin(prim: TduDebugDrawPrimitives; size: Single);
begin
  clear();
  m_prim := prim;
  m_primSize := size;
end;

procedure TduDisplayList.vertex(const x,y,z: Single; color: Cardinal);
var p: PSingle;
begin
  if (m_size+1 >= m_cap) then
    resize(m_cap*2);
  p := @m_pos[m_size*3];
  p[0] := x;
  p[1] := y;
  p[2] := z;
  m_color[m_size] := color;
  Inc(m_size);
end;

procedure TduDisplayList.vertex(const pos: PSingle; color: Cardinal);
begin
  vertex(pos[0],pos[1],pos[2],color);
end;

procedure TduDisplayList.&end();
begin
end;

procedure TduDisplayList.draw(dd: TduDebugDraw);
var i: Integer;
begin
  //if (!dd) then Exit;
  //if (!m_size) then Exit;
  dd.depthMask(m_depthMask);
  dd.&begin(m_prim, m_primSize);
  for i := 0 to m_size - 1 do
    dd.vertex(@m_pos[i*3], m_color[i]);
  dd.&end();
end;

end.