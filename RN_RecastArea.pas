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

unit RN_RecastArea;
interface
uses
  Math, SysUtils, RN_Helper, RN_Recast;


/// Erodes the walkable area within the heightfield by the specified radius.
///  @ingroup recast
///  @param[in,out]  ctx    The build context to use during the operation.
///  @param[in]    radius  The radius of erosion. [Limits: 0 < value < 255] [Units: vx]
///  @param[in,out]  chf    The populated compact heightfield to erode.
///  @returns True if the operation completed successfully.
function rcErodeWalkableArea(ctx: TrcContext; radius: Integer; chf: PrcCompactHeightfield): Boolean;

/// Applies a median filter to walkable area types (based on area id), removing noise.
///  @ingroup recast
///  @param[in,out]  ctx    The build context to use during the operation.
///  @param[in,out]  chf    A populated compact heightfield.
///  @returns True if the operation completed successfully.
function rcMedianFilterWalkableArea(ctx: TrcContext; chf: PrcCompactHeightfield): Boolean;

/// Applies an area id to all spans within the specified bounding box. (AABB)
///  @ingroup recast
///  @param[in,out]  ctx    The build context to use during the operation.
///  @param[in]    bmin  The minimum of the bounding box. [(x, y, z)]
///  @param[in]    bmax  The maximum of the bounding box. [(x, y, z)]
///  @param[in]    areaId  The area id to apply. [Limit: <= #RC_WALKABLE_AREA]
///  @param[in,out]  chf    A populated compact heightfield.
procedure rcMarkBoxArea(ctx: TrcContext; const bmin, bmax: PSingle; areaId: Byte;
           chf: PrcCompactHeightfield);

/// Applies the area id to the all spans within the specified convex polygon.
///  @ingroup recast
///  @param[in,out]  ctx    The build context to use during the operation.
///  @param[in]    verts  The vertices of the polygon [Fomr: (x, y, z) * @p nverts]
///  @param[in]    nverts  The number of vertices in the polygon.
///  @param[in]    hmin  The height of the base of the polygon.
///  @param[in]    hmax  The height of the top of the polygon.
///  @param[in]    areaId  The area id to apply. [Limit: <= #RC_WALKABLE_AREA]
///  @param[in,out]  chf    A populated compact heightfield.
procedure rcMarkConvexPolyArea(ctx: TrcContext; const verts: PSingle; const nverts: Integer;
              const hmin, hmax: Single; areaId: Byte;
              chf: PrcCompactHeightfield);

/// Helper function to offset voncex polygons for rcMarkConvexPolyArea.
///  @ingroup recast
///  @param[in]    verts    The vertices of the polygon [Form: (x, y, z) * @p nverts]
///  @param[in]    nverts    The number of vertices in the polygon.
///  @param[out]  outVerts  The offset vertices (should hold up to 2 * @p nverts) [Form: (x, y, z) * return value]
///  @param[in]    maxOutVerts  The max number of vertices that can be stored to @p outVerts.
///  @returns Number of vertices in the offset polygon or 0 if too few vertices in @p outVerts.
function rcOffsetPoly(const verts: PSingle; const nverts: Integer; const offset: Single;
         out outVerts: PSingle; const maxOutVerts: Integer): Integer;

/// Applies the area id to all spans within the specified cylinder.
///  @ingroup recast
///  @param[in,out]  ctx    The build context to use during the operation.
///  @param[in]    pos    The center of the base of the cylinder. [Form: (x, y, z)]
///  @param[in]    r    The radius of the cylinder.
///  @param[in]    h    The height of the cylinder.
///  @param[in]    areaId  The area id to apply. [Limit: <= #RC_WALKABLE_AREA]
///  @param[in,out]  chf  A populated compact heightfield.
procedure rcMarkCylinderArea(ctx: TrcContext; const pos: PSingle;
            const r, h: Single; areaId: Byte;
            chf: PrcCompactHeightfield);


implementation
uses RN_RecastHelper;


/// @par
///
/// Basically, any spans that are closer to a boundary or obstruction than the specified radius
/// are marked as unwalkable.
///
/// This method is usually called immediately after the heightfield has been built.
///
/// @see rcCompactHeightfield, rcBuildCompactHeightfield, rcConfig::walkableRadius
function rcErodeWalkableArea(ctx: TrcContext; radius: Integer; chf: PrcCompactHeightfield): Boolean;
var w,h: Integer; dist: PByte; x,y,i: Integer; c: PrcCompactCell; s: PrcCompactSpan; nc,dir: Integer;
nx,ny,nidx: Integer; ax,ay,ai: Integer; nd: Byte; as1: PrcCompactSpan; aax,aay,aai: Integer; thr: Byte;
begin
  //rcAssert(ctx);

  w := chf.width;
  h := chf.height;

  ctx.startTimer(RC_TIMER_ERODE_AREA);

  GetMem(dist, sizeof(Byte)*chf.spanCount);

  // Init distance.
  FillChar(dist[0], SizeOf(Byte)*chf.spanCount, $ff);

  // Mark boundary cells.
  for y := 0 to h - 1 do
  begin
    for x := 0 to w - 1 do
    begin
      c := @chf.cells[x+y*w];
      for i := c.index to Integer(c.index+c.count) - 1 do
      begin
        if (chf.areas[i] = RC_NULL_AREA) then
        begin
          dist[i] := 0;
        end
        else
        begin
          s := @chf.spans[i];
          nc := 0;
          for dir := 0 to 3 do
          begin
            if (rcGetCon(s, dir) <> RC_NOT_CONNECTED) then
            begin
              nx := x + rcGetDirOffsetX(dir);
              ny := y + rcGetDirOffsetY(dir);
              nidx := chf.cells[nx+ny*w].index + rcGetCon(s, dir);
              if (chf.areas[nidx] <> RC_NULL_AREA) then
              begin
                Inc(nc);
              end;
            end;
          end;
          // At least one missing neighbour.
          if (nc <> 4) then
            dist[i] := 0;
        end;
      end;
    end;
  end;

  // Pass 1
  for y := 0 to h - 1 do
  begin
    for x := 0 to w - 1 do
    begin
      c := @chf.cells[x+y*w];
      for i := c.index to Integer(c.index+c.count) - 1 do
      begin
        s := @chf.spans[i];

        if (rcGetCon(s, 0) <> RC_NOT_CONNECTED) then
        begin
          // (-1,0)
          ax := x + rcGetDirOffsetX(0);
          ay := y + rcGetDirOffsetY(0);
          ai := chf.cells[ax+ay*w].index + rcGetCon(s, 0);
          as1 := @chf.spans[ai];
          nd := rcMin(dist[ai]+2, 255);
          if (nd < dist[i]) then
            dist[i] := nd;

          // (-1,-1)
          if (rcGetCon(as1, 3) <> RC_NOT_CONNECTED) then
          begin
            aax := ax + rcGetDirOffsetX(3);
            aay := ay + rcGetDirOffsetY(3);
            aai := chf.cells[aax+aay*w].index + rcGetCon(as1, 3);
            nd := rcMin(dist[aai]+3, 255);
            if (nd < dist[i]) then
              dist[i] := nd;
          end;
        end;
        if (rcGetCon(s, 3) <> RC_NOT_CONNECTED) then
        begin
          // (0,-1)
          ax := x + rcGetDirOffsetX(3);
          ay := y + rcGetDirOffsetY(3);
          ai := chf.cells[ax+ay*w].index + rcGetCon(s, 3);
          as1 := @chf.spans[ai];
          nd := rcMin(dist[ai]+2, 255);
          if (nd < dist[i]) then
            dist[i] := nd;

          // (1,-1)
          if (rcGetCon(as1, 2) <> RC_NOT_CONNECTED) then
          begin
            aax := ax + rcGetDirOffsetX(2);
            aay := ay + rcGetDirOffsetY(2);
            aai := chf.cells[aax+aay*w].index + rcGetCon(as1, 2);
            nd := rcMin(dist[aai]+3, 255);
            if (nd < dist[i]) then
              dist[i] := nd;
          end;
        end;
      end;
    end;
  end;

  // Pass 2
  for y := h - 1 downto 0 do
  begin
    for x := w - 1 downto 0 do
    begin
      c := @chf.cells[x+y*w];
      for i := c.index to Integer(c.index+c.count) - 1 do
      begin
        s := @chf.spans[i];

        if (rcGetCon(s, 2) <> RC_NOT_CONNECTED) then
        begin
          // (1,0)
          ax := x + rcGetDirOffsetX(2);
          ay := y + rcGetDirOffsetY(2);
          ai := chf.cells[ax+ay*w].index + rcGetCon(s, 2);
          as1 := @chf.spans[ai];
          nd := rcMin(dist[ai]+2, 255);
          if (nd < dist[i]) then
            dist[i] := nd;

          // (1,1)
          if (rcGetCon(as1, 1) <> RC_NOT_CONNECTED) then
          begin
            aax := ax + rcGetDirOffsetX(1);
            aay := ay + rcGetDirOffsetY(1);
            aai := chf.cells[aax+aay*w].index + rcGetCon(as1, 1);
            nd := rcMin(dist[aai]+3, 255);
            if (nd < dist[i]) then
              dist[i] := nd;
          end;
        end;
        if (rcGetCon(s, 1) <> RC_NOT_CONNECTED) then
        begin
          // (0,1)
          ax := x + rcGetDirOffsetX(1);
          ay := y + rcGetDirOffsetY(1);
          ai := chf.cells[ax+ay*w].index + rcGetCon(s, 1);
          as1 := @chf.spans[ai];
          nd := rcMin(dist[ai]+2, 255);
          if (nd < dist[i]) then
            dist[i] := nd;

          // (-1,1)
          if (rcGetCon(as1, 0) <> RC_NOT_CONNECTED) then
          begin
            aax := ax + rcGetDirOffsetX(0);
            aay := ay + rcGetDirOffsetY(0);
            aai := chf.cells[aax+aay*w].index + rcGetCon(as1, 0);
            nd := rcMin(dist[aai]+3, 255);
            if (nd < dist[i]) then
              dist[i] := nd;
          end;
        end;
      end;
    end;
  end;

  thr := Byte(radius*2);
  for i := 0 to chf.spanCount - 1 do
    if (dist[i] < thr) then
      chf.areas[i] := RC_NULL_AREA;

  FreeMem(dist);

  ctx.stopTimer(RC_TIMER_ERODE_AREA);

  Result := True;
end;

procedure insertSort(a: array of Byte; const n: Integer);
var i, j: Integer; value: Byte;
begin
  for i := 1 to n - 1 do
  begin
    value := a[i];
    //for (j := i - 1; j >= 0 && a[j] > value; j--)
    j := i - 1;
    while (j >= 0) and (a[j] > value) do
    begin
      a[j+1] := a[j];
      Dec(j);
    end;
    a[j+1] := value;
  end;
end;

/// @par
///
/// This filter is usually applied after applying area id's using functions
/// such as #rcMarkBoxArea, #rcMarkConvexPolyArea, and #rcMarkCylinderArea.
///
/// @see rcCompactHeightfield
function rcMedianFilterWalkableArea(ctx: TrcContext; chf: PrcCompactHeightfield): Boolean;
var w,h: Integer; areas: PByte; x,y,i,j,dir,dir2: Integer; c: PrcCompactCell; s,as1: PrcCompactSpan; nei: array [0..8] of Byte;
ax,ay,ai,ax2,ay2,ai2: Integer;
begin
  //rcAssert(ctx);

  w := chf.width;
  h := chf.height;

  ctx.startTimer(RC_TIMER_MEDIAN_AREA);

  GetMem(areas, sizeof(Byte)*chf.spanCount);

  // Init distance.
  FillChar(areas[0], sizeof(Byte)*chf.spanCount, $ff);

  for y := 0 to h - 1 do
  begin
    for x := 0 to w - 1 do
    begin
      c := @chf.cells[x+y*w];
      for i := c.index to Integer(c.index+c.count) - 1 do
      begin
        s := @chf.spans[i];
        if (chf.areas[i] = RC_NULL_AREA) then
        begin
          areas[i] := chf.areas[i];
          continue;
        end;

        for j := 0 to 8 do
          nei[j] := chf.areas[i];

        for dir := 0 to 3 do
        begin
          if (rcGetCon(s, dir) <> RC_NOT_CONNECTED) then
          begin
            ax := x + rcGetDirOffsetX(dir);
            ay := y + rcGetDirOffsetY(dir);
            ai := chf.cells[ax+ay*w].index + rcGetCon(s, dir);
            if (chf.areas[ai] <> RC_NULL_AREA) then
              nei[dir*2+0] := chf.areas[ai];

            as1 := @chf.spans[ai];
            dir2 := (dir+1) and $3;
            if (rcGetCon(as1, dir2) <> RC_NOT_CONNECTED) then
            begin
              ax2 := ax + rcGetDirOffsetX(dir2);
              ay2 := ay + rcGetDirOffsetY(dir2);
              ai2 := chf.cells[ax2+ay2*w].index + rcGetCon(as1, dir2);
              if (chf.areas[ai2] <> RC_NULL_AREA) then
                nei[dir*2+1] := chf.areas[ai2];
            end;
          end;
        end;
        insertSort(nei, 9);
        areas[i] := nei[4];
      end;
    end;
  end;

  Move(areas[0], chf.areas[0], sizeof(Byte)*chf.spanCount);

  FreeMem(areas);

  ctx.stopTimer(RC_TIMER_MEDIAN_AREA);

  Result := true;
end;

/// @par
///
/// The value of spacial parameters are in world units.
///
/// @see rcCompactHeightfield, rcMedianFilterWalkableArea
procedure rcMarkBoxArea(ctx: TrcContext; const bmin, bmax: PSingle; areaId: Byte;
           chf: PrcCompactHeightfield);
var minx, miny, minz, maxx, maxy, maxz: Integer; z,x,i: Integer; c: PrcCompactCell; s: PrcCompactSpan;
begin
  //rcAssert(ctx);

  ctx.startTimer(RC_TIMER_MARK_BOX_AREA);

  minx := Trunc((bmin[0]-chf.bmin[0])/chf.cs);
  miny := Trunc((bmin[1]-chf.bmin[1])/chf.ch);
  minz := Trunc((bmin[2]-chf.bmin[2])/chf.cs);
  maxx := Trunc((bmax[0]-chf.bmin[0])/chf.cs);
  maxy := Trunc((bmax[1]-chf.bmin[1])/chf.ch);
  maxz := Trunc((bmax[2]-chf.bmin[2])/chf.cs);

  if (maxx < 0) then Exit;
  if (minx >= chf.width) then Exit;
  if (maxz < 0) then Exit;
  if (minz >= chf.height) then Exit;

  if (minx < 0) then minx := 0;
  if (maxx >= chf.width) then maxx := chf.width-1;
  if (minz < 0) then minz := 0;
  if (maxz >= chf.height) then maxz := chf.height-1;

  for z := minz to maxz do
  begin
    for x := minx to maxx do
    begin
      c := @chf.cells[x+z*chf.width];
      for i := c.index to Integer(c.index+c.count)- 1 do
      begin
        s := @chf.spans[i];
        if (s.y >= miny) and (s.y <= maxy) then
        begin
          if (chf.areas[i] <> RC_NULL_AREA) then
            chf.areas[i] := areaId;
        end;
      end;
    end;
  end;

  ctx.stopTimer(RC_TIMER_MARK_BOX_AREA);
end;


function pointInPoly(nvert: Integer; const verts: PSingle; const p: PSingle): Boolean;
var i, j: Integer; c: Boolean; vi,vj: PSingle;
begin
  c := False;
  //for (i = 0, j = nvert-1; i < nvert; j = i++)
  i := 0;
  j := nvert-1;
  while (i < nvert) do
  begin
    vi := @verts[i*3];
    vj := @verts[j*3];
    if (((vi[2] > p[2]) <> (vj[2] > p[2])) and
      (p[0] < (vj[0]-vi[0]) * (p[2]-vi[2]) / (vj[2]-vi[2]) + vi[0]) ) then
      c := not c;

    j := i;
    Inc(i);
  end;
  Result := c;
end;

/// @par
///
/// The value of spacial parameters are in world units.
///
/// The y-values of the polygon vertices are ignored. So the polygon is effectively
/// projected onto the xz-plane at @p hmin, then extruded to @p hmax.
///
/// @see rcCompactHeightfield, rcMedianFilterWalkableArea
procedure rcMarkConvexPolyArea(ctx: TrcContext; const verts: PSingle; const nverts: Integer;
              const hmin, hmax: Single; areaId: Byte;
              chf: PrcCompactHeightfield);
var bmin, bmax, p: array [0..2] of Single; i,x,z: Integer; minx, miny, minz, maxx, maxy, maxz: Integer; c: PrcCompactCell; s: PrcCompactSpan;
begin
  //rcAssert(ctx);

  ctx.startTimer(RC_TIMER_MARK_CONVEXPOLY_AREA);

  rcVcopy(@bmin[0], @verts[0]);
  rcVcopy(@bmax[0], @verts[0]);
  for i := 1 to nverts - 1 do
  begin
    rcVmin(@bmin[0], @verts[i*3]);
    rcVmax(@bmax[0], @verts[i*3]);
  end;
  bmin[1] := hmin;
  bmax[1] := hmax;

  minx := Trunc((bmin[0]-chf.bmin[0])/chf.cs);
  miny := Trunc((bmin[1]-chf.bmin[1])/chf.ch);
  minz := Trunc((bmin[2]-chf.bmin[2])/chf.cs);
  maxx := Trunc((bmax[0]-chf.bmin[0])/chf.cs);
  maxy := Trunc((bmax[1]-chf.bmin[1])/chf.ch);
  maxz := Trunc((bmax[2]-chf.bmin[2])/chf.cs);

  if (maxx < 0) then Exit;
  if (minx >= chf.width) then Exit;
  if (maxz < 0) then Exit;
  if (minz >= chf.height) then Exit;

  if (minx < 0) then minx := 0;
  if (maxx >= chf.width) then maxx := chf.width-1;
  if (minz < 0) then minz := 0;
  if (maxz >= chf.height) then maxz := chf.height-1;


  // TODO: Optimize.
  for z := minz to maxz do
  begin
    for x := minx to maxx do
    begin
      c := @chf.cells[x+z*chf.width];
      for i := c.index to Integer(c.index+c.count) - 1 do
      begin
        s := @chf.spans[i];
        if (chf.areas[i] = RC_NULL_AREA) then
          continue;
        if (s.y >= miny) and (s.y <= maxy) then
        begin
          p[0] := chf.bmin[0] + (x+0.5)*chf.cs;
          p[1] := 0;
          p[2] := chf.bmin[2] + (z+0.5)*chf.cs;

          if (pointInPoly(nverts, verts, @p[0])) then
          begin
            chf.areas[i] := areaId;
          end;
        end;
      end;
    end;
  end;

  ctx.stopTimer(RC_TIMER_MARK_CONVEXPOLY_AREA);
end;

function rcOffsetPoly(const verts: PSingle; const nverts: Integer; const offset: Single;
         out outVerts: PSingle; const maxOutVerts: Integer): Integer;
const MITER_LIMIT = 1.20;
var n,i: Integer; a,b,c: Integer; va,vb,vc: PSingle; dx0,dy0,d0,dx1,dy1,d1,dlx0,dly0,dlx1,dly1: Single;
cross,dmx,dmy,dmr2: Single; bevel: Boolean; scale,d: Single;
begin
  n := 0;

  for i := 0 to nverts - 1 do
  begin
    a := (i+nverts-1) mod nverts;
    b := i;
    c := (i+1) mod nverts;
    va := @verts[a*3];
    vb := @verts[b*3];
    vc := @verts[c*3];
    dx0 := vb[0] - va[0];
    dy0 := vb[2] - va[2];
    d0 := dx0*dx0 + dy0*dy0;
    if (d0 > 0.000001) then
    begin
      d0 := 1.0/Sqrt(d0);
      dx0 := dx0 * d0;
      dy0 := dy0 * d0;
    end;
    dx1 := vc[0] - vb[0];
    dy1 := vc[2] - vb[2];
    d1 := dx1*dx1 + dy1*dy1;
    if (d1 > 0.000001) then
    begin
      d1 := 1.0/Sqrt(d1);
      dx1 := dx1 * d1;
      dy1 := dy1 * d1;
    end;
    dlx0 := -dy0;
    dly0 := dx0;
    dlx1 := -dy1;
    dly1 := dx1;
    cross := dx1*dy0 - dx0*dy1;
    dmx := (dlx0 + dlx1) * 0.5;
    dmy := (dly0 + dly1) * 0.5;
    dmr2 := dmx*dmx + dmy*dmy;
    bevel := dmr2 * MITER_LIMIT*MITER_LIMIT < 1.0;
    if (dmr2 > 0.000001) then
    begin
      scale := 1.0 / dmr2;
      dmx := dmx * scale;
      dmy := dmy * scale;
    end;

    if bevel and (cross < 0.0) then
    begin
      if (n+2 >= maxOutVerts) then
        Exit(0);
      d := (1.0 - (dx0*dx1 + dy0*dy1))*0.5;
      outVerts[n*3+0] := vb[0] + (-dlx0+dx0*d)*offset;
      outVerts[n*3+1] := vb[1];
      outVerts[n*3+2] := vb[2] + (-dly0+dy0*d)*offset;
      Inc(n);
      outVerts[n*3+0] := vb[0] + (-dlx1-dx1*d)*offset;
      outVerts[n*3+1] := vb[1];
      outVerts[n*3+2] := vb[2] + (-dly1-dy1*d)*offset;
      Inc(n);
    end
    else
    begin
      if (n+1 >= maxOutVerts) then
        Exit(0);
      outVerts[n*3+0] := vb[0] - dmx*offset;
      outVerts[n*3+1] := vb[1];
      outVerts[n*3+2] := vb[2] - dmy*offset;
      Inc(n);
    end;
  end;

  Result := n;
end;


/// @par
///
/// The value of spacial parameters are in world units.
///
/// @see rcCompactHeightfield, rcMedianFilterWalkableArea
procedure rcMarkCylinderArea(ctx: TrcContext; const pos: PSingle;
            const r, h: Single; areaId: Byte;
            chf: PrcCompactHeightfield);
var bmin, bmax: array [0..2] of Single; r2: Single; minx, miny, minz, maxx, maxy, maxz: Integer; x,z,i: Integer; c: PrcCompactCell; s: PrcCompactSpan;
sx,sz,dx,dz: Single;
begin
  //rcAssert(ctx);

  ctx.startTimer(RC_TIMER_MARK_CYLINDER_AREA);

  bmin[0] := pos[0] - r;
  bmin[1] := pos[1];
  bmin[2] := pos[2] - r;
  bmax[0] := pos[0] + r;
  bmax[1] := pos[1] + h;
  bmax[2] := pos[2] + r;
  r2 := r*r;

  minx := Trunc((bmin[0]-chf.bmin[0])/chf.cs);
  miny := Trunc((bmin[1]-chf.bmin[1])/chf.ch);
  minz := Trunc((bmin[2]-chf.bmin[2])/chf.cs);
  maxx := Trunc((bmax[0]-chf.bmin[0])/chf.cs);
  maxy := Trunc((bmax[1]-chf.bmin[1])/chf.ch);
  maxz := Trunc((bmax[2]-chf.bmin[2])/chf.cs);

  if (maxx < 0) then Exit;
  if (minx >= chf.width) then Exit;
  if (maxz < 0) then Exit;
  if (minz >= chf.height) then Exit;

  if (minx < 0) then minx := 0;
  if (maxx >= chf.width) then maxx := chf.width-1;
  if (minz < 0) then minz := 0;
  if (maxz >= chf.height) then maxz := chf.height-1;


  for z := minz to maxz do
  begin
    for x := minx to maxx do
    begin
      c := @chf.cells[x+z*chf.width];
      for i := c.index to Integer(c.index+c.count) - 1 do
      begin
        s := @chf.spans[i];

        if (chf.areas[i] = RC_NULL_AREA) then
          Continue;

        if (s.y >= miny) and (s.y <= maxy) then
        begin
          sx := chf.bmin[0] + (x+0.5)*chf.cs;
          sz := chf.bmin[2] + (z+0.5)*chf.cs;
          dx := sx - pos[0];
          dz := sz - pos[2];

          if (dx*dx + dz*dz < r2) then
          begin
            chf.areas[i] := areaId;
          end;
        end;
      end;
    end;
  end;

  ctx.stopTimer(RC_TIMER_MARK_CYLINDER_AREA);
end;

end.
