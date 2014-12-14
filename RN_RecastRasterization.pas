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

unit RN_RecastRasterization;
interface
uses
  Math, SysUtils, RN_Helper, RN_Recast;

/// Adds a span to the specified heightfield.
///  @ingroup recast
///  @param[in,out]  ctx        The build context to use during the operation.
///  @param[in,out]  hf        An initialized heightfield.
///  @param[in]    x        The width index where the span is to be added.
///                  [Limits: 0 <= value < rcHeightfield::width]
///  @param[in]    y        The height index where the span is to be added.
///                  [Limits: 0 <= value < rcHeightfield::height]
///  @param[in]    smin      The minimum height of the span. [Limit: < @p smax] [Units: vx]
///  @param[in]    smax      The maximum height of the span. [Limit: <= #RC_SPAN_MAX_HEIGHT] [Units: vx]
///  @param[in]    area      The area id of the span. [Limit: <= #RC_WALKABLE_AREA)
///  @param[in]    flagMergeThr  The merge theshold. [Limit: >= 0] [Units: vx]
procedure rcAddSpan(ctx: TrcContext; var hf: TrcHeightfield; const x, y: Integer;
         const smin, smax: Word;
         const area: Byte; const flagMergeThr: Integer);

/// Rasterizes a triangle into the specified heightfield.
///  @ingroup recast
///  @param[in,out]  ctx        The build context to use during the operation.
///  @param[in]    v0        Triangle vertex 0 [(x, y, z)]
///  @param[in]    v1        Triangle vertex 1 [(x, y, z)]
///  @param[in]    v2        Triangle vertex 2 [(x, y, z)]
///  @param[in]    area      The area id of the triangle. [Limit: <= #RC_WALKABLE_AREA]
///  @param[in,out]  solid      An initialized heightfield.
///  @param[in]    flagMergeThr  The distance where the walkable flag is favored over the non-walkable flag.
///                  [Limit: >= 0] [Units: vx]
procedure rcRasterizeTriangle(ctx: TrcContext; const v0, v1, v2: PSingle;
             const area: Byte; var solid: TrcHeightfield;
             const flagMergeThr: Integer = 1);

/// Rasterizes an indexed triangle mesh into the specified heightfield.
///  @ingroup recast
///  @param[in,out]  ctx        The build context to use during the operation.
///  @param[in]    verts      The vertices. [(x, y, z) * @p nv]
///  @param[in]    nv        The number of vertices.
///  @param[in]    tris      The triangle indices. [(vertA, vertB, vertC) * @p nt]
///  @param[in]    areas      The area id's of the triangles. [Limit: <= #RC_WALKABLE_AREA] [Size: @p nt]
///  @param[in]    nt        The number of triangles.
///  @param[in,out]  solid      An initialized heightfield.
///  @param[in]    flagMergeThr  The distance where the walkable flag is favored over the non-walkable flag.
///                  [Limit: >= 0] [Units: vx]
procedure rcRasterizeTriangles(ctx: TrcContext; const verts: PSingle; const nv: Integer;
              const tris: PInteger; const areas: PByte; const nt: Integer;
              var solid: TrcHeightfield; const flagMergeThr: Integer = 1); overload;

/// Rasterizes an indexed triangle mesh into the specified heightfield.
///  @ingroup recast
///  @param[in,out]  ctx      The build context to use during the operation.
///  @param[in]    verts    The vertices. [(x, y, z) * @p nv]
///  @param[in]    nv      The number of vertices.
///  @param[in]    tris    The triangle indices. [(vertA, vertB, vertC) * @p nt]
///  @param[in]    areas    The area id's of the triangles. [Limit: <= #RC_WALKABLE_AREA] [Size: @p nt]
///  @param[in]    nt      The number of triangles.
///  @param[in,out]  solid    An initialized heightfield.
///  @param[in]    flagMergeThr  The distance where the walkable flag is favored over the non-walkable flag.
///                [Limit: >= 0] [Units: vx]
procedure rcRasterizeTriangles(ctx: TrcContext; const verts: PSingle; const nv: Integer;
              const tris: PWord; const areas: PByte; const nt: Integer;
              var solid: TrcHeightfield; const flagMergeThr: Integer = 1); overload;

/// Rasterizes triangles into the specified heightfield.
///  @ingroup recast
///  @param[in,out]  ctx        The build context to use during the operation.
///  @param[in]    verts      The triangle vertices. [(ax, ay, az, bx, by, bz, cx, by, cx) * @p nt]
///  @param[in]    areas      The area id's of the triangles. [Limit: <= #RC_WALKABLE_AREA] [Size: @p nt]
///  @param[in]    nt        The number of triangles.
///  @param[in,out]  solid      An initialized heightfield.
///  @param[in]    flagMergeThr  The distance where the walkable flag is favored over the non-walkable flag.
///                  [Limit: >= 0] [Units: vx]
procedure rcRasterizeTriangles(ctx: TrcContext; const verts: PSingle; const areas: PByte; const nt: Integer;
              var solid: TrcHeightfield; const flagMergeThr: Integer = 1); overload;


implementation
uses RN_RecastHelper;


function overlapBounds(const amin, amax, bmin, bmax: PSingle): Boolean;
var overlap: Boolean;
begin
  overlap := true;
  if (amin[0] > bmax[0]) or (amax[0] < bmin[0]) then overlap := false;
  if (amin[1] > bmax[1]) or (amax[1] < bmin[1]) then overlap := false;
  if (amin[2] > bmax[2]) or (amax[2] < bmin[2]) then overlap := false;
  Result := overlap;
end;

function overlapInterval(amin, amax, bmin, bmax: Word): Boolean;
begin
  if (amax < bmin) then Exit(false);
  if (amin > bmax) then Exit(false);
  Result := true;
end;

function allocSpan(var hf: TrcHeightfield): PrcSpan;
var pool: PrcSpanPool; freelist,head,it: PrcSpan;
begin
  // If running out of memory, allocate new page and update the freelist.
  if (hf.freelist = nil) or (hf.freelist.next = nil) then
  begin
    // Create new page.
    // Allocate memory for the new pool.
    GetMem(pool, SizeOf(TrcSpanPool));

    // Add the pool into the list of pools.
    pool.next := hf.pools;
    hf.pools := pool;
    // Add new items to the free list.
    freelist := hf.freelist;
    head := @pool.items[0];
    it := @pool.items[RC_SPANS_PER_POOL-1]; Inc(it); // Delphi: Need this trick because Delphi refuses to give pointer to last+1 element
    repeat
      Dec(it);
      it.next := freelist;
      freelist := it;
    until (it = head);
    hf.freelist := it;
  end;

  // Pop item from in front of the free list.
  it := hf.freelist;
  hf.freelist := hf.freelist.next;
  Result := it;
end;

procedure freeSpan(var hf: TrcHeightfield; ptr: PrcSpan);
begin
  if (ptr = nil) then Exit;
  // Add the node in front of the free list.
  ptr.next := hf.freelist;
  hf.freelist := ptr;
end;

procedure addSpan(var hf: TrcHeightfield; const x, y: Integer;
          const smin, smax: Word;
          const area: Byte; const flagMergeThr: Integer);
var idx: Integer; s: PrcSpan; prev,cur,next: PrcSpan;
begin

  idx := x + y*hf.width;

  s := allocSpan(hf);
  s.smin := smin;
  s.smax := smax;
  s.area := area;
  s.next := nil;

  // Empty cell, add the first span.
  if (hf.spans[idx] = nil) then
  begin
    hf.spans[idx] := s;
    Exit;
  end;
  prev := nil;
  cur := hf.spans[idx];

  // Insert and merge spans.
  while (cur <> nil) do
  begin
    if (cur.smin > s.smax) then
    begin
      // Current span is further than the new span, break.
      break;
    end
    else if (cur.smax < s.smin) then
    begin
      // Current span is before the new span advance.
      prev := cur;
      cur := cur.next;
    end
    else
    begin
      // Merge spans.
      if (cur.smin < s.smin) then
        s.smin := cur.smin;
      if (cur.smax > s.smax) then
        s.smax := cur.smax;

      // Merge flags.
      if (Abs(s.smax - cur.smax) <= flagMergeThr) then
        s.area := rcMax(s.area, cur.area);

      // Remove current span.
      next := cur.next;
      freeSpan(hf, cur);
      if (prev <> nil) then
        prev.next := next
      else
        hf.spans[idx] := next;
      cur := next;
    end;
  end;

  // Insert new span.
  if (prev <> nil) then
  begin
    s.next := prev.next;
    prev.next := s;
  end
  else
  begin
    s.next := hf.spans[idx];
    hf.spans[idx] := s;
  end;
end;

/// @par
///
/// The span addition can be set to favor flags. If the span is merged to
/// another span and the new @p smax is within @p flagMergeThr units
/// from the existing span, the span flags are merged.
///
/// @see rcHeightfield, rcSpan.
procedure rcAddSpan(ctx: TrcContext; var hf: TrcHeightfield; const x, y: Integer;
         const smin, smax: Word;
         const area: Byte; const flagMergeThr: Integer);
begin
//  rcAssert(ctx);
  addSpan(hf, x,y, smin, smax, area, flagMergeThr);
end;

// divides a convex polygons into two convex polygons on both sides of a line
procedure dividePoly(&in: PSingle; nin: Integer;
            out1: PSingle; nout1: PInteger;
            out2: PSingle; nout2: PInteger;
            x: Single; axis: Integer);
var d: array [0..11] of Single; m,n,i,j: Integer; ina,inb: Boolean; s: Single;
begin
  for i := 0 to nin - 1 do
    d[i] := x - &in[i*3+axis];

  m := 0; n := 0;
  //for (int i = 0, j = nin-1; i < nin; j=i, ++i)
  i := 0;
  j := nin-1;
  while (i < nin) do
  begin
    ina := d[j] >= 0;
    inb := d[i] >= 0;
    if (ina <> inb) then
    begin
      s := d[j] / (d[j] - d[i]);
      out1[m*3+0] := &in[j*3+0] + (&in[i*3+0] - &in[j*3+0])*s;
      out1[m*3+1] := &in[j*3+1] + (&in[i*3+1] - &in[j*3+1])*s;
      out1[m*3+2] := &in[j*3+2] + (&in[i*3+2] - &in[j*3+2])*s;
      rcVcopy(out2 + n*3, out1 + m*3);
      Inc(m);
      Inc(n);
      // add the i'th point to the right polygon. Do NOT add points that are on the dividing line
      // since these were already added above
      if (d[i] > 0) then
      begin
        rcVcopy(out1 + m*3, &in + i*3);
        Inc(m);
      end
      else if (d[i] < 0) then
      begin
        rcVcopy(out2 + n*3, &in + i*3);
        Inc(n);
      end;
    end
    else // same side
    begin
      // add the i'th point to the right polygon. Addition is done even for points on the dividing line
      if (d[i] >= 0) then
      begin
        rcVcopy(out1 + m*3, &in + i*3);
        Inc(m);
        if (d[i] <> 0) then
        begin
          //C++ seems to be doing loop increase, so do we
          j := i;
          Inc(i);
          continue;
        end;
      end;
      rcVcopy(out2 + n*3, &in + i*3);
      Inc(n);
    end;

    j := i;
    Inc(i);
  end;

  nout1^ := m;
  nout2^ := n;
end;


procedure rasterizeTri(const v0, v1, v2: PSingle;
             const area: Byte; var hf: TrcHeightfield;
             bmin, bmax: PSingle;
             cs,ics,ich: Single;
             const flagMergeThr: Integer);
var w,h: Integer; tmin,tmax: array [0..2] of Single; by: Single; y0,y1: Integer; buf: array [0..7*3*4-1] of Single;
  &in, inrow, p1, p2: PSingle; nvrow,nvIn,x,y,i,nv,nv2: Integer; cx, cz, minX, maxX,smin,smax: Single; x0,x1: Integer;
  ismin,ismax: Word;
begin
  w := hf.width;
  h := hf.height;
  by := bmax[1] - bmin[1];

  // Calculate the bounding box of the triangle.
  rcVcopy(@tmin, v0);
  rcVcopy(@tmax, v0);
  rcVmin(@tmin, v1);
  rcVmin(@tmin, v2);
  rcVmax(@tmax, v1);
  rcVmax(@tmax, v2);

  // If the triangle does not touch the bbox of the heightfield, skip the triagle.
  if (not overlapBounds(bmin, bmax, @tmin, @tmax)) then
    Exit;

  // Calculate the footprint of the triangle on the grid's y-axis
  y0 := Trunc((tmin[2] - bmin[2])*ics);
  y1 := Trunc((tmax[2] - bmin[2])*ics);
  y0 := rcClamp(y0, 0, h-1);
  y1 := rcClamp(y1, 0, h-1);

  // Clip the triangle into all grid cells it touches.
  //float buf[7*3*4];
  //float *in = buf, *inrow = buf+7*3, *p1 = inrow+7*3, *p2 = p1+7*3;
  &in := @buf[0];
  inrow := @buf[7*3];
  p1 := @buf[14*3];
  p2 := @buf[21*3];

  rcVcopy(@&in[0], v0);
  rcVcopy(@&in[1*3], v1);
  rcVcopy(@&in[2*3], v2);
  nvIn := 3;

  for y := y0 to y1 do
  begin
    // Clip polygon to row. Store the remaining polygon as well
    cz := bmin[2] + y*cs;
    dividePoly(&in, nvIn, inrow, @nvrow, p1, @nvIn, cz+cs, 2);
    rcSwap(&in, p1);
    if (nvrow < 3) then continue;

    // find the horizontal bounds in the row
    minX := inrow[0]; maxX := inrow[0];
    for i := 1 to nvrow - 1 do
    begin
      if (minX > inrow[i*3]) then minX := inrow[i*3];
      if (maxX < inrow[i*3]) then maxX := inrow[i*3];
    end;
    x0 := Trunc((minX - bmin[0])*ics);
    x1 := Trunc((maxX - bmin[0])*ics);
    x0 := rcClamp(x0, 0, w-1);
    x1 := rcClamp(x1, 0, w-1);

    nv2 := nvrow;

    for x := x0 to x1 do
    begin
      // Clip polygon to column. store the remaining polygon as well
      cx := bmin[0] + x*cs;
      dividePoly(inrow, nv2, p1, @nv, p2, @nv2, cx+cs, 0);
      rcSwap(inrow, p2);
      if (nv < 3) then continue;

      // Calculate min and max of the span.
      smin := p1[1]; smax := p1[1];
      for i := 1 to nv - 1 do
      begin
        smin := rcMin(smin, p1[i*3+1]);
        smax := rcMax(smax, p1[i*3+1]);
      end;
      smin := smin - bmin[1];
      smax := smax - bmin[1];
      // Skip the span if it is outside the heightfield bbox
      if (smax < 0.0) then continue;
      if (smin > by) then continue;
      // Clamp the span to the heightfield bbox.
      if (smin < 0.0) then smin := 0;
      if (smax > by) then smax := by;

      // Snap the span to the heightfield height grid.
      ismin := rcClamp(floor(smin * ich), 0, RC_SPAN_MAX_HEIGHT);
      ismax := rcClamp(ceil(smax * ich), ismin+1, RC_SPAN_MAX_HEIGHT);

      addSpan(hf, x, y, ismin, ismax, area, flagMergeThr);
    end;
  end;
end;

/// @par
///
/// No spans will be added if the triangle does not overlap the heightfield grid.
///
/// @see rcHeightfield
procedure rcRasterizeTriangle(ctx: TrcContext; const v0, v1, v2: PSingle;
             const area: Byte; var solid: TrcHeightfield;
             const flagMergeThr: Integer = 1);
var ics,ich: Single;
begin
  //rcAssert(ctx);

  ctx.startTimer(RC_TIMER_RASTERIZE_TRIANGLES);

  ics := 1.0/solid.cs;
  ich := 1.0/solid.ch;
  rasterizeTri(v0, v1, v2, area, solid, @solid.bmin, @solid.bmax, solid.cs, ics, ich, flagMergeThr);

  ctx.stopTimer(RC_TIMER_RASTERIZE_TRIANGLES);
end;

/// @par
///
/// Spans will only be added for triangles that overlap the heightfield grid.
///
/// @see rcHeightfield
procedure rcRasterizeTriangles(ctx: TrcContext; const verts: PSingle; const nv: Integer;
              const tris: PInteger; const areas: PByte; const nt: Integer;
              var solid: TrcHeightfield; const flagMergeThr: Integer = 1);
var ics,ich: Single; i: Integer; v0,v1,v2: PSingle;
begin
  //rcAssert(ctx);

  ctx.startTimer(RC_TIMER_RASTERIZE_TRIANGLES);

  ics := 1.0/solid.cs;
  ich := 1.0/solid.ch;
  // Rasterize triangles.
  for i := 0 to nt - 1 do
  begin
    v0 := @verts[tris[i*3+0]*3];
    v1 := @verts[tris[i*3+1]*3];
    v2 := @verts[tris[i*3+2]*3];
    // Rasterize.
    rasterizeTri(v0, v1, v2, areas[i], solid, @solid.bmin, @solid.bmax, solid.cs, ics, ich, flagMergeThr);
  end;

  ctx.stopTimer(RC_TIMER_RASTERIZE_TRIANGLES);
end;

/// @par
///
/// Spans will only be added for triangles that overlap the heightfield grid.
///
/// @see rcHeightfield
procedure rcRasterizeTriangles(ctx: TrcContext; const verts: PSingle; const nv: Integer;
              const tris: PWord; const areas: PByte; const nt: Integer;
              var solid: TrcHeightfield; const flagMergeThr: Integer = 1); overload;
var ics,ich: Single; i: Integer; v0,v1,v2: PSingle;
begin
  //rcAssert(ctx);

  ctx.startTimer(RC_TIMER_RASTERIZE_TRIANGLES);

  ics := 1.0/solid.cs;
  ich := 1.0/solid.ch;
  // Rasterize triangles.
  for i := 0 to nt - 1 do
  begin
    v0 := @verts[tris[i*3+0]*3];
    v1 := @verts[tris[i*3+1]*3];
    v2 := @verts[tris[i*3+2]*3];
    // Rasterize.
    rasterizeTri(v0, v1, v2, areas[i], solid, @solid.bmin, @solid.bmax, solid.cs, ics, ich, flagMergeThr);
  end;

  ctx.stopTimer(RC_TIMER_RASTERIZE_TRIANGLES);
end;

/// @par
///
/// Spans will only be added for triangles that overlap the heightfield grid.
///
/// @see rcHeightfield
procedure rcRasterizeTriangles(ctx: TrcContext; const verts: PSingle; const areas: PByte; const nt: Integer;
              var solid: TrcHeightfield; const flagMergeThr: Integer = 1); overload;
var ics,ich: Single; i: Integer; v0,v1,v2: PSingle;
begin
  //rcAssert(ctx);

  ctx.startTimer(RC_TIMER_RASTERIZE_TRIANGLES);

  ics := 1.0/solid.cs;
  ich := 1.0/solid.ch;
  // Rasterize triangles.
  for i := 0 to nt - 1 do
  begin
    v0 := @verts[(i*3+0)*3];
    v1 := @verts[(i*3+1)*3];
    v2 := @verts[(i*3+2)*3];
    // Rasterize.
    rasterizeTri(v0, v1, v2, areas[i], solid, @solid.bmin, @solid.bmax, solid.cs, ics, ich, flagMergeThr);
  end;

  ctx.stopTimer(RC_TIMER_RASTERIZE_TRIANGLES);
end;

end.
