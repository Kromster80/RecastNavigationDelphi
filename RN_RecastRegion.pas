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

unit RN_RecastRegion;
interface
uses
  Math, SysUtils, System.Generics.Collections,
  RN_Helper, RN_Recast;


/// Builds the distance field for the specified compact heightfield.
///  @ingroup recast
///  @param[in,out]  ctx    The build context to use during the operation.
///  @param[in,out]  chf    A populated compact heightfield.
///  @returns True if the operation completed successfully.
function rcBuildDistanceField(ctx: TrcContext; chf: PrcCompactHeightfield): Boolean;

/// Builds region data for the heightfield using watershed partitioning.
///  @ingroup recast
///  @param[in,out]  ctx        The build context to use during the operation.
///  @param[in,out]  chf        A populated compact heightfield.
///  @param[in]    borderSize    The size of the non-navigable border around the heightfield.
///                  [Limit: >=0] [Units: vx]
///  @param[in]    minRegionArea  The minimum number of cells allowed to form isolated island areas.
///                  [Limit: >=0] [Units: vx].
///  @param[in]    mergeRegionArea    Any regions with a span count smaller than this value will, if possible,
///                  be merged with larger regions. [Limit: >=0] [Units: vx]
///  @returns True if the operation completed successfully.
function rcBuildRegions(ctx: TrcContext; chf: PrcCompactHeightfield;
          const borderSize,  minRegionArea, mergeRegionArea: Integer): Boolean;

/// Builds region data for the heightfield by partitioning the heightfield in non-overlapping layers.
///  @ingroup recast
///  @param[in,out]  ctx        The build context to use during the operation.
///  @param[in,out]  chf        A populated compact heightfield.
///  @param[in]    borderSize    The size of the non-navigable border around the heightfield.
///                  [Limit: >=0] [Units: vx]
///  @param[in]    minRegionArea  The minimum number of cells allowed to form isolated island areas.
///                  [Limit: >=0] [Units: vx].
///  @returns True if the operation completed successfully.
{function rcBuildLayerRegions(ctx: TrcContext; chf: PrcCompactHeightfield;
             const borderSize, minRegionArea: Integer): Boolean;}

/// Builds region data for the heightfield using simple monotone partitioning.
///  @ingroup recast
///  @param[in,out]  ctx        The build context to use during the operation.
///  @param[in,out]  chf        A populated compact heightfield.
///  @param[in]    borderSize    The size of the non-navigable border around the heightfield.
///                  [Limit: >=0] [Units: vx]
///  @param[in]    minRegionArea  The minimum number of cells allowed to form isolated island areas.
///                  [Limit: >=0] [Units: vx].
///  @param[in]    mergeRegionArea  Any regions with a span count smaller than this value will, if possible,
///                  be merged with larger regions. [Limit: >=0] [Units: vx]
///  @returns True if the operation completed successfully.
{function rcBuildRegionsMonotone(ctx: TrcContext; chf: PrcCompactHeightfield;
              const borderSize, minRegionArea, mergeRegionArea: Integer): Boolean;}

implementation
uses RN_RecastAlloc, RN_RecastHelper;

procedure calculateDistanceField(const chf: PrcCompactHeightfield; src: PWord; out maxDist: Word);
var w,h,i,x,y: Integer; c: PrcCompactCell; s,as1: PrcCompactSpan; area: Byte; nc,dir,ax,ay,ai,aax,aay,aai: Integer;
begin
  w := chf.width;
  h := chf.height;

  // Init distance and points.
  for i := 0 to chf.spanCount - 1 do
    src[i] := $ffff;

  // Mark boundary cells.
  for y := 0 to h - 1 do
  begin
    for x := 0 to w - 1 do
    begin
      c := @chf.cells[x+y*w];
      for i := c.index to Integer(c.index+c.count) - 1 do
      begin
        s := @chf.spans[i];
        area := chf.areas[i];

        nc := 0;
        for dir := 0 to 3 do
        begin
          if (rcGetCon(s, dir) <> RC_NOT_CONNECTED) then
          begin
            ax := x + rcGetDirOffsetX(dir);
            ay := y + rcGetDirOffsetY(dir);
            ai := chf.cells[ax+ay*w].index + rcGetCon(s, dir);
            if (area = chf.areas[ai]) then
              Inc(nc);
          end;
        end;
        if (nc <> 4) then
          src[i] := 0;
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
          if (src[ai]+2 < src[i]) then
            src[i] := src[ai]+2;

          // (-1,-1)
          if (rcGetCon(as1, 3) <> RC_NOT_CONNECTED) then
          begin
            aax := ax + rcGetDirOffsetX(3);
            aay := ay + rcGetDirOffsetY(3);
            aai := chf.cells[aax+aay*w].index + rcGetCon(as1, 3);
            if (src[aai]+3 < src[i]) then
              src[i] := src[aai]+3;
          end;
        end;
        if (rcGetCon(s, 3) <> RC_NOT_CONNECTED) then
        begin
          // (0,-1)
          ax := x + rcGetDirOffsetX(3);
          ay := y + rcGetDirOffsetY(3);
          ai := chf.cells[ax+ay*w].index + rcGetCon(s, 3);
          as1 := @chf.spans[ai];
          if (src[ai]+2 < src[i]) then
            src[i] := src[ai]+2;

          // (1,-1)
          if (rcGetCon(as1, 2) <> RC_NOT_CONNECTED) then
          begin
            aax := ax + rcGetDirOffsetX(2);
            aay := ay + rcGetDirOffsetY(2);
            aai := chf.cells[aax+aay*w].index + rcGetCon(as1, 2);
            if (src[aai]+3 < src[i]) then
              src[i] := src[aai]+3;
          end;
        end;
      end;
    end;
  end;

  // Pass 2
  for y := h-1 downto 0 do
  begin
    for x := w-1 downto 0 do
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
          if (src[ai]+2 < src[i]) then
            src[i] := src[ai]+2;

          // (1,1)
          if (rcGetCon(as1, 1) <> RC_NOT_CONNECTED) then
          begin
            aax := ax + rcGetDirOffsetX(1);
            aay := ay + rcGetDirOffsetY(1);
            aai := chf.cells[aax+aay*w].index + rcGetCon(as1, 1);
            if (src[aai]+3 < src[i]) then
              src[i] := src[aai]+3;
          end;
        end;
        if (rcGetCon(s, 1) <> RC_NOT_CONNECTED) then
        begin
          // (0,1)
          ax := x + rcGetDirOffsetX(1);
          ay := y + rcGetDirOffsetY(1);
          ai := chf.cells[ax+ay*w].index + rcGetCon(s, 1);
          as1 := @chf.spans[ai];
          if (src[ai]+2 < src[i]) then
            src[i] := src[ai]+2;

          // (-1,1)
          if (rcGetCon(as1, 0) <> RC_NOT_CONNECTED) then
          begin
            aax := ax + rcGetDirOffsetX(0);
            aay := ay + rcGetDirOffsetY(0);
            aai := chf.cells[aax+aay*w].index + rcGetCon(as1, 0);
            if (src[aai]+3 < src[i]) then
              src[i] := src[aai]+3;
          end;
        end;
      end;
    end;
  end;

  maxDist := 0;
  for i := 0 to chf.spanCount - 1 do
    maxDist := rcMax(src[i], maxDist);

end;

function boxBlur(chf: PrcCompactHeightfield; thr: Integer;
                 src,dst: PWord): PWord;
var w,h,i,x,y: Integer; c: PrcCompactCell; s,as1: PrcCompactSpan; cd: Word; dir,dir2: Integer; d,ax,ay,ai,ax2,ay2,ai2: Integer;
begin
  w := chf.width;
  h := chf.height;

  thr := thr*2;

  for y := 0 to h - 1 do
  begin
    for x := 0 to w - 1 do
    begin
      c := @chf.cells[x+y*w];
      for i := c.index to Integer(c.index+c.count) - 1 do
      begin
        s := @chf.spans[i];
        cd := src[i];
        if (cd <= thr) then
        begin
          dst[i] := cd;
          continue;
        end;

        d := cd;
        for dir := 0 to 3 do
        begin
          if (rcGetCon(s, dir) <> RC_NOT_CONNECTED) then
          begin
            ax := x + rcGetDirOffsetX(dir);
            ay := y + rcGetDirOffsetY(dir);
            ai := chf.cells[ax+ay*w].index + rcGetCon(s, dir);
            Inc(d, src[ai]);

            as1 := @chf.spans[ai];
            dir2 := (dir+1) and $3;
            if (rcGetCon(as1, dir2) <> RC_NOT_CONNECTED) then
            begin
              ax2 := ax + rcGetDirOffsetX(dir2);
              ay2 := ay + rcGetDirOffsetY(dir2);
              ai2 := chf.cells[ax2+ay2*w].index + rcGetCon(as1, dir2);
              Inc(d, src[ai2]);
            end
            else
            begin
              d := d + cd;
            end;
          end
          else
          begin
            d := d + cd*2;
          end;
        end;
        dst[i] := Trunc((d+5)/9);
      end;
    end;
  end;
  Result := dst;
end;


function floodRegion(x, y, i: Integer;
            level,r: Word;
            chf: PrcCompactHeightfield;
            srcReg, srcDist: PWord;
            stack: PrcIntArray): Boolean;
var w,dir: Integer; area: Byte; cs,as1: PrcCompactSpan; lev,ar,nr: Word; count,ci,cx,cy,ax,ay,ai,ax2,ay2,ai2,dir2,nr2: Integer;
begin
  w := chf.width;

  area := chf.areas[i];

  // Flood fill mark region.
  stack.resize(0);
  stack.push(x);
  stack.push(y);
  stack.push(i);
  srcReg[i] := r;
  srcDist[i] := 0;

  if level >= 2 then lev := level-2 else lev := 0;
  count := 0;

  while (stack.size > 0) do
  begin
    ci := stack.pop();
    cy := stack.pop();
    cx := stack.pop();

    cs := @chf.spans[ci];

    // Check if any of the neighbours already have a valid region set.
    ar := 0;
    for dir := 0 to 3 do
    begin
      // 8 connected
      if (rcGetCon(cs, dir) <> RC_NOT_CONNECTED) then
      begin
        ax := cx + rcGetDirOffsetX(dir);
        ay := cy + rcGetDirOffsetY(dir);
        ai := chf.cells[ax+ay*w].index + rcGetCon(cs, dir);
        if (chf.areas[ai] <> area) then
          continue;
        nr := srcReg[ai];
        if (nr and RC_BORDER_REG) <> 0 then// Do not take borders into account.
          continue;
        if (nr <> 0) and (nr <> r) then
        begin
          ar := nr;
          break;
        end;

        as1 := @chf.spans[ai];

        dir2 := (dir+1) and $3;
        if (rcGetCon(as1, dir2) <> RC_NOT_CONNECTED) then
        begin
          ax2 := ax + rcGetDirOffsetX(dir2);
          ay2 := ay + rcGetDirOffsetY(dir2);
          ai2 := chf.cells[ax2+ay2*w].index + rcGetCon(as1, dir2);
          if (chf.areas[ai2] <> area) then
            continue;
          nr2 := srcReg[ai2];
          if (nr2 <> 0) and (nr2 <> r) then
          begin
            ar := nr2;
            break;
          end;
        end;
      end;
    end;

    if (ar <> 0) then
    begin
      srcReg[ci] := 0;
      //C++ seems to be doing loop increase, so do we
      continue;
    end;

    Inc(count);

    // Expand neighbours.
    for dir := 0 to 3 do
    begin
      if (rcGetCon(cs, dir) <> RC_NOT_CONNECTED) then
      begin
        ax := cx + rcGetDirOffsetX(dir);
        ay := cy + rcGetDirOffsetY(dir);
        ai := chf.cells[ax+ay*w].index + rcGetCon(cs, dir);
        if (chf.areas[ai] <> area) then
          continue;
        if (chf.dist[ai] >= lev) and (srcReg[ai] = 0) then
        begin
          srcReg[ai] := r;
          srcDist[ai] := 0;
          stack.push(ax);
          stack.push(ay);
          stack.push(ai);
        end;
      end;
    end;
  end;

  Result := count > 0;
end;

function expandRegions(maxIter: Integer; level: Word;
                   chf: PrcCompactHeightfield;
                   srcReg, srcDist,
                   dstReg, dstDist: PWord;
                   stack: PrcIntArray;
                   fillStack: Boolean): PWord;
var w,h,i,j,x,y: Integer; c: PrcCompactCell; s: PrcCompactSpan; iter,failed,dir: Integer; r,d2: Word; area: Byte; ax,ay,ai: Integer;
begin
  w := chf.width;
  h := chf.height;

  if (fillStack) then
  begin
    // Find cells revealed by the raised level.
    stack.resize(0);
    for y := 0 to h - 1 do
    begin
      for x := 0 to w - 1 do
      begin
        c := @chf.cells[x+y*w];
        for i := c.index to Integer(c.index+c.count) - 1 do
        begin
          if (chf.dist[i] >= level) and (srcReg[i] = 0) and (chf.areas[i] <> RC_NULL_AREA) then
          begin
            stack.push(x);
            stack.push(y);
            stack.push(i);
          end;
        end;
      end;
    end;
  end
  else // use cells in the input stack
  begin
    // mark all cells which already have a region
    //for (int j=0; j<stack.size(); j+=3)
    for j := 0 to stack.size div 3 - 1 do
    begin
      i := stack^[j*3+2];
      if (srcReg[i] <> 0) then
        stack^[j*3+2] := -1;
    end;
  end;

  iter := 0;
  while (stack.size > 0) do
  begin
    failed := 0;

    Move(srcReg^, dstReg^, sizeof(Word)*chf.spanCount);
    Move(srcDist^, dstDist^, sizeof(Word)*chf.spanCount);

    //for (int j = 0; j < stack.size(); j += 3)
    for j := 0 to stack.size div 3 - 1 do
    begin
      x := stack^[j*3+0];
      y := stack^[j*3+1];
      i := stack^[j*3+2];
      if (i < 0) then
      begin
        Inc(failed);
        continue;
      end;

      r := srcReg[i];
      d2 := $ffff;
      area := chf.areas[i];
      s := @chf.spans[i];
      for dir := 0 to 3 do
      begin
        if (rcGetCon(s, dir) = RC_NOT_CONNECTED) then continue;
        ax := x + rcGetDirOffsetX(dir);
        ay := y + rcGetDirOffsetY(dir);
        ai := chf.cells[ax+ay*w].index + rcGetCon(s, dir);
        if (chf.areas[ai] <> area) then continue;
        if (srcReg[ai] > 0) and ((srcReg[ai] and RC_BORDER_REG) = 0) then
        begin
          if (srcDist[ai]+2 < d2) then
          begin
            r := srcReg[ai];
            d2 := srcDist[ai]+2;
          end;
        end;
      end;
      if (r <> 0) then
      begin
        stack^[j*3+2] := -1; // mark as used
        dstReg[i] := r;
        dstDist[i] := d2;
      end
      else
      begin
        Inc(failed);
      end;
    end;

    // rcSwap source and dest.
    rcSwap(Pointer(srcReg), Pointer(dstReg));
    rcSwap(Pointer(srcDist), Pointer(dstDist));

    if (failed*3 = stack.size) then
      break;

    if (level > 0) then
    begin
      Inc(iter);
      if (iter >= maxIter) then
        break;
    end;
  end;

  Result := srcReg;
end;


procedure sortCellsByLevel(startLevel: Word;
                chf: PrcCompactHeightfield;
                srcReg: PWord;
                nbStacks: Integer; const stacks: TArrayOfTrcIntArray;
                loglevelsPerStack: Word); // the levels per stack (2 in our case) as a bit shift
var w,h,i,j,x,y: Integer; c: PrcCompactCell; level,sId: Integer;
begin
  w := chf.width;
  h := chf.height;
  startLevel := startLevel shr loglevelsPerStack;

  for j := 0 to nbStacks - 1 do
    stacks[j].resize(0);

  // put all cells in the level range into the appropriate stacks
  for y := 0 to h - 1 do
  begin
    for x := 0 to w - 1 do
    begin
      c := @chf.cells[x+y*w];
      for i := c.index to Integer(c.index+c.count) - 1 do
      begin
        if (chf.areas[i] = RC_NULL_AREA) or (srcReg[i] <> 0) then
          continue;

        level := chf.dist[i] shr loglevelsPerStack;
        sId := startLevel - level;
        if (sId >= nbStacks) then
          continue;
        if (sId < 0) then
          sId := 0;

        stacks[sId].push(x);
        stacks[sId].push(y);
        stacks[sId].push(i);
      end;
    end;
  end;
end;


procedure appendStacks(srcStack, dstStack: PrcIntArray;
             srcReg: PWord);
var i,j: Integer;
begin
  for j := 0 to srcStack.size div 3 - 1 do
  begin
    i := srcStack^[j*3+2];
    if ((i < 0) or (srcReg[i] <> 0)) then
      continue;
    dstStack.push(srcStack^[j*3]);
    dstStack.push(srcStack^[j*3+1]);
    dstStack.push(srcStack^[j*3+2]);
  end;
end;

type
  TrcRegion = record
    spanCount: Integer;          // Number of spans belonging to this region
    id: Word;        // ID of the region
    areaType: Byte;      // Are type.
    remap: Boolean;
    visited: Boolean;
    overlap: Boolean;
    connectsToBorder: Boolean;
    ymin, ymax: Word;
    connections: TrcIntArray;
    floors: TrcIntArray;
  end;
  PrcRegion = ^TrcRegion;

function rcRegion(i: Word): TrcRegion;
begin
  with Result do
  begin
    spanCount := 0;
    id := i;
    areaType := 0;
    remap := false;
    visited := false;
    overlap := false;
    connectsToBorder := false;
    ymin := $ffff;
    ymax := 0;

    connections.Create(0);
    floors.Create(0);
  end;
end;

// Delphi: Manually dispose of allocated memory within TrcIntArray;
procedure rcRegionFree(r: TrcRegion);
begin
  r.connections.Free;
  r.floors.Free;
end;

procedure removeAdjacentNeighbours(reg: PrcRegion);
var i,j,ni: Integer;
begin
  // Remove adjacent duplicates.
  //for (int i := 0; i < reg.connections.size() and reg.connections.size() > 1; )
  i := 0;
  while(i < reg.connections.size) and (reg.connections.size > 1) do
  begin
    ni := (i+1) mod reg.connections.size;
    if (reg.connections[i] = reg.connections[ni]) then
    begin
      // Remove duplicate
      for j := i to reg.connections.size-1-1 do
        reg.connections[j] := reg.connections[j+1];
      reg.connections.pop();
    end
    else
      Inc(i);
  end;
end;

procedure replaceNeighbour(reg: PrcRegion; oldId, newId: Word);
var neiChanged: Boolean; i: Integer;
begin
  neiChanged := false;
  for i := 0 to reg.connections.size - 1 do
  begin
    if (reg.connections[i] = oldId) then
    begin
      reg.connections[i] := newId;
      neiChanged := true;
    end;
  end;
  for i := 0 to reg.floors.size - 1 do
  begin
    if (reg.floors[i] = oldId) then
      reg.floors[i] := newId;
  end;
  if (neiChanged) then
    removeAdjacentNeighbours(reg);
end;

function canMergeWithRegion(const rega, regb: PrcRegion): Boolean;
var n,i: Integer;
begin
  if (rega.areaType <> regb.areaType) then
    Exit(false);

  n := 0;
  for i := 0 to rega.connections.size - 1 do
  begin
    if (rega.connections[i] = regb.id) then
      Inc(n);
  end;

  if (n > 1) then
    Exit(false);

  for i := 0 to rega.floors.size - 1 do
  begin
    if (rega.floors[i] = regb.id) then
      Exit(false);
  end;
  Result := true;
end;

procedure addUniqueFloorRegion(reg: PrcRegion; n: Integer);
var i: Integer;
begin
  for i := 0 to reg.floors.size - 1 do
    if (reg.floors[i] = n) then
      Exit;
  reg.floors.push(n);
end;

function mergeRegions(rega, regb: PrcRegion): Boolean;
var aid,bid: Word; acon: TrcIntArray; bcon: PrcIntArray; i,j,ni,insa,insb: Integer;
begin
  aid := rega.id;
  bid := regb.id;

  // Duplicate current neighbourhood.
  acon.create(rega.connections.size);
  for i := 0 to rega.connections.size - 1 do
    acon[i] := rega.connections[i];
  bcon := @regb.connections;

  // Find insertion point on A.
  insa := -1;
  for i := 0 to acon.size - 1 do
  begin
    if (acon[i] = bid) then
    begin
      insa := i;
      break;
    end;
  end;
  if (insa = -1) then
    Exit(false);

  // Find insertion point on B.
  insb := -1;
  for i := 0 to bcon.size - 1 do
  begin
    if (bcon^[i] = aid) then
    begin
      insb := i;
      break;
    end;
  end;
  if (insb = -1) then
    Exit(false);

  // Merge neighbours.
  rega.connections.resize(0);
  //for (int i := 0, ni := acon.size(); i < ni-1; ++i)
  i := 0;
  ni := acon.size;
  while (i < ni-1) do
  begin
    rega.connections.push(acon[(insa+1+i) mod ni]);
    Inc(i);
  end;

  //for ( int i := 0, ni := bcon.size(); i < ni-1; ++i)
  i := 0;
  ni := bcon.size;
  while (i < ni-1) do
  begin
    rega.connections.push(bcon^[(insb+1+i) mod ni]);
    Inc(i);
  end;

  removeAdjacentNeighbours(rega);

  for j := 0 to regb.floors.size - 1 do
    addUniqueFloorRegion(rega, regb.floors[j]);
  rega.spanCount := rega.spanCount + regb.spanCount;
  regb.spanCount := 0;
  regb.connections.resize(0);

  // Delphi: Manually release record and buffer it holds within
  acon.Free;

  Result := true;
end;

function isRegionConnectedToBorder(reg: PrcRegion): Boolean;
var i: Integer;
begin
  // Region is connected to border if
  // one of the neighbours is null id.
  for i := 0 to reg.connections.size - 1 do
  begin
    if (reg.connections[i] = 0) then
      Exit(true);
  end;
  Result := false;
end;

function isSolidEdge(chf: PrcCompactHeightfield; srcReg: PWord;
            x,y,i,dir: Integer): Boolean;
var s: PrcCompactSpan; r: Word; ax,ay,ai: Integer;
begin
  s := @chf.spans[i];
  r := 0;
  if (rcGetCon(s, dir) <> RC_NOT_CONNECTED) then
  begin
    ax := x + rcGetDirOffsetX(dir);
    ay := y + rcGetDirOffsetY(dir);
    ai := chf.cells[ax+ay*chf.width].index + rcGetCon(s, dir);
    r := srcReg[ai];
  end;
  if (r = srcReg[i]) then
    Exit(false);

  Result := true;
end;

procedure walkContour(x, y, i, dir: Integer;
            chf: PrcCompactHeightfield;
            srcReg: PWord;
            cont: PrcIntArray);
var startDir,starti: Integer; s,ss: PrcCompactSpan; curReg,r: Word; ax,ay,ai: Integer; iter: Integer; ni,nx,ny: Integer;
nc: PrcCompactCell; j,nj,k: Integer;
begin
  startDir := dir;
  starti := i;

  ss := @chf.spans[i];
  curReg := 0;
  if (rcGetCon(ss, dir) <> RC_NOT_CONNECTED) then
  begin
    ax := x + rcGetDirOffsetX(dir);
    ay := y + rcGetDirOffsetY(dir);
    ai := chf.cells[ax+ay*chf.width].index + rcGetCon(ss, dir);
    curReg := srcReg[ai];
  end;
  cont.push(curReg);

  iter := 0;
  while (iter < 40000) do
  begin
    Inc(iter);
    s := @chf.spans[i];

    if (isSolidEdge(chf, srcReg, x, y, i, dir)) then
    begin
      // Choose the edge corner
      r := 0;
      if (rcGetCon(s, dir) <> RC_NOT_CONNECTED) then
      begin
        ax := x + rcGetDirOffsetX(dir);
        ay := y + rcGetDirOffsetY(dir);
        ai := chf.cells[ax+ay*chf.width].index + rcGetCon(s, dir);
        r := srcReg[ai];
      end;
      if (r <> curReg) then
      begin
        curReg := r;
        cont.push(curReg);
      end;

      dir := (dir+1) and $3;  // Rotate CW
    end
    else
    begin
      ni := -1;
      nx := x + rcGetDirOffsetX(dir);
      ny := y + rcGetDirOffsetY(dir);
      if (rcGetCon(s, dir) <> RC_NOT_CONNECTED) then
      begin
        nc := @chf.cells[nx+ny*chf.width];
        ni := nc.index + rcGetCon(s, dir);
      end;
      if (ni = -1) then
      begin
        // Should not happen.
        Exit;
      end;
      x := nx;
      y := ny;
      i := ni;
      dir := (dir+3) and $3;  // Rotate CCW
    end;

    if (starti = i) and (startDir = dir) then
    begin
      break;
    end;
  end;

  // Remove adjacent duplicates.
  if (cont.size > 1) then
  begin
    //for (int j = 0; j < cont.size(); )
    j := 0;
    while (j < cont.size) do
    begin
      nj := (j+1) mod cont.size;
      if (cont^[j] = cont^[nj]) then
      begin
        for k := j to cont.size-1 - 1 do
          cont^[k] := cont^[k+1];
        cont.pop();
      end
      else
        Inc(j);
    end;
  end;
end;


function mergeAndFilterRegions(ctx: TrcContext; minRegionArea, mergeRegionSize: Integer;
                  maxRegionId: PWord;
                  chf: PrcCompactHeightfield;
                  srcReg: PWord; overlaps: PrcIntArray): Boolean;
var w,h,nreg,i,j,ni,ndir,dir,x,y: Integer; regions: PrcRegion; c: PrcCompactCell; r: Word; reg,creg,neireg,mreg,target: PrcRegion;
floorId: Word; stack,trace: TrcIntArray; connectsToBorder: Boolean; spanCount,ri: Integer; mergeCount,smallest: Integer;
mergeId, oldId, regIdGen, newId: Word;
begin
  w := chf.width;
  h := chf.height;

  nreg := maxRegionId^+1;
  GetMem(regions, sizeof(TrcRegion)*nreg);

  // Construct regions
  for i := 0 to nreg - 1 do
    regions[i] := rcRegion(i);

  // Find edge of a region and find connections around the contour.
  for y := 0 to h - 1 do
  begin
    for x := 0 to w - 1 do
    begin
      c := @chf.cells[x+y*w];
      ni := (c.index+c.count);
      for i := c.index to ni - 1 do
      begin
        r := srcReg[i];
        if (r = 0) or (r >= nreg) then
          continue;

        reg := @regions[r];
        Inc(reg.spanCount);

        // Update floors.
        for j := c.index to ni - 1 do
        begin
          if (i = j) then continue;
          floorId := srcReg[j];
          if (floorId = 0) or (floorId >= nreg) then
            continue;
          if (floorId = r) then
            reg.overlap := true;
          addUniqueFloorRegion(reg, floorId);
        end;

        // Have found contour
        if (reg.connections.size > 0) then
          continue;

        reg.areaType := chf.areas[i];

        // Check if this cell is next to a border.
        ndir := -1;
        for dir := 0 to 3 do
        begin
          if (isSolidEdge(chf, srcReg, x, y, i, dir)) then
          begin
            ndir := dir;
            break;
          end;
        end;

        if (ndir <> -1) then
        begin
          // The cell is at border.
          // Walk around the contour to find all the neighbours.
          walkContour(x, y, i, ndir, chf, srcReg, @reg.connections);
        end;
      end;
    end;
  end;

  // Remove too small regions.
  stack.Create(32);
  trace.Create(32);
  for i := 0 to nreg - 1 do
  begin
    reg := @regions[i];
    if (reg.id = 0) or (reg.id and RC_BORDER_REG <> 0) then
      continue;
    if (reg.spanCount = 0) then
      continue;
    if (reg.visited) then
      continue;

    // Count the total size of all the connected regions.
    // Also keep track of the regions connects to a tile border.
    connectsToBorder := false;
    spanCount := 0;
    stack.resize(0);
    trace.resize(0);

    reg.visited := true;
    stack.push(i);

    while (stack.size <> 0) do
    begin
      // Pop
      ri := stack.pop();

      creg := @regions[ri];

      Inc(spanCount, creg.spanCount);
      trace.push(ri);

      for j := 0 to creg.connections.size - 1 do
      begin
        if (creg.connections[j] and RC_BORDER_REG) <> 0 then
        begin
          connectsToBorder := true;
          continue;
        end;
        neireg := @regions[creg.connections[j]];
        if (neireg.visited) then
          continue;
        if (neireg.id = 0) or (neireg.id and RC_BORDER_REG <> 0) then
          continue;
        // Visit
        stack.push(neireg.id);
        neireg.visited := true;
      end;
    end;

    // If the accumulated regions size is too small, remove it.
    // Do not remove areas which connect to tile borders
    // as their size cannot be estimated correctly and removing them
    // can potentially remove necessary areas.
    if (spanCount < minRegionArea) and (not connectsToBorder) then
    begin
      // Kill all visited regions.
      for j := 0 to trace.size - 1 do
      begin
        regions[trace[j]].spanCount := 0;
        regions[trace[j]].id := 0;
      end;
    end;
  end;

  // Delphi: Manually release record and buffer it holds within
  stack.Free;
  trace.Free;

  // Merge too small regions to neighbour regions.
  mergeCount := 0;
  repeat
    mergeCount := 0;
    for i := 0 to nreg - 1 do
    begin
      reg := @regions[i];
      if (reg.id = 0) or (reg.id and RC_BORDER_REG <> 0) then
        continue;
      if (reg.overlap) then
        continue;
      if (reg.spanCount = 0) then
        continue;

      // Check to see if the region should be merged.
      if (reg.spanCount > mergeRegionSize) and isRegionConnectedToBorder(reg) then
        continue;

      // Small region with more than 1 connection.
      // Or region which is not connected to a border at all.
      // Find smallest neighbour region that connects to this one.
      smallest := $fffffff;
      mergeId := reg.id;
      for j := 0 to reg.connections.size - 1 do
      begin
        if (reg.connections[j] and RC_BORDER_REG <> 0) then continue;
        mreg := @regions[reg.connections[j]];
        if (mreg.id = 0) or (mreg.id and RC_BORDER_REG <> 0) or (mreg.overlap) then continue;
        if (mreg.spanCount < smallest) and
          canMergeWithRegion(reg, mreg) and
          canMergeWithRegion(mreg, reg) then
        begin
          smallest := mreg.spanCount;
          mergeId := mreg.id;
        end;
      end;
      // Found new id.
      if (mergeId <> reg.id) then
      begin
        oldId := reg.id;
        target := @regions[mergeId];

        // Merge neighbours.
        if (mergeRegions(target, reg)) then
        begin
          // Fixup regions pointing to current region.
          for j := 0 to nreg - 1 do
          begin
            if (regions[j].id = 0) or (regions[j].id and RC_BORDER_REG <> 0) then continue;
            // If another region was already merged into current region
            // change the nid of the previous region too.
            if (regions[j].id = oldId) then
              regions[j].id := mergeId;
            // Replace the current region with the new one if the
            // current regions is neighbour.
            replaceNeighbour(@regions[j], oldId, mergeId);
          end;
          Inc(mergeCount);
        end;
      end;
    end;
  until (mergeCount <= 0);

  // Compress region Ids.
  for i := 0 to nreg - 1 do
  begin
    regions[i].remap := false;
    if (regions[i].id = 0) then continue;       // Skip nil regions.
    if (regions[i].id and RC_BORDER_REG <> 0) then continue;    // Skip external regions.
    regions[i].remap := true;
  end;

  regIdGen := 0;
  for i := 0 to nreg - 1 do
  begin
    if (not regions[i].remap) then
      continue;
    oldId := regions[i].id;
    Inc(regIdGen);
    newId := regIdGen;
    for j := i to nreg - 1 do
    begin
      if (regions[j].id = oldId) then
      begin
        regions[j].id := newId;
        regions[j].remap := false;
      end;
    end;
  end;
  maxRegionId^ := regIdGen;

  // Remap regions.
  for i := 0 to chf.spanCount - 1 do
  begin
    if ((srcReg[i] and RC_BORDER_REG) = 0) then
      srcReg[i] := regions[srcReg[i]].id;
  end;

  // Return regions that we found to be overlapping.
  for i := 0 to nreg - 1 do
    if (regions[i].overlap) then
      overlaps.push(regions[i].id);

  for i := 0 to nreg - 1 do
    rcRegionFree(regions[i]);
  FreeMem(regions);

  Result := true;
end;


procedure addUniqueConnection(reg: PrcRegion; n: Integer);
var i: Integer;
begin
  for i := 0 to reg.connections.size - 1 do
    if (reg.connections[i] = n) then
      Exit;
  reg.connections.push(n);
end;

{function mergeAndFilterLayerRegions(ctx: TrcContext; int minRegionArea,
                     unsigned short& maxRegionId,
                     chf: TrcCompactHeightfield;,
                     unsigned short* srcReg, rcIntArray& overlaps): Boolean;
var w,h,i,x,y: Integer; c: PrcCompactCell; s: PrcCompactSpan;
begin
  w := chf.width;
  h := chf.height;

  const int nreg := maxRegionId+1;
  rcRegion* regions := (rcRegion*)rcAlloc(sizeof(rcRegion)*nreg, RC_ALLOC_TEMP);
  if (!regions)
  begin
    ctx.log(RC_LOG_ERROR, 'mergeAndFilterLayerRegions: Out of memory 'regions' (%d).', nreg);
    return false;
  end;

  // Construct regions
  for (int i := 0; i < nreg; ++i)
    new(&regions[i]) rcRegion((unsigned short)i);

  // Find region neighbours and overlapping regions.
  rcIntArray lregs(32);
  for y := 0 to h - 1 do
  begin
    for x := 0 to w - 1 do
    begin
      c := @chf.cells[x+y*w];

      lregs.resize(0);

      for i := c.index to Integer(c.index+c.count) - 1 do
      begin
        s := @chf.spans[i];
        const unsigned short ri := srcReg[i];
        if (ri = 0 or ri >= nreg) continue;
        rcRegion& reg := regions[ri];

        reg.spanCount++;

        reg.ymin := rcMin(reg.ymin, s.y);
        reg.ymax := rcMax(reg.ymax, s.y);

        // Collect all region layers.
        lregs.push(ri);

        // Update neighbours
        for dir := 0 to 3 do
        begin
          if (rcGetCon(s, dir) <> RC_NOT_CONNECTED)
          begin
            const int ax := x + rcGetDirOffsetX(dir);
            const int ay := y + rcGetDirOffsetY(dir);
            const int ai := chf.cells[ax+ay*w].index + rcGetCon(s, dir);
            const unsigned short rai := srcReg[ai];
            if (rai > 0 and rai < nreg and rai <> ri)
              addUniqueConnection(reg, rai);
            if (rai & RC_BORDER_REG)
              reg.connectsToBorder := true;
          end;
        end;

      end;

      // Update overlapping regions.
      for (int i := 0; i < lregs.size()-1; ++i)
      begin
        for (int j := i+1; j < lregs.size(); ++j)
        begin
          if (lregs[i] <> lregs[j])
          begin
            rcRegion& ri := regions[lregs[i]];
            rcRegion& rj := regions[lregs[j]];
            addUniqueFloorRegion(ri, lregs[j]);
            addUniqueFloorRegion(rj, lregs[i]);
          end;
        end;
      end;

    end;
  end;

  // Create 2D layers from regions.
  unsigned short layerId := 1;

  for (int i := 0; i < nreg; ++i)
    regions[i].id := 0;

  // Merge montone regions to create non-overlapping areas.
  rcIntArray stack(32);
  for (int i := 1; i < nreg; ++i)
  begin
    rcRegion& root := regions[i];
    // Skip already visited.
    if (root.id <> 0)
      continue;

    // Start search.
    root.id := layerId;

    stack.resize(0);
    stack.push(i);

    while (stack.size() > 0)
    begin
      // Pop front
      rcRegion& reg := regions[stack[0]];
      for (int j := 0; j < stack.size()-1; ++j)
        stack[j] := stack[j+1];
      stack.resize(stack.size()-1);

      const int ncons := (int)reg.connections.size();
      for (int j := 0; j < ncons; ++j)
      begin
        const int nei := reg.connections[j];
        rcRegion& regn := regions[nei];
        // Skip already visited.
        if (regn.id <> 0)
          continue;
        // Skip if the neighbour is overlapping root region.
        bool overlap := false;
        for (int k := 0; k < root.floors.size(); k++)
        begin
          if (root.floors[k] = nei)
          begin
            overlap := true;
            break;
          end;
        end;
        if (overlap)
          continue;

        // Deepen
        stack.push(nei);

        // Mark layer id
        regn.id := layerId;
        // Merge current layers to root.
        for (int k := 0; k < regn.floors.size(); ++k)
          addUniqueFloorRegion(root, regn.floors[k]);
        root.ymin := rcMin(root.ymin, regn.ymin);
        root.ymax := rcMax(root.ymax, regn.ymax);
        root.spanCount += regn.spanCount;
        regn.spanCount := 0;
        root.connectsToBorder := root.connectsToBorder or regn.connectsToBorder;
      end;
    end;

    layerId++;
  end;

  // Remove small regions
  for (int i := 0; i < nreg; ++i)
  begin
    if (regions[i].spanCount > 0 and regions[i].spanCount < minRegionArea and !regions[i].connectsToBorder)
    begin
      unsigned short reg := regions[i].id;
      for (int j := 0; j < nreg; ++j)
        if (regions[j].id = reg)
          regions[j].id := 0;
    end;
  end;

  // Compress region Ids.
  for (int i := 0; i < nreg; ++i)
  begin
    regions[i].remap := false;
    if (regions[i].id = 0) continue;        // Skip nil regions.
    if (regions[i].id & RC_BORDER_REG) continue;    // Skip external regions.
    regions[i].remap := true;
  end;

  unsigned short regIdGen := 0;
  for (int i := 0; i < nreg; ++i)
  begin
    if (!regions[i].remap)
      continue;
    unsigned short oldId := regions[i].id;
    unsigned short newId := ++regIdGen;
    for (int j := i; j < nreg; ++j)
    begin
      if (regions[j].id = oldId)
      begin
        regions[j].id := newId;
        regions[j].remap := false;
      end;
    end;
  end;
  maxRegionId := regIdGen;

  // Remap regions.
  for for i := 0 to chf.spanCount - 1 do
  begin
    if ((srcReg[i] & RC_BORDER_REG) = 0)
      srcReg[i] := regions[srcReg[i]].id;
  end;

  for (int i := 0; i < nreg; ++i)
    regions[i].~rcRegion();
  rcFree(regions);

  return true;
end;}



/// @par
///
/// This is usually the second to the last step in creating a fully built
/// compact heightfield.  This step is required before regions are built
/// using #rcBuildRegions or #rcBuildRegionsMonotone.
///
/// After this step, the distance data is available via the rcCompactHeightfield::maxDistance
/// and rcCompactHeightfield::dist fields.
///
/// @see rcCompactHeightfield, rcBuildRegions, rcBuildRegionsMonotone
function rcBuildDistanceField(ctx: TrcContext; chf: PrcCompactHeightfield): Boolean;
var maxDist: Word; src,dst: PWord;
begin
  //rcAssert(ctx);

  ctx.startTimer(RC_TIMER_BUILD_DISTANCEFIELD);

  if (chf.dist <> nil) then
  begin
    FreeMem(chf.dist);
    chf.dist := nil;
  end;

  GetMem(src, SizeOf(Word) * chf.spanCount);
  GetMem(dst, SizeOf(Word) * chf.spanCount);

  maxDist := 0;

  ctx.startTimer(RC_TIMER_BUILD_DISTANCEFIELD_DIST);

  calculateDistanceField(chf, src, maxDist);
  chf.maxDistance := maxDist;

  ctx.stopTimer(RC_TIMER_BUILD_DISTANCEFIELD_DIST);

  ctx.startTimer(RC_TIMER_BUILD_DISTANCEFIELD_BLUR);

  // Blur
  if (boxBlur(chf, 1, src, dst) <> src) then
    rcSwap(Pointer(src), Pointer(dst));

  // Store distance.
  chf.dist := src;

  ctx.stopTimer(RC_TIMER_BUILD_DISTANCEFIELD_BLUR);

  ctx.stopTimer(RC_TIMER_BUILD_DISTANCEFIELD);

  FreeMem(dst);

  Result := true;
end;

procedure paintRectRegion(minx, maxx, miny, maxy: Integer; regId: Word;
              chf: PrcCompactHeightfield; srcReg: PWord);
var w,i,x,y: Integer; c: PrcCompactCell;
begin
  w := chf.width;
  for y := miny to maxy - 1 do
  begin
    for x := minx to maxx - 1 do
    begin
      c := @chf.cells[x+y*w];
      for i := c.index to Integer(c.index+c.count) - 1 do
      begin
        if (chf.areas[i] <> RC_NULL_AREA) then
          srcReg[i] := regId;
      end;
    end;
  end;
end;


const RC_NULL_NEI = $ffff;
type
TcSweepSpan = record
  rid: Word;  // row id
  id: Word;  // region id
  ns: Word;  // number samples
  nei: Word;  // neighbour id
end;

/// @par
///
/// Non-null regions will consist of connected, non-overlapping walkable spans that form a single contour.
/// Contours will form simple polygons.
///
/// If multiple regions form an area that is smaller than @p minRegionArea, then all spans will be
/// re-assigned to the zero (null) region.
///
/// Partitioning can result in smaller than necessary regions. @p mergeRegionArea helps
/// reduce unecessarily small regions.
///
/// See the #rcConfig documentation for more information on the configuration parameters.
///
/// The region data will be available via the rcCompactHeightfield::maxRegions
/// and rcCompactSpan::reg fields.
///
/// @warning The distance field must be created using #rcBuildDistanceField before attempting to build regions.
///
/// @see rcCompactHeightfield, rcCompactSpan, rcBuildDistanceField, rcBuildRegionsMonotone, rcConfig
{function rcBuildRegionsMonotone(ctx: TrcContext; chf: PrcCompactHeightfield;
              const borderSize, minRegionArea, mergeRegionArea: Integer): Boolean;
var w,h,i,x,y: Integer; c: PrcCompactCell; s: PrcCompactSpan;
begin
  //rcAssert(ctx);

  ctx.startTimer(RC_TIMER_BUILD_REGIONS);

  w := chf.width;
  h := chf.height;
  unsigned short id := 1;

  rcScopedDelete<unsigned short> srcReg := (unsigned short*)rcAlloc(sizeof(unsigned short)*chf.spanCount, RC_ALLOC_TEMP);
  if (!srcReg)
  begin
    ctx.log(RC_LOG_ERROR, 'rcBuildRegionsMonotone: Out of memory 'src' (%d).', chf.spanCount);
    return false;
  end;
  memset(srcReg,0,sizeof(unsigned short)*chf.spanCount);

  const int nsweeps := rcMax(chf.width,chf.height);
  rcScopedDelete<rcSweepSpan> sweeps := (rcSweepSpan*)rcAlloc(sizeof(rcSweepSpan)*nsweeps, RC_ALLOC_TEMP);
  if (!sweeps)
  begin
    ctx.log(RC_LOG_ERROR, 'rcBuildRegionsMonotone: Out of memory 'sweeps' (%d).', nsweeps);
    return false;
  end;


  // Mark border regions.
  if (borderSize > 0)
  begin
    // Make sure border will not overflow.
    const int bw := rcMin(w, borderSize);
    const int bh := rcMin(h, borderSize);
    // Paint regions
    paintRectRegion(0, bw, 0, h, id|RC_BORDER_REG, chf, srcReg); id++;
    paintRectRegion(w-bw, w, 0, h, id|RC_BORDER_REG, chf, srcReg); id++;
    paintRectRegion(0, w, 0, bh, id|RC_BORDER_REG, chf, srcReg); id++;
    paintRectRegion(0, w, h-bh, h, id|RC_BORDER_REG, chf, srcReg); id++;

    chf.borderSize := borderSize;
  end;

  rcIntArray prev(256);

  // Sweep one line at a time.
  for (int y := borderSize; y < h-borderSize; ++y)
  begin
    // Collect spans from this row.
    prev.resize(id+1);
    memset(&prev[0],0,sizeof(int)*id);
    unsigned short rid := 1;

    for (int x := borderSize; x < w-borderSize; ++x)
    begin
      c := @chf.cells[x+y*w];

      for i := c.index to Integer(c.index+c.count) - 1 do
      begin
        s := @chf.spans[i];
        if (chf.areas[i] = RC_NULL_AREA) continue;

        // -x
        unsigned short previd := 0;
        if (rcGetCon(s, 0) <> RC_NOT_CONNECTED)
        begin
          const int ax := x + rcGetDirOffsetX(0);
          const int ay := y + rcGetDirOffsetY(0);
          const int ai := chf.cells[ax+ay*w].index + rcGetCon(s, 0);
          if ((srcReg[ai] & RC_BORDER_REG) = 0 and chf.areas[i] = chf.areas[ai])
            previd := srcReg[ai];
        end;

        if (!previd)
        begin
          previd := rid++;
          sweeps[previd].rid := previd;
          sweeps[previd].ns := 0;
          sweeps[previd].nei := 0;
        end;

        // -y
        if (rcGetCon(s,3) <> RC_NOT_CONNECTED)
        begin
          const int ax := x + rcGetDirOffsetX(3);
          const int ay := y + rcGetDirOffsetY(3);
          const int ai := chf.cells[ax+ay*w].index + rcGetCon(s, 3);
          if (srcReg[ai] and (srcReg[ai] & RC_BORDER_REG) = 0 and chf.areas[i] = chf.areas[ai])
          begin
            unsigned short nr := srcReg[ai];
            if (!sweeps[previd].nei or sweeps[previd].nei = nr)
            begin
              sweeps[previd].nei := nr;
              sweeps[previd].ns++;
              prev[nr]++;
            end;
            else
            begin
              sweeps[previd].nei := RC_NULL_NEI;
            end;
          end;
        end;

        srcReg[i] := previd;
      end;
    end;

    // Create unique ID.
    for (int i := 1; i < rid; ++i)
    begin
      if (sweeps[i].nei <> RC_NULL_NEI and sweeps[i].nei <> 0 and
        prev[sweeps[i].nei] = (int)sweeps[i].ns)
      begin
        sweeps[i].id := sweeps[i].nei;
      end;
      else
      begin
        sweeps[i].id := id++;
      end;
    end;

    // Remap IDs
    for (int x := borderSize; x < w-borderSize; ++x)
    begin
      c := @chf.cells[x+y*w];

      for i := c.index to Integer(c.index+c.count) - 1 do
      begin
        if (srcReg[i] > 0 and srcReg[i] < rid)
          srcReg[i] := sweeps[srcReg[i]].id;
      end;
    end;
  end;


  ctx.startTimer(RC_TIMER_BUILD_REGIONS_FILTER);

  // Merge regions and filter out small regions.
  rcIntArray overlaps;
  chf.maxRegions := id;
  if (!mergeAndFilterRegions(ctx, minRegionArea, mergeRegionArea, chf.maxRegions, chf, srcReg, overlaps))
    return false;

  // Monotone partitioning does not generate overlapping regions.

  ctx.stopTimer(RC_TIMER_BUILD_REGIONS_FILTER);

  // Store the result out.
  for for i := 0 to chf.spanCount - 1 do
    chf.spans[i].reg := srcReg[i];

  ctx.stopTimer(RC_TIMER_BUILD_REGIONS);

  return true;
end;}

/// @par
///
/// Non-null regions will consist of connected, non-overlapping walkable spans that form a single contour.
/// Contours will form simple polygons.
///
/// If multiple regions form an area that is smaller than @p minRegionArea, then all spans will be
/// re-assigned to the zero (null) region.
///
/// Watershed partitioning can result in smaller than necessary regions, especially in diagonal corridors.
/// @p mergeRegionArea helps reduce unecessarily small regions.
///
/// See the #rcConfig documentation for more information on the configuration parameters.
///
/// The region data will be available via the rcCompactHeightfield::maxRegions
/// and rcCompactSpan::reg fields.
///
/// @warning The distance field must be created using #rcBuildDistanceField before attempting to build regions.
///
/// @see rcCompactHeightfield, rcCompactSpan, rcBuildDistanceField, rcBuildRegionsMonotone, rcConfig
function rcBuildRegions(ctx: TrcContext; chf: PrcCompactHeightfield;
          const borderSize, minRegionArea, mergeRegionArea: Integer): Boolean;
const LOG_NB_STACKS = 3;
const NB_STACKS = 1 shl LOG_NB_STACKS;
const expandIters = 8;
var w,h,i,j,x,y: Integer; buf: PWord; lvlStacks: TArrayOfTrcIntArray;
stack,visited,overlaps: TrcIntArray; srcReg,srcDist,dstReg,dstDist: PWord; regionId,level: Word; bw,bh: Integer; sId: Integer;
begin
  //rcAssert(ctx);

  ctx.startTimer(RC_TIMER_BUILD_REGIONS);

  w := chf.width;
  h := chf.height;

  GetMem(buf, sizeOf(Word)*chf.spanCount*4);

  ctx.startTimer(RC_TIMER_BUILD_REGIONS_WATERSHED);

  SetLength(lvlStacks, NB_STACKS);
  for i := 0 to NB_STACKS - 1 do
    lvlStacks[i].Create(1024);

  stack.Create(1024);
  visited.Create(1024);

  srcReg := buf;
  srcDist := buf + chf.spanCount;
  dstReg := buf + chf.spanCount*2;
  dstDist := buf + chf.spanCount*3;

  FillChar(srcReg[0], sizeof(Word)*chf.spanCount, 0);
  FillChar(srcDist[0], sizeof(Word)*chf.spanCount, 0);

  regionId := 1;
  level := (chf.maxDistance+1) and not 1;

  // TODO: Figure better formula, expandIters defines how much the
  // watershed 'overflows' and simplifies the regions. Tying it to
  // agent radius was usually good indication how greedy it could be.
//  const int expandIters := 4 + walkableRadius * 2;
  //const int expandIters := 8;

  if (borderSize > 0) then
  begin
    // Make sure border will not overflow.
    bw := rcMin(w, borderSize);
    bh := rcMin(h, borderSize);
    // Paint regions
    paintRectRegion(0, bw, 0, h, regionId or RC_BORDER_REG, chf, srcReg); Inc(regionId);
    paintRectRegion(w-bw, w, 0, h, regionId or RC_BORDER_REG, chf, srcReg); Inc(regionId);
    paintRectRegion(0, w, 0, bh, regionId or RC_BORDER_REG, chf, srcReg); Inc(regionId);
    paintRectRegion(0, w, h-bh, h, regionId or RC_BORDER_REG, chf, srcReg); Inc(regionId);

    chf.borderSize := borderSize;
  end;

  sId := -1;
  while (level > 0) do
  begin
    level := IfThen(level >= 2, level-2, 0);
    sId := (sId+1) and (NB_STACKS-1);

//    ctx.startTimer(RC_TIMER_DIVIDE_TO_LEVELS);

    if (sId = 0) then
      sortCellsByLevel(level, chf, srcReg, NB_STACKS, lvlStacks, 1)
    else
      appendStacks(@lvlStacks[sId-1], @lvlStacks[sId], srcReg); // copy left overs from last level

//    ctx.stopTimer(RC_TIMER_DIVIDE_TO_LEVELS);

    ctx.startTimer(RC_TIMER_BUILD_REGIONS_EXPAND);

    // Expand current regions until no empty connected cells found.
    if (expandRegions(expandIters, level, chf, srcReg, srcDist, dstReg, dstDist, @lvlStacks[sId], false) <> srcReg) then
    begin
      rcSwap(Pointer(srcReg), Pointer(dstReg));
      rcSwap(Pointer(srcDist), Pointer(dstDist));
    end;

    ctx.stopTimer(RC_TIMER_BUILD_REGIONS_EXPAND);

    ctx.startTimer(RC_TIMER_BUILD_REGIONS_FLOOD);

    // Mark new regions with IDs.
    for j := 0 to lvlStacks[sId].size div 3 - 1 do
    begin
      x := lvlStacks[sId][j*3];
      y := lvlStacks[sId][j*3+1];
      i := lvlStacks[sId][j*3+2];
      if (i >= 0) and (srcReg[i] = 0) then
      begin
        if (floodRegion(x, y, i, level, regionId, chf, srcReg, srcDist, @stack)) then
          Inc(regionId);
      end;
    end;

    ctx.stopTimer(RC_TIMER_BUILD_REGIONS_FLOOD);
  end;

  // Expand current regions until no empty connected cells found.
  if (expandRegions(expandIters*8, 0, chf, srcReg, srcDist, dstReg, dstDist, @stack, true) <> srcReg) then
  begin
    rcSwap(Pointer(srcReg), Pointer(dstReg));
    rcSwap(Pointer(srcDist), Pointer(dstDist));
  end;

  ctx.stopTimer(RC_TIMER_BUILD_REGIONS_WATERSHED);

  ctx.startTimer(RC_TIMER_BUILD_REGIONS_FILTER);

  // Merge regions and filter out smalle regions.
  overlaps.Create(0);
  chf.maxRegions := regionId;
  if (not mergeAndFilterRegions(ctx, minRegionArea, mergeRegionArea, @chf.maxRegions, chf, srcReg, @overlaps)) then
    Exit(false);

  // If overlapping regions were found during merging, split those regions.
  if (overlaps.size > 0) then
  begin
    ctx.log(RC_LOG_ERROR, Format('rcBuildRegions: %d overlapping regions.', [overlaps.size]));
  end;

  ctx.stopTimer(RC_TIMER_BUILD_REGIONS_FILTER);

  // Write the result out.
  for i := 0 to chf.spanCount - 1 do
    chf.spans[i].reg := srcReg[i];

  FreeMem(buf);

  // Delphi: Manually release record and buffer it holds within
  for i := 0 to NB_STACKS - 1 do
    lvlStacks[i].Free;
  stack.Free;
  visited.Free;

  ctx.stopTimer(RC_TIMER_BUILD_REGIONS);

  Result := true;
end;


{function rcBuildLayerRegions(ctx: TrcContext; chf: PrcCompactHeightfield;
             const borderSize, minRegionArea: Integer): Boolean;
var w,h,i,x,y: Integer; c: PrcCompactCell; s: PrcCompactSpan;
begin
  //rcAssert(ctx);

  ctx.startTimer(RC_TIMER_BUILD_REGIONS);

  w := chf.width;
  h := chf.height;
  unsigned short id := 1;

  rcScopedDelete<unsigned short> srcReg := (unsigned short*)rcAlloc(sizeof(unsigned short)*chf.spanCount, RC_ALLOC_TEMP);
  if (!srcReg)
  begin
    ctx.log(RC_LOG_ERROR, 'rcBuildRegionsMonotone: Out of memory 'src' (%d).', chf.spanCount);
    return false;
  end;
  memset(srcReg,0,sizeof(unsigned short)*chf.spanCount);

  const int nsweeps := rcMax(chf.width,chf.height);
  rcScopedDelete<rcSweepSpan> sweeps := (rcSweepSpan*)rcAlloc(sizeof(rcSweepSpan)*nsweeps, RC_ALLOC_TEMP);
  if (!sweeps)
  begin
    ctx.log(RC_LOG_ERROR, 'rcBuildRegionsMonotone: Out of memory 'sweeps' (%d).', nsweeps);
    return false;
  end;


  // Mark border regions.
  if (borderSize > 0)
  begin
    // Make sure border will not overflow.
    const int bw := rcMin(w, borderSize);
    const int bh := rcMin(h, borderSize);
    // Paint regions
    paintRectRegion(0, bw, 0, h, id|RC_BORDER_REG, chf, srcReg); id++;
    paintRectRegion(w-bw, w, 0, h, id|RC_BORDER_REG, chf, srcReg); id++;
    paintRectRegion(0, w, 0, bh, id|RC_BORDER_REG, chf, srcReg); id++;
    paintRectRegion(0, w, h-bh, h, id|RC_BORDER_REG, chf, srcReg); id++;

    chf.borderSize := borderSize;
  end;

  rcIntArray prev(256);

  // Sweep one line at a time.
  for (int y := borderSize; y < h-borderSize; ++y)
  begin
    // Collect spans from this row.
    prev.resize(id+1);
    memset(&prev[0],0,sizeof(int)*id);
    unsigned short rid := 1;

    for (int x := borderSize; x < w-borderSize; ++x)
    begin
      c := @chf.cells[x+y*w];

      for i := c.index to Integer(c.index+c.count) - 1 do
      begin
        s := @chf.spans[i];
        if (chf.areas[i] = RC_NULL_AREA) continue;

        // -x
        unsigned short previd := 0;
        if (rcGetCon(s, 0) <> RC_NOT_CONNECTED)
        begin
          const int ax := x + rcGetDirOffsetX(0);
          const int ay := y + rcGetDirOffsetY(0);
          const int ai := chf.cells[ax+ay*w].index + rcGetCon(s, 0);
          if ((srcReg[ai] & RC_BORDER_REG) = 0 and chf.areas[i] = chf.areas[ai])
            previd := srcReg[ai];
        end;

        if (!previd)
        begin
          previd := rid++;
          sweeps[previd].rid := previd;
          sweeps[previd].ns := 0;
          sweeps[previd].nei := 0;
        end;

        // -y
        if (rcGetCon(s,3) <> RC_NOT_CONNECTED)
        begin
          const int ax := x + rcGetDirOffsetX(3);
          const int ay := y + rcGetDirOffsetY(3);
          const int ai := chf.cells[ax+ay*w].index + rcGetCon(s, 3);
          if (srcReg[ai] and (srcReg[ai] & RC_BORDER_REG) = 0 and chf.areas[i] = chf.areas[ai])
          begin
            unsigned short nr := srcReg[ai];
            if (!sweeps[previd].nei or sweeps[previd].nei = nr)
            begin
              sweeps[previd].nei := nr;
              sweeps[previd].ns++;
              prev[nr]++;
            end;
            else
            begin
              sweeps[previd].nei := RC_NULL_NEI;
            end;
          end;
        end;

        srcReg[i] := previd;
      end;
    end;

    // Create unique ID.
    for i := 1 to rid - 1 do
    begin
      if (sweeps[i].nei <> RC_NULL_NEI) and (sweeps[i].nei <> 0) and
        (prev[sweeps[i].nei] = sweeps[i].ns) then
      begin
        sweeps[i].id := sweeps[i].nei;
      end;
      else
      begin
        sweeps[i].id := id;
        Inc(id);
      end;
    end;

    // Remap IDs
    for x := borderSize to w-borderSize - 1 do
    begin
      c := @chf.cells[x+y*w];

      for i := c.index to Integer(c.index+c.count) - 1 do
      begin
        if (srcReg[i] > 0) and (srcReg[i] < rid) then
          srcReg[i] := sweeps[srcReg[i]].id;
      end;
    end;
  end;


  ctx.startTimer(RC_TIMER_BUILD_REGIONS_FILTER);

  // Merge monotone regions to layers and remove small regions.
  rcIntArray overlaps;
  chf.maxRegions := id;
  if (not mergeAndFilterLayerRegions(ctx, minRegionArea, chf.maxRegions, chf, srcReg, overlaps)) then
    Exit(false);

  ctx.stopTimer(RC_TIMER_BUILD_REGIONS_FILTER);


  // Store the result out.
  for i := 0 to chf.spanCount - 1 do
    chf.spans[i].reg := srcReg[i];

  ctx.stopTimer(RC_TIMER_BUILD_REGIONS);

  Result := true;
end;}

end.
