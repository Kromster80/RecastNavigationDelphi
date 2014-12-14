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

unit RN_RecastLayers;
interface
uses
  Math, SysUtils, RN_Helper, RN_Recast;


/// @end;
/// @name Layer, Contour, Polymesh, and Detail Mesh Functions
/// @see rcHeightfieldLayer, rcContourSet, rcPolyMesh, rcPolyMeshDetail
/// @begin

/// Builds a layer set from the specified compact heightfield.
///  @ingroup recast
///  @param[in,out]  ctx      The build context to use during the operation.
///  @param[in]    chf      A fully built compact heightfield.
///  @param[in]    borderSize  The size of the non-navigable border around the heightfield. [Limit: >=0]
///                [Units: vx]
///  @param[in]    walkableHeight  Minimum floor to 'ceiling' height that will still allow the floor area
///                to be considered walkable. [Limit: >= 3] [Units: vx]
///  @param[out]  lset    The resulting layer set. (Must be pre-allocated.)
///  @returns True if the operation completed successfully.
function rcBuildHeightfieldLayers(var ctx: TrcContext; chf: TrcCompactHeightfield;
                const borderSize, walkableHeight: Integer;
                out lset: TrcHeightfieldLayerSet): Boolean;

implementation
uses RN_RecastHelper;

const RC_MAX_LAYERS = RC_NOT_CONNECTED;
const RC_MAX_NEIS = 16;

type
TrcLayerRegion = record
  layers: array [0..RC_MAX_LAYERS-1] of Byte;
  neis: array [0..RC_MAX_NEIS-1] of Byte;
  ymin, ymax: Word;
  layerId: Byte;    // Layer ID
  nlayers: Byte;    // Layer count
  nneis: Byte;    // Neighbour count
  base: Byte;      // Flag indicating if the region is the base of merged regions.
end;
PrcLayerRegion = ^TrcLayerRegion;

procedure addUnique(a: PByte; an: PByte; v: Byte);
var n,i: Integer;
begin
  n := an^;
  for i := 0 to n - 1 do
    if (a[i] = v) then
      Exit;
  a[an^] := v;
  Inc(an^);
end;

function contains(a: PByte; an, v: Byte): Boolean;
var n,i: Integer;
begin
  n := an;
  for i := 0 to n - 1 do
    if (a[i] = v) then
      Exit(true);
  Exit(false);
end;

function overlapRange(const amin, amax, bmin, bmax: Word): Boolean;
begin
  Result := not ((amin > bmax) or (amax < bmin));
end;


/// @par
///
/// See the #rcConfig documentation for more information on the configuration parameters.
///
/// @see rcAllocHeightfieldLayerSet, rcCompactHeightfield, rcHeightfieldLayerSet, rcConfig
function rcBuildHeightfieldLayers(var ctx: TrcContext; chf: TrcCompactHeightfield;
                const borderSize, walkableHeight: Integer;
                out lset: TrcHeightfieldLayerSet): Boolean;
type
  TrcLayerSweepSpan = record
    ns: Word;  // number samples
    id: Byte;  // region id
    nei: Byte;  // neighbour id
  end;
const MAX_STACK = 64;
var w,h: Integer; srcReg: PByte; nsweeps: Integer; sweeps: array of TrcLayerSweepSpan; prevCount: array [0..255] of Integer;
regId: Byte; x,y,i,j,k: Integer; sweepId: Byte; c: PrcCompactCell; s,as1: PrcCompactSpan; sid: Byte; ax,ay,ai: Integer; nr: Byte; nregs: Integer;
regs: array of TrcLayerRegion; lregs: array [0..RC_MAX_LAYERS-1] of Byte; nlregs: Integer; regi,dir,rai: Byte; ri,rj: PrcLayerRegion;
layerId: Byte; stack: array [0..MAX_STACK-1] of Byte; nstack: Integer; root: PrcLayerRegion; reg,regn: PrcLayerRegion; nneis: Integer; nei: Byte;
ymin,ymax: Integer; mergeHeight: Word; newId,oldId: Byte; overlap: Boolean; remap: array [0..255] of Byte; lw,lh: Integer; bmin, bmax: PSingle;
curId: Byte; layer: PrcHeightfieldLayer; gridSize: Integer; hmin,hmax: Integer; cx,cy: Integer; lid,alid: Byte; idx: Integer; portal, con: Byte;
nx,ny: Integer;
begin
  //rcAssert(ctx);

  ctx.startTimer(RC_TIMER_BUILD_LAYERS);

  w := chf.width;
  h := chf.height;

  GetMem(srcReg, sizeof(Byte)*chf.spanCount);
  FillChar(srcReg[0], sizeof(Byte)*chf.spanCount, $ff);

  nsweeps := chf.width;
  SetLength(sweeps, nsweeps);

  // Partition walkable area into monotone regions.
  regId := 0;

  for y := borderSize to h-borderSize - 1 do
  begin
    FillChar(prevCount[0], SizeOf(Integer)*regId, #0);
    sweepId := 0;

    for x := borderSize to w-borderSize - 1 do
    begin
      c := @chf.cells[x+y*w];

      for i := c.index to Integer(c.index+c.count) - 1 do
      begin
        s := @chf.spans[i];
        if (chf.areas[i] = RC_NULL_AREA) then continue;

        sid := $ff;

        // -x
        if (rcGetCon(s, 0) <> RC_NOT_CONNECTED) then
        begin
          ax := x + rcGetDirOffsetX(0);
          ay := y + rcGetDirOffsetY(0);
          ai := chf.cells[ax+ay*w].index + rcGetCon(s, 0);
          if (chf.areas[ai] <> RC_NULL_AREA) and(srcReg[ai] <> $ff) then
            sid := srcReg[ai];
        end;

        if (sid = $ff) then
        begin
          sid := sweepId;
          Inc(sweepId);
          sweeps[sid].nei := $ff;
          sweeps[sid].ns := 0;
        end;

        // -y
        if (rcGetCon(s,3) <> RC_NOT_CONNECTED) then
        begin
          ax := x + rcGetDirOffsetX(3);
          ay := y + rcGetDirOffsetY(3);
          ai := chf.cells[ax+ay*w].index + rcGetCon(s, 3);
          nr := srcReg[ai];
          if (nr <> $ff) then
          begin
            // Set neighbour when first valid neighbour is encoutered.
            if (sweeps[sid].ns = 0) then
              sweeps[sid].nei := nr;

            if (sweeps[sid].nei = nr) then
            begin
              // Update existing neighbour
              Inc(sweeps[sid].ns);
              Inc(prevCount[nr]);
            end
            else
            begin
              // This is hit if there is nore than one neighbour.
              // Invalidate the neighbour.
              sweeps[sid].nei := $ff;
            end;
          end;
        end;

        srcReg[i] := sid;
      end;
    end;

    // Create unique ID.
    for i := 0 to sweepId - 1 do
    begin
      // If the neighbour is set and there is only one continuous connection to it,
      // the sweep will be merged with the previous one, else new region is created.
      if (sweeps[i].nei <> $ff) and (prevCount[sweeps[i].nei] = sweeps[i].ns) then
      begin
        sweeps[i].id := sweeps[i].nei;
      end
      else
      begin
        if (regId = 255) then
        begin
          ctx.log(RC_LOG_ERROR, 'rcBuildHeightfieldLayers: Region ID overflow.');
          Exit(false);
        end;
        sweeps[i].id := regId;
        Inc(regId);
      end;
    end;

    // Remap local sweep ids to region ids.
    for x := borderSize to w-borderSize - 1 do
    begin
      c := @chf.cells[x+y*w];
      for i := c.index to Integer(c.index+c.count) - 1 do
      begin
        if (srcReg[i] <> $ff) then
          srcReg[i] := sweeps[srcReg[i]].id;
      end;
    end;
  end;

  // Allocate and init layer regions.
  nregs := regId;
  SetLength(regs, nregs);

  for i := 0 to nregs - 1 do
  begin
    regs[i].layerId := $ff;
    regs[i].ymin := $ffff;
    regs[i].ymax := 0;
  end;

  // Find region neighbours and overlapping regions.
  for y := 0 to h - 1 do
  begin
    for x := 0 to w - 1 do
    begin
      c := @chf.cells[x+y*w];

      nlregs := 0;

      for i := c.index to Integer(c.index+c.count) - 1 do
      begin
        s := @chf.spans[i];
        regi := srcReg[i];
        if (regi = $ff) then continue;

        regs[regi].ymin := rcMin(regs[regi].ymin, s.y);
        regs[regi].ymax := rcMax(regs[regi].ymax, s.y);

        // Collect all region layers.
        if (nlregs < RC_MAX_LAYERS) then
        begin
          lregs[nlregs] := regi;
          Inc(nlregs);
        end;

        // Update neighbours
        for dir := 0 to 3 do
        begin
          if (rcGetCon(s, dir) <> RC_NOT_CONNECTED) then
          begin
            ax := x + rcGetDirOffsetX(dir);
            ay := y + rcGetDirOffsetY(dir);
            ai := chf.cells[ax+ay*w].index + rcGetCon(s, dir);
            rai := srcReg[ai];
            if (rai <> $ff) and (rai <> regi) then
              addUnique(@regs[regi].neis[0], @regs[regi].nneis, rai);
          end;
        end;

      end;

      // Update overlapping regions.
      for i := 0 to nlregs-1-1 do
      begin
        for j := i+1 to nlregs - 1 do
        begin
          if (lregs[i] <> lregs[j]) then
          begin
            ri := @regs[lregs[i]];
            rj := @regs[lregs[j]];
            addUnique(@ri.layers[0], @ri.nlayers, lregs[j]);
            addUnique(@rj.layers[0], @rj.nlayers, lregs[i]);
          end;
        end;
      end;

    end;
  end;

  // Create 2D layers from regions.
  layerId := 0;

  nstack := 0;

  for i := 0 to nregs - 1 do
  begin
    root := @regs[i];
    // Skip alreadu visited.
    if (root.layerId <> $ff) then
      continue;

    // Start search.
    root.layerId := layerId;
    root.base := 1;

    nstack := 0;
    stack[nstack] := i;
    Inc(nstack);

    while (nstack > 0) do
    begin
      // Pop front
      reg := @regs[stack[0]];
      Dec(nstack);
      for j := 0 to nstack - 1 do
        stack[j] := stack[j+1];

      nneis := reg.nneis;
      for j := 0 to nneis - 1 do
      begin
        nei := reg.neis[j];
        regn := @regs[nei];
        // Skip already visited.
        if (regn.layerId <> $ff) then
          continue;
        // Skip if the neighbour is overlapping root region.
        if (contains(@root.layers[0], root.nlayers, nei)) then
          continue;
        // Skip if the height range would become too large.
        ymin := rcMin(root.ymin, regn.ymin);
        ymax := rcMax(root.ymax, regn.ymax);
        if ((ymax - ymin) >= 255) then
           continue;

        if (nstack < MAX_STACK) then
        begin
          // Deepen
          stack[nstack] := nei;
          Inc(nstack);

          // Mark layer id
          regn.layerId := layerId;
          // Merge current layers to root.
          for k := 0 to regn.nlayers - 1 do
            addUnique(@root.layers[0], @root.nlayers, regn.layers[k]);
          root.ymin := rcMin(root.ymin, regn.ymin);
          root.ymax := rcMax(root.ymax, regn.ymax);
        end;
      end;
    end;

    Inc(layerId);
  end;

  // Merge non-overlapping regions that are close in height.
  mergeHeight := walkableHeight * 4;

  for i := 0 to nregs - 1 do
  begin
    ri := @regs[i];
    if (ri.base = 0) then continue;

    newId := ri.layerId;

    while (true) do
    begin
      oldId := $ff;

      for j := 0 to nregs - 1 do
      begin
        if (i = j) then continue;
        rj := @regs[j];
        if (rj.base = 0) then continue;

        // Skip if teh regions are not close to each other.
        if (not overlapRange(ri.ymin,ri.ymax+mergeHeight, rj.ymin,rj.ymax+mergeHeight)) then
          continue;
        // Skip if the height range would become too large.
        ymin := rcMin(ri.ymin, rj.ymin);
        ymax := rcMax(ri.ymax, rj.ymax);
        if ((ymax - ymin) >= 255) then
          continue;

        // Make sure that there is no overlap when mergin 'ri' and 'rj'.
        overlap := false;
        // Iterate over all regions which have the same layerId as 'rj'
        for k := 0 to nregs - 1 do
        begin
          if (regs[k].layerId <> rj.layerId) then
            continue;
          // Check if region 'k' is overlapping region 'ri'
          // Index to 'regs' is the same as region id.
          if (contains(@ri.layers[0],ri.nlayers, k)) then
          begin
            overlap := true;
            break;
          end;
        end;
        // Cannot merge of regions overlap.
        if (overlap) then
          continue;

        // Can merge i and j.
        oldId := rj.layerId;
        break;
      end;

      // Could not find anything to merge with, stop.
      if (oldId = $ff) then
        break;

      // Merge
      for j := 0 to nregs - 1 do
      begin
        rj := @regs[j];
        if (rj.layerId = oldId) then
        begin
          rj.base := 0;
          // Remap layerIds.
          rj.layerId := newId;
          // Add overlaid layers from 'rj' to 'ri'.
          for k := 0 to rj.nlayers - 1 do
            addUnique(@ri.layers[0], @ri.nlayers, rj.layers[k]);
          // Update heigh bounds.
          ri.ymin := rcMin(ri.ymin, rj.ymin);
          ri.ymax := rcMax(ri.ymax, rj.ymax);
        end;
      end;
    end;
  end;

  // Compact layerIds
  FillChar(remap[0], sizeof(Byte)*256, 0);

  // Find number of unique layers.
  layerId := 0;
  for i := 0 to nregs - 1 do
    remap[regs[i].layerId] := 1;
  for i := 0 to 255 do
  begin
    if (remap[i] <> 0) then
    begin
      remap[i] := layerId;
      Inc(layerId);
    end
    else
      remap[i] := $ff;
  end;
  // Remap ids.
  for i := 0 to nregs - 1 do
    regs[i].layerId := remap[regs[i].layerId];

  // No layers, return empty.
  if (layerId = 0) then
  begin
    ctx.stopTimer(RC_TIMER_BUILD_LAYERS);
    Exit(true);
  end;

  // Create layers.
  //rcAssert(lset.layers == 0);

  lw := w - borderSize*2;
  lh := h - borderSize*2;

  // Build contracted bbox for layers.
  rcVcopy(bmin, @chf.bmin);
  rcVcopy(bmax, @chf.bmax);
  bmin[0] := bmin[0] + borderSize*chf.cs;
  bmin[2] := bmin[2] + borderSize*chf.cs;
  bmax[0] := bmax[0] - borderSize*chf.cs;
  bmax[2] := bmax[2] - borderSize*chf.cs;

  lset.nlayers := layerId;

  SetLength(lset.layers, lset.nlayers);

  // Store layers.
  for i := 0 to lset.nlayers - 1 do
  begin
    curId := i;

    // Allocate memory for the current layer.
    layer := @lset.layers[i];
    //memset(layer, 0, sizeof(rcHeightfieldLayer));

    gridSize := lw*lh;

    GetMem(layer.heights, sizeof(Byte)*gridSize);
    FillChar(layer.heights[0], sizeof(Byte)*gridSize, $ff);

    GetMem(layer.areas, sizeof(Byte)*gridSize);
    FillChar(layer.areas[0], sizeof(Byte)*gridSize, 0);

    GetMem(layer.cons, sizeof(Byte)*gridSize);
    FillChar(layer.cons[0], sizeof(Byte)*gridSize, 0);

    // Find layer height bounds.
    hmin := 0;
    hmax := 0;
    for j := 0 to nregs - 1 do
    begin
      if (regs[j].base <> 0) and (regs[j].layerId = curId) then
      begin
        hmin := regs[j].ymin;
        hmax := regs[j].ymax;
      end;
    end;

    layer.width := lw;
    layer.height := lh;
    layer.cs := chf.cs;
    layer.ch := chf.ch;

    // Adjust the bbox to fit the heighfield.
    rcVcopy(@layer.bmin, bmin);
    rcVcopy(@layer.bmax, bmax);
    layer.bmin[1] := bmin[1] + hmin*chf.ch;
    layer.bmax[1] := bmin[1] + hmax*chf.ch;
    layer.hmin := hmin;
    layer.hmax := hmax;

    // Update usable data region.
    layer.minx := layer.width;
    layer.maxx := 0;
    layer.miny := layer.height;
    layer.maxy := 0;

    // Copy height and area from compact heighfield.
    for y := 0 to lh - 1 do
    begin
      for x := 0 to lw - 1 do
      begin
        cx := borderSize+x;
        cy := borderSize+y;
        c := @chf.cells[cx+cy*w];
        for j := c.index to Integer(c.index+c.count) - 1 do
        begin
          s := @chf.spans[j];
          // Skip unassigned regions.
          if (srcReg[j] = $ff) then
            continue;
          // Skip of does nto belong to current layer.
          lid := regs[srcReg[j]].layerId;
          if (lid <> curId) then
            continue;

          // Update data bounds.
          layer.minx := rcMin(layer.minx, x);
          layer.maxx := rcMax(layer.maxx, x);
          layer.miny := rcMin(layer.miny, y);
          layer.maxy := rcMax(layer.maxy, y);

          // Store height and area type.
          idx := x+y*lw;
          layer.heights[idx] := (s.y - hmin);
          layer.areas[idx] := chf.areas[j];

          // Check connection.
          portal := 0;
          con := 0;
          for dir := 0 to 3 do
          begin
            if (rcGetCon(s, dir) <> RC_NOT_CONNECTED) then
            begin
              ax := cx + rcGetDirOffsetX(dir);
              ay := cy + rcGetDirOffsetY(dir);
              ai := chf.cells[ax+ay*w].index + rcGetCon(s, dir);
              if srcReg[ai] <> $ff then alid := regs[srcReg[ai]].layerId else alid := $ff;
              // Portal mask
              if (chf.areas[ai] <> RC_NULL_AREA) and (lid <> alid) then
              begin
                portal := portal or (1 shl dir);
                // Update height so that it matches on both sides of the portal.
                as1 := @chf.spans[ai];
                if (as1.y > hmin) then
                  layer.heights[idx] := rcMax(layer.heights[idx], (as1.y - hmin));
              end;
              // Valid connection mask
              if (chf.areas[ai] <> RC_NULL_AREA) and (lid = alid) then
              begin
                nx := ax - borderSize;
                ny := ay - borderSize;
                if (nx >= 0) and(ny >= 0) and(nx < lw) and(ny < lh) then
                  con := con or (1 shl dir);
              end;
            end;
          end;

          layer.cons[idx] := (portal shl 4) or con;
        end;
      end;
    end;

//    if (layer->minx > layer->maxx)
//      layer->minx = layer->maxx = 0;
//    if (layer->miny > layer->maxy)
//      layer->miny = layer->maxy = 0;
    if (layer.minx > layer.maxx) then
    begin
      layer.minx := 0;
      layer.maxx := 0;
    end;
    if (layer.miny > layer.maxy) then
    begin
      layer.miny := 0;
      layer.maxy := 0;
    end;
  end;

  FreeMem(srcReg);

  ctx.stopTimer(RC_TIMER_BUILD_LAYERS);

  Result := true;
end;

end.
