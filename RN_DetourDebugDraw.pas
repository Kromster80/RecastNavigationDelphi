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
unit RN_DetourDebugDraw;
interface
uses RN_DebugDraw, RN_DetourNavMesh, RN_DetourNavMeshHelper, RN_DetourNavMeshQuery, RN_DetourCommon, RN_DetourNode;

const
  DU_DRAWNAVMESH_OFFMESHCONS = $01;
  DU_DRAWNAVMESH_CLOSEDLIST = $02;
  DU_DRAWNAVMESH_COLOR_TILES = $04;


procedure duDebugDrawNavMesh(dd: TduDebugDraw; mesh: TdtNavMesh; flags: Byte);
procedure duDebugDrawNavMeshWithClosedList(dd: TduDebugDraw; mesh: TdtNavMesh; query: TdtNavMeshQuery; flags: Byte);
procedure duDebugDrawNavMeshNodes(dd: TduDebugDraw; query: TdtNavMeshQuery);

procedure duDebugDrawNavMeshBVTree(dd: TduDebugDraw; mesh: TdtNavMesh);
procedure duDebugDrawNavMeshPortals(dd: TduDebugDraw; mesh: TdtNavMesh);
procedure duDebugDrawNavMeshPolysWithFlags(dd: TduDebugDraw; mesh: TdtNavMesh; polyFlags: Word; col: Cardinal);
procedure duDebugDrawNavMeshPoly(dd: TduDebugDraw; mesh: TdtNavMesh; ref: TdtPolyRef; col: Cardinal);

//procedure duDebugDrawTileCacheLayerAreas(dd: TduDebugDraw; layer: PdtTileCacheLayer; cs, ch: Single);
//procedure duDebugDrawTileCacheLayerRegions(dd: TduDebugDraw; layer: PdtTileCacheLayer; cs, ch: Single);
//procedure duDebugDrawTileCacheContours(dd: TduDebugDraw; lcset: PdtTileCacheContourSet;  orig: PSingle; cs, ch: Single);
//procedure duDebugDrawTileCachePolyMesh(dd: TduDebugDraw; lmesh: PdtTileCachePolyMesh;  orig: PSingle; cs, ch: Single);

implementation
uses Math, RN_DetourStatus;


function distancePtLine2d(pt, p, q: PSingle): Single;
var pqx,pqz,dx,dz,d,t: Single;
begin
  pqx := q[0] - p[0];
  pqz := q[2] - p[2];
  dx := pt[0] - p[0];
  dz := pt[2] - p[2];
  d := pqx*pqx + pqz*pqz;
  t := pqx*dx + pqz*dz;
  if (d <> 0) then t := t / d;
  dx := p[0] + t*pqx - pt[0];
  dz := p[2] + t*pqz - pt[2];
  Result := dx*dx + dz*dz;
end;

procedure drawPolyBoundaries(dd: TduDebugDraw; tile: PdtMeshTile;
                 col: Cardinal; linew: Single;
                 inner: Boolean);
const thr = 0.01*0.01;
var i,j,nj,m,n,k: Integer; p: PdtPoly; pd: PdtPolyDetail; c,kl: Cardinal; con: Boolean; v0,v1: PSingle; t: PByte;
tv: array [0..2] of PSingle;
begin
  dd.&begin(DU_DRAW_LINES, linew);

  for i := 0 to tile.header.polyCount - 1 do
  begin
    p := @tile.polys[i];

    if (p.getType() = DT_POLYTYPE_OFFMESH_CONNECTION) then continue;

    pd := @tile.detailMeshes[i];

    j := 0; nj := p.vertCount;
    while (j < nj) do
    begin
      c := col;
      if (inner) then
      begin
        if (p.neis[j] = 0) then begin Inc(j); continue; end;
        if (p.neis[j] and DT_EXT_LINK) <> 0 then
        begin
          con := false;
          kl := p.firstLink;
          while (kl <> DT_NULL_LINK) do
          begin
            if (tile.links[kl].edge = j) then
            begin
              con := true;
              break;
            end;

            kl := tile.links[kl].next;
          end;
          if (con) then
            c := duRGBA(255,255,255,48)
          else
            c := duRGBA(0,0,0,48);
        end
        else
          c := duRGBA(0,48,64,32);
      end
      else
      begin
        if (p.neis[j] <> 0) then
        begin
          //C++ seems to be doing loop increase, so do we
          Inc(j);
          continue;
        end;
      end;

      v0 := @tile.verts[p.verts[j]*3];
      v1 := @tile.verts[p.verts[(j+1) mod nj]*3];

      // Draw detail mesh edges which align with the actual poly edge.
      // This is really slow.
      for k := 0 to pd.triCount - 1 do
      begin
        t := @tile.detailTris[(pd.triBase+k)*4];
        for m := 0 to 2 do
        begin
          if (t[m] < p.vertCount) then
            tv[m] := @tile.verts[p.verts[t[m]]*3]
          else
            tv[m] := @tile.detailVerts[(pd.vertBase+(t[m]-p.vertCount))*3];
        end;
        m := 0; n := 2;
        while (m < 3) do
        begin
          if (((t[3] shr (n*2)) and $3) = 0) then begin n := m; Inc(m); continue; end;  // Skip inner detail edges.
          if (distancePtLine2d(tv[n],v0,v1) < thr) and
            (distancePtLine2d(tv[m],v0,v1) < thr) then
          begin
            dd.vertex(tv[n], c);
            dd.vertex(tv[m], c);
          end;

          n := m;
          Inc(m);
        end;
      end;

      Inc(j);
    end;
  end;
  dd.&end();
end;

procedure drawMeshTile(dd: TduDebugDraw; mesh: TdtNavMesh; query: TdtNavMeshQuery; const tile: PdtMeshTile; flags: Byte);
var base: TdtPolyRef; tileNum,i,j: Integer; k: Cardinal; p: PdtPoly; pd: PdtPolyDetail; col,col2,vcol: Cardinal; t: PByte;
con: PdtOffMeshConnection; va,vb,v: PSingle; startSet, endSet: Boolean;
begin
  base := mesh.getPolyRefBase(tile);

  tileNum := mesh.decodePolyIdTile(base);

  dd.depthMask(false);

  dd.&begin(DU_DRAW_TRIS);
  for i := 0 to tile.header.polyCount - 1 do
  begin
    p := @tile.polys[i];
    if (p.getType() = DT_POLYTYPE_OFFMESH_CONNECTION)  then // Skip off-mesh links.
      continue;

    pd := @tile.detailMeshes[i];

    if (query <> nil) and (query.isInClosedList(base or TdtPolyRef(i))) then
      col := duRGBA(255,196,0,64)
    else
    begin
      if (flags and DU_DRAWNAVMESH_COLOR_TILES <> 0) then
      begin
        col := duIntToCol(tileNum, 128);
      end
      else
      begin
        if (p.getArea() = 0) then // Treat zero area type as default.
          col := duRGBA(0,192,255,64)
        else
          col := duIntToCol(p.getArea(), 64);
      end;
    end;

    for j := 0 to pd.triCount - 1 do
    begin
      t := @tile.detailTris[(pd.triBase+j)*4];
      for k := 0 to 2 do
      begin
        if (t[k] < p.vertCount) then
          dd.vertex(@tile.verts[p.verts[t[k]]*3], col)
        else
          dd.vertex(@tile.detailVerts[(pd.vertBase+t[k]-p.vertCount)*3], col);
      end;
    end;
  end;
  dd.&end();

  // Draw inter poly boundaries
  drawPolyBoundaries(dd, tile, duRGBA(0,48,64,32), 1.5, true);

  // Draw outer poly boundaries
  drawPolyBoundaries(dd, tile, duRGBA(0,48,64,220), 2.5, false);

  if (flags and DU_DRAWNAVMESH_OFFMESHCONS) <> 0 then
  begin
    dd.&begin(DU_DRAW_LINES, 2.0);
    for i := 0 to tile.header.polyCount - 1 do
    begin
      p := @tile.polys[i];
      if (p.getType() <> DT_POLYTYPE_OFFMESH_CONNECTION) then// Skip regular polys.
        continue;

      if (query <> nil) and (query.isInClosedList(base or TdtPolyRef(i))) then
        col := duRGBA(255,196,0,220)
      else
        col := duDarkenCol(duIntToCol(p.getArea(), 220));

      con := @tile.offMeshCons[i - tile.header.offMeshBase];
      va := @tile.verts[p.verts[0]*3];
      vb := @tile.verts[p.verts[1]*3];

      // Check to see if start and end end-points have links.
      startSet := false;
      endSet := false;
      k := p.firstLink;
      while (k <> DT_NULL_LINK) do
      begin
        if (tile.links[k].edge = 0) then
          startSet := true;
        if (tile.links[k].edge = 1) then
          endSet := true;

        k := tile.links[k].next;
      end;

      // End points and their on-mesh locations.
      dd.vertex(va[0],va[1],va[2], col);
      dd.vertex(con.pos[0],con.pos[1],con.pos[2], col);
      col2 := IfThen(startSet, col, duRGBA(220,32,16,196));
      duAppendCircle(dd, con.pos[0],con.pos[1]+0.1,con.pos[2], con.rad, col2);

      dd.vertex(vb[0],vb[1],vb[2], col);
      dd.vertex(con.pos[3],con.pos[4],con.pos[5], col);
      col2 := IfThen(endSet, col, duRGBA(220,32,16,196));
      duAppendCircle(dd, con.pos[3],con.pos[4]+0.1,con.pos[5], con.rad, col2);

      // End point vertices.
      dd.vertex(con.pos[0],con.pos[1],con.pos[2], duRGBA(0,48,64,196));
      dd.vertex(con.pos[0],con.pos[1]+0.2,con.pos[2], duRGBA(0,48,64,196));

      dd.vertex(con.pos[3],con.pos[4],con.pos[5], duRGBA(0,48,64,196));
      dd.vertex(con.pos[3],con.pos[4]+0.2,con.pos[5], duRGBA(0,48,64,196));

      // Connection arc.
      duAppendArc(dd, con.pos[0],con.pos[1],con.pos[2], con.pos[3],con.pos[4],con.pos[5], 0.25,
            Byte(con.flags and 1) * 0.6, 0.6, col);
    end;
    dd.&end();
  end;

  vcol := duRGBA(0,0,0,196);
  dd.&begin(DU_DRAW_POINTS, 3.0);
  for i := 0 to tile.header.vertCount - 1 do
  begin
    v := @tile.verts[i*3];
    dd.vertex(v[0], v[1], v[2], vcol);
  end;
  dd.&end();

  dd.depthMask(true);
end;

procedure duDebugDrawNavMesh(dd: TduDebugDraw; mesh: TdtNavMesh; flags: Byte);
var i: Integer; tile: PdtMeshTile;
begin
  if (dd = nil) then Exit;

  for i := 0 to mesh.getMaxTiles - 1 do
  begin
    tile := mesh.getTile(i);
    if (tile.header = nil) then continue;
    drawMeshTile(dd, mesh, nil, tile, flags);
  end;
end;

procedure duDebugDrawNavMeshWithClosedList(dd: TduDebugDraw; mesh: TdtNavMesh; query: TdtNavMeshQuery; flags: Byte);
var q: TdtNavMeshQuery; i: Integer; tile: PdtMeshTile;
begin
  if (dd = nil) then Exit;

  if (flags and DU_DRAWNAVMESH_CLOSEDLIST) <> 0 then q := query else q := nil;

  for i := 0 to mesh.getMaxTiles - 1 do
  begin
    tile := mesh.getTile(i);
    if (tile.header = nil) then continue;
    drawMeshTile(dd, mesh, q, tile, flags);
  end;
end;

procedure duDebugDrawNavMeshNodes(dd: TduDebugDraw; query: TdtNavMeshQuery);
var pool: TdtNodePool; off: Single; i: Integer; j: TdtNodeIndex; node,parent: PdtNode;
begin
  if (dd = nil) then Exit;

  pool := query.getNodePool;
  if (pool <> nil) then
  begin
    off := 0.5;
    dd.&begin(DU_DRAW_POINTS, 4.0);
    for i := 0 to pool.getHashSize - 1 do
    begin
      j := pool.getFirst(i);
      while (j <> DT_NULL_IDX) do
      begin
        node := pool.getNodeAtIdx(j+1);
        if (node = nil) then begin j := pool.getNext(j); continue; end;
        dd.vertex(node.pos[0],node.pos[1]+off,node.pos[2], duRGBA(255,192,0,255));

        j := pool.getNext(j);
      end;
    end;
    dd.&end();

    dd.&begin(DU_DRAW_LINES, 2.0);
    for i := 0 to pool.getHashSize - 1 do
    begin
      j := pool.getFirst(i);
      while (j <> DT_NULL_IDX) do
      begin
        node := pool.getNodeAtIdx(j+1);
        if (node = nil) then begin j := pool.getNext(j); continue; end;
        if (node.pidx = 0) then begin j := pool.getNext(j); continue; end;
        parent := pool.getNodeAtIdx(node.pidx);
        if (parent = nil) then begin j := pool.getNext(j); continue; end;
        dd.vertex(node.pos[0],node.pos[1]+off,node.pos[2], duRGBA(255,192,0,128));
        dd.vertex(parent.pos[0],parent.pos[1]+off,parent.pos[2], duRGBA(255,192,0,128));

        j := pool.getNext(j);
      end;
    end;
    dd.&end();
  end;
end;


procedure drawMeshTileBVTree(dd: TduDebugDraw; tile: PdtMeshTile);
var cs: Single; i: Integer; n: PdtBVNode;
begin
  // Draw BV nodes.
  cs := 1.0 / tile.header.bvQuantFactor;
  dd.&begin(DU_DRAW_LINES, 1.0);
  for i := 0 to tile.header.bvNodeCount - 1 do
  begin
    n := @tile.bvTree[i];
    if (n.i < 0) then // Leaf indices are positive.
      continue;
    duAppendBoxWire(dd, tile.header.bmin[0] + n.bmin[0]*cs,
            tile.header.bmin[1] + n.bmin[1]*cs,
            tile.header.bmin[2] + n.bmin[2]*cs,
            tile.header.bmin[0] + n.bmax[0]*cs,
            tile.header.bmin[1] + n.bmax[1]*cs,
            tile.header.bmin[2] + n.bmax[2]*cs,
            duRGBA(255,255,255,128));
  end;
  dd.&end();
end;

procedure duDebugDrawNavMeshBVTree(dd: TduDebugDraw; mesh: TdtNavMesh);
var i: Integer; tile: PdtMeshTile;
begin
  if (dd = nil) then Exit;

  for i := 0 to mesh.getMaxTiles - 1 do
  begin
    tile := mesh.getTile(i);
    if (tile.header = nil) then continue;
    drawMeshTileBVTree(dd, tile);
  end;
end;

procedure drawMeshTilePortal(dd: TduDebugDraw; tile: PdtMeshTile);
const padx = 0.04;
var pady: Single; side,i,j,nv: Integer; m: Word; poly: PdtPoly; va,vb: PSingle; col: Cardinal; x,z: Single;
begin
  // Draw portals
  pady := tile.header.walkableClimb;

  dd.&begin(DU_DRAW_LINES, 2.0);

  for side := 0 to 7 do
  begin
    m := DT_EXT_LINK or side;

    for i := 0 to tile.header.polyCount - 1 do
    begin
      poly := @tile.polys[i];

      // Create new links.
      nv := poly.vertCount;
      for j := 0 to nv - 1 do
      begin
        // Skip edges which do not point to the right side.
        if (poly.neis[j] <> m) then
          continue;

        // Create new links
        va := @tile.verts[poly.verts[j]*3];
        vb := @tile.verts[poly.verts[(j+1) mod nv]*3];

        if (side = 0) or (side = 4) then
        begin
          col := IfThen(side = 0, duRGBA(128,0,0,128), duRGBA(128,0,128,128));

          x := va[0] + IfThen(side = 0, -padx, padx);

          dd.vertex(x,va[1]-pady,va[2], col);
          dd.vertex(x,va[1]+pady,va[2], col);

          dd.vertex(x,va[1]+pady,va[2], col);
          dd.vertex(x,vb[1]+pady,vb[2], col);

          dd.vertex(x,vb[1]+pady,vb[2], col);
          dd.vertex(x,vb[1]-pady,vb[2], col);

          dd.vertex(x,vb[1]-pady,vb[2], col);
          dd.vertex(x,va[1]-pady,va[2], col);
        end
        else if (side = 2) or (side = 6) then
        begin
          col := IfThen(side = 2, duRGBA(0,128,0,128), duRGBA(0,128,128,128));

          z := va[2] + IfThen(side = 2, -padx, padx);

          dd.vertex(va[0],va[1]-pady,z, col);
          dd.vertex(va[0],va[1]+pady,z, col);

          dd.vertex(va[0],va[1]+pady,z, col);
          dd.vertex(vb[0],vb[1]+pady,z, col);

          dd.vertex(vb[0],vb[1]+pady,z, col);
          dd.vertex(vb[0],vb[1]-pady,z, col);

          dd.vertex(vb[0],vb[1]-pady,z, col);
          dd.vertex(va[0],va[1]-pady,z, col);
        end;

      end;
    end;
  end;

  dd.&end();
end;

procedure duDebugDrawNavMeshPortals(dd: TduDebugDraw; mesh: TdtNavMesh);
var i: Integer; tile: PdtMeshTile;
begin
  if (dd = nil) then Exit;

  for i := 0 to mesh.getMaxTiles - 1 do
  begin
    tile := mesh.getTile(i);
    if (tile.header = nil) then continue;
    drawMeshTilePortal(dd, tile);
  end;
end;

procedure duDebugDrawNavMeshPolysWithFlags(dd: TduDebugDraw; mesh: TdtNavMesh; polyFlags: Word; col: Cardinal);
var i,j: Integer; tile: PdtMeshTile; base: TdtPolyRef; p: PdtPoly;
begin
  if (dd = nil) then Exit;

  for i := 0 to mesh.getMaxTiles - 1 do
  begin
    tile := mesh.getTile(i);
    if (tile.header = nil) then continue;
    base := mesh.getPolyRefBase(tile);

    for j := 0 to tile.header.polyCount - 1 do
    begin
      p := @tile.polys[j];
      if ((p.flags and polyFlags) = 0) then continue;
      duDebugDrawNavMeshPoly(dd, mesh, base or TdtPolyRef(j), col);
    end;
  end;
end;

procedure duDebugDrawNavMeshPoly(dd: TduDebugDraw; mesh: TdtNavMesh; ref: TdtPolyRef; col: Cardinal);
var i,j: Integer; tile: PdtMeshTile; poly: PdtPoly; c,ip: Cardinal; con: PdtOffMeshConnection; pd: PdtPolyDetail; t: PByte;
begin
  if (dd = nil) then Exit;

  tile := nil;
  poly := nil;
  if (dtStatusFailed(mesh.getTileAndPolyByRef(ref, @tile, @poly))) then
    Exit;

  dd.depthMask(false);

  c := (col and $00ffffff) or (64 shl 24);
  ip := (poly - tile.polys);

  if (poly.getType() = DT_POLYTYPE_OFFMESH_CONNECTION) then
  begin
    con := @tile.offMeshCons[ip - tile.header.offMeshBase];

    dd.&begin(DU_DRAW_LINES, 2.0);

    // Connection arc.
    duAppendArc(dd, con.pos[0],con.pos[1],con.pos[2], con.pos[3],con.pos[4],con.pos[5], 0.25,
          Byte(con.flags and 1) * 0.6, 0.6, c);

    dd.&end();
  end
  else
  begin
    pd := @tile.detailMeshes[ip];

    dd.&begin(DU_DRAW_TRIS);
    for i := 0 to pd.triCount - 1 do
    begin
      t := @tile.detailTris[(pd.triBase+i)*4];
      for j := 0 to 2 do
      begin
        if (t[j] < poly.vertCount) then
          dd.vertex(@tile.verts[poly.verts[t[j]]*3], c)
        else
          dd.vertex(@tile.detailVerts[(pd.vertBase+t[j]-poly.vertCount)*3], c);
      end;
    end;
    dd.&end();
  end;

  dd.depthMask(true);

end;

{procedure debugDrawTileCachePortals(struct dd: TduDebugDraw; const dtTileCacheLayer& layer, const float cs, const float ch)
begin
  const int w := (int)layer.header.width;
  const int h := (int)layer.header.height;
  const float* bmin := layer.header.bmin;

  // Portals
  unsigned int pcol := duRGBA(255,255,255,255);

  const int segs[4*4] := begin0,0,0,1, 0,1,1,1, 1,1,1,0, 1,0,0,0end;;

  // Layer portals
  dd.&begin(DU_DRAW_LINES, 2.0f);
  for (int y := 0; y < h; ++y)
  begin
    for (int x := 0; x < w; ++x)
    begin
      const int idx := x+y*w;
      const int lh := (int)layer.heights[idx];
      if (lh == 0xff) continue;

      for (int dir := 0; dir < 4; ++dir)
      begin
        if (layer.cons[idx] & (1<<(dir+4)))
        begin
          const int* seg := &segs[dir*4];
          const float ax := bmin[0] + (x+seg[0])*cs;
          const float ay := bmin[1] + (lh+2)*ch;
          const float az := bmin[2] + (y+seg[1])*cs;
          const float bx := bmin[0] + (x+seg[2])*cs;
          const float by := bmin[1] + (lh+2)*ch;
          const float bz := bmin[2] + (y+seg[3])*cs;
          dd.vertex(ax, ay, az, pcol);
          dd.vertex(bx, by, bz, pcol);
        end;
      end;
    end;
  end;
  dd.&end();
end;

procedure duDebugDrawTileCacheLayerAreas(struct dd: TduDebugDraw; const dtTileCacheLayer& layer, const float cs, const float ch)
begin
  const int w := (int)layer.header.width;
  const int h := (int)layer.header.height;
  const float* bmin := layer.header.bmin;
  const float* bmax := layer.header.bmax;
  const int idx := layer.header.tlayer;

  unsigned int color := duIntToCol(idx+1, 255);

  // Layer bounds
  float lbmin[3], lbmax[3];
  lbmin[0] := bmin[0] + layer.header.minx*cs;
  lbmin[1] := bmin[1];
  lbmin[2] := bmin[2] + layer.header.miny*cs;
  lbmax[0] := bmin[0] + (layer.header.maxx+1)*cs;
  lbmax[1] := bmax[1];
  lbmax[2] := bmin[2] + (layer.header.maxy+1)*cs;
  duDebugDrawBoxWire(dd, lbmin[0],lbmin[1],lbmin[2], lbmax[0],lbmax[1],lbmax[2], duTransCol(color,128), 2.0f);

  // Layer height
  dd.&begin(DU_DRAW_QUADS);
  for (int y := 0; y < h; ++y)
  begin
    for (int x := 0; x < w; ++x)
    begin
      const int lidx := x+y*w;
      const int lh := (int)layer.heights[lidx];
      if (lh == 0xff) continue;
      const unsigned char area := layer.areas[lidx];

      unsigned int col;
      if (area == 63)
        col := duLerpCol(color, duRGBA(0,192,255,64), 32);
      else if (area == 0)
        col := duLerpCol(color, duRGBA(0,0,0,64), 32);
      else
        col := duLerpCol(color, duIntToCol(area, 255), 32);

      const float fx := bmin[0] + x*cs;
      const float fy := bmin[1] + (lh+1)*ch;
      const float fz := bmin[2] + y*cs;

      dd.vertex(fx, fy, fz, col);
      dd.vertex(fx, fy, fz+cs, col);
      dd.vertex(fx+cs, fy, fz+cs, col);
      dd.vertex(fx+cs, fy, fz, col);
    end;
  end;
  dd.&end();

  debugDrawTileCachePortals(dd, layer, cs, ch);
end;

procedure duDebugDrawTileCacheLayerRegions(struct dd: TduDebugDraw; const dtTileCacheLayer& layer, const float cs, const float ch)
begin
  const int w := (int)layer.header.width;
  const int h := (int)layer.header.height;
  const float* bmin := layer.header.bmin;
  const float* bmax := layer.header.bmax;
  const int idx := layer.header.tlayer;

  unsigned int color := duIntToCol(idx+1, 255);

  // Layer bounds
  float lbmin[3], lbmax[3];
  lbmin[0] := bmin[0] + layer.header.minx*cs;
  lbmin[1] := bmin[1];
  lbmin[2] := bmin[2] + layer.header.miny*cs;
  lbmax[0] := bmin[0] + (layer.header.maxx+1)*cs;
  lbmax[1] := bmax[1];
  lbmax[2] := bmin[2] + (layer.header.maxy+1)*cs;
  duDebugDrawBoxWire(dd, lbmin[0],lbmin[1],lbmin[2], lbmax[0],lbmax[1],lbmax[2], duTransCol(color,128), 2.0f);

  // Layer height
  dd.&begin(DU_DRAW_QUADS);
  for (int y := 0; y < h; ++y)
  begin
    for (int x := 0; x < w; ++x)
    begin
      const int lidx := x+y*w;
      const int lh := (int)layer.heights[lidx];
      if (lh == 0xff) continue;
      const unsigned char reg := layer.regs[lidx];

      unsigned int col := duLerpCol(color, duIntToCol(reg, 255), 192);

      const float fx := bmin[0] + x*cs;
      const float fy := bmin[1] + (lh+1)*ch;
      const float fz := bmin[2] + y*cs;

      dd.vertex(fx, fy, fz, col);
      dd.vertex(fx, fy, fz+cs, col);
      dd.vertex(fx+cs, fy, fz+cs, col);
      dd.vertex(fx+cs, fy, fz, col);
    end;
  end;
  dd.&end();

  debugDrawTileCachePortals(dd, layer, cs, ch);
end;




/*struct dtTileCacheContour
begin
  int nverts;
  unsigned char* verts;
  unsigned char reg;
  unsigned char area;
end;;

struct dtTileCacheContourSet
begin
  int nconts;
  dtTileCacheContour* conts;
end;;*/

procedure duDebugDrawTileCacheContours(dd: TduDebugDraw; const struct dtTileCacheContourSet& lcset,
                  const float* orig, const float cs, const float ch)
begin
  if (dd = nil) then Exit;

  const unsigned char a := 255;// (unsigned char)(alpha*255.0f);

  const int offs[2*4] := begin-1,0, 0,1, 1,0, 0,-1end;;

  dd.&begin(DU_DRAW_LINES, 2.0f);

  for (int i := 0; i < lcset.nconts; ++i)
  begin
    const dtTileCacheContour& c := lcset.conts[i];
    unsigned int color := 0;

    color := duIntToCol(i, a);

    for (int j := 0; j < c.nverts; ++j)
    begin
      const int k := (j+1) % c.nverts;
      const unsigned char* va := &c.verts[j*4];
      const unsigned char* vb := &c.verts[k*4];
      const float ax := orig[0] + va[0]*cs;
      const float ay := orig[1] + (va[1]+1+(i&1))*ch;
      const float az := orig[2] + va[2]*cs;
      const float bx := orig[0] + vb[0]*cs;
      const float by := orig[1] + (vb[1]+1+(i&1))*ch;
      const float bz := orig[2] + vb[2]*cs;
      unsigned int col := color;
      if ((va[3] & 0xf) != 0xf)
      begin
        // Portal segment
        col := duRGBA(255,255,255,128);
        int d := va[3] & 0xf;

        const float cx := (ax+bx)*0.5f;
        const float cy := (ay+by)*0.5f;
        const float cz := (az+bz)*0.5f;

        const float dx := cx + offs[d*2+0]*2*cs;
        const float dy := cy;
        const float dz := cz + offs[d*2+1]*2*cs;

        dd.vertex(cx,cy,cz,duRGBA(255,0,0,255));
        dd.vertex(dx,dy,dz,duRGBA(255,0,0,255));
      end;

      duAppendArrow(dd, ax,ay,az, bx,by,bz, 0.0f, cs*0.5f, col);
    end;
  end;
  dd.&end();

  dd.&begin(DU_DRAW_POINTS, 4.0f);

  for (int i := 0; i < lcset.nconts; ++i)
  begin
    const dtTileCacheContour& c := lcset.conts[i];
    unsigned int color := 0;

    for (int j := 0; j < c.nverts; ++j)
    begin
      const unsigned char* va := &c.verts[j*4];

      color := duDarkenCol(duIntToCol(i, a));
      if (va[3] & 0x80)
      begin
        // Border vertex
        color := duRGBA(255,0,0,255);
      end;

      float fx := orig[0] + va[0]*cs;
      float fy := orig[1] + (va[1]+1+(i&1))*ch;
      float fz := orig[2] + va[2]*cs;
      dd.vertex(fx,fy,fz, color);
    end;
  end;
  dd.&end();
end;

procedure duDebugDrawTileCachePolyMesh(dd: TduDebugDraw; const struct dtTileCachePolyMesh& lmesh, const float* orig, const float cs, const float ch)
begin
  if (dd = nil) then Exit;

  const int nvp := lmesh.nvp;

  const int offs[2*4] := begin-1,0, 0,1, 1,0, 0,-1end;;

  dd.&begin(DU_DRAW_TRIS);

  for (int i := 0; i < lmesh.npolys; ++i)
  begin
    const unsigned short* p := &lmesh.polys[i*nvp*2];

    unsigned int color;
    if (lmesh.areas[i] == DT_TILECACHE_WALKABLE_AREA)
      color := duRGBA(0,192,255,64);
    else if (lmesh.areas[i] == DT_TILECACHE_NULL_AREA)
      color := duRGBA(0,0,0,64);
    else
      color := duIntToCol(lmesh.areas[i], 255);

    unsigned short vi[3];
    for (int j := 2; j < nvp; ++j)
    begin
      if (p[j] == DT_TILECACHE_NULL_IDX) break;
      vi[0] := p[0];
      vi[1] := p[j-1];
      vi[2] := p[j];
      for (int k := 0; k < 3; ++k)
      begin
        const unsigned short* v := &lmesh.verts[vi[k]*3];
        const float x := orig[0] + v[0]*cs;
        const float y := orig[1] + (v[1]+1)*ch;
        const float z := orig[2] + v[2]*cs;
        dd.vertex(x,y,z, color);
      end;
    end;
  end;
  dd.&end();

  // Draw neighbours edges
  const unsigned int coln := duRGBA(0,48,64,32);
  dd.&begin(DU_DRAW_LINES, 1.5f);
  for (int i := 0; i < lmesh.npolys; ++i)
  begin
    const unsigned short* p := &lmesh.polys[i*nvp*2];
    for (int j := 0; j < nvp; ++j)
    begin
      if (p[j] == DT_TILECACHE_NULL_IDX) break;
      if (p[nvp+j] & 0x8000) continue;
      const int nj := (j+1 >= nvp || p[j+1] == DT_TILECACHE_NULL_IDX) ? 0 : j+1;
      int vi[2] := beginp[j], p[nj]end;;

      for (int k := 0; k < 2; ++k)
      begin
        const unsigned short* v := &lmesh.verts[vi[k]*3];
        const float x := orig[0] + v[0]*cs;
        const float y := orig[1] + (v[1]+1)*ch + 0.1f;
        const float z := orig[2] + v[2]*cs;
        dd.vertex(x, y, z, coln);
      end;
    end;
  end;
  dd.&end();

  // Draw boundary edges
  const unsigned int colb := duRGBA(0,48,64,220);
  dd.&begin(DU_DRAW_LINES, 2.5f);
  for (int i := 0; i < lmesh.npolys; ++i)
  begin
    const unsigned short* p := &lmesh.polys[i*nvp*2];
    for (int j := 0; j < nvp; ++j)
    begin
      if (p[j] == DT_TILECACHE_NULL_IDX) break;
      if ((p[nvp+j] & 0x8000) == 0) continue;
      const int nj := (j+1 >= nvp || p[j+1] == DT_TILECACHE_NULL_IDX) ? 0 : j+1;
      int vi[2] := beginp[j], p[nj]end;;

      unsigned int col := colb;
      if ((p[nvp+j] & 0xf) != 0xf)
      begin
        const unsigned short* va := &lmesh.verts[vi[0]*3];
        const unsigned short* vb := &lmesh.verts[vi[1]*3];

        const float ax := orig[0] + va[0]*cs;
        const float ay := orig[1] + (va[1]+1+(i&1))*ch;
        const float az := orig[2] + va[2]*cs;
        const float bx := orig[0] + vb[0]*cs;
        const float by := orig[1] + (vb[1]+1+(i&1))*ch;
        const float bz := orig[2] + vb[2]*cs;

        const float cx := (ax+bx)*0.5f;
        const float cy := (ay+by)*0.5f;
        const float cz := (az+bz)*0.5f;

        int d := p[nvp+j] & 0xf;

        const float dx := cx + offs[d*2+0]*2*cs;
        const float dy := cy;
        const float dz := cz + offs[d*2+1]*2*cs;

        dd.vertex(cx,cy,cz,duRGBA(255,0,0,255));
        dd.vertex(dx,dy,dz,duRGBA(255,0,0,255));

        col := duRGBA(255,255,255,128);
      end;

      for (int k := 0; k < 2; ++k)
      begin
        const unsigned short* v := &lmesh.verts[vi[k]*3];
        const float x := orig[0] + v[0]*cs;
        const float y := orig[1] + (v[1]+1)*ch + 0.1f;
        const float z := orig[2] + v[2]*cs;
        dd.vertex(x, y, z, col);
      end;
    end;
  end;
  dd.&end();

  dd.&begin(DU_DRAW_POINTS, 3.0f);
  const unsigned int colv := duRGBA(0,0,0,220);
  for (int i := 0; i < lmesh.nverts; ++i)
  begin
    const unsigned short* v := &lmesh.verts[i*3];
    const float x := orig[0] + v[0]*cs;
    const float y := orig[1] + (v[1]+1)*ch + 0.1f;
    const float z := orig[2] + v[2]*cs;
    dd.vertex(x,y,z, colv);
  end;
  dd.&end();
end;}


end.
