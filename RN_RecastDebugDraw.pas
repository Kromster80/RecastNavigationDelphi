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

unit RN_RecastDebugDraw;
interface
uses Math, RN_Helper, RN_Recast, RN_DebugDraw;

procedure duDebugDrawTriMesh(dd: TduDebugDraw; const verts: PSingle; nverts: Integer; const tris: PInteger; const normals: PSingle; ntris: Integer; const flags: PByte; const texScale: Single);
procedure duDebugDrawTriMeshSlope(dd: TduDebugDraw; const verts: PSingle; nverts: Integer; const tris: PInteger; const normals: PSingle; ntris: Integer; const walkableSlopeAngle: Single; const texScale: Single);

procedure duDebugDrawHeightfieldSolid(dd: TduDebugDraw; const hf: PrcHeightfield);
procedure duDebugDrawHeightfieldWalkable(dd: TduDebugDraw; const hf: PrcHeightfield);

procedure duDebugDrawCompactHeightfieldSolid(dd: TduDebugDraw; const chf: PrcCompactHeightfield);
procedure duDebugDrawCompactHeightfieldRegions(dd: TduDebugDraw; const chf: PrcCompactHeightfield);
procedure duDebugDrawCompactHeightfieldDistance(dd: TduDebugDraw; const chf: PrcCompactHeightfield);

{procedure duDebugDrawHeightfieldLayer(duDebugDraw* dd, const struct rcHeightfieldLayer& layer, const int idx);
procedure duDebugDrawHeightfieldLayers(duDebugDraw* dd, const struct rcHeightfieldLayerSet& lset);
procedure duDebugDrawHeightfieldLayersRegions(duDebugDraw* dd, const struct rcHeightfieldLayerSet& lset);}

procedure duDebugDrawRegionConnections(dd: TduDebugDraw; const cset: PrcContourSet; const alpha: Single = 1.0);
procedure duDebugDrawRawContours(dd: TduDebugDraw; const cset: PrcContourSet; const alpha: Single = 1.0);
procedure duDebugDrawContours(dd: TduDebugDraw; const cset: PrcContourSet; const alpha: Single = 1.0);
procedure duDebugDrawPolyMesh(dd: TduDebugDraw; const mesh: PrcPolyMesh);
procedure duDebugDrawPolyMeshDetail(dd: TduDebugDraw; const dmesh: PrcPolyMeshDetail);

implementation

procedure duDebugDrawTriMesh(dd: TduDebugDraw; const verts: PSingle; nverts: Integer;
            const tris: PInteger; const normals: PSingle; ntris: Integer;
            const flags: PByte; const texScale: Single);
var uva,uvb,uvc: array [0..1] of Single; unwalkable,color: Cardinal; i: Integer; norm,va,vb,vc: PSingle; a: Byte;
ax,ay: Integer;
begin
  if (dd = nil) then Exit;
  if (verts = nil) then Exit;
  if (tris = nil) then Exit;
  if (normals = nil) then Exit;

  unwalkable := duRGBA(192,128,0,255);

  dd.texture(true);

  dd.&begin(DU_DRAW_TRIS);
  for i := 0 to ntris - 1 do
  begin
    norm := @normals[i*3];
    a := Byte(Trunc(220*(2+norm[0]+norm[1])/4));
    if (flags <> nil) and (not flags[i] <> 0) then
      color := duLerpCol(duRGBA(a,a,a,255), unwalkable, 64)
    else
      color := duRGBA(a,a,a,255);

    va := @verts[tris[i*3+0]*3];
    vb := @verts[tris[i*3+1]*3];
    vc := @verts[tris[i*3+2]*3];

    ax := 0; ay := 0;
    if (Abs(norm[1]) > Abs(norm[ax])) then
      ax := 1;
    if (Abs(norm[2]) > Abs(norm[ax])) then
      ax := 2;
    ax := (1 shl ax) and 3; // +1 mod 3
    ay := (1 shl ax) and 3; // +1 mod 3

    uva[0] := va[ax]*texScale;
    uva[1] := va[ay]*texScale;
    uvb[0] := vb[ax]*texScale;
    uvb[1] := vb[ay]*texScale;
    uvc[0] := vc[ax]*texScale;
    uvc[1] := vc[ay]*texScale;

    dd.vertex(va, color, @uva);
    dd.vertex(vb, color, @uvb);
    dd.vertex(vc, color, @uvc);
  end;
  dd.&end();
  dd.texture(false);
end;

procedure duDebugDrawTriMeshSlope(dd: TduDebugDraw; const verts: PSingle; nverts: Integer; const tris: PInteger; const normals: PSingle; ntris: Integer; const walkableSlopeAngle: Single; const texScale: Single);
//var walkableThr: Single; uva,uvb,uvc: array [0..1] of Single; unwalkable,color: Cardinal; i: Integer; norm,va,vb,vc: PSingle; a: Byte;
//ax,ay: Integer;
begin
  //Delphi: Not sure why, but this mode blackens everything way too much

  {if (dd = nil) then Exit;
  if (verts = nil) then Exit;
  if (tris = nil) then Exit;
  if (normals = nil) then Exit;

  walkableThr := cos(walkableSlopeAngle/180.0*PI);

  dd.texture(true);

  unwalkable := duRGBA(192,128,0,255);

  dd.&begin(DU_DRAW_TRIS);
  for i := 0 to ntris - 1 do
  begin
    norm := @normals[i*3];
    a := Byte(Trunc(220*(2+norm[0]+norm[1])/4));
    if (norm[1] < walkableThr) then
      color := duLerpCol(duRGBA(a,a,a,255), unwalkable, 64)
    else
      color := duRGBA(a,a,a,255);

    va := @verts[tris[i*3+0]*3];
    vb := @verts[tris[i*3+1]*3];
    vc := @verts[tris[i*3+2]*3];

    ax := 0; ay := 0;
    if (Abs(norm[1]) > Abs(norm[ax])) then
      ax := 1;
    if (Abs(norm[2]) > Abs(norm[ax])) then
      ax := 2;
    ax := (1 shl ax) and 3; // +1 mod 3
    ay := (1 shl ax) and 3; // +1 mod 3

    uva[0] := va[ax]*texScale;
    uva[1] := va[ay]*texScale;
    uvb[0] := vb[ax]*texScale;
    uvb[1] := vb[ay]*texScale;
    uvc[0] := vc[ax]*texScale;
    uvc[1] := vc[ay]*texScale;

    dd.vertex(va, color, @uva);
    dd.vertex(vb, color, @uvb);
    dd.vertex(vc, color, @uvc);
  end;
  dd.&end();

  dd.texture(false);}
end;

procedure duDebugDrawHeightfieldSolid(dd: TduDebugDraw; const hf: PrcHeightfield);
var orig: PSingle; cs,ch: Single; w,h: Integer; fcol: array [0..5] of Cardinal; x,y: Integer; fx,fz: Single; s: PrcSpan;
begin
  if (dd = nil) then Exit;

  orig := @hf.bmin;
  cs := hf.cs;
  ch := hf.ch;

  w := hf.width;
  h := hf.height;

  //unsigned int fcol[6];
  duCalcBoxColors(fcol, duRGBA(255,255,255,255), duRGBA(255,255,255,255));

  dd.&begin(DU_DRAW_QUADS);

  for y := 0 to h - 1 do
  begin
    for x := 0 to w - 1 do
    begin
      fx := orig[0] + x*cs;
      fz := orig[2] + y*cs;
      s := hf.spans[x + y*w];
      while (s <> nil) do
      begin
        duAppendBox(dd, fx, orig[1]+s.smin*ch, fz, fx+cs, orig[1] + s.smax*ch, fz+cs, @fcol);
        s := s.next;
      end;
    end;
  end;
  dd.&end();
end;

procedure duDebugDrawHeightfieldWalkable(dd: TduDebugDraw; const hf: PrcHeightfield);
var orig: PSingle; cs,ch: Single; w,h: Integer; fcol: array [0..5] of Cardinal; x,y: Integer; fx,fz: Single; s: PrcSpan;
begin
  if (dd = nil) then Exit;

  orig := @hf.bmin;
  cs := hf.cs;
  ch := hf.ch;

  w := hf.width;
  h := hf.height;

  //unsigned int fcol[6];
  duCalcBoxColors(fcol, duRGBA(255,255,255,255), duRGBA(217,217,217,255));

  dd.&begin(DU_DRAW_QUADS);

  for y := 0 to h - 1 do
  begin
    for x := 0 to w - 1 do
    begin
      fx := orig[0] + x*cs;
      fz := orig[2] + y*cs;
      s := hf.spans[x + y*w];
      while (s <> nil) do
      begin
        if (s.area = RC_WALKABLE_AREA) then
          fcol[0] := duRGBA(64,128,160,255)
        else if (s.area = RC_NULL_AREA) then
          fcol[0] := duRGBA(64,64,64,255)
        else
          fcol[0] := duMultCol(duIntToCol(s.area, 255), 200);

        duAppendBox(dd, fx, orig[1]+s.smin*ch, fz, fx+cs, orig[1] + s.smax*ch, fz+cs, @fcol);
        s := s.next;
      end;
    end;
  end;

  dd.&end();
end;

procedure duDebugDrawCompactHeightfieldSolid(dd: TduDebugDraw; const chf: PrcCompactHeightfield);
var cs,ch: Single; x,y,i: Integer; fx,fy,fz: Single; c: PrcCompactCell; s: PrcCompactSpan; color: Cardinal;
begin
  if (dd = nil) then Exit;

  cs := chf.cs;
  ch := chf.ch;

  dd.&begin(DU_DRAW_QUADS);

  for y := 0 to chf.height - 1 do
  begin
    for x := 0 to chf.width - 1 do
    begin
      fx := chf.bmin[0] + x*cs;
      fz := chf.bmin[2] + y*cs;
      c := @chf.cells[x+y*chf.width];

      for i := c.index to Integer(c.index+c.count) - 1 do
      begin
        s := @chf.spans[i];

        if (chf.areas[i] = RC_WALKABLE_AREA) then
          color := duRGBA(0,192,255,64)
        else if (chf.areas[i] = RC_NULL_AREA) then
          color := duRGBA(0,0,0,64)
        else
          color := duIntToCol(chf.areas[i], 255);

        fy := chf.bmin[1] + (s.y+1)*ch;
        dd.vertex(fx, fy, fz, color);
        dd.vertex(fx, fy, fz+cs, color);
        dd.vertex(fx+cs, fy, fz+cs, color);
        dd.vertex(fx+cs, fy, fz, color);
      end;
    end;
  end;
  dd.&end();
end;

procedure duDebugDrawCompactHeightfieldRegions(dd: TduDebugDraw; const chf: PrcCompactHeightfield);
var cs,ch: Single; x,y,i: Integer; fx,fy,fz: Single; c: PrcCompactCell; s: PrcCompactSpan; color: Cardinal;
begin
  if (dd = nil) then Exit;

  cs := chf.cs;
  ch := chf.ch;

  dd.&begin(DU_DRAW_QUADS);

  for y := 0 to chf.height - 1 do
  begin
    for x := 0 to chf.width - 1 do
    begin
      fx := chf.bmin[0] + x*cs;
      fz := chf.bmin[2] + y*cs;
      c := @chf.cells[x+y*chf.width];

      for i := c.index to Integer(c.index+c.count) - 1 do
      begin
        s := @chf.spans[i];
        fy := chf.bmin[1] + (s.y)*ch;

        if (s.reg <> 0) then
          color := duIntToCol(s.reg, 192)
        else
          color := duRGBA(0,0,0,64);

        dd.vertex(fx, fy, fz, color);
        dd.vertex(fx, fy, fz+cs, color);
        dd.vertex(fx+cs, fy, fz+cs, color);
        dd.vertex(fx+cs, fy, fz, color);
      end;
    end;
  end;

  dd.&end();
end;


procedure duDebugDrawCompactHeightfieldDistance(dd: TduDebugDraw; const chf: PrcCompactHeightfield);
var cs,ch,maxd,dscale: Single; x,y,i: Integer; fx,fy,fz: Single; c: PrcCompactCell; s: PrcCompactSpan; cd: Byte; color: Cardinal;
begin
  if (dd = nil) then Exit;
  //if (!chf.dist) return;

  cs := chf.cs;
  ch := chf.ch;

  maxd := chf.maxDistance;
  if (maxd < 1.0) then maxd := 1;
  dscale := 255.0 / maxd;

  dd.&begin(DU_DRAW_QUADS);

  for y := 0 to chf.height - 1 do
  begin
    for x := 0 to chf.width - 1 do
    begin
      fx := chf.bmin[0] + x*cs;
      fz := chf.bmin[2] + y*cs;
      c := @chf.cells[x+y*chf.width];

      for i := c.index to Integer(c.index+c.count) - 1 do
      begin
        s := @chf.spans[i];
        fy := chf.bmin[1] + (s.y+1)*ch;
        cd := Trunc(chf.dist[i] * dscale);
        color := duRGBA(cd,cd,cd,255);
        dd.vertex(fx, fy, fz, color);
        dd.vertex(fx, fy, fz+cs, color);
        dd.vertex(fx+cs, fy, fz+cs, color);
        dd.vertex(fx+cs, fy, fz, color);
      end;
    end;
  end;
  dd.&end();
end;

{procedure drawLayerPortals(duDebugDraw* dd, const rcHeightfieldLayer* layer)
begin
  const float cs := layer.cs;
  const float ch := layer.ch;
  const int w := layer.width;
  const int h := layer.height;

  unsigned int pcol := duRGBA(255,255,255,255);

  const int segs[4*4] := begin0,0,0,1, 0,1,1,1, 1,1,1,0, 1,0,0,0end;;

  // Layer portals
  dd.&begin(DU_DRAW_LINES, 2.0f);
  for y := 0 to h - 1 do
  begin
    for x := 0 to w - 1 do
    begin
      const int idx := x+y*w;
      const int lh := (int)layer.heights[idx];
      if (lh == 255) continue;

      for dir := 0 to 3 do
      begin
        if (layer.cons[idx] & (1<<(dir+4)))
        begin
          const int* seg := &segs[dir*4];
          const float ax := layer.bmin[0] + (x+seg[0])*cs;
          const float ay := layer.bmin[1] + (lh+2)*ch;
          const float az := layer.bmin[2] + (y+seg[1])*cs;
          const float bx := layer.bmin[0] + (x+seg[2])*cs;
          const float by := layer.bmin[1] + (lh+2)*ch;
          const float bz := layer.bmin[2] + (y+seg[3])*cs;
          dd.vertex(ax, ay, az, pcol);
          dd.vertex(bx, by, bz, pcol);
        end;
      end;
    end;
  end;
  dd.&end();
end;

procedure duDebugDrawHeightfieldLayer(duDebugDraw* dd, const struct rcHeightfieldLayer& layer, const int idx)
begin
  const float cs := layer.cs;
  const float ch := layer.ch;
  const int w := layer.width;
  const int h := layer.height;

  unsigned int color := duIntToCol(idx+1, 255);

  // Layer bounds
  float bmin[3], bmax[3];
  bmin[0] := layer.bmin[0] + layer.minx*cs;
  bmin[1] := layer.bmin[1];
  bmin[2] := layer.bmin[2] + layer.miny*cs;
  bmax[0] := layer.bmin[0] + (layer.maxx+1)*cs;
  bmax[1] := layer.bmax[1];
  bmax[2] := layer.bmin[2] + (layer.maxy+1)*cs;
  duDebugDrawBoxWire(dd, bmin[0],bmin[1],bmin[2], bmax[0],bmax[1],bmax[2], duTransCol(color,128), 2.0f);

  // Layer height
  dd.&begin(DU_DRAW_QUADS);
  for y := 0 to h - 1 do
  begin
    for x := 0 to w - 1 do
    begin
      const int lidx := x+y*w;
      const int lh := (int)layer.heights[lidx];
      if (h == 0xff) continue;
      const unsigned char area := layer.areas[lidx];

      unsigned int col;
      if (area == RC_WALKABLE_AREA)
        col := duLerpCol(color, duRGBA(0,192,255,64), 32);
      else if (area == RC_NULL_AREA)
        col := duLerpCol(color, duRGBA(0,0,0,64), 32);
      else
        col := duLerpCol(color, duIntToCol(area, 255), 32);

      const float fx := layer.bmin[0] + x*cs;
      const float fy := layer.bmin[1] + (lh+1)*ch;
      const float fz := layer.bmin[2] + y*cs;

      dd.vertex(fx, fy, fz, col);
      dd.vertex(fx, fy, fz+cs, col);
      dd.vertex(fx+cs, fy, fz+cs, col);
      dd.vertex(fx+cs, fy, fz, col);
    end;
  end;
  dd.&end();

  // Portals
  drawLayerPortals(dd, &layer);
end;

procedure duDebugDrawHeightfieldLayers(duDebugDraw* dd, const struct rcHeightfieldLayerSet& lset)
begin
  if (!dd) return;
  for (int i := 0; i < lset.nlayers; ++i)
    duDebugDrawHeightfieldLayer(dd, lset.layers[i], i);
end;

/*
procedure duDebugDrawLayerContours(duDebugDraw* dd, const struct rcLayerContourSet& lcset)
begin
  if (!dd) return;

  const float* orig := lcset.bmin;
  const float cs := lcset.cs;
  const float ch := lcset.ch;

  const unsigned char a := 255;// (unsigned char)(alpha*255.0f);

  const int offs[2*4] := begin-1,0, 0,1, 1,0, 0,-1end;;

  dd.&begin(DU_DRAW_LINES, 2.0f);

  for (int i := 0; i < lcset.nconts; ++i)
  begin
    const rcLayerContour& c := lcset.conts[i];
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
    const rcLayerContour& c := lcset.conts[i];
    unsigned int color := 0;

    for (int j := 0; j < c.nverts; ++j)
    begin
      const unsigned char* va := &c.verts[j*4];

      color := duDarkenCol(duIntToCol(i, a));
      if (va[3] & 0x80)
        color := duRGBA(255,0,0,255);

      float fx := orig[0] + va[0]*cs;
      float fy := orig[1] + (va[1]+1+(i&1))*ch;
      float fz := orig[2] + va[2]*cs;
      dd.vertex(fx,fy,fz, color);
    end;
  end;
  dd.&end();
end;

procedure duDebugDrawLayerPolyMesh(duDebugDraw* dd, const struct rcLayerPolyMesh& lmesh)
begin
  if (!dd) return;

  const int nvp := lmesh.nvp;
  const float cs := lmesh.cs;
  const float ch := lmesh.ch;
  const float* orig := lmesh.bmin;

  const int offs[2*4] := begin-1,0, 0,1, 1,0, 0,-1end;;

  dd.&begin(DU_DRAW_TRIS);

  for (int i := 0; i < lmesh.npolys; ++i)
  begin
    const unsigned short* p := &lmesh.polys[i*nvp*2];

    unsigned int color;
    if (lmesh.areas[i] == RC_WALKABLE_AREA)
      color := duRGBA(0,192,255,64);
    else if (lmesh.areas[i] == RC_NULL_AREA)
      color := duRGBA(0,0,0,64);
    else
      color := duIntToCol(lmesh.areas[i], 255);

    unsigned short vi[3];
    for (int j := 2; j < nvp; ++j)
    begin
      if (p[j] == RC_MESH_NULL_IDX) break;
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
    for j := 0 to nvp - 1 do
    begin
      if (p[j] == RC_MESH_NULL_IDX) break;
      if (p[nvp+j] & 0x8000) continue;
      const int nj := (j+1 >= nvp || p[j+1] == RC_MESH_NULL_IDX) ? 0 : j+1;
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
    for j := 0 to nvp - 1 do
    begin
      if (p[j] == RC_MESH_NULL_IDX) break;
      if ((p[nvp+j] & 0x8000) == 0) continue;
      const int nj := (j+1 >= nvp || p[j+1] == RC_MESH_NULL_IDX) ? 0 : j+1;
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
end;
*/}

procedure getContourCenter(const cont: PrcContour; const orig: PSingle; cs,ch: Single; center: PSingle);
var i: Integer; v: PInteger; s: Single;
begin
  center[0] := 0;
  center[1] := 0;
  center[2] := 0;
  if (cont.nverts = 0) then
    Exit;
  for  i := 0 to cont.nverts - 1 do
  begin
    v := @cont.verts[i*4];
    center[0] := center[0] + v[0];
    center[1] := center[1] + v[1];
    center[2] := center[2] + v[2];
  end;
  s := 1.0 / cont.nverts;
  center[0] := center[0] * s * cs;
  center[1] := center[1] * s * ch;
  center[2] := center[2] * s * cs;
  center[0] := center[0] + orig[0];
  center[1] := center[1] + orig[1] + 4*ch;
  center[2] := center[2] + orig[2];
end;

function findContourFromSet(const cset: PrcContourSet; reg: Word): PrcContour;
var i: Integer;
begin
  for i := 0 to cset.nconts - 1 do
  begin
    if (cset.conts[i].reg = reg) then
      Exit(@cset.conts[i]);
  end;
  Result := nil;
end;

procedure duDebugDrawRegionConnections(dd: TduDebugDraw; const cset: PrcContourSet; const alpha: Single = 1.0);
var orig: PSingle; cs,ch: Single; pos,pos2: array [0..2] of Single; color: Cardinal; i,j: Integer; cont,cont2: PrcContour; v: PInteger;
a: Byte; col: Cardinal;
begin
  if (dd = nil) then Exit;

  orig := @cset.bmin;
  cs := cset.cs;
  ch := cset.ch;

  // Draw centers

  color := duRGBA(0,0,0,196);

  dd.&begin(DU_DRAW_LINES, 2.0);

  for i := 0 to cset.nconts - 1 do
  begin
    cont := @cset.conts[i];
    getContourCenter(cont, orig, cs, ch, @pos);
    for j := 0 to cont.nverts - 1 do
    begin
      v := @cont.verts[j*4];
      if (Word(v[3]) = 0) or (Word(v[3]) < cont.reg) then continue;
      cont2 := findContourFromSet(cset, Word(v[3]));
      if (cont2 <> nil) then
      begin
        getContourCenter(cont2, orig, cs, ch, @pos2);
        duAppendArc(dd, pos[0],pos[1],pos[2], pos2[0],pos2[1],pos2[2], 0.25, 0.6, 0.6, color);
      end;
    end;
  end;

  dd.&end();

  a := Trunc(alpha * 255.0);

  dd.&begin(DU_DRAW_POINTS, 7.0);

  for i := 0 to cset.nconts - 1 do
  begin
    cont := @cset.conts[i];
    col := duDarkenCol(duIntToCol(cont.reg,a));
    getContourCenter(cont, orig, cs, ch, @pos);
    dd.vertex(@pos, col);
  end;
  dd.&end();
end;

procedure duDebugDrawRawContours(dd: TduDebugDraw; const cset: PrcContourSet; const alpha: Single = 1.0);
var orig: PSingle; cs,ch: Single; a: Byte; i,j: Integer; c: PrcContour; color,colv: Cardinal; v: PInteger; fx,fy,fz: Single;
off: Single;
begin
  if (dd = nil) then Exit;

  orig := @cset.bmin;
  cs := cset.cs;
  ch := cset.ch;

  a := Trunc(alpha*255.0);

  dd.&begin(DU_DRAW_LINES, 2.0);

  for i := 0 to cset.nconts - 1 do
  begin
    c := @cset.conts[i];
    color := duIntToCol(c.reg, a);

    for j := 0 to c.nrverts - 1 do
    begin
      v := @c.rverts[j*4];
      fx := orig[0] + v[0]*cs;
      fy := orig[1] + (v[1]+1+(i and 1))*ch;
      fz := orig[2] + v[2]*cs;
      dd.vertex(fx,fy,fz,color);
      if (j > 0) then
        dd.vertex(fx,fy,fz,color);
    end;
    // Loop last segment.
    v := @c.rverts[0];
    fx := orig[0] + v[0]*cs;
    fy := orig[1] + (v[1]+1+(i and 1))*ch;
    fz := orig[2] + v[2]*cs;
    dd.vertex(fx,fy,fz,color);
  end;
  dd.&end();

  dd.&begin(DU_DRAW_POINTS, 2.0);

  for i := 0 to cset.nconts - 1 do
  begin
    c := @cset.conts[i];
    color := duDarkenCol(duIntToCol(c.reg, a));

    for j := 0 to c.nrverts - 1 do
    begin
      v := @c.rverts[j*4];
      off := 0;
      colv := color;
      if (v[3] and RC_BORDER_VERTEX) <> 0 then
      begin
        colv := duRGBA(255,255,255,a);
        off := ch*2;
      end;

      fx := orig[0] + v[0]*cs;
      fy := orig[1] + (v[1]+1+(i and 1))*ch + off;
      fz := orig[2] + v[2]*cs;
      dd.vertex(fx,fy,fz, colv);
    end;
  end;
  dd.&end();
end;

procedure duDebugDrawContours(dd: TduDebugDraw; const cset: PrcContourSet; const alpha: Single = 1.0);
var orig: PSingle; cs,ch: Single; a: Byte; i,j,k: Integer; c: PrcContour; color,bcolor,col,colv: Cardinal; va,vb,v: PInteger; fx,fy,fz: Single;
off: Single;
begin
  if (dd = nil) then Exit;

  orig := @cset.bmin;
  cs := cset.cs;
  ch := cset.ch;

  a := Trunc(alpha*255.0);

  dd.&begin(DU_DRAW_LINES, 2.5);

  for i := 0 to cset.nconts - 1 do
  begin
    c := @cset.conts[i];
    if (c.nverts = 0) then
      continue;
    color := duIntToCol(c.reg, a);
    bcolor := duLerpCol(color,duRGBA(255,255,255,a),128);
    //for (int j := 0, k := c.nverts-1; j < c.nverts; k=j++)
    j := 0;
    k := c.nverts-1;
    while (j < c.nverts) do
    begin
      va := @c.verts[k*4];
      vb := @c.verts[j*4];
      col := IfThen(va[3] and RC_AREA_BORDER <> 0, bcolor, color);

      fx := orig[0] + va[0]*cs;
      fy := orig[1] + (va[1]+1+(i and 1))*ch;
      fz := orig[2] + va[2]*cs;
      dd.vertex(fx,fy,fz, col);
      fx := orig[0] + vb[0]*cs;
      fy := orig[1] + (vb[1]+1+(i and 1))*ch;
      fz := orig[2] + vb[2]*cs;
      dd.vertex(fx,fy,fz, col);

      k := j;
      Inc(j);
    end;
  end;
  dd.&end();

  dd.&begin(DU_DRAW_POINTS, 3.0);

  for i := 0 to cset.nconts - 1 do
  begin
    c := @cset.conts[i];
    color := duDarkenCol(duIntToCol(c.reg, a));
    for j := 0 to c.nverts - 1 do
    begin
      v := @c.verts[j*4];
      off := 0;
      colv := color;
      if (v[3] and RC_BORDER_VERTEX) <> 0 then
      begin
        colv := duRGBA(255,255,255,a);
        off := ch*2;
      end;

      fx := orig[0] + v[0]*cs;
      fy := orig[1] + (v[1]+1+(i and 1))*ch + off;
      fz := orig[2] + v[2]*cs;
      dd.vertex(fx,fy,fz, colv);
    end;
  end;
  dd.&end();
end;

procedure duDebugDrawPolyMesh(dd: TduDebugDraw; const mesh: PrcPolyMesh);
var nvp: Integer; cs,ch: Single; orig: PSingle; i,j,k: Integer; p,v: PWord; color,col,coln,colb,colv: Cardinal;
vi: array [0..2] of Word; x,y,z: Single; nj: Integer;
begin
  if (dd = nil) then Exit;

  nvp := mesh.nvp;
  cs := mesh.cs;
  ch := mesh.ch;
  orig := @mesh.bmin;

  dd.&begin(DU_DRAW_TRIS);

  for i := 0 to mesh.npolys - 1 do
  begin
    p := @mesh.polys[i*nvp*2];

    if (mesh.areas[i] = RC_WALKABLE_AREA) then
      color := duRGBA(0,192,255,64)
    else if (mesh.areas[i] = RC_NULL_AREA) then
      color := duRGBA(0,0,0,64)
    else
      color := duIntToCol(mesh.areas[i], 255);

    for j := 2 to nvp - 1 do
    begin
      if (p[j] = RC_MESH_NULL_IDX) then break;
      vi[0] := p[0];
      vi[1] := p[j-1];
      vi[2] := p[j];
      for k := 0 to 2 do
      begin
        v := @mesh.verts[vi[k]*3];
        x := orig[0] + v[0]*cs;
        y := orig[1] + (v[1]+1)*ch;
        z := orig[2] + v[2]*cs;
        dd.vertex(x,y,z, color);
      end;
    end;
  end;
  dd.&end();

  // Draw neighbours edges
  coln := duRGBA(0,48,64,32);
  dd.&begin(DU_DRAW_LINES, 1.5);
  for i := 0 to mesh.npolys - 1 do
  begin
    p := @mesh.polys[i*nvp*2];
    for j := 0 to nvp - 1 do
    begin
      if (p[j] = RC_MESH_NULL_IDX) then break;
      if (p[nvp+j] and $8000 <> 0) then continue;
      nj := IfThen((j+1 >= nvp) or (p[j+1] = RC_MESH_NULL_IDX), 0, j+1);
      vi[0] := p[j]; vi[1] := p[nj];

      for k := 0 to 1 do
      begin
        v := @mesh.verts[vi[k]*3];
        x := orig[0] + v[0]*cs;
        y := orig[1] + (v[1]+1)*ch + 0.1;
        z := orig[2] + v[2]*cs;
        dd.vertex(x, y, z, coln);
      end;
    end;
  end;
  dd.&end();

  // Draw boundary edges
  colb := duRGBA(0,48,64,220);
  dd.&begin(DU_DRAW_LINES, 2.5);
  for i := 0 to mesh.npolys - 1 do
  begin
    p := @mesh.polys[i*nvp*2];
    for j := 0 to nvp - 1 do
    begin
      if (p[j] = RC_MESH_NULL_IDX) then break;
      if ((p[nvp+j] and $8000) = 0) then continue;
      nj := IfThen((j+1 >= nvp) or (p[j+1] = RC_MESH_NULL_IDX), 0, j+1);
      vi[0] := p[j]; vi[1] := p[nj];

      col := colb;
      if ((p[nvp+j] and $f) <> $f) then
        col := duRGBA(255,255,255,128);
      for k := 0 to 1 do
      begin
        v := @mesh.verts[vi[k]*3];
        x := orig[0] + v[0]*cs;
        y := orig[1] + (v[1]+1)*ch + 0.1;
        z := orig[2] + v[2]*cs;
        dd.vertex(x, y, z, col);
      end;
    end;
  end;
  dd.&end();

  dd.&begin(DU_DRAW_POINTS, 3.0);
  colv := duRGBA(0,0,0,220);
  for i := 0 to mesh.nverts - 1 do
  begin
    v := @mesh.verts[i*3];
    x := orig[0] + v[0]*cs;
    y := orig[1] + (v[1]+1)*ch + 0.1;
    z := orig[2] + v[2]*cs;
    dd.vertex(x,y,z, colv);
  end;
  dd.&end();
end;

procedure duDebugDrawPolyMeshDetail(dd: TduDebugDraw; const dmesh: PrcPolyMeshDetail);
var i,j,k,kp: Integer; m: PCardinal; bverts, btris: Cardinal; ntris: Integer; verts: PSingle; tris,t: PByte; color,coli,cole,colv: Cardinal;
ef: Byte; nverts: Integer;
begin
  if (dd = nil) then Exit;

  dd.&begin(DU_DRAW_TRIS);

  for i := 0 to dmesh.nmeshes - 1 do
  begin
    m := @dmesh.meshes[i*4];
    bverts := m[0];
    btris := m[2];
    ntris := integer(m[3]);
    verts := @dmesh.verts[bverts*3];
    tris := @dmesh.tris[btris*4];

    color := duIntToCol(i, 192);

    for j := 0 to ntris - 1 do
    begin
      dd.vertex(@verts[tris[j*4+0]*3], color);
      dd.vertex(@verts[tris[j*4+1]*3], color);
      dd.vertex(@verts[tris[j*4+2]*3], color);
    end;
  end;
  dd.&end();

  // Internal edges.
  dd.&begin(DU_DRAW_LINES, 1.0);
  coli := duRGBA(0,0,0,64);
  for i := 0 to dmesh.nmeshes - 1 do
  begin
    m := @dmesh.meshes[i*4];
    bverts := m[0];
    btris := m[2];
    ntris := Integer(m[3]);
    verts := @dmesh.verts[bverts*3];
    tris := @dmesh.tris[btris*4];

    for j := 0 to ntris - 1 do
    begin
      t := @tris[j*4];
      k := 0;
      kp := 2;
      while (k < 3) do
      begin
        ef := (t[3] shr (kp*2)) and $3;
        if (ef = 0) then
        begin
          // Internal edge
          if (t[kp] < t[k]) then
          begin
            dd.vertex(@verts[t[kp]*3], coli);
            dd.vertex(@verts[t[k]*3], coli);
          end;
        end;

        kp := k;
        Inc(k);
      end;
    end;
  end;
  dd.&end();

  // External edges.
  dd.&begin(DU_DRAW_LINES, 2.0);
  cole := duRGBA(0,0,0,64);
  for i := 0 to dmesh.nmeshes - 1 do
  begin
    m := @dmesh.meshes[i*4];
    bverts := m[0];
    btris := m[2];
    ntris := Integer(m[3]);
    verts := @dmesh.verts[bverts*3];
    tris := @dmesh.tris[btris*4];

    for j := 0 to ntris - 1 do
    begin
      t := @tris[j*4];
      k := 0;
      kp := 2;
      while (k < 3) do
      begin
        ef := (t[3] shr (kp*2)) and $3;
        if (ef <> 0) then
        begin
          // Ext edge
          dd.vertex(@verts[t[kp]*3], cole);
          dd.vertex(@verts[t[k]*3], cole);
        end;

        kp := k;
        Inc(k);
      end;
    end;
  end;
  dd.&end();

  dd.&begin(DU_DRAW_POINTS, 3.0);
  colv := duRGBA(0,0,0,64);
  for i := 0 to dmesh.nmeshes - 1 do
  begin
    m := @dmesh.meshes[i*4];
    bverts := m[0];
    nverts := Integer(m[1]);
    verts := @dmesh.verts[bverts*3];
    for j := 0 to nverts - 1 do
      dd.vertex(@verts[j*3], colv);
  end;
  dd.&end();
end;

end.
