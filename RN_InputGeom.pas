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

unit RN_InputGeom;
interface
uses Math, SysUtils, RN_Helper, RN_ChunkyTriMesh, RN_MeshLoaderObj, RN_Recast, RN_DebugDraw;

const MAX_CONVEXVOL_PTS = 12;

type
  TConvexVolume = record
    verts: array [0..MAX_CONVEXVOL_PTS*3-1] of Single;
    hmin, hmax: Single;
    nverts: Integer;
    area: Integer;
  end;
  PConvexVolume = ^TConvexVolume;

  const MAX_VOLUMES = 256;
  type
    TIGConvexVolumeArray = array [0..MAX_VOLUMES-1] of TConvexVolume;

  TInputGeom = class
  private
    m_chunkyMesh: TrcChunkyTriMesh;
    m_mesh: TrcMeshLoaderObj;
    m_meshBMin, m_meshBMax: array [0..2] of Single;

    /// @name Off-Mesh connections.
    ///@{
    const MAX_OFFMESH_CONNECTIONS = 256;
    var
    m_offMeshConVerts: array [0..MAX_OFFMESH_CONNECTIONS*3*2-1] of Single;
    m_offMeshConRads: array [0..MAX_OFFMESH_CONNECTIONS-1] of Single;
    m_offMeshConDirs: array [0..MAX_OFFMESH_CONNECTIONS-1] of Byte;
    m_offMeshConAreas: array [0..MAX_OFFMESH_CONNECTIONS-1] of Byte;
    m_offMeshConFlags: array [0..MAX_OFFMESH_CONNECTIONS-1] of Word;
    m_offMeshConId: array [0..MAX_OFFMESH_CONNECTIONS-1] of Cardinal;
    m_offMeshConCount: Integer;
    ///@}

    /// @name Convex Volumes.
    ///@{
    var
    m_volumes: TIGConvexVolumeArray;
    m_volumeCount: Integer;
    ///@}

  public
    destructor Destroy; override;
    function loadMesh(ctx: TrcContext; const filepath: string): Boolean;

    function load(ctx: TrcContext; const filepath: string): Boolean;
    function save(const filepath: string): Boolean;

    /// Method to return static mesh data.
    property getMesh: TrcMeshLoaderObj read m_mesh;
    function getMeshBoundsMin: PSingle;
    function getMeshBoundsMax: PSingle;
    property getChunkyMesh: TrcChunkyTriMesh read m_chunkyMesh;
    function raycastMesh(src, dst: PSingle; tmin: PSingle): Boolean;

    /// @name Off-Mesh connections.
    ///@{
    property getOffMeshConnectionCount: Integer read m_offMeshConCount;
    function getOffMeshConnectionVerts: PSingle;
    function getOffMeshConnectionRads: PSingle;
    function getOffMeshConnectionDirs: PByte;
    function getOffMeshConnectionAreas: PByte;
    function getOffMeshConnectionFlags: PWord;
    function getOffMeshConnectionId: PCardinal;
    procedure addOffMeshConnection(const spos, epos: PSingle; const rad: Single;
                  bidir, area: Byte; flags: Word);
    procedure deleteOffMeshConnection(i: Integer);
    procedure drawOffMeshConnections(dd: TduDebugDraw; hilight: Boolean = false);
    ///@}

    /// @name Box Volumes.
    ///@{
    property getConvexVolumeCount: Integer read m_volumeCount;
    property getConvexVolumes: TIGConvexVolumeArray read m_volumes;
    procedure addConvexVolume(const verts: PSingle; const nverts: Integer;
               const minh, maxh: Single; area: Byte);
    procedure deleteConvexVolume(i: Integer);
    procedure drawConvexVolumes(dd: TduDebugDraw; hilight: Boolean = false);
    ///@}
  end;


implementation
uses RN_RecastHelper;

function intersectSegmentTriangle(sp, sq, a, b, c: PSingle; t: PSingle): Boolean;
var v,w,d: Single; ab,ac,qp,ap,norm,e: array [0..2] of Single;
begin
  rcVsub(@ab[0], b, a);
  rcVsub(@ac[0], c, a);
  rcVsub(@qp[0], sp, sq);

  // Compute triangle normal. Can be precalculated or cached if
  // intersecting multiple segments against the same triangle
  rcVcross(@norm[0], @ab[0], @ac[0]);

  // Compute denominator d. If d <= 0, segment is parallel to or points
  // away from triangle, so exit early
  d := rcVdot(@qp[0], @norm[0]);
  if (d <= 0.0) then Exit(false);

  // Compute intersection t value of pq with plane of triangle. A ray
  // intersects iff 0 <= t. Segment intersects iff 0 <= t <= 1. Delay
  // dividing by d until intersection has been found to pierce triangle
  rcVsub(@ap[0], sp, a);
  t^ := rcVdot(@ap[0], @norm[0]);
  if (t^ < 0.0) then Exit(false);
  if (t^ > d) then Exit(false); // For segment; exclude this code line for a ray test

  // Compute barycentric coordinate components and test if within bounds
  rcVcross(@e[0], @qp[0], @ap[0]);
  v := rcVdot(@ac[0], @e[0]);
  if (v < 0.0) or (v > d) then Exit(false);
  w := -rcVdot(@ab[0], @e[0]);
  if (w < 0.0) or (v + w > d) then Exit(false);

  // Segment/ray intersects triangle. Perform delayed division
  t^ := t^/d;

  Result := True;
end;

{static char* parseRow(char* buf, char* bufEnd, char* row, int len)
begin
  bool start := true;
  bool done := false;
  int n := 0;
  while (not done and (buf < bufEnd) do
  begin
    char c := *buf;
    buf++;
    // multirow
    switch (c)
    begin
      case '\n':
        if (start) break;
        done := true;
        break;
      case '\r':
        break;
      case '\t':
      case ' ':
        if (start) break;
      default:
        start := false;
        row[n++] := c;
        if (n >= len-1)
          done := true;
        break;
    end;
  end;
  row[n] := '\0';
  return buf;
end;}

destructor TInputGeom.Destroy;
begin
  FreeAndNil(m_mesh);
  inherited;
end;

function TInputGeom.getMeshBoundsMax: PSingle;
begin
  Result := @m_meshBMax[0];
end;

function TInputGeom.getMeshBoundsMin: PSingle;
begin
  Result := @m_meshBMin[0];
end;

function TInputGeom.getOffMeshConnectionAreas: PByte;
begin
  Result := @m_offMeshConAreas[0];
end;

function TInputGeom.getOffMeshConnectionDirs: PByte;
begin
  Result := @m_offMeshConDirs[0];
end;

function TInputGeom.getOffMeshConnectionFlags: PWord;
begin
  Result := @m_offMeshConFlags[0];
end;

function TInputGeom.getOffMeshConnectionId: PCardinal;
begin
  Result := @m_offMeshConId[0];
end;

function TInputGeom.getOffMeshConnectionRads: PSingle;
begin
  Result := @m_offMeshConRads[0];
end;

function TInputGeom.getOffMeshConnectionVerts: PSingle;
begin
  Result := @m_offMeshConVerts[0];
end;

function TInputGeom.loadMesh(ctx: TrcContext; const filepath: string): Boolean;
begin
  if (m_mesh <> nil) then
  begin
    m_mesh.Free;
    m_mesh := nil;
  end;
  m_offMeshConCount := 0;
  m_volumeCount := 0;

  m_mesh := TrcMeshLoaderObj.Create;
  if (not m_mesh.load(filepath)) then
  begin
    ctx.log(RC_LOG_ERROR, Format('buildTiledNavigation: Could not load ''%s''', [filepath]));
    Exit(false);
  end;

  rcCalcBounds(m_mesh.getVerts, m_mesh.getVertCount, @m_meshBMin[0], @m_meshBMax[0]);

  if (not rcCreateChunkyTriMesh(m_mesh.getVerts, m_mesh.getTris, m_mesh.getTriCount, 256, @m_chunkyMesh)) then
  begin
    ctx.log(RC_LOG_ERROR, 'buildTiledNavigation: Failed to build chunky mesh.');
    Exit(false);
  end;

  Result := true;
end;

function TInputGeom.load(ctx: TrcContext; const filepath: string): Boolean;
begin
{  char* buf := 0;
  FILE* fp := fopen(filePath, 'rb');
  if (!fp)
    return false;
  fseek(fp, 0, SEEK_END);
  int bufSize := ftell(fp);
  fseek(fp, 0, SEEK_SET);
  buf := new char[bufSize];
  if (!buf)
  begin
    fclose(fp);
    return false;
  end;
  size_t readLen := fread(buf, bufSize, 1, fp);
  fclose(fp);
  if (readLen != 1)
  begin
    return false;
  end;

  m_offMeshConCount := 0;
  m_volumeCount := 0;
  m_mesh.Free;
  m_mesh := nil;

  char* src := buf;
  char* srcEnd := buf + bufSize;
  char row[512];
  while (src < srcEnd)
  begin
    // Parse one row
    row[0] := '\0';
    src := parseRow(src, srcEnd, row, sizeof(row)/sizeof(char));
    if (row[0] == 'f')
    begin
      // File name.
      const char* name := row+1;
      // Skip white spaces
      while (*name && isspace(*name))
        name++;
      if (*name)
      begin
        if (!loadMesh(ctx, name))
        begin
          delete [] buf;
          return false;
        end;
      end;
    end;
    else if (row[0] == 'c')
    begin
      // Off-mesh connection
      if (m_offMeshConCount < MAX_OFFMESH_CONNECTIONS)
      begin
        float* v := &m_offMeshConVerts[m_offMeshConCount*3*2];
        int bidir, area := 0, flags := 0;
        float rad;
        sscanf(row+1, '%f %f %f  %f %f %f %f %d %d %d',
             &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &rad, &bidir, &area, &flags);
        m_offMeshConRads[m_offMeshConCount] := rad;
        m_offMeshConDirs[m_offMeshConCount] := (unsigned char)bidir;
        m_offMeshConAreas[m_offMeshConCount] := (unsigned char)area;
        m_offMeshConFlags[m_offMeshConCount] := (unsigned short)flags;
        m_offMeshConCount++;
      end;
    end;
    else if (row[0] == 'v')
    begin
      // Convex volumes
      if (m_volumeCount < MAX_VOLUMES)
      begin
        ConvexVolume* vol := &m_volumes[m_volumeCount++];
        sscanf(row+1, '%d %d %f %f', &vol.nverts, &vol.area, &vol.hmin, &vol.hmax);
        for (int i := 0; i < vol.nverts; ++i)
        begin
          row[0] := '\0';
          src := parseRow(src, srcEnd, row, sizeof(row)/sizeof(char));
          sscanf(row, '%f %f %f', &vol.verts[i*3+0], &vol.verts[i*3+1], &vol.verts[i*3+2]);
        end;
      end;
    end;
  end;

  delete [] buf;
}
  Result := true;
end;

function TInputGeom.save(const filepath: string): Boolean;
begin
  if (m_mesh = nil) then Exit(false);

{  FILE* fp := fopen(filepath, 'w');
  if (!fp) return false;

  // Store mesh filename.
  fprintf(fp, 'f %s\n', m_mesh.getFileName());

  // Store off-mesh links.
  for (int i := 0; i < m_offMeshConCount; ++i)
  begin
    const float* v := &m_offMeshConVerts[i*3*2];
    const float rad := m_offMeshConRads[i];
    const int bidir := m_offMeshConDirs[i];
    const int area := m_offMeshConAreas[i];
    const int flags := m_offMeshConFlags[i];
    fprintf(fp, 'c %f %f %f  %f %f %f  %f %d %d %d\n',
        v[0], v[1], v[2], v[3], v[4], v[5], rad, bidir, area, flags);
  end;

  // Convex volumes
  for (int i := 0; i < m_volumeCount; ++i)
  begin
    ConvexVolume* vol := &m_volumes[i];
    fprintf(fp, 'v %d %d %f %f\n', vol.nverts, vol.area, vol.hmin, vol.hmax);
    for (int j := 0; j < vol.nverts; ++j)
      fprintf(fp, '%f %f %f\n', vol.verts[j*3+0], vol.verts[j*3+1], vol.verts[j*3+2]);
  end;

  fclose(fp);
}
  Result := true;
end;

function isectSegAABB(const sp, sq, amin, amax: PSingle;
             tmin, tmax: PSingle): Boolean;
const EPS = 0.000001;
var d: array [0..2] of Single; i: Integer; ood,t1,t2,tmp: Single;
begin
  d[0] := sq[0] - sp[0];
  d[1] := sq[1] - sp[1];
  d[2] := sq[2] - sp[2];
  tmin^ := 0.0;
  tmax^ := 1.0;

  for i := 0 to 2 do
  begin
    if (Abs(d[i]) < EPS) then
    begin
      if (sp[i] < amin[i]) or (sp[i] > amax[i]) then
        Exit(false);
    end
    else
    begin
      ood := 1.0 / d[i];
      t1 := (amin[i] - sp[i]) * ood;
      t2 := (amax[i] - sp[i]) * ood;
      if (t1 > t2) then begin tmp := t1; t1 := t2; t2 := tmp; end;
      if (t1 > tmin^) then tmin^ := t1;
      if (t2 < tmax^) then tmax^ := t2;
      if (tmin^ > tmax^) then Exit(false);
    end;
  end;

  Result := true;
end;


function TInputGeom.raycastMesh(src, dst: PSingle; tmin: PSingle): Boolean;
var dir: array [0..2] of Single; p,q: array [0..1] of Single; btmin,btmax,t: Single; hit: Boolean; ncid: Integer; verts: PSingle;
cid: array [0..511] of Integer; tris: PInteger; i,j,ntris: Integer; node: PrcChunkyTriMeshNode;
begin
  rcVsub(@dir[0], dst, src);

  // Prune hit ray.

  if (not isectSegAABB(src, dst, @m_meshBMin[0], @m_meshBMax[0], @btmin, @btmax)) then Exit(false);

  p[0] := src[0] + (dst[0]-src[0])*btmin;
  p[1] := src[2] + (dst[2]-src[2])*btmin;
  q[0] := src[0] + (dst[0]-src[0])*btmax;
  q[1] := src[2] + (dst[2]-src[2])*btmax;

  ncid := rcGetChunksOverlappingSegment(@m_chunkyMesh, @p[0], @q[0], @cid[0], 512);
  if (ncid = 0) then
    Exit(false);

  tmin^ := 1.0;
  hit := false;
  verts := m_mesh.getVerts;

  for i := 0 to ncid - 1 do
  begin
    node := @m_chunkyMesh.nodes[cid[i]];
    tris := @m_chunkyMesh.tris[node.i*3];
    ntris := node.n;

    for j := 0 to ntris - 1 do
    begin
      t := 1;
      if (intersectSegmentTriangle(src, dst,
                     @verts[tris[j*3]*3],
                     @verts[tris[j*3+1]*3],
                     @verts[tris[j*3+2]*3], @t)) then
      begin
        if (t < tmin^) then
          tmin^ := t;
        hit := true;
      end;
    end;
  end;

  Result := hit;
end;

procedure TInputGeom.addOffMeshConnection(const spos, epos: PSingle; const rad: Single;
                  bidir, area: Byte; flags: Word);
var v: PSingle;
begin
  if (m_offMeshConCount >= MAX_OFFMESH_CONNECTIONS) then Exit;
  v := @m_offMeshConVerts[m_offMeshConCount*3*2];
  m_offMeshConRads[m_offMeshConCount] := rad;
  m_offMeshConDirs[m_offMeshConCount] := bidir;
  m_offMeshConAreas[m_offMeshConCount] := area;
  m_offMeshConFlags[m_offMeshConCount] := flags;
  m_offMeshConId[m_offMeshConCount] := 1000 + m_offMeshConCount;
  rcVcopy(PSingle(v[0]), spos);
  rcVcopy(PSingle(v[3]), epos);
  Inc(m_offMeshConCount);
end;

procedure TInputGeom.deleteOffMeshConnection(i: Integer);
var src,dst: PSingle;
begin
  Dec(m_offMeshConCount);
  src := @m_offMeshConVerts[m_offMeshConCount*3*2];
  dst := @m_offMeshConVerts[i*3*2];
  rcVcopy(PSingle(dst[0]), PSingle(src[0]));
  rcVcopy(PSingle(dst[3]), PSingle(src[3]));
  m_offMeshConRads[i] := m_offMeshConRads[m_offMeshConCount];
  m_offMeshConDirs[i] := m_offMeshConDirs[m_offMeshConCount];
  m_offMeshConAreas[i] := m_offMeshConAreas[m_offMeshConCount];
  m_offMeshConFlags[i] := m_offMeshConFlags[m_offMeshConCount];
end;

procedure TInputGeom.drawOffMeshConnections(dd: TduDebugDraw; hilight: Boolean = false);
var conColor,baseColor: Cardinal; i: Integer; v: PSingle;
begin
  conColor := duRGBA(192,0,128,192);
  baseColor := duRGBA(0,0,0,64);
  dd.depthMask(false);

  dd.&begin(DU_DRAW_LINES, 2.0);
  for i := 0 to m_offMeshConCount - 1 do
  begin
    v := @m_offMeshConVerts[i*3*2];

    dd.vertex(v[0],v[1],v[2], baseColor);
    dd.vertex(v[0],v[1]+0.2,v[2], baseColor);

    dd.vertex(v[3],v[4],v[5], baseColor);
    dd.vertex(v[3],v[4]+0.2,v[5], baseColor);

    duAppendCircle(dd, v[0],v[1]+0.1,v[2], m_offMeshConRads[i], baseColor);
    duAppendCircle(dd, v[3],v[4]+0.1,v[5], m_offMeshConRads[i], baseColor);

    if (hilight) then
    begin
      duAppendArc(dd, v[0],v[1],v[2], v[3],v[4],v[5], 0.25,
            Byte(m_offMeshConDirs[i] and 1) * 0.6, 0.6, conColor);
    end;
  end;
  dd.&end();

  dd.depthMask(true);
end;

procedure TInputGeom.addConvexVolume(const verts: PSingle; const nverts: Integer;
               const minh, maxh: Single; area: Byte);
var vol: PConvexVolume;
begin
  if (m_volumeCount >= MAX_VOLUMES) then Exit;
  vol := @m_volumes[m_volumeCount];
  Inc(m_volumeCount);

  Move(verts^, vol.verts[0], sizeof(Single)*3*nverts);
  vol.hmin := minh;
  vol.hmax := maxh;
  vol.nverts := nverts;
  vol.area := area;
end;

procedure TInputGeom.deleteConvexVolume(i: Integer);
begin
  Dec(m_volumeCount);
  m_volumes[i] := m_volumes[m_volumeCount];
end;

procedure TInputGeom.drawConvexVolumes(dd: TduDebugDraw; hilight: Boolean = false);
var i,k,j: Integer; vol: PConvexVolume; col: Cardinal; va,vb: PSingle;
begin
  dd.depthMask(false);

  dd.&begin(DU_DRAW_TRIS);

  for i := 0 to m_volumeCount - 1 do
  begin
    vol := @m_volumes[i];
    col := duIntToCol(vol.area, 32);
    j := 0;
    k := vol.nverts-1;
    while (j < vol.nverts) do
    begin
      va := @vol.verts[k*3];
      vb := @vol.verts[j*3];

      dd.vertex(vol.verts[0],vol.hmax,vol.verts[2], col);
      dd.vertex(vb[0],vol.hmax,vb[2], col);
      dd.vertex(va[0],vol.hmax,va[2], col);

      dd.vertex(va[0],vol.hmin,va[2], duDarkenCol(col));
      dd.vertex(va[0],vol.hmax,va[2], col);
      dd.vertex(vb[0],vol.hmax,vb[2], col);

      dd.vertex(va[0],vol.hmin,va[2], duDarkenCol(col));
      dd.vertex(vb[0],vol.hmax,vb[2], col);
      dd.vertex(vb[0],vol.hmin,vb[2], duDarkenCol(col));

      k := j;
      Inc(j);
    end;
  end;

  dd.&end();

  dd.&begin(DU_DRAW_LINES, 2.0);
  for i := 0 to m_volumeCount - 1 do
  begin
    vol := @m_volumes[i];
    col := duIntToCol(vol.area, 220);
    j := 0;
    k := vol.nverts-1;
    while (j < vol.nverts) do
    begin
      va := @vol.verts[k*3];
      vb := @vol.verts[j*3];
      dd.vertex(va[0],vol.hmin,va[2], duDarkenCol(col));
      dd.vertex(vb[0],vol.hmin,vb[2], duDarkenCol(col));
      dd.vertex(va[0],vol.hmax,va[2], col);
      dd.vertex(vb[0],vol.hmax,vb[2], col);
      dd.vertex(va[0],vol.hmin,va[2], duDarkenCol(col));
      dd.vertex(va[0],vol.hmax,va[2], col);

      k := j;
      Inc(j);
    end;
  end;
  dd.&end();

  dd.&begin(DU_DRAW_POINTS, 3.0);
  for i := 0 to m_volumeCount - 1 do
  begin
    vol := @m_volumes[i];
    col := duDarkenCol(duIntToCol(vol.area, 255));
    for j := 0 to vol.nverts - 1 do
    begin
      dd.vertex(vol.verts[j*3+0],vol.verts[j*3+1]+0.1,vol.verts[j*3+2], col);
      dd.vertex(vol.verts[j*3+0],vol.hmin,vol.verts[j*3+2], col);
      dd.vertex(vol.verts[j*3+0],vol.hmax,vol.verts[j*3+2], col);
    end;
  end;
  dd.&end();

  dd.depthMask(true);
end;

end.
