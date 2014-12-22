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

unit RN_DetourNavMeshBuilder;
interface

type
  PPByte = ^PByte;

/// Represents the source data used to build an navigation mesh tile.
/// @ingroup detour
PdtNavMeshCreateParams = ^TdtNavMeshCreateParams;
TdtNavMeshCreateParams = record
  /// @name Polygon Mesh Attributes
  /// Used to create the base navigation graph.
  /// See #rcPolyMesh for details related to these attributes.
  /// @{

  verts: PWord;      ///< The polygon mesh vertices. [(x, y, z) * #vertCount] [Unit: vx]
  vertCount: Integer;              ///< The number vertices in the polygon mesh. [Limit: >= 3]
  polys: PWord;      ///< The polygon data. [Size: #polyCount * 2 * #nvp]
  polyFlags: PWord;    ///< The user defined flags assigned to each polygon. [Size: #polyCount]
  polyAreas: PByte;      ///< The user defined area ids assigned to each polygon. [Size: #polyCount]
  polyCount: Integer;              ///< Number of polygons in the mesh. [Limit: >= 1]
  nvp: Integer;                ///< Number maximum number of vertices per polygon. [Limit: >= 3]

  /// @}
  /// @name Height Detail Attributes (Optional)
  /// See #rcPolyMeshDetail for details related to these attributes.
  /// @{

  detailMeshes: PCardinal;    ///< The height detail sub-mesh data. [Size: 4 * #polyCount]
  detailVerts: PSingle;        ///< The detail mesh vertices. [Size: 3 * #detailVertsCount] [Unit: wu]
  detailVertsCount: Integer;          ///< The number of vertices in the detail mesh.
  detailTris: PByte;    ///< The detail mesh triangles. [Size: 4 * #detailTriCount]
  detailTriCount: Integer;            ///< The number of triangles in the detail mesh.

  /// @}
  /// @name Off-Mesh Connections Attributes (Optional)
  /// Used to define a custom point-to-point edge within the navigation graph, an
  /// off-mesh connection is a user defined traversable connection made up to two vertices,
  /// at least one of which resides within a navigation mesh polygon.
  /// @{

  /// Off-mesh connection vertices. [(ax, ay, az, bx, by, bz) * #offMeshConCount] [Unit: wu]
  offMeshConVerts: PSingle;
  /// Off-mesh connection radii. [Size: #offMeshConCount] [Unit: wu]
  offMeshConRad: PSingle;
  /// User defined flags assigned to the off-mesh connections. [Size: #offMeshConCount]
  offMeshConFlags: PWord;
  /// User defined area ids assigned to the off-mesh connections. [Size: #offMeshConCount]
  offMeshConAreas: PByte;
  /// The permitted travel direction of the off-mesh connections. [Size: #offMeshConCount]
  ///
  /// 0 = Travel only from endpoint A to endpoint B.<br/>
  /// #DT_OFFMESH_CON_BIDIR = Bidirectional travel.
  offMeshConDir: PByte;
  /// The user defined ids of the off-mesh connection. [Size: #offMeshConCount]
  offMeshConUserID: PCardinal;
  /// The number of off-mesh connections. [Limit: >= 0]
  offMeshConCount: Integer;

  /// @}
  /// @name Tile Attributes
  /// @note The tile grid/layer data can be left at zero if the destination is a single tile mesh.
  /// @{

  userId: Cardinal;  ///< The user defined id of the tile.
  tileX: Integer;        ///< The tile's x-grid location within the multi-tile destination mesh. (Along the x-axis.)
  tileY: Integer;        ///< The tile's y-grid location within the multi-tile desitation mesh. (Along the z-axis.)
  tileLayer: Integer;      ///< The tile's layer within the layered destination mesh. [Limit: >= 0] (Along the y-axis.)
  bmin: array [0..2] of Single;      ///< The minimum bounds of the tile. [(x, y, z)] [Unit: wu]
  bmax: array [0..2] of Single;      ///< The maximum bounds of the tile. [(x, y, z)] [Unit: wu]

  /// @}
  /// @name General Configuration Attributes
  /// @{

  walkableHeight: Single;  ///< The agent height. [Unit: wu]
  walkableRadius: Single;  ///< The agent radius. [Unit: wu]
  walkableClimb: Single;  ///< The agent maximum traversable ledge. (Up/Down) [Unit: wu]
  cs: Single;        ///< The xz-plane cell size of the polygon mesh. [Limit: > 0] [Unit: wu]
  ch: Single;        ///< The y-axis cell height of the polygon mesh. [Limit: > 0] [Unit: wu]

  /// True if a bounding volume tree should be built for the tile.
  /// @note The BVTree is not normally needed for layered navigation meshes.
  buildBvTree: Boolean;

  /// @}
end;

/// Builds navigation mesh tile data from the provided tile creation data.
/// @ingroup detour
///  @param[in]    params    Tile creation data.
///  @param[out]  outData    The resulting tile data.
///  @param[out]  outDataSize  The size of the tile data array.
/// @return True if the tile data was successfully created.
function dtCreateNavMeshData(params: PdtNavMeshCreateParams; outData: PPByte; outDataSize: PInteger): Boolean;

/// Swaps the endianess of the tile data's header (#dtMeshHeader).
///  @param[in,out]  data    The tile data array.
///  @param[in]    dataSize  The size of the data array.
//function dtNavMeshHeaderSwapEndian(data: PByte; dataSize: Integer): Boolean;

/// Swaps endianess of the tile data.
///  @param[in,out]  data    The tile data array.
///  @param[in]    dataSize  The size of the data array.
//function dtNavMeshDataSwapEndian(data: PByte; dataSize: Integer): Boolean;


// This section contains detailed documentation for members that don't have
// a source file. It reduces clutter in the main section of the header.

(**

@struct dtNavMeshCreateParams
@par

This structure is used to marshal data between the Recast mesh generation pipeline and Detour navigation components.

See the rcPolyMesh and rcPolyMeshDetail documentation for detailed information related to mesh structure.

Units are usually in voxels (vx) or world units (wu). The units for voxels, grid size, and cell size
are all based on the values of #cs and #ch.

The standard navigation mesh build process is to create tile data using dtCreateNavMeshData, then add the tile
to a navigation mesh using either the dtNavMesh single tile <tt>init()</tt> function or the dtNavMesh::addTile()
function.

@see dtCreateNavMeshData

*)

implementation
uses Math, RN_DetourCommon, RN_DetourNavMesh, RN_Helper;

const
  MESH_NULL_IDX = $ffff;

type
  PBVItem = ^TBVItem;
  TBVItem = record
    bmin: array [0..2] of Word;
    bmax: array [0..2] of Word;
    i: Integer;
  end;

function compareItemX(const va, vb: Pointer): Integer;
var a,b: PBVItem;
begin
  a := PBVItem(va);
  b := PBVItem(vb);
  if (a.bmin[0] < b.bmin[0]) then
    Exit(-1);
  if (a.bmin[0] > b.bmin[0]) then
    Exit(1);
  Result := 0;
end;

function compareItemY(const va, vb: Pointer): Integer;
var a,b: PBVItem;
begin
  a := PBVItem(va);
  b := PBVItem(vb);
  if (a.bmin[1] < b.bmin[1]) then
    Exit(-1);
  if (a.bmin[1] > b.bmin[1]) then
    Exit(1);
  Result := 0;
end;

function compareItemZ(const va, vb: Pointer): Integer;
var a,b: PBVItem;
begin
  a := PBVItem(va);
  b := PBVItem(vb);
  if (a.bmin[2] < b.bmin[2]) then
    Exit(-1);
  if (a.bmin[2] > b.bmin[2]) then
    Exit(1);
  Result := 0;
end;

procedure calcExtends(items: PBVItem; nitems: Integer; imin, imax: Integer; bmin, bmax: PWord);
var i: Integer; it: PBVItem;
begin
  bmin[0] := items[imin].bmin[0];
  bmin[1] := items[imin].bmin[1];
  bmin[2] := items[imin].bmin[2];

  bmax[0] := items[imin].bmax[0];
  bmax[1] := items[imin].bmax[1];
  bmax[2] := items[imin].bmax[2];

  for i := imin+1 to imax - 1 do
  begin
    it := @items[i];
    if (it.bmin[0] < bmin[0]) then bmin[0] := it.bmin[0];
    if (it.bmin[1] < bmin[1]) then bmin[1] := it.bmin[1];
    if (it.bmin[2] < bmin[2]) then bmin[2] := it.bmin[2];

    if (it.bmax[0] > bmax[0]) then bmax[0] := it.bmax[0];
    if (it.bmax[1] > bmax[1]) then bmax[1] := it.bmax[1];
    if (it.bmax[2] > bmax[2]) then bmax[2] := it.bmax[2];
  end;
end;

function longestAxis(x,y,z: Word): Integer;
var axis: Integer; maxVal: Word;
begin
  axis := 0;
  maxVal := x;
  if (y > maxVal) then
  begin
    axis := 1;
    maxVal := y;
  end;
  if (z > maxVal) then
  begin
    axis := 2;
    maxVal := z;
  end;
  Result := axis;
end;

procedure subdivide(items: PBVItem; nitems, imin, imax: Integer; curNode: PInteger; nodes: PdtBVNode);
var inum,icur,axis,isplit,iescape: Integer; node: PdtBVNode;
begin
  inum := imax - imin;
  icur := curNode^;

  node := @nodes[curNode^];
  Inc(curNode^);

  if (inum = 1) then
  begin
    // Leaf
    node.bmin[0] := items[imin].bmin[0];
    node.bmin[1] := items[imin].bmin[1];
    node.bmin[2] := items[imin].bmin[2];

    node.bmax[0] := items[imin].bmax[0];
    node.bmax[1] := items[imin].bmax[1];
    node.bmax[2] := items[imin].bmax[2];

    node.i := items[imin].i;
  end
  else
  begin
    // Split
    calcExtends(items, nitems, imin, imax, @node.bmin[0], @node.bmax[0]);

    axis := longestAxis(node.bmax[0] - node.bmin[0],
                 node.bmax[1] - node.bmin[1],
                 node.bmax[2] - node.bmin[2]);

    if (axis = 0) then
    begin
      // Sort along x-axis
      qsort(items+imin, inum, sizeof(TBVItem), compareItemX);
    end
    else if (axis = 1) then
    begin
      // Sort along y-axis
      qsort(items+imin, inum, sizeof(TBVItem), compareItemY);
    end
    else
    begin
      // Sort along z-axis
      qsort(items+imin, inum, sizeof(TBVItem), compareItemZ);
    end;

    isplit := imin+inum div 2;

    // Left
    subdivide(items, nitems, imin, isplit, curNode, nodes);
    // Right
    subdivide(items, nitems, isplit, imax, curNode, nodes);

    iescape := curNode^ - icur;
    // Negative index means escape.
    node.i := -iescape;
  end;
end;

function createBVTree(verts: PWord; nverts: Integer;
            polys: PWord; npolys, nvp: Integer;
            cs, ch: Single;
            nnodes: Integer; nodes: PdtBVNode): Integer;
var items: PBVItem; i,j: Integer; it: PBVItem; p: PWord; x,y,z: Word; curNode: Integer;
begin
  // Build tree
  GetMem(items, sizeof(TBVItem)*npolys);
  for i := 0 to npolys - 1 do
  begin
    it := @items[i];
    it.i := i;
    // Calc polygon bounds.
    p := @polys[i*nvp*2];
    it.bmin[0] := verts[p[0]*3+0]; it.bmax[0] := verts[p[0]*3+0];
    it.bmin[1] := verts[p[0]*3+1]; it.bmax[1] := verts[p[0]*3+1];
    it.bmin[2] := verts[p[0]*3+2]; it.bmax[2] := verts[p[0]*3+2];

    for j := 1 to nvp - 1 do
    begin
      if (p[j] = MESH_NULL_IDX) then break;
      x := verts[p[j]*3+0];
      y := verts[p[j]*3+1];
      z := verts[p[j]*3+2];

      if (x < it.bmin[0]) then it.bmin[0] := x;
      if (y < it.bmin[1]) then it.bmin[1] := y;
      if (z < it.bmin[2]) then it.bmin[2] := z;

      if (x > it.bmax[0]) then it.bmax[0] := x;
      if (y > it.bmax[1]) then it.bmax[1] := y;
      if (z > it.bmax[2]) then it.bmax[2] := z;
    end;
    // Remap y
    it.bmin[1] := Floor(it.bmin[1]*ch/cs);
    it.bmax[1] := Ceil(it.bmax[1]*ch/cs);
  end;

  curNode := 0;
  subdivide(items, npolys, 0, npolys, @curNode, nodes);

  FreeMem(items);

  Result := curNode;
end;

function classifyOffMeshPoint(pt, bmin, bmax: PSingle): Byte;
const XP = 1 shl 0;
const ZP = 1 shl 1;
const XM = 1 shl 2;
const ZM = 1 shl 3;
var outcode: Byte;
begin
  outcode := 0;
  outcode := outcode or IfThen(pt[0] >= bmax[0], XP, 0);
  outcode := outcode or IfThen(pt[2] >= bmax[2], ZP, 0);
  outcode := outcode or IfThen(pt[0] < bmin[0], XM, 0);
  outcode := outcode or IfThen(pt[2] < bmin[2], ZM, 0);

  case (outcode) of
    XP: Exit(0);
    XP or ZP: Exit(1);
    ZP: Exit(2);
    XM or ZP: Exit(3);
    XM: Exit(4);
    XM or ZM: Exit(5);
    ZM: Exit(6);
    XP or ZM: Exit(7);
  end;;

  Result := $ff;
end;

// TODO: Better error handling.

/// @par
///
/// The output data array is allocated using the detour allocator (dtAlloc()).  The method
/// used to free the memory will be determined by how the tile is added to the navigation
/// mesh.
///
/// @see dtNavMesh, dtNavMesh::addTile()
function dtCreateNavMeshData(params: PdtNavMeshCreateParams; outData: PPByte; outDataSize: PInteger): Boolean;
var nvp: Integer; offMeshConClass: PByte; storedOffMeshConCount,offMeshConLinkCount: Integer; hmin,hmax,h: Single;
i,j: Integer; iv,p: PWord; bmin,bmax: array [0..2] of Single; p0,p1: PSingle; totPolyCount, totVertCount: Integer;
edgeCount, portalCount: Integer; dir: Word; maxLinkCount, uniqueDetailVertCount, detailTriCount,ndv,nv: Integer;
headerSize,vertsSize,polysSize,linksSize,detailMeshesSize,detailVertsSize,detailTrisSize,bvTreeSize,offMeshConsSize,dataSize: Integer;
data,d: PByte; header: PdtMeshHeader; navVerts: PSingle; navPolys: PdtPoly; navDMeshes: PdtPolyDetail; navDVerts: PSingle;
navDTris: PByte; navBvtree: PdtBVNode; offMeshCons: PdtOffMeshConnection; offMeshVertsBase, offMeshPolyBase: Integer;
v,linkv: PSingle; n: Integer; src: PWord; pp: PdtPoly; vbase: Word; dtl: PdtPolyDetail; vb,tbase: Integer; t: PByte;
con: PdtOffMeshConnection; endPts: PSingle;
begin
  if (params.nvp > DT_VERTS_PER_POLYGON) then
    Exit(false);
  if (params.vertCount >= $ffff) then
    Exit(false);
  if (params.vertCount = 0) or (params.verts = nil) then
    Exit(false);
  if (params.polyCount = 0) or (params.polys = nil) then
    Exit(false);

  nvp := params.nvp;

  // Classify off-mesh connection points. We store only the connections
  // whose start point is inside the tile.
  offMeshConClass := nil;
  storedOffMeshConCount := 0;
  offMeshConLinkCount := 0;

  if (params.offMeshConCount > 0) then
  begin
     GetMem(offMeshConClass, sizeof(Byte)*params.offMeshConCount*2);

    // Find tight heigh bounds, used for culling out off-mesh start locations.
    hmin := MaxSingle;
    hmax := -MaxSingle;

    if (params.detailVerts <> nil) and (params.detailVertsCount <> 0) then
    begin
      for i := 0 to params.detailVertsCount - 1 do
      begin
        h := params.detailVerts[i*3+1];
        hmin := dtMin(hmin,h);
        hmax := dtMax(hmax,h);
      end;
    end
    else
    begin
      for i := 0 to params.vertCount - 1 do
      begin
        iv := @params.verts[i*3];
        h := params.bmin[1] + iv[1] * params.ch;
        hmin := dtMin(hmin,h);
        hmax := dtMax(hmax,h);
      end;
    end;
    hmin := hmin - params.walkableClimb;
    hmax := hmax + params.walkableClimb;

    dtVcopy(@bmin[0], @params.bmin[0]);
    dtVcopy(@bmax[0], @params.bmax[0]);
    bmin[1] := hmin;
    bmax[1] := hmax;

    for i := 0 to params.offMeshConCount - 1 do
    begin
      p0 := @params.offMeshConVerts[(i*2+0)*3];
      p1 := @params.offMeshConVerts[(i*2+1)*3];
      offMeshConClass[i*2+0] := classifyOffMeshPoint(p0, @bmin[0], @bmax[0]);
      offMeshConClass[i*2+1] := classifyOffMeshPoint(p1, @bmin[0], @bmax[0]);

      // Zero out off-mesh start positions which are not even potentially touching the mesh.
      if (offMeshConClass[i*2+0] = $ff) then
      begin
        if (p0[1] < bmin[1]) or (p0[1] > bmax[1]) then
          offMeshConClass[i*2+0] := 0;
      end;

      // Cound how many links should be allocated for off-mesh connections.
      if (offMeshConClass[i*2+0] = $ff) then
        Inc(offMeshConLinkCount);
      if (offMeshConClass[i*2+1] = $ff) then
        Inc(offMeshConLinkCount);

      if (offMeshConClass[i*2+0] = $ff) then
        Inc(storedOffMeshConCount);
    end;
  end;

  // Off-mesh connectionss are stored as polygons, adjust values.
  totPolyCount := params.polyCount + storedOffMeshConCount;
  totVertCount := params.vertCount + storedOffMeshConCount*2;

  // Find portal edges which are at tile borders.
  edgeCount := 0;
  portalCount := 0;
  for i := 0 to params.polyCount - 1 do
  begin
    p := @params.polys[i*2*nvp];
    for j := 0 to nvp - 1 do
    begin
      if (p[j] = MESH_NULL_IDX) then break;
      Inc(edgeCount);

      if (p[nvp+j] and $8000) <> 0 then
      begin
        dir := p[nvp+j] and $f;
        if (dir <> $f) then
          Inc(portalCount);
      end;
    end;
  end;

  maxLinkCount := edgeCount + portalCount*2 + offMeshConLinkCount*2;

  // Find unique detail vertices.
  uniqueDetailVertCount := 0;
  detailTriCount := 0;
  if (params.detailMeshes <> nil) then
  begin
    // Has detail mesh, count unique detail vertex count and use input detail tri count.
    detailTriCount := params.detailTriCount;
    for i := 0 to params.polyCount - 1 do
    begin
      p := @params.polys[i*nvp*2];
      ndv := params.detailMeshes[i*4+1];
      nv := 0;
      for j := 0 to nvp - 1 do
      begin
        if (p[j] = MESH_NULL_IDX) then break;
        Inc(nv);
      end;
      Dec(ndv, nv);
      Inc(uniqueDetailVertCount, ndv);
    end;
  end
  else
  begin
    // No input detail mesh, build detail mesh from nav polys.
    uniqueDetailVertCount := 0; // No extra detail verts.
    detailTriCount := 0;
    for i := 0 to params.polyCount - 1 do
    begin
      p := @params.polys[i*nvp*2];
      nv := 0;
      for j := 0 to nvp - 1 do
      begin
        if (p[j] = MESH_NULL_IDX) then break;
        Inc(nv);
      end;
      Inc(detailTriCount, nv-2);
    end;
  end;

  // Calculate data size
  headerSize := dtAlign4(sizeof(TdtMeshHeader));
  vertsSize := dtAlign4(sizeof(Single)*3*totVertCount);
  polysSize := dtAlign4(sizeof(TdtPoly)*totPolyCount);
  linksSize := dtAlign4(sizeof(TdtLink)*maxLinkCount);
  detailMeshesSize := dtAlign4(sizeof(TdtPolyDetail)*params.polyCount);
  detailVertsSize := dtAlign4(sizeof(Single)*3*uniqueDetailVertCount);
  detailTrisSize := dtAlign4(sizeof(Byte)*4*detailTriCount);
  bvTreeSize := IfThen(params.buildBvTree, dtAlign4(sizeof(TdtBVNode)*params.polyCount*2), 0);
  offMeshConsSize := dtAlign4(sizeof(TdtOffMeshConnection)*storedOffMeshConCount);

  dataSize := headerSize + vertsSize + polysSize + linksSize +
             detailMeshesSize + detailVertsSize + detailTrisSize +
             bvTreeSize + offMeshConsSize;

  GetMem(data, sizeof(Byte)*dataSize);
  FillChar(data[0], dataSize, 0);

  d := data;
  header := PdtMeshHeader(d); Inc(d, headerSize);
  navVerts := PSingle(d); Inc(d, vertsSize);
  navPolys := PdtPoly(d); Inc(d, polysSize);
  Inc(d, linksSize);
  navDMeshes := PdtPolyDetail(d); Inc(d, detailMeshesSize);
  navDVerts := PSingle(d); Inc(d, detailVertsSize);
  navDTris := PByte(d); Inc(d, detailTrisSize);
  navBvtree := PdtBVNode(d); Inc(d, bvTreeSize);
  offMeshCons := PdtOffMeshConnection(d); Inc(d, offMeshConsSize);


  // Store header
  header.magic := DT_NAVMESH_MAGIC;
  header.version := DT_NAVMESH_VERSION;
  header.x := params.tileX;
  header.y := params.tileY;
  header.layer := params.tileLayer;
  header.userId := params.userId;
  header.polyCount := totPolyCount;
  header.vertCount := totVertCount;
  header.maxLinkCount := maxLinkCount;
  dtVcopy(@header.bmin[0], @params.bmin[0]);
  dtVcopy(@header.bmax[0], @params.bmax[0]);
  header.detailMeshCount := params.polyCount;
  header.detailVertCount := uniqueDetailVertCount;
  header.detailTriCount := detailTriCount;
  header.bvQuantFactor := 1.0 / params.cs;
  header.offMeshBase := params.polyCount;
  header.walkableHeight := params.walkableHeight;
  header.walkableRadius := params.walkableRadius;
  header.walkableClimb := params.walkableClimb;
  header.offMeshConCount := storedOffMeshConCount;
  header.bvNodeCount := IfThen(params.buildBvTree, params.polyCount*2, 0);

  offMeshVertsBase := params.vertCount;
  offMeshPolyBase := params.polyCount;

  // Store vertices
  // Mesh vertices
  for i := 0 to params.vertCount - 1 do
  begin
    iv := @params.verts[i*3];
    v := @navVerts[i*3];
    v[0] := params.bmin[0] + iv[0] * params.cs;
    v[1] := params.bmin[1] + iv[1] * params.ch;
    v[2] := params.bmin[2] + iv[2] * params.cs;
  end;
  // Off-mesh link vertices.
  n := 0;
  for i := 0 to params.offMeshConCount - 1 do
  begin
    // Only store connections which start from this tile.
    if (offMeshConClass[i*2+0] = $ff) then
    begin
      linkv := @params.offMeshConVerts[i*2*3];
      v := @navVerts[(offMeshVertsBase + n*2)*3];
      dtVcopy(@v[0], @linkv[0]);
      dtVcopy(@v[3], @linkv[3]);
      Inc(n);
    end;
  end;

  // Store polygons
  // Mesh polys
  src := params.polys;
  for i := 0 to params.polyCount - 1 do
  begin
    pp := @navPolys[i];
    pp.vertCount := 0;
    pp.flags := params.polyFlags[i];
    pp.setArea(params.polyAreas[i]);
    pp.setType(DT_POLYTYPE_GROUND);
    for j := 0 to nvp - 1 do
    begin
      if (src[j] = MESH_NULL_IDX) then break;
      pp.verts[j] := src[j];
      if (src[nvp+j] and $8000) <> 0 then
      begin
        // Border or portal edge.
        dir := src[nvp+j] and $f;
        if (dir = $f) then // Border
          pp.neis[j] := 0
        else if (dir = 0) then // Portal x-
          pp.neis[j] := DT_EXT_LINK or 4
        else if (dir = 1) then // Portal z+
          pp.neis[j] := DT_EXT_LINK or 2
        else if (dir = 2) then // Portal x+
          pp.neis[j] := DT_EXT_LINK or 0
        else if (dir = 3) then // Portal z-
          pp.neis[j] := DT_EXT_LINK or 6;
      end
      else
      begin
        // Normal connection
        pp.neis[j] := src[nvp+j]+1;
      end;

      Inc(pp.vertCount);
    end;
    Inc(src, nvp*2);
  end;
  // Off-mesh connection vertices.
  n := 0;
  for i := 0 to params.offMeshConCount - 1 do
  begin
    // Only store connections which start from this tile.
    if (offMeshConClass[i*2+0] = $ff) then
    begin
      pp := @navPolys[offMeshPolyBase+n];
      pp.vertCount := 2;
      pp.verts[0] := (offMeshVertsBase + n*2+0);
      pp.verts[1] := (offMeshVertsBase + n*2+1);
      pp.flags := params.offMeshConFlags[i];
      pp.setArea(params.offMeshConAreas[i]);
      pp.setType(DT_POLYTYPE_OFFMESH_CONNECTION);
      Inc(n);
    end;
  end;

  // Store detail meshes and vertices.
  // The nav polygon vertices are stored as the first vertices on each mesh.
  // We compress the mesh data by skipping them and using the navmesh coordinates.
  if (params.detailMeshes <> nil) then
  begin
    vbase := 0;
    for i := 0 to params.polyCount - 1 do
    begin
      dtl := @navDMeshes[i];
      vb := params.detailMeshes[i*4+0];
      ndv := params.detailMeshes[i*4+1];
      nv := navPolys[i].vertCount;
      dtl.vertBase := vbase;
      dtl.vertCount := (ndv-nv);
      dtl.triBase := params.detailMeshes[i*4+2];
      dtl.triCount := params.detailMeshes[i*4+3];
      // Copy vertices except the first 'nv' verts which are equal to nav poly verts.
      if (ndv-nv) <> 0 then
      begin
        Move(params.detailVerts[(vb+nv)*3], navDVerts[vbase*3], sizeof(Single)*3*(ndv-nv));
        Inc(vbase, (ndv-nv));
      end;
    end;
    // Store triangles.
    Move(params.detailTris[0], navDTris[0], sizeof(Byte)*4*params.detailTriCount);
  end
  else
  begin
    // Create dummy detail mesh by triangulating polys.
    tbase := 0;
    for i := 0 to params.polyCount - 1 do
    begin
      dtl := @navDMeshes[i];
      nv := navPolys[i].vertCount;
      dtl.vertBase := 0;
      dtl.vertCount := 0;
      dtl.triBase := tbase;
      dtl.triCount := (nv-2);
      // Triangulate polygon (local indices).
      for j := 2 to nv - 1 do
      begin
        t := @navDTris[tbase*4];
        t[0] := 0;
        t[1] := (j-1);
        t[2] := j;
        // Bit for each edge that belongs to poly boundary.
        t[3] := (1 shl 2);
        if (j = 2) then t[3] := t[3] or (1 shl 0);
        if (j = nv-1) then t[3] := t[3] or (1 shl 4);
        Inc(tbase);
      end;
    end;
  end;

  // Store and create BVtree.
  // TODO: take detail mesh into account! use byte per bbox extent?
  if (params.buildBvTree) then
  begin
    createBVTree(params.verts, params.vertCount, params.polys, params.polyCount,
           nvp, params.cs, params.ch, params.polyCount*2, navBvtree);
  end;

  // Store Off-Mesh connections.
  n := 0;
  for i := 0 to params.offMeshConCount - 1 do
  begin
    // Only store connections which start from this tile.
    if (offMeshConClass[i*2+0] = $ff) then
    begin
      con := @offMeshCons[n];
      con.poly := (offMeshPolyBase + n);
      // Copy connection end-points.
      endPts := @params.offMeshConVerts[i*2*3];
      dtVcopy(@con.pos[0], @endPts[0]);
      dtVcopy(@con.pos[3], @endPts[3]);
      con.rad := params.offMeshConRad[i];
      con.flags := IfThen(params.offMeshConDir[i] <> 0, DT_OFFMESH_CON_BIDIR, 0);
      con.side := offMeshConClass[i*2+1];
      if (params.offMeshConUserID <> nil) then
        con.userId := params.offMeshConUserID[i];
      Inc(n);
    end;
  end;

  FreeMem(offMeshConClass);

  outData^ := data;
  outDataSize^ := dataSize;

  Result := true;
end;

{function dtNavMeshHeaderSwapEndian(data: PByte; dataSize: Integer): Boolean;
var header: PdtMeshHeader; swappedMagic,swappedVersion: Integer;
begin
  header := PdtMeshHeader(data);

  swappedMagic := DT_NAVMESH_MAGIC;
  swappedVersion := DT_NAVMESH_VERSION;
  dtSwapEndian(PInteger(@swappedMagic));
  dtSwapEndian(PInteger(@swappedVersion));

  if ((header.magic <> DT_NAVMESH_MAGIC) or (header.version <> DT_NAVMESH_VERSION) and
    (header.magic <> swappedMagic) or (header.version <> swappedVersion)) then
  begin
    Exit(false);
  end;

  dtSwapEndian(PInteger(@header.magic));
  dtSwapEndian(PInteger(@header.version));
  dtSwapEndian(PInteger(@header.x));
  dtSwapEndian(PInteger(@header.y));
  dtSwapEndian(PInteger(@header.layer));
  dtSwapEndian(PCardinal(@header.userId));
  dtSwapEndian(PInteger(@header.polyCount));
  dtSwapEndian(PInteger(@header.vertCount));
  dtSwapEndian(PInteger(@header.maxLinkCount));
  dtSwapEndian(PInteger(@header.detailMeshCount));
  dtSwapEndian(PInteger(@header.detailVertCount));
  dtSwapEndian(PInteger(@header.detailTriCount));
  dtSwapEndian(PInteger(@header.bvNodeCount));
  dtSwapEndian(PInteger(@header.offMeshConCount));
  dtSwapEndian(PInteger(@header.offMeshBase));
  dtSwapEndian(PSingle(@&header.walkableHeight));
  dtSwapEndian(PSingle(@header.walkableRadius));
  dtSwapEndian(PSingle(@header.walkableClimb));
  dtSwapEndian(PSingle(@header.bmin[0]));
  dtSwapEndian(PSingle(@header.bmin[1]));
  dtSwapEndian(PSingle(@header.bmin[2]));
  dtSwapEndian(PSingle(@header.bmax[0]));
  dtSwapEndian(PSingle(@header.bmax[1]));
  dtSwapEndian(PSingle(@header.bmax[2]));
  dtSwapEndian(PSingle(@header.bvQuantFactor));

  // Freelist index and pointers are updated when tile is added, no need to swap.

  Result := true;
end;}

/// @par
///
/// @warning This function assumes that the header is in the correct endianess already.
/// Call #dtNavMeshHeaderSwapEndian() first on the data if the data is expected to be in wrong endianess
/// to start with. Call #dtNavMeshHeaderSwapEndian() after the data has been swapped if converting from
/// native to foreign endianess.
{function dtNavMeshDataSwapEndian(data: PByte; dataSize: Integer): Boolean;
begin
  // Make sure the data is in right format.
  dtMeshHeader* header := (dtMeshHeader*)data;
  if (header.magic != DT_NAVMESH_MAGIC)
    return false;
  if (header.version != DT_NAVMESH_VERSION)
    return false;

  // Patch header pointers.
  const int headerSize := dtAlign4(sizeof(dtMeshHeader));
  const int vertsSize := dtAlign4(sizeof(float)*3*header.vertCount);
  const int polysSize := dtAlign4(sizeof(dtPoly)*header.polyCount);
  const int linksSize := dtAlign4(sizeof(dtLink)*(header.maxLinkCount));
  const int detailMeshesSize := dtAlign4(sizeof(dtPolyDetail)*header.detailMeshCount);
  const int detailVertsSize := dtAlign4(sizeof(float)*3*header.detailVertCount);
  const int detailTrisSize := dtAlign4(sizeof(unsigned char)*4*header.detailTriCount);
  const int bvtreeSize := dtAlign4(sizeof(dtBVNode)*header.bvNodeCount);
  const int offMeshLinksSize := dtAlign4(sizeof(dtOffMeshConnection)*header.offMeshConCount);

  unsigned char* d := data + headerSize;
  float* verts := (float*)d; d += vertsSize;
  dtPoly* polys := (dtPoly*)d; d += polysSize;
  /*dtLink* links := (dtLink*)d;*/ d += linksSize;
  dtPolyDetail* detailMeshes := (dtPolyDetail*)d; d += detailMeshesSize;
  float* detailVerts := (float*)d; d += detailVertsSize;
  /*unsigned char* detailTris := (unsigned char*)d;*/ d += detailTrisSize;
  dtBVNode* bvTree := (dtBVNode*)d; d += bvtreeSize;
  dtOffMeshConnection* offMeshCons := (dtOffMeshConnection*)d; d += offMeshLinksSize;

  // Vertices
  for (int i := 0; i < header.vertCount*3; ++i)
  begin
    dtSwapEndian(&verts[i]);
  end;

  // Polys
  for (int i := 0; i < header.polyCount; ++i)
  begin
    dtPoly* p := &polys[i];
    // poly.firstLink is update when tile is added, no need to swap.
    for (int j := 0; j < DT_VERTS_PER_POLYGON; ++j)
    begin
      dtSwapEndian(&p.verts[j]);
      dtSwapEndian(&p.neis[j]);
    end;
    dtSwapEndian(&p.flags);
  end;

  // Links are rebuild when tile is added, no need to swap.

  // Detail meshes
  for (int i := 0; i < header.detailMeshCount; ++i)
  begin
    dtPolyDetail* pd := &detailMeshes[i];
    dtSwapEndian(&pd.vertBase);
    dtSwapEndian(&pd.triBase);
  end;

  // Detail verts
  for (int i := 0; i < header.detailVertCount*3; ++i)
  begin
    dtSwapEndian(&detailVerts[i]);
  end;

  // BV-tree
  for (int i := 0; i < header.bvNodeCount; ++i)
  begin
    dtBVNode* node := &bvTree[i];
    for (int j := 0; j < 3; ++j)
    begin
      dtSwapEndian(&node.bmin[j]);
      dtSwapEndian(&node.bmax[j]);
    end;
    dtSwapEndian(&node.i);
  end;

  // Off-mesh Connections.
  for (int i := 0; i < header.offMeshConCount; ++i)
  begin
    dtOffMeshConnection* con := &offMeshCons[i];
    for (int j := 0; j < 6; ++j)
      dtSwapEndian(&con.pos[j]);
    dtSwapEndian(&con.rad);
    dtSwapEndian(&con.poly);
  end;

  Result := true;
end;}

end.
