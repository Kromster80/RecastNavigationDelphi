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

unit RN_SampleSoloMesh;
interface
uses
  Classes, Controls, Math, OpenGL, StrUtils, SysUtils,
  RN_Helper, RN_InputGeom, RN_Sample, RN_Recast, RN_RecastHelper;

type
  TDrawMode = (
    DRAWMODE_NAVMESH,
    DRAWMODE_NAVMESH_TRANS,
    DRAWMODE_NAVMESH_BVTREE,
    DRAWMODE_NAVMESH_NODES,
    DRAWMODE_NAVMESH_INVIS,
    DRAWMODE_MESH,
    DRAWMODE_VOXELS,
    DRAWMODE_VOXELS_WALKABLE,
    DRAWMODE_COMPACT,
    DRAWMODE_COMPACT_DISTANCE,
    DRAWMODE_COMPACT_REGIONS,
    DRAWMODE_REGION_CONNECTIONS,
    DRAWMODE_RAW_CONTOURS,
    DRAWMODE_BOTH_CONTOURS,
    DRAWMODE_CONTOURS,
    DRAWMODE_POLYMESH,
    DRAWMODE_POLYMESH_DETAIL,
    MAX_DRAWMODE
  );

  TSample_SoloMesh = class(TSample)
  protected
    fOwner: TWinControl;
    m_keepInterResults: Boolean;
    m_totalBuildTimeMs: Single;

    m_triareas: PByte;
    m_solid: TrcHeightfield;
    m_cfg: TrcConfig;

    m_drawMode: TDrawMode;

    procedure cleanup();
  public
    m_chf: TrcCompactHeightfield;
    m_cset: TrcContourSet;
    m_pmesh: TrcPolyMesh;
    m_dmesh: TrcPolyMeshDetail;
    property drawMode: TDrawMode read m_drawMode write m_drawMode;
    property keepIntermediateResults: Boolean read m_keepInterResults write m_keepInterResults;
    function getToolItems: string;
    function getDrawModeItems: string;
    procedure setToolType(aType: TSampleToolType);

    constructor Create(aOwner: TWinControl);
    destructor Destroy; override;

    procedure handleSettings(); override;
    procedure handleTools(); override;
    procedure handleDebugMode(); override;

    procedure handleRender(); override;
    procedure handleRenderOverlay(proj, model: PDouble; view: PInteger); override;
    procedure handleMeshChanged(geom: TInputGeom); override;
    function handleBuild(): Boolean; override;
  end;

implementation
uses
  RN_DebugDraw, RN_RecastDump, RN_DetourDump, RN_NavMeshTesterTool, RN_NavMeshPruneTool, RN_CrowdTool,
  RN_DetourDebugDraw, RN_DetourNavMesh, RN_DetourNavMeshBuilder, RN_DetourStatus,
  RN_RecastDebugDraw, RN_RecastArea, RN_RecastContour, RN_RecastFilter, RN_RecastMesh, RN_RecastMeshDetail,
  RN_RecastRasterization, RN_RecastRegion, RN_SampleInterfaces;

constructor TSample_SoloMesh.Create(aOwner: TWinControl);
begin
  inherited Create;

  fOwner := aOwner;

  m_keepInterResults := true;

  m_drawMode := DRAWMODE_NAVMESH;

  setTool(TNavMeshTesterTool.Create(fOwner));
end;

destructor TSample_SoloMesh.Destroy;
begin
  cleanup();

  inherited;
end;

procedure TSample_SoloMesh.cleanup();
begin
  m_triareas := nil;
  FillChar(m_solid, SizeOf(m_solid), #0);
  FillChar(m_chf, SizeOf(m_chf), #0);
  FillChar(m_cset, SizeOf(m_cset), #0);
  FillChar(m_pmesh, SizeOf(m_pmesh), #0);
  FillChar(m_dmesh, SizeOf(m_dmesh), #0);
  FreeAndNil(m_navMesh);
end;

procedure TSample_SoloMesh.handleSettings;
begin
  inherited;

  //
end;

procedure TSample_SoloMesh.handleTools;
begin
  // Delphi. Refactored
end;

procedure TSample_SoloMesh.setToolType(aType: TSampleToolType);
begin
  case aType of
    TOOL_NONE:                setTool(nil);
    TOOL_NAVMESH_TESTER:      setTool(TNavMeshTesterTool.Create(fOwner));
    TOOL_NAVMESH_PRUNE:       setTool(TNavMeshPruneTool.Create(fOwner));
    //todo: TOOL_OFFMESH_CONNECTION:  setTool(TOffMeshConnectionTool.Create);
    //todo: TOOL_CONVEX_VOLUME:       setTool(TConvexVolumeTool.Create);
    TOOL_CROWD:               setTool(TCrowdTool.Create(fOwner));
  else
    setTool(nil);
  end;

  if m_tool <> nil then
    m_tool.handleMenu(nil);
end;

function TSample_SoloMesh.getToolItems: string;
begin
  Result := ' ' + sLineBreak +
            ' ' + sLineBreak +
            ' ' + sLineBreak +
            ' ' + sLineBreak +
            'Test Navmesh' + sLineBreak +
            'Prune Navmesh' + sLineBreak +
            'Create Off-Mesh Connections' + sLineBreak +
            'Create Convex Volumes' + sLineBreak +
            'Create Crowds';
end;

procedure TSample_SoloMesh.handleDebugMode();
begin
  // Delphi. Refactored
end;

function TSample_SoloMesh.getDrawModeItems: string;
var valid: array [TDrawMode] of Boolean; i: TDrawMode;
begin
  for i := Low(TDrawMode) to High(TDrawMode) do
    valid[i] := false;

  if (m_geom <> nil) then
  begin
    valid[DRAWMODE_NAVMESH] := m_navMesh <> nil;
    valid[DRAWMODE_NAVMESH_TRANS] := m_navMesh <> nil;
    valid[DRAWMODE_NAVMESH_BVTREE] := m_navMesh <> nil;
    valid[DRAWMODE_NAVMESH_NODES] := m_navQuery <> nil;
    valid[DRAWMODE_NAVMESH_INVIS] := m_navMesh <> nil;
    valid[DRAWMODE_MESH] := true;
    valid[DRAWMODE_VOXELS] := true; //m_solid <> nil;
    valid[DRAWMODE_VOXELS_WALKABLE] := true; //m_solid <> nil;
    valid[DRAWMODE_COMPACT] := true; //m_chf <> nil;
    valid[DRAWMODE_COMPACT_DISTANCE] := true; //m_chf <> nil;
    valid[DRAWMODE_COMPACT_REGIONS] := true; //m_chf <> nil;
    valid[DRAWMODE_REGION_CONNECTIONS] := true; //m_cset <> nil;
    valid[DRAWMODE_RAW_CONTOURS] := true; //m_cset <> nil;
    valid[DRAWMODE_BOTH_CONTOURS] := true; //m_cset <> nil;
    valid[DRAWMODE_CONTOURS] := true; //m_cset <> nil;
    valid[DRAWMODE_POLYMESH] := true; //m_pmesh <> nil;
    valid[DRAWMODE_POLYMESH_DETAIL] := true; //m_dmesh <> nil;
  end;

  Result := 'Navmesh' + IfThen(not valid[DRAWMODE_NAVMESH], ' ') + sLineBreak +
            'Navmesh Trans' + IfThen(not valid[DRAWMODE_NAVMESH_TRANS], ' ') + sLineBreak +
            'Navmesh BVTree' + IfThen(not valid[DRAWMODE_NAVMESH_BVTREE], ' ') + sLineBreak +
            'Navmesh Nodes' + IfThen(not valid[DRAWMODE_NAVMESH_NODES], ' ') + sLineBreak +
            'Navmesh Invis' + IfThen(not valid[DRAWMODE_NAVMESH_INVIS], ' ') + sLineBreak +
            'Input Mesh' + IfThen(not valid[DRAWMODE_MESH], ' ') + sLineBreak +
            'Voxels' + IfThen(not valid[DRAWMODE_VOXELS], ' ') + sLineBreak +
            'Walkable Voxels' + IfThen(not valid[DRAWMODE_VOXELS_WALKABLE], ' ') + sLineBreak +
            'Compact' + IfThen(not valid[DRAWMODE_COMPACT], ' ') + sLineBreak +
            'Compact Distance' + IfThen(not valid[DRAWMODE_COMPACT_DISTANCE], ' ') + sLineBreak +
            'Compact Regions' + IfThen(not valid[DRAWMODE_COMPACT_REGIONS], ' ') + sLineBreak +
            'Region Connections' + IfThen(not valid[DRAWMODE_REGION_CONNECTIONS], ' ') + sLineBreak +
            'Raw Contours' + IfThen(not valid[DRAWMODE_RAW_CONTOURS], ' ') + sLineBreak +
            'Both Contours' + IfThen(not valid[DRAWMODE_BOTH_CONTOURS], ' ') + sLineBreak +
            'Contours' + IfThen(not valid[DRAWMODE_CONTOURS], ' ') + sLineBreak +
            'Poly Mesh' + IfThen(not valid[DRAWMODE_POLYMESH], ' ') + sLineBreak +
            'Poly Mesh Detail' + IfThen(not valid[DRAWMODE_POLYMESH_DETAIL], ' ');
end;


procedure TSample_SoloMesh.handleRender();
var dd: TDebugDrawGL; texScale: Single; bmin, bmax: PSingle;
begin
  inherited;

  if (m_geom.getMesh = nil) then
    Exit;

  dd := TDebugDrawGL.Create;

  glEnable(GL_FOG);
  glDepthMask(GL_TRUE);

  texScale := 1.0 / (m_cellSize * 10.0);
  
  if (m_drawMode <> DRAWMODE_NAVMESH_TRANS) then
  begin
    // Draw mesh
    duDebugDrawTriMeshSlope(dd, m_geom.getMesh.getVerts, m_geom.getMesh.getVertCount,
                m_geom.getMesh.getTris, m_geom.getMesh.getNormals, m_geom.getMesh.getTriCount,
                m_agentMaxSlope, texScale);
    m_geom.drawOffMeshConnections(dd);
  end;

  glDisable(GL_FOG);
  glDepthMask(GL_FALSE);

  // Draw bounds
  bmin := m_geom.getMeshBoundsMin;
  bmax := m_geom.getMeshBoundsMax;
  duDebugDrawBoxWire(dd, bmin[0],bmin[1],bmin[2], bmax[0],bmax[1],bmax[2], duRGBA(255,255,255,128), 1.0);
  dd.&begin(DU_DRAW_POINTS, 5.0);
  dd.vertex(bmin[0],bmin[1],bmin[2],duRGBA(255,255,255,128));
  dd.&end();

  if (m_navMesh <> nil) and (m_navQuery <> nil) and
    (m_drawMode = DRAWMODE_NAVMESH) or
    (m_drawMode = DRAWMODE_NAVMESH_TRANS) or
    (m_drawMode = DRAWMODE_NAVMESH_BVTREE) or
    (m_drawMode = DRAWMODE_NAVMESH_NODES) or
    (m_drawMode = DRAWMODE_NAVMESH_INVIS) then
  begin
    if (m_drawMode <> DRAWMODE_NAVMESH_INVIS) then
      duDebugDrawNavMeshWithClosedList(dd, m_navMesh, m_navQuery, m_navMeshDrawFlags);
    if (m_drawMode = DRAWMODE_NAVMESH_BVTREE) then
      duDebugDrawNavMeshBVTree(dd, m_navMesh);
    if (m_drawMode = DRAWMODE_NAVMESH_NODES) then
      duDebugDrawNavMeshNodes(dd, m_navQuery);
    duDebugDrawNavMeshPolysWithFlags(dd, m_navMesh, SAMPLE_POLYFLAGS_DISABLED, duRGBA(0,0,0,128));
  end;

  glDepthMask(GL_TRUE);

  if (m_drawMode = DRAWMODE_COMPACT) then
    duDebugDrawCompactHeightfieldSolid(dd, @m_chf);

  if (m_drawMode = DRAWMODE_COMPACT_DISTANCE) then
    duDebugDrawCompactHeightfieldDistance(dd, @m_chf);
  if (m_drawMode = DRAWMODE_COMPACT_REGIONS) then
    duDebugDrawCompactHeightfieldRegions(dd, @m_chf);
  if (m_drawMode = DRAWMODE_VOXELS) then
  begin
    //glEnable(GL_FOG);
    duDebugDrawHeightfieldSolid(dd, @m_solid);
    //glDisable(GL_FOG);
  end;
  if (m_drawMode = DRAWMODE_VOXELS_WALKABLE) then
  begin
    //glEnable(GL_FOG);
    duDebugDrawHeightfieldWalkable(dd, @m_solid);
    //glDisable(GL_FOG);
  end;
  if (m_drawMode = DRAWMODE_RAW_CONTOURS) then
  begin
    glDepthMask(GL_FALSE);
    duDebugDrawRawContours(dd, @m_cset);
    glDepthMask(GL_TRUE);
  end;
  if (m_drawMode = DRAWMODE_BOTH_CONTOURS) then
  begin
    glDepthMask(GL_FALSE);
    duDebugDrawRawContours(dd, @m_cset, 0.5);
    duDebugDrawContours(dd, @m_cset);
    glDepthMask(GL_TRUE);
  end;
  if (m_drawMode = DRAWMODE_CONTOURS) then
  begin
    glDepthMask(GL_FALSE);
    duDebugDrawContours(dd, @m_cset);
    glDepthMask(GL_TRUE);
  end;
  if (m_drawMode = DRAWMODE_REGION_CONNECTIONS) then
  begin
    duDebugDrawCompactHeightfieldRegions(dd, @m_chf);

    glDepthMask(GL_FALSE);
    duDebugDrawRegionConnections(dd, @m_cset);
    glDepthMask(GL_TRUE);
  end;
  if (m_drawMode = DRAWMODE_POLYMESH) then
  begin
    glDepthMask(GL_FALSE);
    duDebugDrawPolyMesh(dd, @m_pmesh);
    glDepthMask(GL_TRUE);
  end;
  if (m_drawMode = DRAWMODE_POLYMESH_DETAIL) then
  begin
    //glDepthMask(GL_FALSE);
    duDebugDrawPolyMeshDetail(dd, @m_dmesh);
    glDepthMask(GL_TRUE);
  end;

  m_geom.drawConvexVolumes(dd);

  if (m_tool <> nil) then
    m_tool.handleRender();
  renderToolStates();

  glDepthMask(GL_TRUE);

  dd.Free;
end;

procedure TSample_SoloMesh.handleRenderOverlay(proj, model: PDouble; view: PInteger);
begin
  if (m_tool <> nil) then
    m_tool.handleRenderOverlay(proj, model, view);
  renderOverlayToolStates(proj, model, view);
end;

procedure TSample_SoloMesh.handleMeshChanged(geom: TInputGeom);
begin
  inherited handleMeshChanged(geom);

  m_navMesh.Free;
  m_navMesh := nil;

  if (m_tool <> nil) then
  begin
    m_tool.reset();
    m_tool.init(Self);
  end;
  resetToolStates();
  initToolStates(Self);
end;


function TSample_SoloMesh.handleBuild: Boolean;
var bmin, bmax: PSingle; verts: PSingle; tris: PInteger; nverts,ntris: Integer;
vols: TIGConvexVolumeArray; i: Integer; navData: PByte; navDataSize: Integer;
params: TdtNavMeshCreateParams; status: TdtStatus;
begin
  if (m_geom = nil) or (m_geom.getMesh = nil) then
  begin
    m_ctx.log(RC_LOG_ERROR, 'buildNavigation: Input mesh is not specified.');
    Exit(false);
  end;

  cleanup();

  bmin := m_geom.getMeshBoundsMin;
  bmax := m_geom.getMeshBoundsMax;
  verts := m_geom.getMesh.getVerts;
  nverts := m_geom.getMesh.getVertCount;
  tris := m_geom.getMesh.getTris;
  ntris := m_geom.getMesh.getTriCount;

  //
  // Step 1. Initialize build config.
  //

  // Init build configuration from GUI
  FillChar(m_cfg, sizeof(m_cfg), #0);
  m_cfg.cs := m_cellSize;
  m_cfg.ch := m_cellHeight;
  m_cfg.walkableSlopeAngle := m_agentMaxSlope;
  m_cfg.walkableHeight := ceil(m_agentHeight / m_cfg.ch);
  m_cfg.walkableClimb := floor(m_agentMaxClimb / m_cfg.ch);
  m_cfg.walkableRadius := ceil(m_agentRadius / m_cfg.cs);
  m_cfg.maxEdgeLen := Trunc(m_edgeMaxLen / m_cellSize);
  m_cfg.maxSimplificationError := m_edgeMaxError;
  m_cfg.minRegionArea := Trunc(Sqr(m_regionMinSize));    // Note: area := size*size
  m_cfg.mergeRegionArea := Trunc(Sqr(m_regionMergeSize));  // Note: area := size*size
  m_cfg.maxVertsPerPoly := Trunc(m_vertsPerPoly);
  m_cfg.detailSampleDist := IfThen(m_detailSampleDist < 0.9, 0, m_cellSize * m_detailSampleDist);
  m_cfg.detailSampleMaxError := m_cellHeight * m_detailSampleMaxError;

  // Set the area where the navigation will be build.
  // Here the bounds of the input mesh are used, but the
  // area could be specified by an user defined box, etc.
  rcVcopy(@m_cfg.bmin[0], bmin);
  rcVcopy(@m_cfg.bmax[0], bmax);
  rcCalcGridSize(@m_cfg.bmin[0], @m_cfg.bmax[0], m_cfg.cs, @m_cfg.width, @m_cfg.height);

  // Reset build times gathering.
  m_ctx.resetTimers();

  // Start the build process.
  m_ctx.startTimer(RC_TIMER_TOTAL);

  m_ctx.log(RC_LOG_PROGRESS, 'Building navigation:');
  m_ctx.log(RC_LOG_PROGRESS, Format(' - %d x %d cells', [m_cfg.width, m_cfg.height]));
  m_ctx.log(RC_LOG_PROGRESS, Format(' - %.1fK verts, %.1fK tris', [nverts/1000.0, ntris/1000.0]));
  
  //
  // Step 2. Rasterize input polygon soup.
  //
  
  // Allocate voxel heightfield where we rasterize our input data to.
  {m_solid := rcAllocHeightfield();
  if (!m_solid)
  begin
    m_ctx.log(RC_LOG_ERROR, 'buildNavigation: Out of memory 'solid'.');
    Exit(false);
  end;}
  if (not rcCreateHeightfield(m_ctx, m_solid, m_cfg.width, m_cfg.height, @m_cfg.bmin[0], @m_cfg.bmax[0], m_cfg.cs, m_cfg.ch)) then
  begin
    m_ctx.log(RC_LOG_ERROR, 'buildNavigation: Could not create solid heightfield.');
    Exit(false);
  end;

  // Allocate array that can hold triangle area types.
  // If you have multiple meshes you need to process, allocate
  // and array which can hold the max number of triangles you need to process.
  GetMem(m_triareas, sizeof(Byte)*ntris);

  // Find triangles which are walkable based on their slope and rasterize them.
  // If your input data is multiple meshes, you can transform them here, calculate
  // the are type for each of the meshes and rasterize them.
  FillChar(m_triareas[0], ntris*sizeof(Byte), 0);
  rcMarkWalkableTriangles(m_ctx, m_cfg.walkableSlopeAngle, verts, nverts, tris, ntris, m_triareas);
  rcRasterizeTriangles(m_ctx, verts, nverts, tris, m_triareas, ntris, m_solid, m_cfg.walkableClimb);

  if (not m_keepInterResults) then
  begin
    FreeMem(m_triareas);
  end;
  
  //
  // Step 3. Filter walkables surfaces.
  //
  
  // Once all geoemtry is rasterized, we do initial pass of filtering to
  // remove unwanted overhangs caused by the conservative rasterization
  // as well as filter spans where the character cannot possibly stand.
  rcFilterLowHangingWalkableObstacles(m_ctx, m_cfg.walkableClimb, m_solid);
  rcFilterLedgeSpans(m_ctx, m_cfg.walkableHeight, m_cfg.walkableClimb, m_solid);
  rcFilterWalkableLowHeightSpans(m_ctx, m_cfg.walkableHeight, m_solid);

  duDumpHeightfield(@m_solid, 'dump_3_hf_d.txt');

  //
  // Step 4. Partition walkable surface to simple regions.
  //

  // Compact the heightfield so that it is faster to handle from now on.
  // This will result more cache coherent data as well as the neighbours
  // between walkable cells will be calculated.
  if (not rcBuildCompactHeightfield(m_ctx, m_cfg.walkableHeight, m_cfg.walkableClimb, @m_solid, @m_chf)) then
  begin
    m_ctx.log(RC_LOG_ERROR, 'buildNavigation: Could not build compact data.');
    Exit(false);
  end;

  if (not m_keepInterResults) then
  begin
    //rcFreeHeightField(m_solid);
    //m_solid := nil;
  end;

  // Erode the walkable area by agent radius.
  if (not rcErodeWalkableArea(m_ctx, m_cfg.walkableRadius, @m_chf)) then
  begin
    m_ctx.log(RC_LOG_ERROR, 'buildNavigation: Could not erode.');
    Exit(false);
  end;

  // (Optional) Mark areas.
  vols := m_geom.getConvexVolumes;
  for i := 0 to m_geom.getConvexVolumeCount - 1 do
    rcMarkConvexPolyArea(m_ctx, @vols[i].verts[0], vols[i].nverts, vols[i].hmin, vols[i].hmax, vols[i].area, @m_chf);


  // Partition the heightfield so that we can use simple algorithm later to triangulate the walkable areas.
  // There are 3 martitioning methods, each with some pros and cons:
  // 1) Watershed partitioning
  //   - the classic Recast partitioning
  //   - creates the nicest tessellation
  //   - usually slowest
  //   - partitions the heightfield into nice regions without holes or overlaps
  //   - the are some corner cases where this method creates produces holes and overlaps
  //      - holes may appear when a small obstacles is close to large open area (triangulation can handle this)
  //      - overlaps may occur if you have narrow spiral corridors (i.e stairs), this make triangulation to fail
  //   * generally the best choice if you precompute the nacmesh, use this if you have large open areas
  // 2) Monotone partioning
  //   - fastest
  //   - partitions the heightfield into regions without holes and overlaps (guaranteed)
  //   - creates long thin polygons, which sometimes causes paths with detours
  //   * use this if you want fast navmesh generation
  // 3) Layer partitoining
  //   - quite fast
  //   - partitions the heighfield into non-overlapping regions
  //   - relies on the triangulation code to cope with holes (thus slower than monotone partitioning)
  //   - produces better triangles than monotone partitioning
  //   - does not have the corner cases of watershed partitioning
  //   - can be slow and create a bit ugly tessellation (still better than monotone)
  //     if you have large open areas with small obstacles (not a problem if you use tiles)
  //   * good choice to use for tiled navmesh with medium and small sized tiles

  if (m_partitionType = SAMPLE_PARTITION_WATERSHED) then
  begin
    // Prepare for region partitioning, by calculating distance field along the walkable surface.
    if (not rcBuildDistanceField(m_ctx, @m_chf)) then
    begin
      m_ctx.log(RC_LOG_ERROR, 'buildNavigation: Could not build distance field.');
      Exit(false);
    end;

    // Partition the walkable surface into simple regions without holes.
    if (not rcBuildRegions(m_ctx, @m_chf, 0, m_cfg.minRegionArea, m_cfg.mergeRegionArea)) then
    begin
      m_ctx.log(RC_LOG_ERROR, 'buildNavigation: Could not build watershed regions.');
      Exit(false);
    end;
  end
  else if (m_partitionType = SAMPLE_PARTITION_MONOTONE) then
  begin
    {// Partition the walkable surface into simple regions without holes.
    // Monotone partitioning does not need distancefield.
    if (not rcBuildRegionsMonotone(m_ctx, @m_chf, 0, m_cfg.minRegionArea, m_cfg.mergeRegionArea)) then
    begin
      m_ctx.log(RC_LOG_ERROR, 'buildNavigation: Could not build monotone regions.');
      Exit(false);
    end;}
  end
  else // SAMPLE_PARTITION_LAYERS
  begin
    {// Partition the walkable surface into simple regions without holes.
    if (not rcBuildLayerRegions(m_ctx, @m_chf, 0, m_cfg.minRegionArea)) then
    begin
      m_ctx.log(RC_LOG_ERROR, 'buildNavigation: Could not build layer regions.');
      Exit(false);
    end;}
  end;

  duDumpCompactHeightfield(@m_chf, 'dump_4_chf_d.txt');

  //
  // Step 5. Trace and simplify region contours.
  //

  // Create contours.
  if (not rcBuildContours(m_ctx, @m_chf, m_cfg.maxSimplificationError, m_cfg.maxEdgeLen, @m_cset)) then
  begin
    m_ctx.log(RC_LOG_ERROR, 'buildNavigation: Could not create contours.');
    Exit(false);
  end;

  duDumpContourSet(@m_cset, 'dump_5_cset_d.txt');

  //
  // Step 6. Build polygons mesh from contours.
  //

  // Build polygon navmesh from the contours.
  if (not rcBuildPolyMesh(m_ctx, @m_cset, m_cfg.maxVertsPerPoly, @m_pmesh)) then
  begin
    m_ctx.log(RC_LOG_ERROR, 'buildNavigation: Could not triangulate contours.');
    Exit(false);
  end;

  duDumpPolyMeshToObj(@m_pmesh, 'dump_6_pmesh_d.txt');

  //
  // Step 7. Create detail mesh which allows to access approximate height on each polygon.
  //
  if (not rcBuildPolyMeshDetail(m_ctx, @m_pmesh, @m_chf, m_cfg.detailSampleDist, m_cfg.detailSampleMaxError, @m_dmesh)) then
  begin
    m_ctx.log(RC_LOG_ERROR, 'buildNavigation: Could not build detail mesh.');
    Exit(false);
  end;

  duDumpPolyMeshDetailToObj(@m_dmesh, 'dump_7_dmesh_d.txt');

  if (not m_keepInterResults) then
  begin
    //rcFreeCompactHeightfield(m_chf);
    //m_chf := 0;
    //rcFreeContourSet(m_cset);
    //m_cset := 0;
  end;

  // At this point the navigation mesh data is ready, you can access it from m_pmesh.
  // See duDebugDrawPolyMesh or dtCreateNavMeshData as examples how to access the data.

  //
  // (Optional) Step 8. Create Detour data from Recast poly mesh.
  //

  // The GUI may allow more max points per polygon than Detour can handle.
  // Only build the detour navmesh if we do not exceed the limit.
  if (m_cfg.maxVertsPerPoly <= DT_VERTS_PER_POLYGON) then
  begin
    navData := nil;
    navDataSize := 0;

    // Update poly flags from areas.
    for i := 0 to m_pmesh.npolys - 1 do
    begin
      if (m_pmesh.areas[i] = RC_WALKABLE_AREA) then
        m_pmesh.areas[i] := Byte(SAMPLE_POLYAREA_GROUND);

      if (m_pmesh.areas[i] = Byte(SAMPLE_POLYAREA_GROUND)) or
        (m_pmesh.areas[i] = Byte(SAMPLE_POLYAREA_GRASS)) or
        (m_pmesh.areas[i] = Byte(SAMPLE_POLYAREA_ROAD)) then
      begin
        m_pmesh.flags[i] := Byte(SAMPLE_POLYFLAGS_WALK);
      end
      else if (m_pmesh.areas[i] = Byte(SAMPLE_POLYAREA_WATER)) then
      begin
        m_pmesh.flags[i] := Byte(SAMPLE_POLYFLAGS_SWIM);
      end
      else if (m_pmesh.areas[i] = Byte(SAMPLE_POLYAREA_DOOR)) then
      begin
        m_pmesh.flags[i] := Byte(SAMPLE_POLYFLAGS_WALK) or Byte(SAMPLE_POLYFLAGS_DOOR);
      end;
    end;

    FillChar(params, sizeof(params), 0);
    params.verts := m_pmesh.verts;
    params.vertCount := m_pmesh.nverts;
    params.polys := m_pmesh.polys;
    params.polyAreas := m_pmesh.areas;
    params.polyFlags := m_pmesh.flags;
    params.polyCount := m_pmesh.npolys;
    params.nvp := m_pmesh.nvp;
    params.detailMeshes := m_dmesh.meshes;
    params.detailVerts := m_dmesh.verts;
    params.detailVertsCount := m_dmesh.nverts;
    params.detailTris := m_dmesh.tris;
    params.detailTriCount := m_dmesh.ntris;
    params.offMeshConVerts := m_geom.getOffMeshConnectionVerts;
    params.offMeshConRad := m_geom.getOffMeshConnectionRads;
    params.offMeshConDir := m_geom.getOffMeshConnectionDirs;
    params.offMeshConAreas := m_geom.getOffMeshConnectionAreas;
    params.offMeshConFlags := m_geom.getOffMeshConnectionFlags;
    params.offMeshConUserID := m_geom.getOffMeshConnectionId;
    params.offMeshConCount := m_geom.getOffMeshConnectionCount;
    params.walkableHeight := m_agentHeight;
    params.walkableRadius := m_agentRadius;
    params.walkableClimb := m_agentMaxClimb;
    rcVcopy(@params.bmin[0], @m_pmesh.bmin[0]);
    rcVcopy(@params.bmax[0], @m_pmesh.bmax[0]);
    params.cs := m_cfg.cs;
    params.ch := m_cfg.ch;
    params.buildBvTree := true;

    if (not dtCreateNavMeshData(@params, @navData, @navDataSize)) then
    begin
      m_ctx.log(RC_LOG_ERROR, 'Could not build Detour navmesh.');
      Exit(false);
    end;

    m_navMesh := TdtNavMesh.Create;

    status := m_navMesh.init(navData, navDataSize, DT_TILE_FREE_DATA);
    if (dtStatusFailed(status)) then
    begin
      FreeMem(navData);
      m_ctx.log(RC_LOG_ERROR, 'Could not init Detour navmesh');
      Exit(false);
    end;

    status := m_navQuery.init(m_navMesh, 2048);
    if (dtStatusFailed(status)) then
    begin
      m_ctx.log(RC_LOG_ERROR, 'Could not init Detour navmesh query');
      Exit(false);
    end;
  end;

  m_ctx.stopTimer(RC_TIMER_TOTAL);

  // Show performance stats.
  duLogBuildTimes(m_ctx, m_ctx.getAccumulatedTime(RC_TIMER_TOTAL));
  m_ctx.log(RC_LOG_PROGRESS, Format('>> Polymesh: %d vertices  %d polygons', [m_pmesh.nverts, m_pmesh.npolys]));

  m_totalBuildTimeMs := m_ctx.getAccumulatedTime(RC_TIMER_TOTAL)/1000.0;

  if (m_tool <> nil) then
    m_tool.init(Self);
  initToolStates(Self);

  Result := true;
end;

end.
