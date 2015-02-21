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

unit RN_SampleTileMesh;
interface
uses
  Classes, Controls, Math, OpenGL, StrUtils, SysUtils, ExtCtrls,
  Unit_FrameSampleTileMesh,
  RN_InputGeom, RN_Sample, RN_DetourNavMesh, RN_Recast, RN_ChunkyTriMesh;

type
  TSample_TileMesh = class(TSample)
  protected
    fSampleFrame: TWinControl;
    fToolFrame: TWinControl;
    fFrame: TFrameSampleTileMesh;
    fTools: TRadioGroup;
    fUpdateUI: Boolean;

    m_keepInterResults: Boolean;
    m_buildAll: Boolean;
    m_totalBuildTimeMs: Single;

    m_triareas: PByte;
    m_solid: TrcHeightfield;
    m_chf: TrcCompactHeightfield;
    m_cset: TrcContourSet;
    m_pmesh: TrcPolyMesh;
    m_dmesh: TrcPolyMeshDetail;
    m_cfg: TrcConfig;

    type
    TDrawMode =
    (
      DRAWMODE_NAVMESH,
      DRAWMODE_NAVMESH_TRANS,
      DRAWMODE_NAVMESH_BVTREE,
      DRAWMODE_NAVMESH_NODES,
      DRAWMODE_NAVMESH_PORTALS,
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

    var
    m_drawMode: TDrawMode;

    m_maxTiles: Integer;
    m_maxPolysPerTile: Integer;
    m_tileSize: Single;

    m_tileCol: Cardinal;
    m_tileBmin: array [0..2] of Single;
    m_tileBmax: array [0..2] of Single;
    m_tileBuildTime: Single;
    m_tileMemUsage: Single;
    m_tileTriCount: Integer;

    function getDrawModeItems: string;
    function getToolItems: string;
    procedure setToolType(aType: TSampleToolType);
    function buildTileMesh(tx, ty: Integer; bmin, bmax: PSingle; dataSize: PInteger): PByte;

    procedure cleanup();

    //procedure saveAll(const path: string; mesh: TdtNavMesh);
    //function loadAll(const path: string): TdtNavMesh;

  public
    constructor Create(aSampleFrame, aToolFrame: TWinControl; aTools: TRadioGroup);
    destructor Destroy; override;

    procedure handleSettings(); override;
    procedure handleTools(Sender: TObject); override;
    procedure handleDebugMode(); override;
    procedure handleRender(); override;
    procedure handleRenderOverlay(proj, model: PDouble; view: PInteger); override;
    procedure handleMeshChanged(geom: TInputGeom); override;
    function handleBuild(): Boolean; override;
    procedure handleGUI(Sender: TObject);

    procedure getTilePos(pos: PSingle; tx, ty: PInteger);

    procedure buildTile(pos: PSingle);
    procedure removeTile(pos: PSingle);
    procedure buildAllTiles();
    procedure removeAllTiles();
  end;


implementation
uses
  RN_DebugDraw, RN_RecastDebugDraw, RN_DetourNavMeshHelper, RN_DetourStatus, RN_DetourNavMeshBuilder, RN_DetourDebugDraw,
  RN_NavMeshTesterTool, RN_NavMeshPruneTool, RN_NavMeshTileTool, RN_SampleInterfaces, RN_RecastHelper, RN_RecastRasterization,
  RN_RecastFilter, RN_RecastArea, RN_RecastRegion, RN_RecastContour, RN_RecastMesh, RN_RecastMeshDetail, RN_RecastDump,
  {RN_OffMeshConnectionTool, RN_ConvexVolumeTool,} RN_CrowdTool;


function NextPow2(v: Cardinal): Cardinal;
begin
  if v > 0 then
  begin
    Dec(v);
    v := v or (v shr 1);
    v := v or (v shr 2);
    v := v or (v shr 4);
    v := v or (v shr 8);
    v := v or (v shr 16);
    Inc(v);
  end;
  Result := v;
end;

function Ilog2(v: Cardinal): Cardinal;
var r, shift: Cardinal;
begin
  r := Byte(v > $ffff) shl 4; v := v shr r;
  shift := Byte(v > $ff) shl 3; v := v shr shift; r := r or shift;
  shift := Byte(v > $f) shl 2; v := v shr shift; r := r or shift;
  shift := Byte(v > $3) shl 1; v := v shr shift; r := r or shift;
  r := r or Byte(v shr 1);
  Result := r;
end;


constructor TSample_TileMesh.Create(aSampleFrame, aToolFrame: TWinControl; aTools: TRadioGroup);
begin
  inherited Create;

  m_buildAll := true;
  m_drawMode := DRAWMODE_NAVMESH;
  m_tileSize := 240;
  m_tileCol := duRGBA(0,0,0,32);

  fSampleFrame := aSampleFrame;
  fToolFrame := aToolFrame;
  fTools := aTools;
  fTools.OnClick := handleTools;

  fFrame := TFrameSampleTileMesh.Create(fSampleFrame);
  fFrame.Align := alClient;
  fFrame.Parent := fSampleFrame;
  fFrame.Visible := True;

  fFrame.seCellSize.OnChange := handleGUI;
  fFrame.seMaxSampleError.OnChange := handleGUI;
  fFrame.seSampleDistance.OnChange := handleGUI;
  fFrame.seVertsPerPoly.OnChange := handleGUI;
  fFrame.seMaxEdgeError.OnChange := handleGUI;
  fFrame.seMaxEdgeLength.OnChange := handleGUI;
  fFrame.rgPartitioning.OnClick := handleGUI;
  fFrame.seMergedRegionSize.OnChange := handleGUI;
  fFrame.seMinRegionSize.OnChange := handleGUI;
  fFrame.seMaxSlope.OnChange := handleGUI;
  fFrame.seMaxClimb.OnChange := handleGUI;
  fFrame.seAgentRadius.OnChange := handleGUI;
  fFrame.seAgentHeight.OnChange := handleGUI;
  fFrame.seCellHeight.OnChange := handleGUI;
  fFrame.cbKeepIntermediateResults.OnClick := handleGUI;
  fFrame.cbBuildAll.OnClick := handleGUI;
  fFrame.tbTileSize.OnChange := handleGUI;
  fFrame.rgDrawMode.OnClick := handleGUI;

  handleGUI(nil);

  resetCommonSettings();
  FillChar(m_tileBmin[0], sizeof(m_tileBmin), 0);
  FillChar(m_tileBmax[0], sizeof(m_tileBmax), 0);

  setTool(TNavMeshTileTool.Create(fToolFrame));
end;

destructor TSample_TileMesh.Destroy;
begin
  cleanup();
  m_navMesh.Free;
  m_navMesh := nil;
  fFrame.Free;
  inherited;
end;

procedure TSample_TileMesh.cleanup();
begin
  FreeMem(m_triareas);
  m_triareas := nil;
end;


const NAVMESHSET_MAGIC: Integer = Ord('M') shl 24 + Ord('S') shl 16 + Ord('E') shl 8 + Ord('T'); //'MSET';
const NAVMESHSET_VERSION: Integer = 1;

type
  TNavMeshSetHeader = record
    magic: Integer;
    version: Integer;
    numTiles: Integer;
    params: TdtNavMeshParams;
  end;

  TNavMeshTileHeader = record
    tileRef: TdtTileRef;
    dataSize: Integer;
  end;

{procedure TSample_TileMesh.saveAll(const path: string; mesh: TdtNavMesh);
begin
  if (mesh = nil) then Exit;

  FILE* fp := fopen(path, "wb");
  if (!fp)
    return;

  // Store header.
  NavMeshSetHeader header;
  header.magic := NAVMESHSET_MAGIC;
  header.version := NAVMESHSET_VERSION;
  header.numTiles := 0;
  for (int i := 0; i < mesh->getMaxTiles(); ++i)
  begin
    const dtMeshTile* tile := mesh->getTile(i);
    if (!tile || !tile->header || !tile->dataSize) continue;
    header.numTiles++;
  end;
  memcpy(&header.params, mesh->getParams(), sizeof(dtNavMeshParams));
  fwrite(&header, sizeof(NavMeshSetHeader), 1, fp);

  // Store tiles.
  for (int i := 0; i < mesh->getMaxTiles(); ++i)
  begin
    const dtMeshTile* tile := mesh->getTile(i);
    if (!tile || !tile->header || !tile->dataSize) continue;

    NavMeshTileHeader tileHeader;
    tileHeader.tileRef := mesh->getTileRef(tile);
    tileHeader.dataSize := tile->dataSize;
    fwrite(&tileHeader, sizeof(tileHeader), 1, fp);

    fwrite(tile->data, tile->dataSize, 1, fp);
  end;

  fclose(fp);
end;}

{function TSample_TileMesh.loadAll(const path: string): TdtNavMesh;
var mesh: TdtNavMesh;
begin
  FILE* fp := fopen(path, "rb");
  if (!fp) return 0;

  // Read header.
  NavMeshSetHeader header;
  size_t readLen := fread(&header, sizeof(NavMeshSetHeader), 1, fp);
  if (readLen != 1)
  begin
    fclose(fp);
    return 0;
  end;
  if (header.magic != NAVMESHSET_MAGIC)
  begin
    fclose(fp);
    return 0;
  end;
  if (header.version != NAVMESHSET_VERSION)
  begin
    fclose(fp);
    return 0;
  end;

  dtNavMesh* mesh := dtAllocNavMesh();
  if (!mesh)
  begin
    fclose(fp);
    return 0;
  end;
  dtStatus status := mesh->init(&header.params);
  if (dtStatusFailed(status))
  begin
    fclose(fp);
    return 0;
  end;

  // Read tiles.
  for (int i := 0; i < header.numTiles; ++i)
  begin
    NavMeshTileHeader tileHeader;
    readLen := fread(&tileHeader, sizeof(tileHeader), 1, fp);
    if (readLen != 1)
      return 0;

    if (!tileHeader.tileRef || !tileHeader.dataSize)
      break;

    unsigned char* data := (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
    if (!data) break;
    memset(data, 0, tileHeader.dataSize);
    readLen := fread(data, tileHeader.dataSize, 1, fp);
    if (readLen != 1)
      return 0;

    mesh->addTile(data, tileHeader.dataSize, DT_TILE_FREE_DATA, tileHeader.tileRef, 0);
  end;

  fclose(fp);

  Result := mesh;
end;}

procedure TSample_TileMesh.handleSettings();
var bmin, bmax: PSingle; gw, gh, ts, tw, th: Integer; tileBits, polyBits: Integer;
begin
  handleCommonSettings();

  if (m_geom <> nil) then
  begin
    bmin := m_geom.getMeshBoundsMin;
    bmax := m_geom.getMeshBoundsMax;
    //char text[64];
    gw := 0; gh := 0;
    rcCalcGridSize(bmin, bmax, m_cellSize, @gw, @gh);
    ts := Trunc(m_tileSize);
    tw := (gw + ts-1) div ts;
    th := (gh + ts-1) div ts;
    //snprintf(text, 64, "Tiles  %d x %d", tw, th);
    //imguiValue(text);

    // Max tiles and max polys affect how the tile IDs are caculated.
    // There are 22 bits available for identifying a tile and a polygon.
    tileBits := Min(Trunc(ilog2(nextPow2(tw*th))), 14);
    if (tileBits > 14) then tileBits := 14;
    polyBits := 22 - tileBits;
    m_maxTiles := 1 shl tileBits;
    m_maxPolysPerTile := 1 shl polyBits;
    //snprintf(text, 64, "Max Tiles  %d", m_maxTiles);
    //imguiValue(text);
    //snprintf(text, 64, "Max Polys  %d", m_maxPolysPerTile);
    //imguiValue(text);
  end
  else
  begin
    m_maxTiles := 0;
    m_maxPolysPerTile := 0;
  end;

{  imguiSeparator();

  imguiIndent();
  imguiIndent();

  if (imguiButton("Save"))
  begin
    saveAll("all_tiles_navmesh.bin", m_navMesh);
  end;

  if (imguiButton("Load"))
  begin
    dtFreeNavMesh(m_navMesh);
    m_navMesh := loadAll("all_tiles_navmesh.bin");
    m_navQuery->init(m_navMesh, 2048);
  end;

  imguiUnindent();
  imguiUnindent();

  char msg[64];
  snprintf(msg, 64, "Build Time: %.1fms", m_totalBuildTimeMs);
  imguiLabel(msg);

  imguiSeparator();

  imguiSeparator();
}
end;

function TSample_TileMesh.getToolItems: string;
begin
  Result := ' ' + sLineBreak +
            'Tile Edit' + sLineBreak +
            ' ' + sLineBreak +
            ' ' + sLineBreak +
            'Test Navmesh' + sLineBreak +
            'Prune Navmesh' + sLineBreak +
            'Create Off-Mesh Links' + sLineBreak +
            'Create Convex Volumes' + sLineBreak +
            'Create Crowds';
end;

procedure TSample_TileMesh.handleTools(Sender: TObject);
begin
  setToolType(TSampleToolType(fTools.ItemIndex));
end;

procedure TSample_TileMesh.setToolType(aType: TSampleToolType);
begin
  // Delphi. Dispose of any previous tool and it's Frame
  setTool(nil);

  case aType of
    TOOL_NONE:                setTool(nil);
    TOOL_NAVMESH_TESTER:      setTool(TNavMeshTesterTool.Create(fToolFrame));
    TOOL_NAVMESH_PRUNE:       setTool(TNavMeshPruneTool.Create(fToolFrame));
    //todo: TOOL_OFFMESH_CONNECTION:  setTool(TOffMeshConnectionTool.Create);
    //todo: TOOL_CONVEX_VOLUME:       setTool(TConvexVolumeTool.Create);
    TOOL_CROWD:               setTool(TCrowdTool.Create(fToolFrame));
  else
    setTool(nil);
  end;

  if m_tool <> nil then
    m_tool.handleMenu(nil);
end;

function TSample_TileMesh.getDrawModeItems: string;
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
    valid[DRAWMODE_NAVMESH_PORTALS] := m_navMesh <> nil;
    valid[DRAWMODE_NAVMESH_INVIS] := m_navMesh <> nil;
    valid[DRAWMODE_MESH] := true;
    valid[DRAWMODE_VOXELS] := true; //m_solid != 0;
    valid[DRAWMODE_VOXELS_WALKABLE] := true; //m_solid != 0;
    valid[DRAWMODE_COMPACT] := true; //m_chf != 0;
    valid[DRAWMODE_COMPACT_DISTANCE] := true; //m_chf != 0;
    valid[DRAWMODE_COMPACT_REGIONS] := true; //m_chf != 0;
    valid[DRAWMODE_REGION_CONNECTIONS] := true; //m_cset != 0;
    valid[DRAWMODE_RAW_CONTOURS] := true; //m_cset != 0;
    valid[DRAWMODE_BOTH_CONTOURS] := true; //m_cset != 0;
    valid[DRAWMODE_CONTOURS] := true; //m_cset != 0;
    valid[DRAWMODE_POLYMESH] := true; //m_pmesh != 0;
    valid[DRAWMODE_POLYMESH_DETAIL] := true; //m_dmesh != 0;
  end;


  Result := 'Navmesh' + IfThen(not valid[DRAWMODE_NAVMESH], ' ') + sLineBreak +
            'Navmesh Trans' + IfThen(not valid[DRAWMODE_NAVMESH_TRANS], ' ') + sLineBreak +
            'Navmesh BVTree' + IfThen(not valid[DRAWMODE_NAVMESH_BVTREE], ' ') + sLineBreak +
            'Navmesh Nodes' + IfThen(not valid[DRAWMODE_NAVMESH_NODES], ' ') + sLineBreak +
            'Navmesh Portals' + IfThen(not valid[DRAWMODE_NAVMESH_PORTALS], ' ') + sLineBreak +
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

procedure TSample_TileMesh.handleDebugMode();
begin
  //Delphi. Refactored
end;

procedure TSample_TileMesh.handleGUI(Sender: TObject);
var
  I: Integer;
begin
  if fUpdateUI then Exit;

  if Sender = nil then
  begin
    fUpdateUI := True;
    fFrame.seCellSize.Value := Round(m_cellSize * 10);
    fFrame.seCellHeight.Value := Round(m_cellHeight * 10);
    fFrame.seAgentHeight.Value := Round(m_agentHeight * 10);
    fFrame.seAgentRadius.Value := Round(m_agentRadius * 10);
    fFrame.seMaxClimb.Value := Round(m_agentMaxClimb * 10);
    fFrame.seMaxSlope.Value := Round(m_agentMaxSlope);

    fFrame.seMinRegionSize.Value := Round(m_regionMinSize);
    fFrame.seMergedRegionSize.Value := Round(m_regionMergeSize);
    fFrame.seMaxEdgeLength.Value := Round(m_edgeMaxLen);
    fFrame.seMaxEdgeError.Value := Round(m_edgeMaxError * 10);
    fFrame.seVertsPerPoly.Value := Round(m_vertsPerPoly);
    fFrame.seSampleDistance.Value := Round(m_detailSampleDist);
    fFrame.seMaxSampleError.Value := Round(m_detailSampleMaxError);
    fFrame.rgPartitioning.ItemIndex := Byte(m_partitionType);

    fFrame.cbKeepIntermediateResults.Checked := m_keepInterResults;
    fFrame.cbBuildAll.Checked := m_buildAll;

    fFrame.tbTileSize.Position := Round(m_tileSize);

    fFrame.rgDrawMode.Items.Text := getDrawModeItems;
    for I := 0 to fFrame.rgDrawMode.Items.Count - 1 do
      fFrame.rgDrawMode.Buttons[I].Enabled := RightStr(fFrame.rgDrawMode.Items[I], 1) <> ' ';
    fFrame.rgDrawMode.ItemIndex := Byte(m_drawMode);

    fTools.Items.Text := getToolItems;
    for I := 0 to fTools.Items.Count - 1 do
      fTools.Buttons[I].Enabled := RightStr(fTools.Items[I], 1) <> ' ';
    fTools.ItemIndex := 0;

    fUpdateUI := False;
    Exit;
  end;

  m_cellSize := fFrame.seCellSize.Value / 10;
  m_cellHeight := fFrame.seCellHeight.Value / 10;
  m_agentHeight := fFrame.seAgentHeight.Value / 10;
  m_agentRadius := fFrame.seAgentRadius.Value / 10;
  m_agentMaxClimb := fFrame.seMaxClimb.Value / 10;
  m_agentMaxSlope := fFrame.seMaxSlope.Value;
  m_regionMinSize := fFrame.seMinRegionSize.Value;
  m_regionMergeSize := fFrame.seMergedRegionSize.Value;
  m_edgeMaxLen := fFrame.seMaxEdgeLength.Value;
  m_edgeMaxError := fFrame.seMaxEdgeError.Value / 10;
  m_vertsPerPoly := fFrame.seVertsPerPoly.Value;
  m_detailSampleDist := fFrame.seSampleDistance.Value;
  m_detailSampleMaxError := fFrame.seMaxSampleError.Value;
  m_partitionType := TSamplePartitionType(fFrame.rgPartitioning.ItemIndex);

  m_keepInterResults := fFrame.cbKeepIntermediateResults.Checked;
  m_buildAll := fFrame.cbBuildAll.Checked;
  m_tileSize := fFrame.tbTileSize.Position;

  m_drawMode := TDrawMode(fFrame.rgDrawMode.ItemIndex);
end;

procedure TSample_TileMesh.handleRender();
var dd: TDebugDrawGL; texScale: Single; bmin, bmax: PSingle; gw, gh, tw, th: Integer; s: Single;
begin
  if (m_geom = nil) or (m_geom.getMesh = nil) then
    Exit;

  dd := TDebugDrawGL.Create;

  texScale := 1.0 / (m_cellSize * 10.0);

  // Draw mesh
  if (m_drawMode <> DRAWMODE_NAVMESH_TRANS) then
  begin
    // Draw mesh
    duDebugDrawTriMeshSlope(dd, m_geom.getMesh.getVerts, m_geom.getMesh.getVertCount,
                m_geom.getMesh.getTris, m_geom.getMesh.getNormals, m_geom.getMesh.getTriCount,
                m_agentMaxSlope, texScale);
    m_geom.drawOffMeshConnections(dd);
  end;
    
  glDepthMask(GL_FALSE);
  
  // Draw bounds
  bmin := m_geom.getMeshBoundsMin;
  bmax := m_geom.getMeshBoundsMax;
  duDebugDrawBoxWire(dd, bmin[0],bmin[1],bmin[2], bmax[0],bmax[1],bmax[2], duRGBA(255,255,255,128), 1.0);
  
  // Tiling grid.
  gw := 0; gh := 0;
  rcCalcGridSize(bmin, bmax, m_cellSize, @gw, @gh);
  tw := (gw + Trunc(m_tileSize)-1) div Trunc(m_tileSize);
  th := (gh + Trunc(m_tileSize)-1) div Trunc(m_tileSize);
  s := m_tileSize*m_cellSize;
  duDebugDrawGridXZ(dd, bmin[0],bmin[1],bmin[2], tw,th, s, duRGBA(0,0,0,64), 1.0);
  
  // Draw active tile
  duDebugDrawBoxWire(dd, m_tileBmin[0],m_tileBmin[1],m_tileBmin[2],
             m_tileBmax[0],m_tileBmax[1],m_tileBmax[2], m_tileCol, 1.0);
    
  if (m_navMesh <> nil) and (m_navQuery <> nil) and
    (m_drawMode in [DRAWMODE_NAVMESH,
                    DRAWMODE_NAVMESH_TRANS,
                    DRAWMODE_NAVMESH_BVTREE,
                    DRAWMODE_NAVMESH_NODES,
                    DRAWMODE_NAVMESH_PORTALS,
                    DRAWMODE_NAVMESH_INVIS]) then
  begin
    if (m_drawMode <> DRAWMODE_NAVMESH_INVIS) then
      duDebugDrawNavMeshWithClosedList(dd, m_navMesh, m_navQuery, m_navMeshDrawFlags);
    if (m_drawMode = DRAWMODE_NAVMESH_BVTREE) then
      duDebugDrawNavMeshBVTree(dd, m_navMesh);
    if (m_drawMode = DRAWMODE_NAVMESH_PORTALS) then
      duDebugDrawNavMeshPortals(dd, m_navMesh);
    if (m_drawMode = DRAWMODE_NAVMESH_NODES) then
      duDebugDrawNavMeshNodes(dd, m_navQuery);
    duDebugDrawNavMeshPolysWithFlags(dd, m_navMesh, SAMPLE_POLYFLAGS_DISABLED, duRGBA(0,0,0,128));
  end;


  glDepthMask(GL_TRUE);

  if {(m_chf <> nil) and} (m_drawMode = DRAWMODE_COMPACT) then
    duDebugDrawCompactHeightfieldSolid(dd, @m_chf);

  if {(m_chf <> nil) and} (m_drawMode = DRAWMODE_COMPACT_DISTANCE) then
    duDebugDrawCompactHeightfieldDistance(dd, @m_chf);
  if {(m_chf <> nil) and} (m_drawMode = DRAWMODE_COMPACT_REGIONS) then
    duDebugDrawCompactHeightfieldRegions(dd, @m_chf);
  if {(m_solid <> nil) and} (m_drawMode = DRAWMODE_VOXELS) then
  begin
    glEnable(GL_FOG);
    duDebugDrawHeightfieldSolid(dd, @m_solid);
    glDisable(GL_FOG);
  end;
  if {(m_solid <> nil) and} (m_drawMode = DRAWMODE_VOXELS_WALKABLE) then
  begin
    glEnable(GL_FOG);
    duDebugDrawHeightfieldWalkable(dd, @m_solid);
    glDisable(GL_FOG);
  end;

  if {(m_cset <> nil) and} (m_drawMode = DRAWMODE_RAW_CONTOURS) then
  begin
    glDepthMask(GL_FALSE);
    duDebugDrawRawContours(dd, @m_cset);
    glDepthMask(GL_TRUE);
  end;

  if {(m_cset <> nil) and} (m_drawMode = DRAWMODE_BOTH_CONTOURS) then
  begin
    glDepthMask(GL_FALSE);
    duDebugDrawRawContours(dd, @m_cset, 0.5);
    duDebugDrawContours(dd, @m_cset);
    glDepthMask(GL_TRUE);
  end;
  if {(m_cset <> nil) and} (m_drawMode = DRAWMODE_CONTOURS) then
  begin
    glDepthMask(GL_FALSE);
    duDebugDrawContours(dd, @m_cset);
    glDepthMask(GL_TRUE);
  end;
  if ({m_chf && m_cset &&} m_drawMode = DRAWMODE_REGION_CONNECTIONS) then
  begin
    duDebugDrawCompactHeightfieldRegions(dd, @m_chf);

    glDepthMask(GL_FALSE);
    duDebugDrawRegionConnections(dd, @m_cset);
    glDepthMask(GL_TRUE);
  end;
  if ({m_pmesh &&} m_drawMode = DRAWMODE_POLYMESH) then
  begin
    glDepthMask(GL_FALSE);
    duDebugDrawPolyMesh(dd, @m_pmesh);
    glDepthMask(GL_TRUE);
  end;
  if ({m_dmesh &&} m_drawMode = DRAWMODE_POLYMESH_DETAIL) then
  begin
    glDepthMask(GL_FALSE);
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

procedure TSample_TileMesh.handleRenderOverlay(proj, model: PDouble; view: PInteger);
begin
{  GLdouble x, y, z;

  // Draw start and end point labels
  if (m_tileBuildTime > 0.0f && gluProject((GLdouble)(m_tileBmin[0]+m_tileBmax[0])/2, (GLdouble)(m_tileBmin[1]+m_tileBmax[1])/2, (GLdouble)(m_tileBmin[2]+m_tileBmax[2])/2,
                       model, proj, view, &x, &y, &z))
  begin
    char text[32];
    snprintf(text,32,"%.3fms / %dTris / %.1fkB", m_tileBuildTime, m_tileTriCount, m_tileMemUsage);
    imguiDrawText((int)x, (int)y-25, IMGUI_ALIGN_CENTER, text, imguiRGBA(0,0,0,220));
  end;

  if (m_tool)
    m_tool.handleRenderOverlay(proj, model, view);
  renderOverlayToolStates(proj, model, view);}
end;

procedure TSample_TileMesh.handleMeshChanged(geom: TInputGeom);
begin
  inherited handleMeshChanged(geom);

  cleanup();

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

function TSample_TileMesh.handleBuild(): Boolean;
var
  params: TdtNavMeshParams; status: TdtStatus;
begin
  if (m_geom = nil) or (m_geom.getMesh = nil) then
  begin
    m_ctx.log(RC_LOG_ERROR, 'buildTiledNavigation: No vertices and triangles.');
    Exit(False);
  end;

  m_navMesh.Free;

  m_navMesh := TdtNavMesh.Create;

  rcVcopy(@params.orig[0], m_geom.getMeshBoundsMin);
  params.tileWidth := m_tileSize*m_cellSize;
  params.tileHeight := m_tileSize*m_cellSize;
  params.maxTiles := m_maxTiles;
  params.maxPolys := m_maxPolysPerTile;

  status := m_navMesh.init(@params);
  if (dtStatusFailed(status)) then
  begin
    m_ctx.log(RC_LOG_ERROR, 'buildTiledNavigation: Could not init navmesh.');
    Exit(False);
  end;

  status := m_navQuery.init(m_navMesh, 2048);
  if (dtStatusFailed(status)) then
  begin
    m_ctx.log(RC_LOG_ERROR, 'buildTiledNavigation: Could not init Detour navmesh query');
    Exit(False);
  end;

  if (m_buildAll) then
    buildAllTiles;

  handleGUI(nil);

  if (m_tool <> nil) then
    m_tool.init(Self);
  initToolStates(Self);

  Result := true;
end;

procedure TSample_TileMesh.buildTile(pos: PSingle);
begin
{  if (m_geom = nil) then Exit;
  if (m_navMesh = nil) then Exit;

  const float* bmin := m_geom.getMeshBoundsMin();
  const float* bmax := m_geom.getMeshBoundsMax();
  
  const float ts := m_tileSize*m_cellSize;
  const int tx := (int)((pos[0] - bmin[0]) / ts);
  const int ty := (int)((pos[2] - bmin[2]) / ts);
  
  m_tileBmin[0] := bmin[0] + tx*ts;
  m_tileBmin[1] := bmin[1];
  m_tileBmin[2] := bmin[2] + ty*ts;
  
  m_tileBmax[0] := bmin[0] + (tx+1)*ts;
  m_tileBmax[1] := bmax[1];
  m_tileBmax[2] := bmin[2] + (ty+1)*ts;
  
  m_tileCol := duRGBA(255,255,255,64);
  
  m_ctx.resetLog();
  
  int dataSize := 0;
  unsigned char* data := buildTileMesh(tx, ty, m_tileBmin, m_tileBmax, dataSize);

  // Remove any previous data (navmesh owns and deletes the data).
  m_navMesh.removeTile(m_navMesh.getTileRefAt(tx,ty,0),0,0);

  // Add tile, or leave the location empty.
  if (data)
  begin
    // Let the navmesh own the data.
    dtStatus status := m_navMesh.addTile(data,dataSize,DT_TILE_FREE_DATA,0,0);
    if (dtStatusFailed(status))
      dtFree(data);
  end;

  m_ctx.dumpLog("Build Tile (%d,%d):", tx,ty);}
end;

procedure TSample_TileMesh.getTilePos(pos: PSingle; tx, ty: PInteger);
var bmin: PSingle; ts: Single;
begin
  if (m_geom = nil) then Exit;

  bmin := m_geom.getMeshBoundsMin();

  ts := m_tileSize*m_cellSize;
  tx^ := Trunc((pos[0] - bmin[0]) / ts);
  ty^ := Trunc((pos[2] - bmin[2]) / ts);
end;

procedure TSample_TileMesh.removeTile(pos: PSingle);
var bmin, bmax: PSingle; ts: Single; tx, ty: Integer;
begin
  if (m_geom = nil) then Exit;
  if (m_navMesh = nil) then Exit;

  bmin := m_geom.getMeshBoundsMin();
  bmax := m_geom.getMeshBoundsMax();

  ts := m_tileSize*m_cellSize;
  tx := Trunc((pos[0] - bmin[0]) / ts);
  ty := Trunc((pos[2] - bmin[2]) / ts);

  m_tileBmin[0] := bmin[0] + tx*ts;
  m_tileBmin[1] := bmin[1];
  m_tileBmin[2] := bmin[2] + ty*ts;

  m_tileBmax[0] := bmin[0] + (tx+1)*ts;
  m_tileBmax[1] := bmax[1];
  m_tileBmax[2] := bmin[2] + (ty+1)*ts;

  m_tileCol := duRGBA(128,32,16,64);

  m_navMesh.removeTile(m_navMesh.getTileRefAt(tx,ty,0),nil,nil);
end;

procedure TSample_TileMesh.buildAllTiles();
var bmin, bmax: PSingle; gw, gh, ts, tw, th: Integer; tcs: Single; x, y: Integer; dataSize: Integer; data: PByte; status: TdtStatus;
begin
  if (m_geom = nil) then Exit;
  if (m_navMesh = nil) then Exit;

  bmin := m_geom.getMeshBoundsMin;
  bmax := m_geom.getMeshBoundsMax;
  gw := 0; gh := 0;
  rcCalcGridSize(bmin, bmax, m_cellSize, @gw, @gh);
  ts := Trunc(m_tileSize);
  tw := (gw + ts-1) div ts;
  th := (gh + ts-1) div ts;
  tcs := m_tileSize*m_cellSize;

  // Start the build process.
  m_ctx.startTimer(RC_TIMER_TEMP);

  for y := 0 to th - 1 do
  begin
    for x := 0 to tw - 1 do
    begin
      m_tileBmin[0] := bmin[0] + x*tcs;
      m_tileBmin[1] := bmin[1];
      m_tileBmin[2] := bmin[2] + y*tcs;

      m_tileBmax[0] := bmin[0] + (x+1)*tcs;
      m_tileBmax[1] := bmax[1];
      m_tileBmax[2] := bmin[2] + (y+1)*tcs;

      dataSize := 0;
      data := buildTileMesh(x, y, @m_tileBmin[0], @m_tileBmax[0], @dataSize);
      if (data <> nil) then
      begin
        // Remove any previous data (navmesh owns and deletes the data).
        m_navMesh.removeTile(m_navMesh.getTileRefAt(x,y,0),nil,nil);
        // Let the navmesh own the data.
        status := m_navMesh.addTile(data,dataSize,DT_TILE_FREE_DATA,0,nil);
        if (dtStatusFailed(status)) then
          FreeMem(data);
      end;
    end;
  end;

  // Start the build process.
  m_ctx.stopTimer(RC_TIMER_TEMP);

  m_totalBuildTimeMs := m_ctx.getAccumulatedTime(RC_TIMER_TEMP)/1000.0;
end;

procedure TSample_TileMesh.removeAllTiles();
var bmin, bmax: PSingle; gw, gh, ts, tw, th, x, y: Integer;
begin
  bmin := m_geom.getMeshBoundsMin();
  bmax := m_geom.getMeshBoundsMax();
  gw := 0; gh := 0;
  rcCalcGridSize(bmin, bmax, m_cellSize, @gw, @gh);
  ts := Trunc(m_tileSize);
  tw := (gw + ts-1) div ts;
  th := (gh + ts-1) div ts;

  for y := 0 to th - 1 do
    for x := 0 to tw - 1 do
      m_navMesh.removeTile(m_navMesh.getTileRefAt(x,y,0),nil,nil);
end;


function TSample_TileMesh.buildTileMesh(tx, ty: Integer; bmin, bmax: PSingle; dataSize: PInteger): PByte;
var verts: PSingle; nverts, ntris: Integer; chunkyMesh: PrcChunkyTriMesh; tbmin, tbmax: array [0..1] of Single;
cid: array [0..511] of Integer; ncid: Integer; i: Integer; node: PrcChunkyTriMeshNode; ctris: PInteger; nctris: Integer;
vols: TIGConvexVolumeArray; navData: PByte; navDataSize: Integer; params: TdtNavMeshCreateParams;
begin
  if (m_geom = nil) or (m_geom.getMesh = nil) {or (m_geom.getChunkyMesh = nil)} then
  begin
    m_ctx.log(RC_LOG_ERROR, 'buildNavigation: Input mesh is not specified.');
    Exit(nil);
  end;

  m_tileMemUsage := 0;
  m_tileBuildTime := 0;

  cleanup();

  verts := m_geom.getMesh.getVerts;
  nverts := m_geom.getMesh.getVertCount;
  ntris := m_geom.getMesh.getTriCount;
  chunkyMesh := @m_geom.getChunkyMesh;

  // Init build configuration from GUI
  FillChar(m_cfg, sizeof(m_cfg), #0);
  m_cfg.cs := m_cellSize;
  m_cfg.ch := m_cellHeight;
  m_cfg.walkableSlopeAngle := m_agentMaxSlope;
  m_cfg.walkableHeight := Ceil(m_agentHeight / m_cfg.ch);
  m_cfg.walkableClimb := Floor(m_agentMaxClimb / m_cfg.ch);
  m_cfg.walkableRadius := Ceil(m_agentRadius / m_cfg.cs);
  m_cfg.maxEdgeLen := Trunc(m_edgeMaxLen / m_cellSize);
  m_cfg.maxSimplificationError := m_edgeMaxError;
  m_cfg.minRegionArea := Trunc(Sqr(m_regionMinSize));    // Note: area := size*size
  m_cfg.mergeRegionArea := Trunc(Sqr(m_regionMergeSize));  // Note: area := size*size
  m_cfg.maxVertsPerPoly := Trunc(m_vertsPerPoly);
  m_cfg.tileSize := Trunc(m_tileSize);
  m_cfg.borderSize := m_cfg.walkableRadius + 3; // Reserve enough padding.
  m_cfg.width := m_cfg.tileSize + m_cfg.borderSize*2;
  m_cfg.height := m_cfg.tileSize + m_cfg.borderSize*2;
  m_cfg.detailSampleDist := IfThen(m_detailSampleDist < 0.9, 0, m_cellSize * m_detailSampleDist);
  m_cfg.detailSampleMaxError := m_cellHeight * m_detailSampleMaxError;
  
  // Expand the heighfield bounding box by border size to find the extents of geometry we need to build this tile.
  //
  // This is done in order to make sure that the navmesh tiles connect correctly at the borders,
  // and the obstacles close to the border work correctly with the dilation process.
  // No polygons (or contours) will be created on the border area.
  //
  // IMPORTANT!
  //
  //   :''''''''':
  //   : +-----+ :
  //   : |     | :
  //   : |     |<--- tile to build
  //   : |     | :
  //   : +-----+ :<-- geometry needed
  //   :.........:
  //
  // You should use this bounding box to query your input geometry.
  //
  // For example if you build a navmesh for terrain, and want the navmesh tiles to match the terrain tile size
  // you will need to pass in data from neighbour terrain tiles too! In a simple case, just pass in all the 8 neighbours,
  // or use the bounding box below to only pass in a sliver of each of the 8 neighbours.
  rcVcopy(@m_cfg.bmin[0], bmin);
  rcVcopy(@m_cfg.bmax[0], bmax);
  m_cfg.bmin[0] := m_cfg.bmin[0] - m_cfg.borderSize * m_cfg.cs;
  m_cfg.bmin[2] := m_cfg.bmin[2] - m_cfg.borderSize * m_cfg.cs;
  m_cfg.bmax[0] := m_cfg.bmax[0] + m_cfg.borderSize * m_cfg.cs;
  m_cfg.bmax[2] := m_cfg.bmax[2] + m_cfg.borderSize * m_cfg.cs;

  // Reset build times gathering.
  m_ctx.resetTimers();

  // Start the build process.
  m_ctx.startTimer(RC_TIMER_TOTAL);

  m_ctx.log(RC_LOG_PROGRESS, 'Building navigation:');
  m_ctx.log(RC_LOG_PROGRESS, Format(' - %d x %d cells', [m_cfg.width, m_cfg.height]));
  m_ctx.log(RC_LOG_PROGRESS, Format(' - %.1fK verts, %.1fK tris', [nverts/1000.0, ntris/1000.0]));

  // Delphi. We need to reset the m_solid
  SetLength(m_solid.spans, 0);

  // Allocate voxel heightfield where we rasterize our input data to.
  if (not rcCreateHeightfield(m_ctx, m_solid, m_cfg.width, m_cfg.height, @m_cfg.bmin[0], @m_cfg.bmax[0], m_cfg.cs, m_cfg.ch)) then
  begin
    m_ctx.log(RC_LOG_ERROR, 'buildNavigation: Could not create solid heightfield.');
    Exit(nil);
  end;

  // Allocate array that can hold triangle flags.
  // If you have multiple meshes you need to process, allocate
  // and array which can hold the max number of triangles you need to process.
  GetMem(m_triareas, sizeof(Byte)*chunkyMesh.maxTrisPerChunk);

  tbmin[0] := m_cfg.bmin[0];
  tbmin[1] := m_cfg.bmin[2];
  tbmax[0] := m_cfg.bmax[0];
  tbmax[1] := m_cfg.bmax[2];
  //int cid[512];// TODO: Make grow when returning too many items.
  ncid := rcGetChunksOverlappingRect(chunkyMesh, @tbmin[0], @tbmax[0], @cid[0], 512);
  if (ncid = 0) then
    Exit(nil);

  m_tileTriCount := 0;

  for i := 0 to ncid - 1 do
  begin
    node := @chunkyMesh.nodes[cid[i]];
    ctris := @chunkyMesh.tris[node.i*3];
    nctris := node.n;

    Inc(m_tileTriCount, nctris);

    FillChar(m_triareas[0], nctris*sizeof(Byte), 0);
    rcMarkWalkableTriangles(m_ctx, m_cfg.walkableSlopeAngle,
                verts, nverts, ctris, nctris, m_triareas);

    rcRasterizeTriangles(m_ctx, verts, nverts, ctris, m_triareas, nctris, m_solid, m_cfg.walkableClimb);
  end;

  if (not m_keepInterResults) then
  begin
    FreeMem(m_triareas);
    m_triareas := nil;
  end;

  // Once all geometry is rasterized, we do initial pass of filtering to
  // remove unwanted overhangs caused by the conservative rasterization
  // as well as filter spans where the character cannot possibly stand.
  rcFilterLowHangingWalkableObstacles(m_ctx, m_cfg.walkableClimb, m_solid);
  rcFilterLedgeSpans(m_ctx, m_cfg.walkableHeight, m_cfg.walkableClimb, m_solid);
  rcFilterWalkableLowHeightSpans(m_ctx, m_cfg.walkableHeight, m_solid);

  SetLength(m_chf.cells, 0);
  SetLength(m_chf.spans, 0);

  // Compact the heightfield so that it is faster to handle from now on.
  // This will result more cache coherent data as well as the neighbours
  // between walkable cells will be calculated.
  if (not rcBuildCompactHeightfield(m_ctx, m_cfg.walkableHeight, m_cfg.walkableClimb, @m_solid, @m_chf)) then
  begin
    m_ctx.log(RC_LOG_ERROR, 'buildNavigation: Could not build compact data.');
    Exit(nil);
  end;

  if (not m_keepInterResults) then
  begin
    //rcFreeHeightField(m_solid);
    //m_solid := 0;
  end;

  // Erode the walkable area by agent radius.
  if (not rcErodeWalkableArea(m_ctx, m_cfg.walkableRadius, @m_chf)) then
  begin
    m_ctx.log(RC_LOG_ERROR, 'buildNavigation: Could not erode.');
    Exit(nil);
  end;

  // (Optional) Mark areas.
  vols := m_geom.getConvexVolumes;
  for i := 0 to m_geom.getConvexVolumeCount - 1 do
    rcMarkConvexPolyArea(m_ctx, @vols[i].verts[0], vols[i].nverts, vols[i].hmin, vols[i].hmax, Byte(vols[i].area), @m_chf);


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
      Exit(nil);
    end;

    // Partition the walkable surface into simple regions without holes.
    if (not rcBuildRegions(m_ctx, @m_chf, m_cfg.borderSize, m_cfg.minRegionArea, m_cfg.mergeRegionArea)) then
    begin
      m_ctx.log(RC_LOG_ERROR, 'buildNavigation: Could not build watershed regions.');
      Exit(nil);
    end;
  end
  {else if (m_partitionType = SAMPLE_PARTITION_MONOTONE) then
  begin
    // Partition the walkable surface into simple regions without holes.
    // Monotone partitioning does not need distancefield.
    if (not rcBuildRegionsMonotone(m_ctx, @m_chf, m_cfg.borderSize, m_cfg.minRegionArea, m_cfg.mergeRegionArea)) then
    begin
      m_ctx.log(RC_LOG_ERROR, 'buildNavigation: Could not build monotone regions.');
      Exit(nil);
    end;
  end
  else // SAMPLE_PARTITION_LAYERS
  begin
    // Partition the walkable surface into simple regions without holes.
    if (not rcBuildLayerRegions(m_ctx, @m_chf, m_cfg.borderSize, m_cfg.minRegionArea)) then
    begin
      m_ctx.log(RC_LOG_ERROR, 'buildNavigation: Could not build layer regions.');
      Exit(nil);
    end;
  end};

  // Create contours.
  if (not rcBuildContours(m_ctx, @m_chf, m_cfg.maxSimplificationError, m_cfg.maxEdgeLen, @m_cset)) then
  begin
    m_ctx.log(RC_LOG_ERROR, 'buildNavigation: Could not create contours.');
    Exit(nil);
  end;

  if (m_cset.nconts = 0) then
  begin
    Exit(nil);
  end;

  // Build polygon navmesh from the contours.
  if (not rcBuildPolyMesh(m_ctx, @m_cset, m_cfg.maxVertsPerPoly, @m_pmesh)) then
  begin
    m_ctx.log(RC_LOG_ERROR, 'buildNavigation: Could not triangulate contours.');
    Exit(nil);
  end;

  // Build detail mesh.
  if (not rcBuildPolyMeshDetail(m_ctx, @m_pmesh, @m_chf,
                 m_cfg.detailSampleDist, m_cfg.detailSampleMaxError,
                 @m_dmesh)) then
  begin
    m_ctx.log(RC_LOG_ERROR, 'buildNavigation: Could build polymesh detail.');
    Exit(nil);
  end;

  if (not m_keepInterResults) then
  begin
    //rcFreeCompactHeightfield(m_chf);
    //m_chf := 0;
    //rcFreeContourSet(m_cset);
    //m_cset := 0;
  end;

  navData := nil;
  navDataSize := 0;
  if (m_cfg.maxVertsPerPoly <= DT_VERTS_PER_POLYGON) then
  begin
    if (m_pmesh.nverts >= $ffff) then
    begin
      // The vertex indices are ushorts, and cannot point to more than 0xffff vertices.
      m_ctx.log(RC_LOG_ERROR, Format('Too many vertices per tile %d (max: %d).', [m_pmesh.nverts, $ffff]));
      Exit(nil);
    end;

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
    params.offMeshConVerts := m_geom.getOffMeshConnectionVerts();
    params.offMeshConRad := m_geom.getOffMeshConnectionRads();
    params.offMeshConDir := m_geom.getOffMeshConnectionDirs();
    params.offMeshConAreas := m_geom.getOffMeshConnectionAreas();
    params.offMeshConFlags := m_geom.getOffMeshConnectionFlags();
    params.offMeshConUserID := m_geom.getOffMeshConnectionId();
    params.offMeshConCount := m_geom.getOffMeshConnectionCount;
    params.walkableHeight := m_agentHeight;
    params.walkableRadius := m_agentRadius;
    params.walkableClimb := m_agentMaxClimb;
    params.tileX := tx;
    params.tileY := ty;
    params.tileLayer := 0;
    rcVcopy(@params.bmin[0], @m_pmesh.bmin[0]);
    rcVcopy(@params.bmax[0], @m_pmesh.bmax[0]);
    params.cs := m_cfg.cs;
    params.ch := m_cfg.ch;
    params.buildBvTree := true;

    if (not dtCreateNavMeshData(@params, @navData, @navDataSize)) then
    begin
      m_ctx.log(RC_LOG_ERROR, 'Could not build Detour navmesh.');
      Exit(nil);
    end;
  end;
  m_tileMemUsage := navDataSize/1024.0;

  m_ctx.stopTimer(RC_TIMER_TOTAL);

  // Show performance stats.
  duLogBuildTimes(m_ctx, m_ctx.getAccumulatedTime(RC_TIMER_TOTAL));
  m_ctx.log(RC_LOG_PROGRESS, Format('>> Polymesh: %d vertices  %d polygons', [m_pmesh.nverts, m_pmesh.npolys]));

  m_tileBuildTime := m_ctx.getAccumulatedTime(RC_TIMER_TOTAL)/1000.0;

  dataSize^ := navDataSize;
  Result := navData;
end;

end.
