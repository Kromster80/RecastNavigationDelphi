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

unit RN_Sample;
interface
uses Math, SysUtils,
  RN_DetourCrowd, RN_DetourNavMesh, RN_DetourNavMeshQuery, RN_Helper, RN_Recast, RN_InputGeom, RN_SampleInterfaces;

/// Tool types.
type
TSampleToolType =
(
  TOOL_NONE,
  TOOL_TILE_EDIT,
  TOOL_TILE_HIGHLIGHT,
  TOOL_TEMP_OBSTACLE,
  TOOL_NAVMESH_TESTER,
  TOOL_NAVMESH_PRUNE,
  TOOL_OFFMESH_CONNECTION,
  TOOL_CONVEX_VOLUME,
  TOOL_CROWD,
  MAX_TOOLS
);

/// These are just sample areas to use consistent values across the samples.
/// The use should specify these base on his needs.
TSamplePolyAreas =
(
  SAMPLE_POLYAREA_GROUND,
  SAMPLE_POLYAREA_WATER,
  SAMPLE_POLYAREA_ROAD,
  SAMPLE_POLYAREA_DOOR,
  SAMPLE_POLYAREA_GRASS,
  SAMPLE_POLYAREA_JUMP
);
const
  SAMPLE_POLYFLAGS_WALK    = $01;    // Ability to walk (ground, grass, road)
  SAMPLE_POLYFLAGS_SWIM    = $02;    // Ability to swim (water).
  SAMPLE_POLYFLAGS_DOOR    = $04;    // Ability to move through doors.
  SAMPLE_POLYFLAGS_JUMP    = $08;    // Ability to jump.
  SAMPLE_POLYFLAGS_DISABLED  = $10;    // Disabled polygon
  SAMPLE_POLYFLAGS_ALL    = $ffff;  // All abilities.

type
  TSamplePartitionType =
  (
    SAMPLE_PARTITION_WATERSHED,
    SAMPLE_PARTITION_MONOTONE,
    SAMPLE_PARTITION_LAYERS
  );

  TSample = class;

  TSampleTool = class // Delphi: Could be a record, but they are not allowed to have abstract methods
    &type: TSampleToolType;
    procedure init(sample: TSample); virtual; abstract;
    procedure reset(); virtual; abstract;
    procedure handleMenu(Sender: TObject); virtual; abstract;
    procedure handleClick(s,p: PSingle; shift: Boolean); virtual; abstract;
    procedure handleRender(); virtual; abstract;
    procedure handleRenderOverlay(proj, model: PDouble; view: PInteger); virtual; abstract;
    procedure handleToggle(); virtual; abstract;
    procedure handleStep(); virtual; abstract;
    procedure handleUpdate(dt: Single); virtual; abstract;
  end;

  TSampleToolState = class // Delphi: Could be a record, but they are not allowed to have abstract methods
    procedure init(sample: TSample); virtual; abstract;
    procedure reset; virtual; abstract;
    procedure handleRender; virtual; abstract;
    procedure handleRenderOverlay(proj, model: PDouble; view: PInteger); virtual; abstract;
    procedure handleUpdate(dt: Single); virtual; abstract;
  end;

  TSample = class
  protected
    m_geom: TInputGeom;
    m_navMesh: TdtNavMesh;
    m_navQuery: TdtNavMeshQuery;
    m_crowd: TdtCrowd;

    m_navMeshDrawFlags: Byte;

    m_tool: TSampleTool;
    m_toolStates: array [TSampleToolType] of TSampleToolState;

    m_ctx: TBuildContext;

  public
    // Delphi. Making it public does no harm, but makes GUI easier to rig
    m_cellSize: Single;
    m_cellHeight: Single;
    m_agentHeight: Single;
    m_agentRadius: Single;
    m_agentMaxClimb: Single;
    m_agentMaxSlope: Single;
    m_regionMinSize: Single;
    m_regionMergeSize: Single;
    m_edgeMaxLen: Single;
    m_edgeMaxError: Single;
    m_vertsPerPoly: Single;
    m_detailSampleDist: Single;
    m_detailSampleMaxError: Single;
    m_partitionType: TSamplePartitionType; // Delphi. Use specific type instead of Int

    constructor Create;
    destructor Destroy; override;

    property setContext: TBuildContext write m_ctx;

    procedure setTool(tool: TSampleTool);
    function getToolState(&type: TSampleToolType): TSampleToolState; { return m_toolStates[type]; }
    procedure setToolState(&type: TSampleToolType; s: TSampleToolState); { m_toolStates[type] = s; }

    procedure handleSettings(); virtual;
    procedure handleTools(); virtual;
    procedure handleDebugMode(); virtual;
    procedure handleClick(const s, p: PSingle; shift: Boolean); virtual;
    procedure handleMenu(Sender: TObject); virtual;
    procedure handleToggle(); virtual;
    procedure handleStep(); virtual;
    procedure handleRender(); virtual;
    procedure handleRenderOverlay(proj, model: PDouble; view: PInteger); virtual;
    procedure handleMeshChanged(geom: TInputGeom); virtual;
    function handleBuild(): Boolean; virtual;
    procedure handleUpdate(const dt: Single); virtual;

    property getInputGeom: TInputGeom read m_geom;
    property getNavMesh: TdtNavMesh read m_navMesh;
    property getNavMeshQuery: TdtNavMeshQuery read m_navQuery;
    property getCrowd: TdtCrowd read m_crowd;
    property getAgentRadius: Single read m_agentRadius;
    property getAgentHeight: Single read m_agentHeight;
    property getAgentClimb: Single read m_agentMaxClimb;
    function getBoundsMin: PSingle;
    function getBoundsMax: PSingle;

    property getNavMeshDrawFlags: Byte read m_navMeshDrawFlags write m_navMeshDrawFlags;

    procedure updateToolStates(const dt: Single);
    procedure initToolStates(sample: TSample);
    procedure resetToolStates();
    procedure renderToolStates();
    procedure renderOverlayToolStates(proj, model: PDouble; view: PInteger);

    procedure resetCommonSettings();
    procedure handleCommonSettings();
  end;

implementation
uses RN_DebugDraw, RN_DetourDebugDraw, RN_RecastDebugDraw;


constructor TSample.Create;
begin
  inherited;

  m_navMeshDrawFlags := DU_DRAWNAVMESH_OFFMESHCONS or DU_DRAWNAVMESH_CLOSEDLIST;

  resetCommonSettings();
  m_navQuery := dtAllocNavMeshQuery();
  m_crowd := dtAllocCrowd;
end;

destructor TSample.Destroy;
begin
  dtFreeNavMeshQuery(m_navQuery);
  FreeAndNil(m_navMesh);
  dtFreeCrowd(m_crowd);

  inherited;
end;

procedure TSample.setTool(tool: TSampleTool);
begin
  FreeAndNil(m_tool);
  m_tool := tool;
  if (tool <> nil) then
    m_tool.init(Self);
end;

function TSample.getToolState(&type: TSampleToolType): TSampleToolState;
begin
  Result := m_toolStates[&type];
end;

procedure TSample.setToolState(&type: TSampleToolType; s: TSampleToolState);
begin
  m_toolStates[&type] := s;
end;

procedure TSample.handleSettings();
begin
end;

procedure TSample.handleTools();
begin
end;

procedure TSample.handleDebugMode();
begin
end;

procedure TSample.handleRender();
var dd: TDebugDrawGL; bmin,bmax: PSingle;
begin
  if (m_geom = nil) then
    Exit;

  dd := TDebugDrawGL.Create;

  // Draw mesh
  duDebugDrawTriMesh(dd, m_geom.getMesh.getVerts, m_geom.getMesh.getVertCount,
             m_geom.getMesh.getTris, m_geom.getMesh.getNormals, m_geom.getMesh.getTriCount, nil, 1.0);
  // Draw bounds
  bmin := m_geom.getMeshBoundsMin();
  bmax := m_geom.getMeshBoundsMax();
  duDebugDrawBoxWire(dd, bmin[0],bmin[1],bmin[2], bmax[0],bmax[1],bmax[2], duRGBA(255,255,255,128), 1.0);

  dd.Free;
end;

procedure TSample.handleRenderOverlay(proj, model: PDouble; view: PInteger);
begin
end;

procedure TSample.handleMenu(Sender: TObject);
begin
  if m_tool <> nil then
    m_tool.handleMenu(Sender);
end;

procedure TSample.handleMeshChanged(geom: TInputGeom);
begin
  m_geom := geom;
end;

function TSample.getBoundsMin(): PSingle;
begin
  if (m_geom = nil) then Exit(nil);
  Result := m_geom.getMeshBoundsMin;
end;

function TSample.getBoundsMax(): PSingle;
begin
  if (m_geom = nil) then Exit(nil);
  Result := m_geom.getMeshBoundsMax;
end;

procedure TSample.resetCommonSettings();
begin
  m_cellSize := 0.3;
  m_cellHeight := 0.2;
  m_agentHeight := 2.0;
  m_agentRadius := 0.6;
  m_agentMaxClimb := 0.9;
  m_agentMaxSlope := 45.0;
  m_regionMinSize := 8;
  m_regionMergeSize := 20;
  m_edgeMaxLen := 12.0;
  m_edgeMaxError := 1.3;
  m_vertsPerPoly := 6.0;
  m_detailSampleDist := 6.0;
  m_detailSampleMaxError := 1.0;
  m_partitionType := SAMPLE_PARTITION_WATERSHED;
end;

procedure TSample.handleCommonSettings();
begin
 // Delphi. Refactored into DFM
end;

procedure TSample.handleClick(const s, p: PSingle; shift: Boolean);
begin
  if (m_tool <> nil) then
    m_tool.handleClick(s, p, shift);
end;

procedure TSample.handleToggle();
begin
  if (m_tool <> nil) then
    m_tool.handleToggle();
end;

procedure TSample.handleStep();
begin
  if (m_tool <> nil) then
    m_tool.handleStep();
end;

function TSample.handleBuild(): Boolean;
begin
  Result := True;
end;

procedure TSample.handleUpdate(const dt: Single);
begin
  if (m_tool <> nil) then
    m_tool.handleUpdate(dt);
  updateToolStates(dt);
end;


procedure TSample.updateToolStates(const dt: Single);
var i: TSampleToolType;
begin
  for i := Low(TSampleToolType) to High(TSampleToolType) do
  begin
    if (m_toolStates[i] <> nil) then
      m_toolStates[i].handleUpdate(dt);
  end;
end;

procedure TSample.initToolStates(sample: TSample);
var i: TSampleToolType;
begin
  for i := Low(TSampleToolType) to High(TSampleToolType) do
  begin
    if (m_toolStates[i] <> nil) then
      m_toolStates[i].init(sample);
  end;
end;

procedure TSample.resetToolStates();
var i: TSampleToolType;
begin
  for i := Low(TSampleToolType) to High(TSampleToolType) do
  begin
    if (m_toolStates[i] <> nil) then
      m_toolStates[i].reset();
  end;
end;

procedure TSample.renderToolStates();
var i: TSampleToolType;
begin
  for i := Low(TSampleToolType) to High(TSampleToolType) do
  begin
    if (m_toolStates[i] <> nil) then
      m_toolStates[i].handleRender();
  end;
end;

procedure TSample.renderOverlayToolStates(proj, model: PDouble; view: PInteger);
var i: TSampleToolType;
begin
  for i := Low(TSampleToolType) to High(TSampleToolType) do
  begin
    if (m_toolStates[i] <> nil) then
      m_toolStates[i].handleRenderOverlay(proj, model, view);
  end;
end;

end.
