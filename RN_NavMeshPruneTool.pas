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

unit RN_NavMeshPruneTool;
interface
uses Classes, Controls, StdCtrls, RN_Sample, RN_DetourNavMesh, RN_DetourNavMeshHelper, Unit_FramePruneTool;

// Prune navmesh to accessible locations from a point.
type
  PTileFlags = ^TTileFlags;
  TTileFlags = record
    flags: PByte;
    nflags: Integer;
    base: TdtPolyRef;
    procedure purge;
  end;

  TNavmeshFlags = class
  private
    m_nav: TdtNavMesh;
    m_tiles: PTileFlags;
    m_ntiles: Integer;
  public
    destructor Destroy; override;
    function init(nav: TdtNavMesh): Boolean;
    procedure clearAllFlags;
    function getFlags(ref: TdtPolyRef): Byte;
    procedure setFlags(ref: TdtPolyRef; flags: Byte);
  end;

  TNavMeshPruneTool = class (TSampleTool)
  private
    fFrame: TFramePruneTool;
    m_sample: TSample;

    m_flags: TNavmeshFlags;

    m_hitPos: array [0..2] of Single;
    m_hitPosSet: Boolean;
  public
    constructor Create(aOwner: TWinControl);
    destructor Destroy; override;

    procedure init(sample: TSample); override;
    procedure reset(); override;
    procedure handleMenu(Sender: TObject); override;
    procedure handleClick(s,p: PSingle; shift: Boolean); override;
    procedure handleToggle(); override;
    procedure handleStep(); override;
    procedure handleUpdate(dt: Single); override;
    procedure handleRender(); override;
    procedure handleRenderOverlay(proj, model: PDouble; view: PInteger); override;
  end;

implementation
uses Math,
  RN_InputGeom, RN_SampleInterfaces, RN_DebugDraw,
  RN_Recast, RN_RecastHelper, RN_RecastDebugDraw,
  RN_DetourNavMeshBuilder, RN_DetourNavMeshQuery, RN_DetourStatus, RN_DetourDebugDraw, RN_DetourCommon;

// Copy/paste from Recast int array
type
  TPolyRefArray = class
  private
    m_size, m_cap: Integer;
    //inline PolyRefArray(const PolyRefArray&);
    //inline PolyRefArray& operator=(const PolyRefArray&);
  public
    m_data: PdtPolyRef;
    constructor Create; overload;
    constructor Create(n: Integer); overload;
    destructor Destroy; override;
    procedure resize(n: Integer);
    procedure push(item: TdtPolyRef);
    function pop: TdtPolyRef;
    function size: Integer;
  end;

constructor TPolyRefArray.Create;
begin
end;

constructor TPolyRefArray.Create(n: Integer);
begin
  resize(n);
end;

destructor TPolyRefArray.Destroy;
begin
  FreeMem(m_data);
  inherited;
end;

procedure TPolyRefArray.resize(n: Integer);
var newData: PdtPolyRef;
begin
  if (n > m_cap) then
  begin
    if (m_cap = 0) then m_cap := n;
    while (m_cap < n) do m_cap := m_cap * 2;
    GetMem(newData, m_cap*sizeof(TdtPolyRef));
    if (m_size <> 0) and (newData <> nil) then Move(m_data^, newData^, m_size*sizeof(TdtPolyRef));
    FreeMem(m_data);
    m_data := newData;
  end;
  m_size := n;
end;

procedure TPolyRefArray.push(item: TdtPolyRef);
begin
  resize(m_size+1); m_data[m_size-1] := item;
end;

function TPolyRefArray.Pop: TdtPolyRef;
begin
  if (m_size > 0) then Dec(m_size); Result := m_data[m_size];
end;

function TPolyRefArray.size: Integer;
begin
  Result := m_size;
end;


procedure TTileFlags.purge; begin FreeMem(flags); end;

destructor TNavmeshFlags.Destroy;
var i: Integer;
begin
  for i := 0 to m_ntiles - 1 do
    m_tiles[i].purge;
  FreeMem(m_tiles);

  inherited;
end;

function TNavmeshFlags.init(nav: TdtNavMesh): Boolean;
var i: Integer; tile: PdtMeshTile; tf: PTileFlags;
begin
  m_ntiles := nav.getMaxTiles;
  if (m_ntiles = 0) then
    Exit(true);
  GetMem(m_tiles, sizeof(TTileFlags)*m_ntiles);
  if (m_tiles = nil) then
  begin
    Exit(false);
  end;
  FillChar(m_tiles[0], sizeof(TTileFlags)*m_ntiles, 0);

  // Alloc flags for each tile.
  for i := 0 to nav.getMaxTiles - 1 do
  begin
    tile := nav.getTile(i);
    if (tile.header = nil) then continue;
    tf := @m_tiles[i];
    tf.nflags := tile.header.polyCount;
    tf.base := nav.getPolyRefBase(tile);
    if (tf.nflags <> 0) then
    begin
      GetMem(tf.flags, tf.nflags);
      if (tf.flags = nil) then
        Exit(false);
      FillChar(tf.flags[0], tf.nflags, 0);
    end;
  end;

  m_nav := nav;

  Result := false;
end;

procedure TNavmeshFlags.clearAllFlags;
var i: Integer; tf: PTileFlags;
begin
  for i := 0 to m_ntiles - 1 do
  begin
    tf := @m_tiles[i];
    if (tf.nflags <> 0) then
      FillChar(tf.flags[0], tf.nflags, 0);
  end;
end;

function TNavmeshFlags.getFlags(ref: TdtPolyRef): Byte;
var salt, it, ip: Cardinal;
begin
  Assert(m_nav <> nil);
  Assert(m_ntiles <> 0);
  // Assume the ref is valid, no bounds checks.
  m_nav.decodePolyId(ref, @salt, @it, @ip);
  Result := m_tiles[it].flags[ip];
end;

procedure TNavmeshFlags.setFlags(ref: TdtPolyRef; flags: Byte);
var salt, it, ip: Cardinal;
begin
  Assert(m_nav <> nil);
  Assert(m_ntiles <> 0);
  // Assume the ref is valid, no bounds checks.
  m_nav.decodePolyId(ref, @salt, @it, @ip);
  m_tiles[it].flags[ip] := flags;
end;

procedure floodNavmesh(nav: TdtNavMesh; flags: TNavmeshFlags; start: TdtPolyRef; flag: Byte);
var openList: TPolyRefArray; ref,neiRef: TdtPolyRef; tile: PdtMeshTile; poly: PdtPoly; i: Cardinal;
begin
  // If already visited, skip.
  if (flags.getFlags(start) <> 0) then
    Exit;

  openList := TPolyRefArray.Create;
  openList.push(start);

  while (openList.size <> 0) do
  begin
    ref := openList.pop;
    // Get current poly and tile.
    // The API input has been cheked already, skip checking internal data.
    tile := nil;
    poly := nil;
    nav.getTileAndPolyByRefUnsafe(ref, @tile, @poly);

    // Visit linked polygons.
    i := poly.firstLink;
    while (i <> DT_NULL_LINK) do
    begin
      neiRef := tile.links[i].ref;
      // Skip invalid and already visited.
      if (neiRef = 0) or (flags.getFlags(neiRef) <> 0) then
      begin i := tile.links[i].next; continue; end;
      // Mark as visited
      flags.setFlags(neiRef, flag);
      // Visit neighbours
      openList.push(neiRef);

      i := tile.links[i].next;
    end;
  end;
end;

procedure disableUnvisitedPolys(nav: TdtNavMesh; flags: TNavmeshFlags);
var i,j: Integer; tile: PdtMeshTile; base,ref: TdtPolyRef; f: Word;
begin
  for i := 0 to nav.getMaxTiles - 1 do
  begin
    tile := nav.getTile(i);
    if (tile.header = nil) then continue;
    base := nav.getPolyRefBase(tile);
    for j := 0 to tile.header.polyCount - 1 do
    begin
      ref := base or j;
      if (flags.getFlags(ref) = 0) then
      begin
        f := 0;
        nav.getPolyFlags(ref, @f);
        nav.setPolyFlags(ref, f or SAMPLE_POLYFLAGS_DISABLED);
      end;
    end;
  end;
end;

constructor TNavMeshPruneTool.Create(aOwner: TWinControl);
begin
  inherited Create;
  &type := TOOL_NAVMESH_PRUNE;

  fFrame := TFramePruneTool.Create(aOwner);
  fFrame.Align := alClient;
  fFrame.Parent := aOwner;
  fFrame.Visible := True;
  fFrame.btnClearSelection.OnClick := handleMenu;
  fFrame.btnPruneUnselected.OnClick := handleMenu;
end;

destructor TNavMeshPruneTool.Destroy;
begin
  m_flags.Free;
  fFrame.Free;
  inherited;
end;

procedure TNavMeshPruneTool.init(sample: TSample);
begin
  m_sample := sample;
end;

procedure TNavMeshPruneTool.reset();
begin
  m_hitPosSet := false;
  m_flags.Free;
  m_flags := nil;
end;

procedure TNavMeshPruneTool.handleMenu(Sender: TObject);
var nav: TdtNavMesh;
begin
  nav := m_sample.getNavMesh;
  if (nav = nil) then Exit;
  if (m_flags = nil) then Exit;

  if Sender = fFrame.btnClearSelection then
  begin
    m_flags.clearAllFlags();
  end;

  if Sender = fFrame.btnPruneUnselected then
  begin
    disableUnvisitedPolys(nav, m_flags);
    m_flags.Free;
    m_flags := nil;
  end;
end;

procedure TNavMeshPruneTool.handleClick(s,p: PSingle; shift: Boolean);
var geom: TInputGeom; nav: TdtNavMesh; query: TdtNavMeshQuery; ext: array [0..2] of Single; filter: TdtQueryFilter; ref: TdtPolyRef;
begin
  //rcIgnoreUnused(s);
  //rcIgnoreUnused(shift);

  if (m_sample = nil) then Exit;
  geom := m_sample.getInputGeom;
  if (geom = nil) then Exit;
  nav := m_sample.getNavMesh;
  if (nav = nil) then Exit;
  query := m_sample.getNavMeshQuery;
  if (query = nil) then Exit;

  dtVcopy(@m_hitPos[0], p);
  m_hitPosSet := true;

  if (m_flags = nil) then
  begin
    m_flags := TNavmeshFlags.Create;
    m_flags.init(nav);
  end;

  ext[0] := 2; ext[1] := 4; ext[2] := 2;
  // Delphi: Assume C++ invokes constructor
  filter := TdtQueryFilter.Create;
  ref := 0;
  query.findNearestPoly(p, @ext[0], filter, @ref, nil);

  floodNavmesh(nav, m_flags, ref, 1);

  // Delphi: Assume C++ does the same when var goes out of scope
  filter.Free;
end;

procedure TNavMeshPruneTool.handleToggle();
begin
end;

procedure TNavMeshPruneTool.handleStep();
begin
end;

procedure TNavMeshPruneTool.handleUpdate(dt: Single);
begin
end;

procedure TNavMeshPruneTool.handleRender();
var dd: TDebugDrawGL; s: Single; col: Cardinal; nav: TdtNavMesh; i,j: Integer; tile: PdtMeshTile; base,ref: TdtPolyRef;
begin
  dd := TDebugDrawGL.Create;

  if (m_hitPosSet) then
  begin
    s := m_sample.getAgentRadius;
    col := duRGBA(255,255,255,255);
    dd.&begin(DU_DRAW_LINES);
    dd.vertex(m_hitPos[0]-s,m_hitPos[1],m_hitPos[2], col);
    dd.vertex(m_hitPos[0]+s,m_hitPos[1],m_hitPos[2], col);
    dd.vertex(m_hitPos[0],m_hitPos[1]-s,m_hitPos[2], col);
    dd.vertex(m_hitPos[0],m_hitPos[1]+s,m_hitPos[2], col);
    dd.vertex(m_hitPos[0],m_hitPos[1],m_hitPos[2]-s, col);
    dd.vertex(m_hitPos[0],m_hitPos[1],m_hitPos[2]+s, col);
    dd.&end();
  end;

  nav := m_sample.getNavMesh;
  if (m_flags <> nil) and (nav <> nil) then
  begin
    for i := 0 to nav.getMaxTiles - 1 do
    begin
      tile := nav.getTile(i);
      if (tile.header = nil) then continue;
      base := nav.getPolyRefBase(tile);
      for j := 0 to tile.header.polyCount - 1 do
      begin
        ref := base or j;
        if (m_flags.getFlags(ref) <> 0) then
        begin
          duDebugDrawNavMeshPoly(dd, nav, ref, duRGBA(255,255,255,128));
        end;
      end;
    end;
  end;

  dd.Free;
end;

procedure TNavMeshPruneTool.handleRenderOverlay(proj, model: PDouble; view: PInteger);
begin
  //rcIgnoreUnused(model);
  //rcIgnoreUnused(proj);

  // Tool help
  //const int h := view[3];

  //imguiDrawText(280, h-40, IMGUI_ALIGN_LEFT, "LMB: Click fill area.", imguiRGBA(255,255,255,192));
end;

end.