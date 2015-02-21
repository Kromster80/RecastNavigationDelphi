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

unit RN_NavMeshTesterTool;
interface
uses
  Classes, Controls, StdCtrls, ExtCtrls, Unit_FrameTesterTool,
  RN_DetourNavMesh, RN_DetourNavMeshHelper, RN_DetourNavMeshQuery, RN_DetourStatus, RN_Sample;

type
  TToolMode =
  (
    TOOLMODE_PATHFIND_FOLLOW,
    TOOLMODE_PATHFIND_STRAIGHT,
    TOOLMODE_PATHFIND_SLICED,
    TOOLMODE_RAYCAST,
    TOOLMODE_DISTANCE_TO_WALL,
    TOOLMODE_FIND_POLYS_IN_CIRCLE,
    TOOLMODE_FIND_POLYS_IN_SHAPE,
    TOOLMODE_FIND_LOCAL_NEIGHBOURHOOD
  );

  const MAX_POLYS = 256;
  const MAX_SMOOTH = 2048;
  const MAX_RAND_POINTS = 64;
  const MAX_STEER_POINTS = 10;

type
  TNavMeshTesterTool = class (TSampleTool)
  private
    fFrame: TFrameTesterTool;
    fUpdateUI: Boolean;

    m_sample: TSample;

    m_navMesh: TdtNavMesh;
    m_navQuery: TdtNavMeshQuery;

    m_filter: TdtQueryFilter;

    m_pathFindStatus: TdtStatus;

    m_toolMode: TToolMode;

    m_straightPathOptions: TdtStraightPathOptions;

    m_startRef: TdtPolyRef;
    m_endRef: TdtPolyRef;
    m_polys: array [0..MAX_POLYS-1] of TdtPolyRef;
    m_parent: array [0..MAX_POLYS-1] of TdtPolyRef;
    m_npolys: Integer;
    m_straightPath: array [0..MAX_POLYS*3-1] of Single;
    m_straightPathFlags: array [0..MAX_POLYS-1] of Byte;
    m_straightPathPolys: array [0..MAX_POLYS-1] of TdtPolyRef;
    m_nstraightPath: Integer;
    m_polyPickExt: array [0..2] of Single;
    m_smoothPath: array [0..MAX_SMOOTH*3-1] of Single;
    m_nsmoothPath: Integer;
    m_queryPoly: array [0..4*3-1] of Single;

    m_randPoints: array [0..MAX_RAND_POINTS*3-1] of Single;
    m_nrandPoints: Integer;
    m_randPointsInCircle: Boolean;

    m_spos: array [0..2] of Single;
    m_epos: array [0..2] of Single;
    m_hitPos: array [0..2] of Single;
    m_hitNormal: array [0..2] of Single;
    m_hitResult: Boolean;
    m_distanceToWall: Single;
    m_neighbourhoodRadius: Single;
    m_randomRadius: Single;
    m_sposSet: Boolean;
    m_eposSet: Boolean;

    m_pathIterNum: Integer;
    m_pathIterPolys: array [0..MAX_POLYS-1] of TdtPolyRef;
    m_pathIterPolyCount: Integer;
    m_prevIterPos, m_iterPos, m_steerPos, m_targetPos: array [0..2] of Single;

    m_steerPoints: array [0..MAX_STEER_POINTS*3] of Single;
    m_steerPointCount: Integer;
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

    procedure recalc();
    procedure drawAgent(pos: PSingle; r, h, c: Single; col: Cardinal);
  end;

implementation
uses Math,
  RN_Recast, RN_RecastHelper, RN_DebugDraw, RN_RecastDebugDraw, RN_SampleInterfaces,
  RN_DetourNavMeshBuilder, RN_DetourDebugDraw, RN_DetourCommon;

// Uncomment this to dump all the requests in stdout.
//#define DUMP_REQS

// Returns a random number [0..1)
//static float frand()
{
//  return ((float)(rand() & 0xffff)/(float)0xffff);
  return (float)rand()/(float)RAND_MAX;
}

function inRange(v1,v2: PSingle; r,h: Single): Boolean;
var dx,dy,dz: Single;
begin
  dx := v2[0] - v1[0];
  dy := v2[1] - v1[1];
  dz := v2[2] - v1[2];
  Result := ((dx*dx + dz*dz) < r*r) and (abs(dy) < h);
end;


function fixupCorridor(path: PdtPolyRef; npath, maxPath: Integer; visited: PdtPolyRef; nvisited: Integer): Integer;
var furthestPath, furthestVisited, i, j: Integer; found: Boolean; req, orig, size: Integer;
begin
  furthestPath := -1;
  furthestVisited := -1;

  // Find furthest common polygon.
  for i := npath-1 downto 0 do
  begin
    found := false;
    for j := nvisited-1 downto 0 do
    begin
      if (path[i] = visited[j]) then
      begin
        furthestPath := i;
        furthestVisited := j;
        found := true;
      end;
    end;
    if (found) then
      break;
  end;

  // If no intersection found just return current path.
  if (furthestPath = -1) or (furthestVisited = -1) then
    Exit(npath);

  // Concatenate paths.

  // Adjust beginning of the buffer to include the visited.
  req := nvisited - furthestVisited;
  orig := rcMin(furthestPath+1, npath);
  size := rcMax(0, npath-orig);
  if (req+size > maxPath) then
    size := maxPath-req;
  if (size <> 0) then
    Move((path+orig)^, (path+req)^, size*sizeof(TdtPolyRef));

  // Store visited
  for i := 0 to req - 1 do
    path[i] := visited[(nvisited-1)-i];

  Result := req+size;
end;

// This function checks if the path has a small U-turn, that is,
// a polygon further in the path is adjacent to the first polygon
// in the path. If that happens, a shortcut is taken.
// This can happen if the target (T) location is at tile boundary,
// and we're (S) approaching it parallel to the tile edge.
// The choice at the vertex can be arbitrary,
//  +---+---+
//  |:::|:::|
//  +-S-+-T-+
//  |:::|   | <-- the step can end up in here, resulting U-turn path.
//  +---+---+
function fixupShortcuts(path: PdtPolyRef; npath: Integer; navQuery: TdtNavMeshQuery): Integer;
const maxNeis = 16; maxLookAhead = 6;
var neis: array [0..maxNeis-1] of TdtPolyRef; nneis: Integer; tile: PdtMeshTile; poly: PdtPoly; k: Cardinal; link: PdtLink;
cut,i,j,offset: Integer;
begin
  if (npath < 3) then
    Exit(npath);

  // Get connected polygons
  nneis := 0;

  tile := nil;
  poly := nil;
  if (dtStatusFailed(navQuery.getAttachedNavMesh.getTileAndPolyByRef(path[0], @tile, @poly))) then
    Exit(npath);

  k := poly.firstLink;
  while (k <> DT_NULL_LINK) do
  begin
    link := @tile.links[k];
    if (link.ref <> 0) then
    begin
      if (nneis < maxNeis) then
      begin
        neis[nneis] := link.ref;
        Inc(nneis);
      end;
    end;

    k := tile.links[k].next;
  end;

  // If any of the neighbour polygons is within the next few polygons
  // in the path, short cut to that polygon directly.
  cut := 0;
  i := dtMin(maxLookAhead, npath) - 1;
  while (i > 1) and (cut = 0) do
  begin
    for j := 0 to nneis - 1 do
    begin
      if (path[i] = neis[j]) then
      begin
        cut := i;
        break;
      end;
    end;
    Dec(i);
  end;
  if (cut > 1) then
  begin
    offset := cut-1;
    Dec(npath, offset);
    for i := 1 to npath - 1 do
      path[i] := path[i+offset];
  end;

  Result := npath;
end;

function getSteerTarget(navQuery: TdtNavMeshQuery; startPos, endPos: PSingle;
               minTargetDist: Single;
               path: PdtPolyRef; pathSize: Integer;
               steerPos: PSingle; steerPosFlag: PByte; steerPosRef: PdtPolyRef;
               outPoints: PSingle = nil; outPointCount: PInteger = nil): Boolean;
const MAX_STEER_POINTS = 3;
var steerPath: array [0..MAX_STEER_POINTS*3-1] of Single; steerPathFlags: array [0..MAX_STEER_POINTS-1] of Byte;
steerPathPolys: array [0..MAX_STEER_POINTS-1] of TdtPolyRef; nsteerPath,i,ns: Integer;
begin
  // Find steer target.
  nsteerPath := 0;
  navQuery.findStraightPath(startPos, endPos, path, pathSize,
                 @steerPath[0], @steerPathFlags[0], @steerPathPolys[0], @nsteerPath, MAX_STEER_POINTS);
  if (nsteerPath = 0) then
    Exit(false);

  if (outPoints <> nil) and (outPointCount <> nil) then
  begin
    outPointCount^ := nsteerPath;
    for i := 0 to nsteerPath - 1 do
      dtVcopy(@outPoints[i*3], @steerPath[i*3]);
  end;


  // Find vertex far enough to steer to.
  ns := 0;
  while (ns < nsteerPath) do
  begin
    // Stop at Off-Mesh link or when point is further than slop away.
    if ((steerPathFlags[ns] and Byte(DT_STRAIGHTPATH_OFFMESH_CONNECTION) <> 0) or
      not inRange(@steerPath[ns*3], startPos, minTargetDist, 1000.0)) then
      break;
    Inc(ns);
  end;
  // Failed to find good point to steer to.
  if (ns >= nsteerPath) then
    Exit(false);

  dtVcopy(steerPos, @steerPath[ns*3]);
  steerPos[1] := startPos[1];
  steerPosFlag^ := steerPathFlags[ns];
  steerPosRef^ := steerPathPolys[ns];

  Result := true;
end;


constructor TNavMeshTesterTool.Create(aOwner: TWinControl);
begin
  inherited Create;

  &type := TOOL_NAVMESH_TESTER;

  fFrame := TFrameTesterTool.Create(aOwner);
  fFrame.Align := alClient;
  fFrame.Parent := aOwner;
  fFrame.Visible := True;

  fFrame.rgToolMode.OnClick := handleMenu;
  fFrame.rgVerticesAtCrossings.OnClick := handleMenu;
  fFrame.btnSetRandomStart.OnClick := handleMenu;
  fFrame.btnSetRandomEnd.OnClick := handleMenu;
  fFrame.btnMakeRandomPoints.OnClick := handleMenu;
  fFrame.btnMakeRandomPointsAround.OnClick := handleMenu;
  fFrame.cbInclWalk.OnClick := handleMenu;
  fFrame.cbInclSwim.OnClick := handleMenu;
  fFrame.cbInclDoor.OnClick := handleMenu;
  fFrame.cbInclJump.OnClick := handleMenu;
  fFrame.cbExclWalk.OnClick := handleMenu;
  fFrame.cbExclSwim.OnClick := handleMenu;
  fFrame.cbExclDoor.OnClick := handleMenu;
  fFrame.cbExclJump.OnClick := handleMenu;

  // Delphi: Assume C++ invokes constructor
  m_filter := TdtQueryFilter.Create;

  m_pathFindStatus := DT_FAILURE;
  m_toolMode := TOOLMODE_PATHFIND_FOLLOW;

  m_filter.setIncludeFlags(SAMPLE_POLYFLAGS_ALL xor SAMPLE_POLYFLAGS_DISABLED);
  m_filter.setExcludeFlags(0);

  m_polyPickExt[0] := 2;
  m_polyPickExt[1] := 4;
  m_polyPickExt[2] := 2;

  m_neighbourhoodRadius := 2.5;
  m_randomRadius := 5.0;
end;

destructor TNavMeshTesterTool.Destroy;
begin
  // Delphi: Assume C++ does the same when var goes out of scope
  m_filter.Free;
  fFrame.Free;
  inherited;
end;

procedure TNavMeshTesterTool.init(sample: TSample);
begin
  m_sample := sample;
  m_navMesh := sample.getNavMesh;
  m_navQuery := sample.getNavMeshQuery;
  recalc();

  if (m_navQuery <> nil) then
  begin
    // Change costs.
    m_filter.setAreaCost(Byte(SAMPLE_POLYAREA_GROUND), 1.0);
    m_filter.setAreaCost(Byte(SAMPLE_POLYAREA_WATER), 10.0);
    m_filter.setAreaCost(Byte(SAMPLE_POLYAREA_ROAD), 1.0);
    m_filter.setAreaCost(Byte(SAMPLE_POLYAREA_DOOR), 1.0);
    m_filter.setAreaCost(Byte(SAMPLE_POLYAREA_GRASS), 2.0);
    m_filter.setAreaCost(Byte(SAMPLE_POLYAREA_JUMP), 1.5);
  end;

  m_neighbourhoodRadius := sample.getAgentRadius * 20.0;
  m_randomRadius := sample.getAgentRadius * 30.0;
end;

procedure TNavMeshTesterTool.handleMenu(Sender: TObject);
var status: TdtStatus; i: Integer; pt: array[0..2] of Single; ref: TdtPolyRef;
begin
  if fUpdateUI then Exit;

  // Delphi: When the Sender is NIL, we fill the controls with current state values
  if Sender = nil then
  begin
    fUpdateUI := True;
    fFrame.rgToolMode.ItemIndex := Byte(m_toolMode);
    fFrame.rgVerticesAtCrossings.Enabled := m_toolMode = TOOLMODE_PATHFIND_STRAIGHT;
    fFrame.rgVerticesAtCrossings.ItemIndex := Byte(m_straightPathOptions);
    fFrame.btnSetRandomEnd.Enabled := m_sposSet;
    fFrame.btnMakeRandomPointsAround.Enabled := m_sposSet;

    fFrame.cbInclWalk.Checked := (m_filter.getIncludeFlags() and SAMPLE_POLYFLAGS_WALK) <> 0;
    fFrame.cbInclSwim.Checked := (m_filter.getIncludeFlags() and SAMPLE_POLYFLAGS_SWIM) <> 0;
    fFrame.cbInclDoor.Checked := (m_filter.getIncludeFlags() and SAMPLE_POLYFLAGS_DOOR) <> 0;
    fFrame.cbInclJump.Checked := (m_filter.getIncludeFlags() and SAMPLE_POLYFLAGS_JUMP) <> 0;

    fFrame.cbExclWalk.Checked := (m_filter.getExcludeFlags() and SAMPLE_POLYFLAGS_WALK) <> 0;
    fFrame.cbExclSwim.Checked := (m_filter.getExcludeFlags() and SAMPLE_POLYFLAGS_SWIM) <> 0;
    fFrame.cbExclDoor.Checked := (m_filter.getExcludeFlags() and SAMPLE_POLYFLAGS_DOOR) <> 0;
    fFrame.cbExclJump.Checked := (m_filter.getExcludeFlags() and SAMPLE_POLYFLAGS_JUMP) <> 0;
    fUpdateUI := False;
  end;

  if Sender = fFrame.rgToolMode then
  begin
    m_toolMode := TToolMode(fFrame.rgToolMode.ItemIndex);
    recalc();
  end;

  fFrame.rgVerticesAtCrossings.Visible := (m_toolMode = TOOLMODE_PATHFIND_STRAIGHT);
  if (m_toolMode = TOOLMODE_PATHFIND_STRAIGHT) then
  begin
    m_straightPathOptions := TdtStraightPathOptions(fFrame.rgVerticesAtCrossings.ItemIndex);
    recalc();
  end;

  if Sender = fFrame.btnSetRandomStart then
  begin
    status := m_navQuery.findRandomPoint(m_filter, {frand,} @m_startRef, @m_spos[0]);
    if (dtStatusSucceed(status)) then
    begin
      m_sposSet := true;
      recalc();
    end;
  end;
  if Sender = fFrame.btnSetRandomEnd then
  begin
    if (m_sposSet) then
    begin
      status := m_navQuery.findRandomPointAroundCircle(m_startRef, @m_spos[0], m_randomRadius, m_filter, {frand,} @m_endRef, @m_epos[0]);
      if (dtStatusSucceed(status)) then
      begin
        m_eposSet := true;
        recalc();
      end;
    end;
  end;

  if Sender = fFrame.btnMakeRandomPoints then
  begin
    m_randPointsInCircle := false;
    m_nrandPoints := 0;
    for i := 0 to MAX_RAND_POINTS - 1 do
    begin
      status := m_navQuery.findRandomPoint(m_filter, {frand,} @ref, @pt[0]);
      if (dtStatusSucceed(status)) then
      begin
        dtVcopy(@m_randPoints[m_nrandPoints*3], @pt[0]);
        Inc(m_nrandPoints);
      end;
    end;
  end;
  if Sender = fFrame.btnMakeRandomPointsAround then
  begin
    if (m_sposSet) then
    begin
      m_nrandPoints := 0;
      m_randPointsInCircle := true;
      for i := 0 to MAX_RAND_POINTS - 1 do
      begin
        status := m_navQuery.findRandomPointAroundCircle(m_startRef, @m_spos[0], m_randomRadius, m_filter, {frand,} @ref, @pt[0]);
        if (dtStatusSucceed(status)) then
        begin
          dtVcopy(@m_randPoints[m_nrandPoints*3], @pt[0]);
          Inc(m_nrandPoints);
        end;
      end;
    end;
  end;

  m_filter.setIncludeFlags(Byte(fFrame.cbInclWalk.Checked) * SAMPLE_POLYFLAGS_WALK +
                           Byte(fFrame.cbInclSwim.Checked) * SAMPLE_POLYFLAGS_SWIM +
                           Byte(fFrame.cbInclDoor.Checked) * SAMPLE_POLYFLAGS_DOOR +
                           Byte(fFrame.cbInclJump.Checked) * SAMPLE_POLYFLAGS_JUMP);
  recalc();

  m_filter.setExcludeFlags(Byte(fFrame.cbExclWalk.Checked) * SAMPLE_POLYFLAGS_WALK +
                           Byte(fFrame.cbExclSwim.Checked) * SAMPLE_POLYFLAGS_SWIM +
                           Byte(fFrame.cbExclDoor.Checked) * SAMPLE_POLYFLAGS_DOOR +
                           Byte(fFrame.cbExclJump.Checked) * SAMPLE_POLYFLAGS_JUMP);
  recalc();
end;

procedure TNavMeshTesterTool.handleClick(s,p: PSingle; shift: Boolean);
begin
  if (shift) then
  begin
    m_sposSet := true;
    dtVcopy(@m_spos[0], p);
  end
  else
  begin
    m_eposSet := true;
    dtVcopy(@m_epos[0], p);
  end;
  recalc();
end;

procedure TNavMeshTesterTool.handleStep();
begin
end;

procedure TNavMeshTesterTool.handleToggle();
const STEP_SIZE = 0.5;
const SLOP = 0.01;
var steerPos,delta,moveTgt,reslt,startPos,endPos: array [0..2] of Single;  steerPosFlag: Byte; steerPosRef,prevRef,polyRef: TdtPolyRef;
endOfPath, offMeshConnection: Boolean; len: Single; visited: array [0..15] of TdtPolyRef; nvisited,npos,i: Integer; h,eh: Single;
status: TdtStatus;
begin
  // TODO: merge separate to a path iterator. Use same code in recalc() too.
  if (m_toolMode <> TOOLMODE_PATHFIND_FOLLOW) then
    Exit;

  if (not m_sposSet or not m_eposSet or (m_startRef = 0) or (m_endRef = 0)) then
    Exit;

  if (m_pathIterNum = 0) then
  begin
    m_navQuery.findPath(m_startRef, m_endRef, @m_spos[0], @m_epos[0], m_filter, @m_polys[0], @m_npolys, MAX_POLYS);
    m_nsmoothPath := 0;

    m_pathIterPolyCount := m_npolys;
    if (m_pathIterPolyCount <> 0) then
      Move(m_polys[0], m_pathIterPolys[0], sizeof(TdtPolyRef)*m_pathIterPolyCount);

    if (m_pathIterPolyCount <> 0) then
    begin
      // Iterate over the path to find smooth path on the detail mesh surface.
      m_navQuery.closestPointOnPoly(m_startRef, @m_spos[0], @m_iterPos[0], nil);
      m_navQuery.closestPointOnPoly(m_pathIterPolys[m_pathIterPolyCount-1], @m_epos[0], @m_targetPos[0], nil);

      m_nsmoothPath := 0;

      dtVcopy(@m_smoothPath[m_nsmoothPath*3], @m_iterPos[0]);
      Inc(m_nsmoothPath);
    end;
  end;

  dtVcopy(@m_prevIterPos[0], @m_iterPos[0]);

  Inc(m_pathIterNum);

  if (m_pathIterPolyCount = 0) then
    Exit;

  if (m_nsmoothPath >= MAX_SMOOTH) then
    Exit;

  // Move towards target a small advancement at a time until target reached or
  // when ran out of memory to store the path.

  // Find location to steer towards.

  if (not getSteerTarget(m_navQuery, @m_iterPos[0], @m_targetPos[0], SLOP,
            @m_pathIterPolys[0], m_pathIterPolyCount, @steerPos[0], @steerPosFlag, @steerPosRef,
            @m_steerPoints[0], @m_steerPointCount)) then
    Exit;

  dtVcopy(@m_steerPos[0], @steerPos[0]);

  endOfPath := (steerPosFlag and Byte(DT_STRAIGHTPATH_END)) <> 0;
  offMeshConnection := (steerPosFlag and Byte(DT_STRAIGHTPATH_OFFMESH_CONNECTION)) <> 0;

  // Find movement delta.
  dtVsub(@delta[0], @steerPos[0], @m_iterPos[0]);
  len := sqrt(dtVdot(@delta[0],@delta[0]));
  // If the steer target is end of path or off-mesh link, do not move past the location.
  if (endOfPath or offMeshConnection) and (len < STEP_SIZE) then
    len := 1
  else
    len := STEP_SIZE / len;
  dtVmad(@moveTgt[0], @m_iterPos[0], @delta[0], len);

  // Move
  nvisited := 0;
  m_navQuery.moveAlongSurface(m_pathIterPolys[0], @m_iterPos[0], @moveTgt[0], m_filter,
                 @reslt[0], @visited[0], @nvisited, 16);
  m_pathIterPolyCount := fixupCorridor(@m_pathIterPolys[0], m_pathIterPolyCount, MAX_POLYS, @visited[0], nvisited);
  m_pathIterPolyCount := fixupShortcuts(@m_pathIterPolys[0], m_pathIterPolyCount, m_navQuery);

  h := 0;
  m_navQuery.getPolyHeight(m_pathIterPolys[0], @reslt[0], @h);
  reslt[1] := h;
  dtVcopy(@m_iterPos[0], @reslt[0]);

  // Handle end of path and off-mesh links when close enough.
  if (endOfPath and inRange(@m_iterPos[0], @steerPos[0], SLOP, 1.0)) then
  begin
    // Reached end of path.
    dtVcopy(@m_iterPos[0], @m_targetPos[0]);
    if (m_nsmoothPath < MAX_SMOOTH) then
    begin
      dtVcopy(@m_smoothPath[m_nsmoothPath*3], @m_iterPos[0]);
      Inc(m_nsmoothPath);
    end;
    Exit;
  end
  else if (offMeshConnection and inRange(@m_iterPos[0], @steerPos[0], SLOP, 1.0)) then
  begin
    // Reached off-mesh connection.

    // Advance the path up to and over the off-mesh connection.
    prevRef := 0; polyRef := m_pathIterPolys[0];
    npos := 0;
    while (npos < m_pathIterPolyCount) and (polyRef <> steerPosRef) do
    begin
      prevRef := polyRef;
      polyRef := m_pathIterPolys[npos];
      Inc(npos);
    end;
    for i := npos to m_pathIterPolyCount - 1 do
      m_pathIterPolys[i-npos] := m_pathIterPolys[i];
    Dec(m_pathIterPolyCount, npos);

    // Handle the connection.
    status := m_navMesh.getOffMeshConnectionPolyEndPoints(prevRef, polyRef, @startPos[0], @endPos[0]);
    if (dtStatusSucceed(status)) then
    begin
      if (m_nsmoothPath < MAX_SMOOTH) then
      begin
        dtVcopy(@m_smoothPath[m_nsmoothPath*3], @startPos[0]);
        Inc(m_nsmoothPath);
        // Hack to make the dotted path not visible during off-mesh connection.
        if (m_nsmoothPath and 1) <> 0 then
        begin
          dtVcopy(@m_smoothPath[m_nsmoothPath*3], @startPos[0]);
          Inc(m_nsmoothPath);
        end;
      end;
      // Move position at the other side of the off-mesh link.
      dtVcopy(@m_iterPos[0], @endPos[0]);
      eh := 0.0;
      m_navQuery.getPolyHeight(m_pathIterPolys[0], @m_iterPos[0], @eh);
      m_iterPos[1] := eh;
    end;
  end;

  // Store results.
  if (m_nsmoothPath < MAX_SMOOTH) then
  begin
    dtVcopy(@m_smoothPath[m_nsmoothPath*3], @m_iterPos[0]);
    Inc(m_nsmoothPath);
  end;

end;

procedure TNavMeshTesterTool.handleUpdate(dt: Single);
var epos: array [0..2] of Single;
begin
  if (m_toolMode = TOOLMODE_PATHFIND_SLICED) then
  begin
    if (dtStatusInProgress(m_pathFindStatus)) then
    begin
      m_pathFindStatus := m_navQuery.updateSlicedFindPath(1, nil);
    end;
    if (dtStatusSucceed(m_pathFindStatus)) then
    begin
      m_navQuery.finalizeSlicedFindPath(@m_polys[0], @m_npolys, MAX_POLYS);
      m_nstraightPath := 0;
      if (m_npolys <> 0) then
      begin
        // In case of partial path, make sure the end point is clamped to the last polygon.

        dtVcopy(@epos[0], @m_epos[0]);
        if (m_polys[m_npolys-1] <> m_endRef) then
        m_navQuery.closestPointOnPoly(m_polys[m_npolys-1], @m_epos[0], @epos[0], nil);

        m_navQuery.findStraightPath(@m_spos[0], @epos[0], @m_polys[0], m_npolys,
                       @m_straightPath[0], @m_straightPathFlags[0],
                       @m_straightPathPolys[0], @m_nstraightPath, MAX_POLYS, Byte(DT_STRAIGHTPATH_ALL_CROSSINGS));
      end;

      m_pathFindStatus := DT_FAILURE;
    end;
  end;
end;

procedure TNavMeshTesterTool.reset();
begin
  m_startRef := 0;
  m_endRef := 0;
  m_npolys := 0;
  m_nstraightPath := 0;
  m_nsmoothPath := 0;
  FillChar(m_hitPos[0], sizeof(m_hitPos), 0);
  FillChar(m_hitNormal[0], sizeof(m_hitNormal), 0);
  m_distanceToWall := 0;
end;


procedure TNavMeshTesterTool.recalc();
const STEP_SIZE = 0.5;
const SLOP = 0.01;
var polys: array [0..MAX_POLYS-1] of TdtPolyRef; npolys: Integer; iterPos,targetPos,steerPos,delta,moveTgt,reslt,startPos,endPos,epos: array [0..2] of Single;
steerPosFlag: Byte; steerPosRef,prevRef,polyRef: TdtPolyRef; endOfPath,offMeshConnection: Boolean; len: Single;
visited: array [0..15] of TdtPolyRef; nvisited,npos,i: Integer; h,eh,t: Single; status: TdtStatus; dx,dz,dist: Single;
nx,nz,agentHeight: Single;
begin
  if (m_navMesh = nil) then
    Exit;

  if (m_sposSet) then
    m_navQuery.findNearestPoly(@m_spos[0], @m_polyPickExt[0], m_filter, @m_startRef, nil)
  else
    m_startRef := 0;

  if (m_eposSet) then
    m_navQuery.findNearestPoly(@m_epos[0], @m_polyPickExt[0], m_filter, @m_endRef, nil)
  else
    m_endRef := 0;

  m_pathFindStatus := DT_FAILURE;

  if (m_toolMode = TOOLMODE_PATHFIND_FOLLOW) then
  begin
    m_pathIterNum := 0;
    if (m_sposSet and m_eposSet and (m_startRef <> 0) and (m_endRef <> 0)) then
    begin
{$ifdef DUMP_REQS}
      printf("pi  %f %f %f  %f %f %f  0x%x 0x%x\n",
           m_spos[0],m_spos[1],m_spos[2], m_epos[0],m_epos[1],m_epos[2],
           m_filter.getIncludeFlags(), m_filter.getExcludeFlags());
{$endif}

      m_navQuery.findPath(m_startRef, m_endRef, @m_spos[0], @m_epos[0], m_filter, @m_polys[0], @m_npolys, MAX_POLYS);

      m_nsmoothPath := 0;

      if (m_npolys <> 0) then
      begin
        // Iterate over the path to find smooth path on the detail mesh surface.
        Move(m_polys[0], polys[0], sizeof(TdtPolyRef)*m_npolys);
        npolys := m_npolys;

        m_navQuery.closestPointOnPoly(m_startRef, @m_spos[0], @iterPos[0], nil);
        m_navQuery.closestPointOnPoly(polys[npolys-1], @m_epos[0], @targetPos[0], nil);

        m_nsmoothPath := 0;

        dtVcopy(@m_smoothPath[m_nsmoothPath*3], @iterPos[0]);
        Inc(m_nsmoothPath);

        // Move towards target a small advancement at a time until target reached or
        // when ran out of memory to store the path.
        while (npolys <> 0) and (m_nsmoothPath < MAX_SMOOTH) do
        begin
          // Find location to steer towards.
          if (not getSteerTarget(m_navQuery, @iterPos[0], @targetPos[0], SLOP,
                    @polys[0], npolys, @steerPos[0], @steerPosFlag, @steerPosRef)) then
            break;

          endOfPath := (steerPosFlag and Byte(DT_STRAIGHTPATH_END)) <> 0;
          offMeshConnection := (steerPosFlag and Byte(DT_STRAIGHTPATH_OFFMESH_CONNECTION)) <> 0;

          // Find movement delta.
          dtVsub(@delta[0], @steerPos[0], @iterPos[0]);
          len := Sqrt(dtVdot(@delta[0],@delta[0]));
          // If the steer target is end of path or off-mesh link, do not move past the location.
          if (endOfPath or offMeshConnection) and (len < STEP_SIZE) then
            len := 1
          else
            len := STEP_SIZE / len;

          dtVmad(@moveTgt[0], @iterPos[0], @delta[0], len);

          // Move
          nvisited := 0;
          m_navQuery.moveAlongSurface(polys[0], @iterPos[0], @moveTgt[0], m_filter,
                         @reslt[0], @visited[0], @nvisited, 16);

          npolys := fixupCorridor(@polys[0], npolys, MAX_POLYS, @visited[0], nvisited);
          npolys := fixupShortcuts(@polys[0], npolys, m_navQuery);

          h := 0;
          m_navQuery.getPolyHeight(polys[0], @reslt[0], @h);
          reslt[1] := h;
          dtVcopy(@iterPos[0], @reslt[0]);

          // Handle end of path and off-mesh links when close enough.
          if (endOfPath and inRange(@iterPos[0], @steerPos[0], SLOP, 1.0)) then
          begin
            // Reached end of path.
            dtVcopy(@iterPos[0], @targetPos[0]);
            if (m_nsmoothPath < MAX_SMOOTH) then
            begin
              dtVcopy(@m_smoothPath[m_nsmoothPath*3], @iterPos[0]);
              Inc(m_nsmoothPath);
            end;
            break;
          end
          else if (offMeshConnection and inRange(@iterPos[0], @steerPos[0], SLOP, 1.0)) then
          begin
            // Reached off-mesh connection.

            // Advance the path up to and over the off-mesh connection.
            prevRef := 0; polyRef := polys[0];
            npos := 0;
            while (npos < npolys) and (polyRef <> steerPosRef) do
            begin
              prevRef := polyRef;
              polyRef := polys[npos];
              Inc(npos);
            end;
            for i := npos to npolys - 1 do
              polys[i-npos] := polys[i];
            Dec(npolys, npos);

            // Handle the connection.
            status := m_navMesh.getOffMeshConnectionPolyEndPoints(prevRef, polyRef, @startPos[0], @endPos[0]);
            if (dtStatusSucceed(status)) then
            begin
              if (m_nsmoothPath < MAX_SMOOTH) then
              begin
                dtVcopy(@m_smoothPath[m_nsmoothPath*3], @startPos[0]);
                Inc(m_nsmoothPath);
                // Hack to make the dotted path not visible during off-mesh connection.
                if (m_nsmoothPath and 1) <> 0 then
                begin
                  dtVcopy(@m_smoothPath[m_nsmoothPath*3], @startPos[0]);
                  Inc(m_nsmoothPath);
                end;
              end;
              // Move position at the other side of the off-mesh link.
              dtVcopy(@iterPos[0], @endPos[0]);
              eh := 0.0;
              m_navQuery.getPolyHeight(polys[0], @iterPos[0], @eh);
              iterPos[1] := eh;
            end;
          end;

          // Store results.
          if (m_nsmoothPath < MAX_SMOOTH) then
          begin
            dtVcopy(@m_smoothPath[m_nsmoothPath*3], @iterPos[0]);
            Inc(m_nsmoothPath);
          end;
        end;
      end;

    end
    else
    begin
      m_npolys := 0;
      m_nsmoothPath := 0;
    end;
  end
  else if (m_toolMode = TOOLMODE_PATHFIND_STRAIGHT) then
  begin
    if (m_sposSet and m_eposSet and (m_startRef <> 0) and (m_endRef <> 0)) then
    begin
{$ifdef DUMP_REQS}
      printf("ps  %f %f %f  %f %f %f  0x%x 0x%x\n",
           m_spos[0],m_spos[1],m_spos[2], m_epos[0],m_epos[1],m_epos[2],
           m_filter.getIncludeFlags(), m_filter.getExcludeFlags());
{$endif}
      m_navQuery.findPath(m_startRef, m_endRef, @m_spos[0], @m_epos[0], m_filter, @m_polys[0], @m_npolys, MAX_POLYS);
      m_nstraightPath := 0;
      if (m_npolys <> 0) then
      begin
        // In case of partial path, make sure the end point is clamped to the last polygon.
        dtVcopy(@epos[0], @m_epos[0]);
        if (m_polys[m_npolys-1] <> m_endRef) then
          m_navQuery.closestPointOnPoly(m_polys[m_npolys-1], @m_epos[0], @epos[0], nil);

        m_navQuery.findStraightPath(@m_spos[0], @epos[0], @m_polys[0], m_npolys,
                       @m_straightPath[0], @m_straightPathFlags[0],
                       @m_straightPathPolys[0], @m_nstraightPath, MAX_POLYS, Byte(m_straightPathOptions));
      end;
    end
    else
    begin
      m_npolys := 0;
      m_nstraightPath := 0;
    end;
  end
  else if (m_toolMode = TOOLMODE_PATHFIND_SLICED) then
  begin
    if (m_sposSet and m_eposSet and (m_startRef <> 0) and (m_endRef <> 0)) then
    begin
{$ifdef DUMP_REQS}
      printf("ps  %f %f %f  %f %f %f  0x%x 0x%x\n",
           m_spos[0],m_spos[1],m_spos[2], m_epos[0],m_epos[1],m_epos[2],
           m_filter.getIncludeFlags(), m_filter.getExcludeFlags());
{$endif}
      m_npolys := 0;
      m_nstraightPath := 0;

      m_pathFindStatus := m_navQuery.initSlicedFindPath(m_startRef, m_endRef, @m_spos[0], @m_epos[0], m_filter, Byte(DT_FINDPATH_ANY_ANGLE));
    end
    else
    begin
      m_npolys := 0;
      m_nstraightPath := 0;
    end;
  end
  else if (m_toolMode = TOOLMODE_RAYCAST) then
  begin
    m_nstraightPath := 0;
    if (m_sposSet and m_eposSet and (m_startRef <> 0)) then
    begin
{$ifdef DUMP_REQS}
      printf("rc  %f %f %f  %f %f %f  0x%x 0x%x\n",
           m_spos[0],m_spos[1],m_spos[2], m_epos[0],m_epos[1],m_epos[2],
           m_filter.getIncludeFlags(), m_filter.getExcludeFlags());
{$endif}
      t := 0;
      m_npolys := 0;
      m_nstraightPath := 2;
      m_straightPath[0] := m_spos[0];
      m_straightPath[1] := m_spos[1];
      m_straightPath[2] := m_spos[2];
      m_navQuery.raycast(m_startRef, @m_spos[0], @m_epos[0], m_filter, @t, @m_hitNormal[0], @m_polys[0], @m_npolys, MAX_POLYS);
      if (t > 1) then
      begin
        // No hit
        dtVcopy(@m_hitPos[0], @m_epos[0]);
        m_hitResult := false;
      end
      else
      begin
        // Hit
        dtVlerp(@m_hitPos[0], @m_spos[0], @m_epos[0], t);
        m_hitResult := true;
      end;
      // Adjust height.
      if (m_npolys > 0) then
      begin
        h := 0;
        m_navQuery.getPolyHeight(m_polys[m_npolys-1], @m_hitPos[0], @h);
        m_hitPos[1] := h;
      end;
      dtVcopy(@m_straightPath[3], @m_hitPos[0]);
    end;
  end
  else if (m_toolMode = TOOLMODE_DISTANCE_TO_WALL) then
  begin
    m_distanceToWall := 0;
    if (m_sposSet and (m_startRef <> 0)) then
    begin
{$ifdef DUMP_REQS}
      printf("dw  %f %f %f  %f  0x%x 0x%x\n",
           m_spos[0],m_spos[1],m_spos[2], 100.0f,
           m_filter.getIncludeFlags(), m_filter.getExcludeFlags());
{$endif}
      m_distanceToWall := 0.0;
      m_navQuery.findDistanceToWall(m_startRef, @m_spos[0], 100.0, m_filter, @m_distanceToWall, @m_hitPos[0], @m_hitNormal[0]);
    end;
  end
  else if (m_toolMode = TOOLMODE_FIND_POLYS_IN_CIRCLE) then
  begin
    if (m_sposSet and m_eposSet and (m_startRef <> 0)) then
    begin
      dx := m_epos[0] - m_spos[0];
      dz := m_epos[2] - m_spos[2];
      dist := sqrt(dx*dx + dz*dz);
{$ifdef DUMP_REQS}
      printf("fpc  %f %f %f  %f  0x%x 0x%x\n",
           m_spos[0],m_spos[1],m_spos[2], dist,
           m_filter.getIncludeFlags(), m_filter.getExcludeFlags());
{$endif}
      m_navQuery.findPolysAroundCircle(m_startRef, @m_spos[0], dist, m_filter,
                        @m_polys[0], @m_parent[0], nil, @m_npolys, MAX_POLYS);

    end;
  end
  else if (m_toolMode = TOOLMODE_FIND_POLYS_IN_SHAPE) then
  begin
    if (m_sposSet and m_eposSet and (m_startRef <> 0)) then
    begin
      nx := (m_epos[2] - m_spos[2])*0.25;
      nz := -(m_epos[0] - m_spos[0])*0.25;
      if m_sample <> nil then agentHeight := m_sample.getAgentHeight else agentHeight := 0;

      m_queryPoly[0] := m_spos[0] + nx*1.2;
      m_queryPoly[1] := m_spos[1] + agentHeight/2;
      m_queryPoly[2] := m_spos[2] + nz*1.2;

      m_queryPoly[3] := m_spos[0] - nx*1.3;
      m_queryPoly[4] := m_spos[1] + agentHeight/2;
      m_queryPoly[5] := m_spos[2] - nz*1.3;

      m_queryPoly[6] := m_epos[0] - nx*0.8;
      m_queryPoly[7] := m_epos[1] + agentHeight/2;
      m_queryPoly[8] := m_epos[2] - nz*0.8;

      m_queryPoly[9] := m_epos[0] + nx;
      m_queryPoly[10] := m_epos[1] + agentHeight/2;
      m_queryPoly[11] := m_epos[2] + nz;

{$ifdef DUMP_REQS}
      printf("fpp  %f %f %f  %f %f %f  %f %f %f  %f %f %f  0x%x 0x%x\n",
           m_queryPoly[0],m_queryPoly[1],m_queryPoly[2],
           m_queryPoly[3],m_queryPoly[4],m_queryPoly[5],
           m_queryPoly[6],m_queryPoly[7],m_queryPoly[8],
           m_queryPoly[9],m_queryPoly[10],m_queryPoly[11],
           m_filter.getIncludeFlags(), m_filter.getExcludeFlags());
{$endif}
      m_navQuery.findPolysAroundShape(m_startRef, @m_queryPoly[0], 4, m_filter,
                       @m_polys[0], @m_parent[0], nil, @m_npolys, MAX_POLYS);
    end;
  end
  else if (m_toolMode = TOOLMODE_FIND_LOCAL_NEIGHBOURHOOD) then
  begin
    if (m_sposSet and (m_startRef <> 0)) then
    begin
{$ifdef DUMP_REQS}
      printf("fln  %f %f %f  %f  0x%x 0x%x\n",
           m_spos[0],m_spos[1],m_spos[2], m_neighbourhoodRadius,
           m_filter.getIncludeFlags(), m_filter.getExcludeFlags());
{$endif}
      //todo: m_navQuery.findLocalNeighbourhood(m_startRef, m_spos, m_neighbourhoodRadius, m_filter, m_polys, m_parent, &m_npolys, MAX_POLYS);
    end;
  end;
end;

procedure getPolyCenter(navMesh: TdtNavMesh; ref: TdtPolyRef; center: PSingle);
var tile: PdtMeshTile; poly: PdtPoly; status: TdtStatus; i: Integer; v: PSingle; s: Single;
begin
  center[0] := 0;
  center[1] := 0;
  center[2] := 0;

  tile := nil;
  poly := nil;
  status := navMesh.getTileAndPolyByRef(ref, @tile, @poly);
  if (dtStatusFailed(status)) then
    Exit;

  for i := 0 to poly.vertCount - 1 do
  begin
    v := @tile.verts[poly.verts[i]*3];
    center[0] := center[0] + v[0];
    center[1] := center[1] + v[1];
    center[2] := center[2] + v[2];
  end;
  s := 1.0 / poly.vertCount;
  center[0] := center[0] * s;
  center[1] := center[1] * s;
  center[2] := center[2] * s;
end;



procedure TNavMeshTesterTool.handleRender();
const MAX_SEGS = DT_VERTS_PER_POLYGON*4;
var dd: TDebugDrawGL;
  startCol, endCol, pathCol, spathCol, prevCol, curCol, steerCol, offMeshCol, col, hitCol: Cardinal;
  agentRadius, agentHeight, agentClimb: Single; i,j: Integer;
  p0,p1,delta,norm: array [0..2] of Single; pp0,pp1,s,p: PSingle; dx,dz,dist,tseg,distSqr: Single;
  segs: array [0..MAX_SEGS*6-1] of Single;
  refs: array [0..MAX_SEGS-1] of TdtPolyRef;
  nsegs: Integer;
begin
  dd := TDebugDrawGL.Create;

  startCol := duRGBA(128,25,0,192);
  endCol := duRGBA(51,102,0,129);
  pathCol := duRGBA(0,0,0,64);

  agentRadius := m_sample.getAgentRadius;
  agentHeight := m_sample.getAgentHeight;
  agentClimb := m_sample.getAgentClimb;

  dd.depthMask(false);
  if (m_sposSet) then
    drawAgent(@m_spos[0], agentRadius, agentHeight, agentClimb, startCol);
  if (m_eposSet) then
    drawAgent(@m_epos[0], agentRadius, agentHeight, agentClimb, endCol);
  dd.depthMask(true);

  if (m_navMesh = nil) then
  begin
    Exit;
  end;

  if (m_toolMode = TOOLMODE_PATHFIND_FOLLOW) then
  begin
    duDebugDrawNavMeshPoly(dd, m_navMesh, m_startRef, startCol);
    duDebugDrawNavMeshPoly(dd, m_navMesh, m_endRef, endCol);

    if (m_npolys <> 0) then
    begin
      for i := 0 to m_npolys - 1 do
      begin
        if (m_polys[i] = m_startRef) or (m_polys[i] = m_endRef) then
          continue;
        duDebugDrawNavMeshPoly(dd, m_navMesh, m_polys[i], pathCol);
      end;
    end;

    if (m_nsmoothPath <> 0) then
    begin
      dd.depthMask(false);
      spathCol := duRGBA(0,0,0,220);
      dd.&begin(DU_DRAW_LINES, 3.0);
      for i := 0 to m_nsmoothPath - 1 do
        dd.vertex(m_smoothPath[i*3], m_smoothPath[i*3+1]+0.1, m_smoothPath[i*3+2], spathCol);
      dd.&end();
      dd.depthMask(true);
    end;

    if (m_pathIterNum <> 0) then
    begin
      duDebugDrawNavMeshPoly(dd, m_navMesh, m_pathIterPolys[0], duRGBA(255,255,255,128));

      dd.depthMask(false);
      dd.&begin(DU_DRAW_LINES, 1.0);

      prevCol := duRGBA(255,192,0,220);
      curCol := duRGBA(255,255,255,220);
      steerCol := duRGBA(0,192,255,220);

      dd.vertex(m_prevIterPos[0],m_prevIterPos[1]-0.3,m_prevIterPos[2], prevCol);
      dd.vertex(m_prevIterPos[0],m_prevIterPos[1]+0.3,m_prevIterPos[2], prevCol);

      dd.vertex(m_iterPos[0],m_iterPos[1]-0.3,m_iterPos[2], curCol);
      dd.vertex(m_iterPos[0],m_iterPos[1]+0.3,m_iterPos[2], curCol);

      dd.vertex(m_prevIterPos[0],m_prevIterPos[1]+0.3,m_prevIterPos[2], prevCol);
      dd.vertex(m_iterPos[0],m_iterPos[1]+0.3,m_iterPos[2], prevCol);

      dd.vertex(m_prevIterPos[0],m_prevIterPos[1]+0.3,m_prevIterPos[2], steerCol);
      dd.vertex(m_steerPos[0],m_steerPos[1]+0.3,m_steerPos[2], steerCol);

      for i := 0 to m_steerPointCount-1 - 1 do
      begin
        dd.vertex(m_steerPoints[i*3+0],m_steerPoints[i*3+1]+0.2,m_steerPoints[i*3+2], duDarkenCol(steerCol));
        dd.vertex(m_steerPoints[(i+1)*3+0],m_steerPoints[(i+1)*3+1]+0.2,m_steerPoints[(i+1)*3+2], duDarkenCol(steerCol));
      end;

      dd.&end();
      dd.depthMask(true);
    end;
  end
  else if (m_toolMode = TOOLMODE_PATHFIND_STRAIGHT) or
       (m_toolMode = TOOLMODE_PATHFIND_SLICED) then
  begin
    duDebugDrawNavMeshPoly(dd, m_navMesh, m_startRef, startCol);
    duDebugDrawNavMeshPoly(dd, m_navMesh, m_endRef, endCol);

    if (m_npolys <> 0) then
    begin
      for i := 0 to m_npolys - 1 do
      begin
        if (m_polys[i] = m_startRef) or (m_polys[i] = m_endRef) then
          continue;
        duDebugDrawNavMeshPoly(dd, m_navMesh, m_polys[i], pathCol);
      end;
    end;

    if (m_nstraightPath <> 0) then
    begin
      dd.depthMask(false);
      spathCol := duRGBA(64,16,0,220);
      offMeshCol := duRGBA(128,96,0,220);
      dd.&begin(DU_DRAW_LINES, 2.0);
      for i := 0 to m_nstraightPath-1 - 1 do
      begin
        if (m_straightPathFlags[i] and Byte(DT_STRAIGHTPATH_OFFMESH_CONNECTION)) <> 0 then
          col := offMeshCol
        else
          col := spathCol;

        dd.vertex(m_straightPath[i*3], m_straightPath[i*3+1]+0.4, m_straightPath[i*3+2], col);
        dd.vertex(m_straightPath[(i+1)*3], m_straightPath[(i+1)*3+1]+0.4, m_straightPath[(i+1)*3+2], col);
      end;
      dd.&end();
      dd.&begin(DU_DRAW_POINTS, 6.0);
      for i := 0 to m_nstraightPath - 1 do
      begin
        if (m_straightPathFlags[i] and Byte(DT_STRAIGHTPATH_START)) <> 0 then
          col := startCol
        else if (m_straightPathFlags[i] and Byte(DT_STRAIGHTPATH_END)) <> 0 then
          col := endCol
        else if (m_straightPathFlags[i] and Byte(DT_STRAIGHTPATH_OFFMESH_CONNECTION)) <> 0 then
          col := offMeshCol
        else
          col := spathCol;
        dd.vertex(m_straightPath[i*3], m_straightPath[i*3+1]+0.4, m_straightPath[i*3+2], col);
      end;
      dd.&end();
      dd.depthMask(true);
    end;
  end
  else if (m_toolMode = TOOLMODE_RAYCAST) then
  begin
    duDebugDrawNavMeshPoly(dd, m_navMesh, m_startRef, startCol);

    if (m_nstraightPath <> 0) then
    begin
      for i := 1 to m_npolys - 1 do
        duDebugDrawNavMeshPoly(dd, m_navMesh, m_polys[i], pathCol);

      dd.depthMask(false);
      spathCol := IfThen(m_hitResult, duRGBA(64,16,0,220), duRGBA(240,240,240,220));
      dd.&begin(DU_DRAW_LINES, 2.0);
      for i := 0 to m_nstraightPath-1 - 1 do
      begin
        dd.vertex(m_straightPath[i*3], m_straightPath[i*3+1]+0.4, m_straightPath[i*3+2], spathCol);
        dd.vertex(m_straightPath[(i+1)*3], m_straightPath[(i+1)*3+1]+0.4, m_straightPath[(i+1)*3+2], spathCol);
      end;
      dd.&end();
      dd.&begin(DU_DRAW_POINTS, 4.0);
      for i := 0 to m_nstraightPath - 1 do
        dd.vertex(m_straightPath[i*3], m_straightPath[i*3+1]+0.4, m_straightPath[i*3+2], spathCol);
      dd.&end();

      if (m_hitResult) then
      begin
        hitCol := duRGBA(0,0,0,128);
        dd.&begin(DU_DRAW_LINES, 2.0);
        dd.vertex(m_hitPos[0], m_hitPos[1] + 0.4, m_hitPos[2], hitCol);
        dd.vertex(m_hitPos[0] + m_hitNormal[0]*agentRadius,
              m_hitPos[1] + 0.4 + m_hitNormal[1]*agentRadius,
              m_hitPos[2] + m_hitNormal[2]*agentRadius, hitCol);
        dd.&end();
      end;
      dd.depthMask(true);
    end;
  end
  else if (m_toolMode = TOOLMODE_DISTANCE_TO_WALL) then
  begin
    duDebugDrawNavMeshPoly(dd, m_navMesh, m_startRef, startCol);
    dd.depthMask(false);
    duDebugDrawCircle(dd, m_spos[0], m_spos[1]+agentHeight/2, m_spos[2], m_distanceToWall, duRGBA(64,16,0,220), 2.0);
    dd.&begin(DU_DRAW_LINES, 3.0);
    dd.vertex(m_hitPos[0], m_hitPos[1] + 0.02, m_hitPos[2], duRGBA(0,0,0,192));
    dd.vertex(m_hitPos[0], m_hitPos[1] + agentHeight, m_hitPos[2], duRGBA(0,0,0,192));
    dd.&end();
    dd.depthMask(true);
  end
  else if (m_toolMode = TOOLMODE_FIND_POLYS_IN_CIRCLE) then
  begin
    for i := 0 to m_npolys - 1 do
    begin
      duDebugDrawNavMeshPoly(dd, m_navMesh, m_polys[i], pathCol);
      dd.depthMask(false);
      if (m_parent[i] <> 0) then
      begin
        dd.depthMask(false);
        getPolyCenter(m_navMesh, m_parent[i], @p0[0]);
        getPolyCenter(m_navMesh, m_polys[i], @p1[0]);
        duDebugDrawArc(dd, p0[0],p0[1],p0[2], p1[0],p1[1],p1[2], 0.25, 0.0, 0.4, duRGBA(0,0,0,128), 2.0);
        dd.depthMask(true);
      end;
      dd.depthMask(true);
    end;

    if (m_sposSet and m_eposSet) then
    begin
      dd.depthMask(false);
      dx := m_epos[0] - m_spos[0];
      dz := m_epos[2] - m_spos[2];
      dist := sqrt(dx*dx + dz*dz);
      duDebugDrawCircle(dd, m_spos[0], m_spos[1]+agentHeight/2, m_spos[2], dist, duRGBA(64,16,0,220), 2.0);
      dd.depthMask(true);
    end;
  end
  else if (m_toolMode = TOOLMODE_FIND_POLYS_IN_SHAPE) then
  begin
    for i := 0 to m_npolys - 1 do
    begin
      duDebugDrawNavMeshPoly(dd, m_navMesh, m_polys[i], pathCol);
      dd.depthMask(false);
      if (m_parent[i] <> 0) then
      begin
        dd.depthMask(false);
        getPolyCenter(m_navMesh, m_parent[i], @p0[0]);
        getPolyCenter(m_navMesh, m_polys[i], @p1[0]);
        duDebugDrawArc(dd, p0[0],p0[1],p0[2], p1[0],p1[1],p1[2], 0.25, 0.0, 0.4, duRGBA(0,0,0,128), 2.0);
        dd.depthMask(true);
      end;
      dd.depthMask(true);
    end;

    if (m_sposSet and m_eposSet) then
    begin
      dd.depthMask(false);
      col := duRGBA(64,16,0,220);
      dd.&begin(DU_DRAW_LINES, 2.0);
      for i := 0 to 3 do
      begin
        j := (i+4-1) mod 4;
        pp0 := @m_queryPoly[j*3];
        pp1 := @m_queryPoly[i*3];
        dd.vertex(pp0, col);
        dd.vertex(pp1, col);
      end;
      dd.&end();
      dd.depthMask(true);
    end;
  end
  else if (m_toolMode = TOOLMODE_FIND_LOCAL_NEIGHBOURHOOD) then
  begin
    for i := 0 to m_npolys - 1 do
    begin
      duDebugDrawNavMeshPoly(dd, m_navMesh, m_polys[i], pathCol);
      dd.depthMask(false);
      if (m_parent[i] <> 0) then
      begin
        dd.depthMask(false);
        getPolyCenter(m_navMesh, m_parent[i], @p0[0]);
        getPolyCenter(m_navMesh, m_polys[i], @p1[0]);
        duDebugDrawArc(dd, p0[0],p0[1],p0[2], p1[0],p1[1],p1[2], 0.25, 0.0, 0.4, duRGBA(0,0,0,128), 2.0);
        dd.depthMask(true);
      end;

      FillChar(refs[0], sizeof(TdtPolyRef)*MAX_SEGS, 0);
      nsegs := 0;
      m_navQuery.getPolyWallSegments(m_polys[i], m_filter, @segs[0], @refs[0], @nsegs, MAX_SEGS);
      dd.&begin(DU_DRAW_LINES, 2.0);
      for j := 0 to nsegs - 1 do
      begin
        s := @segs[j*6];

        // Skip too distant segments.
        distSqr := dtDistancePtSegSqr2D(@m_spos[0], s, s+3, @tseg);
        if (distSqr > Sqr(m_neighbourhoodRadius)) then
          continue;

        dtVsub(@delta[0], s+3,s);
        dtVmad(@p0[0], s, @delta[0], 0.5);
        norm[0] := delta[2];
        norm[1] := 0;
        norm[2] := -delta[0];
        dtVnormalize(@norm[0]);
        dtVmad(@p1[0], @p0[0], @norm[0], agentRadius*0.5);

        // Skip backfacing segments.
        if (refs[j] <> 0) then
        begin
          col := duRGBA(255,255,255,32);
          dd.vertex(s[0],s[1]+agentClimb,s[2],col);
          dd.vertex(s[3],s[4]+agentClimb,s[5],col);
        end
        else
        begin
          col := duRGBA(192,32,16,192);
          if (dtTriArea2D(@m_spos[0], s, s+3) < 0.0) then
            col := duRGBA(96,32,16,192);

          dd.vertex(p0[0],p0[1]+agentClimb,p0[2],col);
          dd.vertex(p1[0],p1[1]+agentClimb,p1[2],col);

          dd.vertex(s[0],s[1]+agentClimb,s[2],col);
          dd.vertex(s[3],s[4]+agentClimb,s[5],col);
        end;
      end;
      dd.&end();

      dd.depthMask(true);
    end;

    if (m_sposSet) then
    begin
      dd.depthMask(false);
      duDebugDrawCircle(dd, m_spos[0], m_spos[1]+agentHeight/2, m_spos[2], m_neighbourhoodRadius, duRGBA(64,16,0,220), 2.0);
      dd.depthMask(true);
    end;
  end;

  if (m_nrandPoints > 0) then
  begin
    dd.&begin(DU_DRAW_POINTS, 6.0);
    for i := 0 to m_nrandPoints - 1 do
    begin
      p := @m_randPoints[i*3];
      dd.vertex(p[0],p[1]+0.1,p[2], duRGBA(220,32,16,192));
    end;
    dd.&end();

    if (m_randPointsInCircle and m_sposSet) then
    begin
      duDebugDrawCircle(dd, m_spos[0], m_spos[1]+agentHeight/2, m_spos[2], m_randomRadius, duRGBA(64,16,0,220), 2.0);
    end;
  end;

  dd.Free;
end;

procedure TNavMeshTesterTool.handleRenderOverlay(proj, model: PDouble; view: PInteger);
//var x,y,z: GLDouble; h: Integer;
begin
{  // Draw start and end point labels
  if (m_sposSet and (gluProject(m_spos[0], m_spos[1], m_spos[2],
                PGLMatrixd4(model)^, PGLMatrixd4(proj)^, PVector4i(view)^, @x, @y, @z) <> 0)) then
  begin
    //todo: imguiDrawText((int)x, (int)(y-25), IMGUI_ALIGN_CENTER, "Start", imguiRGBA(0,0,0,220));
  end;
  if (m_eposSet and (gluProject(m_epos[0], m_epos[1], m_epos[2],
                PGLMatrixd4(model)^, PGLMatrixd4(proj)^, PVector4i(view)^, @x, @y, @z) <> 0)) then
  begin
    //todo: imguiDrawText((int)x, (int)(y-25), IMGUI_ALIGN_CENTER, "End", imguiRGBA(0,0,0,220));
  end;

  // Tool help
  h := view[3];
  //todo: imguiDrawText(280, h-40, IMGUI_ALIGN_LEFT, "LMB+SHIFT: Set start location  LMB: Set end location", imguiRGBA(255,255,255,192));}
end;

procedure TNavMeshTesterTool.drawAgent(pos: PSingle; r, h, c: Single; col: Cardinal);
var dd: TDebugDrawGL; colb: Cardinal;
begin
  dd := TDebugDrawGL.Create;

  dd.depthMask(false);

  // Agent dimensions.
  duDebugDrawCylinderWire(dd, pos[0]-r, pos[1]+0.02, pos[2]-r, pos[0]+r, pos[1]+h, pos[2]+r, col, 2.0);

  duDebugDrawCircle(dd, pos[0],pos[1]+c,pos[2],r,duRGBA(0,0,0,64),1.0);

  colb := duRGBA(0,0,0,196);
  dd.&begin(DU_DRAW_LINES);
  dd.vertex(pos[0], pos[1]-c, pos[2], colb);
  dd.vertex(pos[0], pos[1]+c, pos[2], colb);
  dd.vertex(pos[0]-r/2, pos[1]+0.02, pos[2], colb);
  dd.vertex(pos[0]+r/2, pos[1]+0.02, pos[2], colb);
  dd.vertex(pos[0], pos[1]+0.02, pos[2]-r/2, colb);
  dd.vertex(pos[0], pos[1]+0.02, pos[2]+r/2, colb);
  dd.&end();

  dd.depthMask(true);

  dd.Free;
end;

end.