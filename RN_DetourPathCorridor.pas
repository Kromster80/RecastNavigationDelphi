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
unit RN_DetourPathCorridor;
interface
uses RN_DetourNavMeshQuery, RN_DetourNavMeshHelper;

/// Represents a dynamic polygon corridor used to plan agent movement.
/// @ingroup crowd, detour
type
  TdtPathCorridor = class
  private
    m_pos: array [0..2] of Single;
    m_target: array [0..2] of Single;

    m_path: PdtPolyRef;
    m_npath: Integer;
    m_maxPath: Integer;

  public
    constructor Create;
    destructor Destroy; override;

    /// Allocates the corridor's path buffer.
    ///  @param[in]    maxPath    The maximum path size the corridor can handle.
    /// @return True if the initialization succeeded.
    function init(const maxPath: Integer): Boolean;
  
    /// Resets the path corridor to the specified position.
    ///  @param[in]    ref    The polygon reference containing the position.
    ///  @param[in]    pos    The new position in the corridor. [(x, y, z)]
    procedure reset(ref: TdtPolyRef; const pos: PSingle);
  
    /// Finds the corners in the corridor from the position toward the target. (The straightened path.)
    ///  @param[out]  cornerVerts    The corner vertices. [(x, y, z) * cornerCount] [Size: <= maxCorners]
    ///  @param[out]  cornerFlags    The flag for each corner. [(flag) * cornerCount] [Size: <= maxCorners]
    ///  @param[out]  cornerPolys    The polygon reference for each corner. [(polyRef) * cornerCount]
    ///                  [Size: <= @p maxCorners]
    ///  @param[in]    maxCorners    The maximum number of corners the buffers can hold.
    ///  @param[in]    navquery    The query object used to build the corridor.
    ///  @param[in]    filter      The filter to apply to the operation.
    /// @return The number of corners returned in the corner buffers. [0 <= value <= @p maxCorners]
    function findCorners(cornerVerts: PSingle; cornerFlags: PByte;
            cornerPolys: PdtPolyRef; const maxCorners: Integer;
            navquery: TdtNavMeshQuery; const filter: TdtQueryFilter): Integer;

    /// Attempts to optimize the path if the specified point is visible from the current position.
    ///  @param[in]    next          The point to search toward. [(x, y, z])
    ///  @param[in]    pathOptimizationRange  The maximum range to search. [Limit: > 0]
    ///  @param[in]    navquery        The query object used to build the corridor.
    ///  @param[in]    filter          The filter to apply to the operation.
    procedure optimizePathVisibility(const next: PSingle; const pathOptimizationRange: Single;
                  navquery: TdtNavMeshQuery; const filter: TdtQueryFilter);

    /// Attempts to optimize the path using a local area search. (Partial replanning.)
    ///  @param[in]    navquery  The query object used to build the corridor.
    ///  @param[in]    filter    The filter to apply to the operation.
    function optimizePathTopology(navquery: TdtNavMeshQuery; const filter: TdtQueryFilter): Boolean;

    function moveOverOffmeshConnection(offMeshConRef: TdtPolyRef; refs: PdtPolyRef;
                     startPos, endPos: PSingle;
                     navquery: TdtNavMeshQuery): Boolean;

    function fixPathStart(safeRef: TdtPolyRef; const safePos: PSingle): Boolean;

    function trimInvalidPath(safeRef: TdtPolyRef; const safePos: PSingle;
               navquery: TdtNavMeshQuery; const filter: TdtQueryFilter): Boolean;

    /// Checks the current corridor path to see if its polygon references remain valid.
    ///  @param[in]    maxLookAhead  The number of polygons from the beginning of the corridor to search.
    ///  @param[in]    navquery    The query object used to build the corridor.
    ///  @param[in]    filter      The filter to apply to the operation.
    function isValid(const maxLookAhead: Integer; navquery: TdtNavMeshQuery; const filter: TdtQueryFilter): Boolean;

    /// Moves the position from the current location to the desired location, adjusting the corridor
    /// as needed to reflect the change.
    ///  @param[in]    npos    The desired new position. [(x, y, z)]
    ///  @param[in]    navquery  The query object used to build the corridor.
    ///  @param[in]    filter    The filter to apply to the operation.
    /// @return Returns true if move succeeded.
    function movePosition(const npos: PSingle; navquery: TdtNavMeshQuery; const filter: TdtQueryFilter): Boolean;

    /// Moves the target from the curent location to the desired location, adjusting the corridor
    /// as needed to reflect the change.
    ///  @param[in]    npos    The desired new target position. [(x, y, z)]
    ///  @param[in]    navquery  The query object used to build the corridor.
    ///  @param[in]    filter    The filter to apply to the operation.
    /// @return Returns true if move succeeded.
    function moveTargetPosition(const npos: PSingle; navquery: TdtNavMeshQuery; const filter: TdtQueryFilter): Boolean;
  
    /// Loads a new path and target into the corridor.
    ///  @param[in]    target    The target location within the last polygon of the path. [(x, y, z)]
    ///  @param[in]    path    The path corridor. [(polyRef) * @p npolys]
    ///  @param[in]    npath    The number of polygons in the path.
    procedure setCorridor(const target: PSingle; const path: PdtPolyRef; const npath: Integer);

    /// Gets the current position within the corridor. (In the first polygon.)
    /// @return The current position within the corridor.
    function getPos(): PSingle; { return m_pos; }

    /// Gets the current target within the corridor. (In the last polygon.)
    /// @return The current target within the corridor.
    function getTarget(): PSingle; { return m_target; }

    /// The polygon reference id of the first polygon in the corridor, the polygon containing the position.
    /// @return The polygon reference id of the first polygon in the corridor. (Or zero if there is no path.)
    function getFirstPoly(): TdtPolyRef; { return m_npath ? m_path[0] : 0; }

    /// The polygon reference id of the last polygon in the corridor, the polygon containing the target.
    /// @return The polygon reference id of the last polygon in the corridor. (Or zero if there is no path.)
    function getLastPoly(): TdtPolyRef; { return m_npath ? m_path[m_npath-1] : 0; }

    /// The corridor's path.
    /// @return The corridor's path. [(polyRef) * #getPathCount()]
    function getPath(): PdtPolyRef; { return m_path; }

    /// The number of polygons in the current corridor path.
    /// @return The number of polygons in the current corridor path.
    function getPathCount(): Integer; { return m_npath; }
  end;

  function dtMergeCorridorStartMoved(path: PdtPolyRef; const npath, maxPath: Integer;
                  const visited: PdtPolyRef; const nvisited: Integer): Integer;

  function dtMergeCorridorEndMoved(path: PdtPolyRef; const npath, maxPath: Integer;
                const visited: PdtPolyRef; const nvisited: Integer): Integer;

  function dtMergeCorridorStartShortcut(path: PdtPolyRef; const npath, maxPath: Integer;
                   const visited: PdtPolyRef; const nvisited: Integer): Integer;


implementation
uses
  RN_DetourCommon, RN_DetourNavMesh, RN_DetourStatus;


function dtMergeCorridorStartMoved(path: PdtPolyRef; const npath, maxPath: Integer;
                  const visited: PdtPolyRef; const nvisited: Integer): Integer;
var furthestPath, furthestVisited, i, j, req, orig, size: Integer; found: Boolean;
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
  orig := dtMin(furthestPath+1, npath);
  size := dtMax(0, npath-orig);
  if (req+size > maxPath) then
    size := maxPath-req;
  if (size <> 0) then
    Move(path[orig], path[req], size*sizeof(TdtPolyRef));

  // Store visited
  for i := 0 to req - 1 do
    path[i] := visited[(nvisited-1)-i];

  Result := req+size;
end;

function dtMergeCorridorEndMoved(path: PdtPolyRef; const npath, maxPath: Integer;
                const visited: PdtPolyRef; const nvisited: Integer): Integer;
var furthestPath, furthestVisited, i, j, req, orig, size, ppos, vpos, count: Integer; found: Boolean;
begin
  furthestPath := -1;
  furthestVisited := -1;

  // Find furthest common polygon.
  for i := 0 to npath - 1 do
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
  ppos := furthestPath+1;
  vpos := furthestVisited+1;
  count := dtMin(nvisited-vpos, maxPath-ppos);
  Assert(ppos+count <= maxPath);
  if (count <> 0) then
    Move(visited[vpos], path[ppos], sizeof(TdtPolyRef)*count);
  
  Result := ppos+count;
end;

function dtMergeCorridorStartShortcut(path: PdtPolyRef; const npath, maxPath: Integer;
                   const visited: PdtPolyRef; const nvisited: Integer): Integer;
var furthestPath, furthestVisited, i, j, req, orig, size, ppos, vpos, count: Integer; found: Boolean;
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
  req := furthestVisited;
  if (req <= 0) then
    Exit(npath);

  orig := furthestPath;
  size := dtMax(0, npath-orig);
  if (req+size > maxPath) then
    size := maxPath-req;
  if (size <> 0) then
    Move(path[orig], path[req], size*sizeof(TdtPolyRef));

  // Store visited
  for i := 0 to req - 1 do
    path[i] := visited[i];

  Result := req+size;
end;

(**
@class dtPathCorridor
@par

The corridor is loaded with a path, usually obtained from a #dtNavMeshQuery::findPath() query. The corridor
is then used to plan local movement, with the corridor automatically updating as needed to deal with inaccurate
agent locomotion.

Example of a common use case:

-# Construct the corridor object and call #init() to allocate its path buffer.
-# Obtain a path from a #dtNavMeshQuery object.
-# Use #reset() to set the agent's current position. (At the beginning of the path.)
-# Use #setCorridor() to load the path and target.
-# Use #findCorners() to plan movement. (This handles dynamic path straightening.)
-# Use #movePosition() to feed agent movement back into the corridor. (The corridor will automatically adjust as needed.)
-# If the target is moving, use #moveTargetPosition() to update the end of the corridor.
   (The corridor will automatically adjust as needed.)
-# Repeat the previous 3 steps to continue to move the agent.

The corridor position and target are always constrained to the navigation mesh.

One of the difficulties in maintaining a path is that floating point errors, locomotion inaccuracies, and/or local
steering can result in the agent crossing the boundary of the path corridor, temporarily invalidating the path.
This class uses local mesh queries to detect and update the corridor as needed to handle these types of issues.

The fact that local mesh queries are used to move the position and target locations results in two beahviors that
need to be considered:

Every time a move function is used there is a chance that the path will become non-optimial. Basically, the further
the target is moved from its original location, and the further the position is moved outside the original corridor,
the more likely the path will become non-optimal. This issue can be addressed by periodically running the
#optimizePathTopology() and #optimizePathVisibility() methods.

All local mesh queries have distance limitations. (Review the #dtNavMeshQuery methods for details.) So the most accurate
use case is to move the position and target in small increments. If a large increment is used, then the corridor
may not be able to accurately find the new location.  Because of this limiation, if a position is moved in a large
increment, then compare the desired and resulting polygon references. If the two do not match, then path replanning
may be needed.  E.g. If you move the target, check #getLastPoly() to see if it is the expected polygon.

*)

constructor TdtPathCorridor.Create;
begin
  inherited;
  //m_path(0),
  //m_npath(0),
  //m_maxPath(0)
end;

destructor TdtPathCorridor.Destroy;
begin
  FreeMem(m_path);
  inherited;
end;

/// @par
///
/// @warning Cannot be called more than once.
function TdtPathCorridor.init(const maxPath: Integer): Boolean;
begin
  Assert(m_path = nil);
  GetMem(m_path, sizeof(TdtPolyRef)*maxPath);
  m_npath := 0;
  m_maxPath := maxPath;
  Result := true;
end;

/// @par
///
/// Essentially, the corridor is set of one polygon in size with the target
/// equal to the position.
procedure TdtPathCorridor.reset(ref: TdtPolyRef; const pos: PSingle);
begin
  Assert(m_path <> nil);
  dtVcopy(@m_pos[0], pos);
  dtVcopy(@m_target[0], pos);
  m_path[0] := ref;
  m_npath := 1;
end;

(**
@par

This is the function used to plan local movement within the corridor. One or more corners can be 
detected in order to plan movement. It performs essentially the same function as #dtNavMeshQuery::findStraightPath.

Due to internal optimizations, the maximum number of corners returned will be (@p maxCorners - 1)
For example: If the buffers are sized to hold 10 corners, the function will never return more than 9 corners. 
So if 10 corners are needed, the buffers should be sized for 11 corners.

If the target is within range, it will be the last corner and have a polygon reference id of zero.
*)
function TdtPathCorridor.findCorners(cornerVerts: PSingle; cornerFlags: PByte;
            cornerPolys: PdtPolyRef; const maxCorners: Integer;
            navquery: TdtNavMeshQuery; const filter: TdtQueryFilter): Integer;
const MIN_TARGET_DIST = 0.01;
var ncorners, i: Integer;
begin
  Assert(m_path <> nil);
  Assert(m_npath <> 0);

  ncorners := 0;
  navquery.findStraightPath(@m_pos[0], @m_target[0], m_path, m_npath,
                 cornerVerts, cornerFlags, cornerPolys, @ncorners, maxCorners);
  
  // Prune points in the beginning of the path which are too close.
  while (ncorners <> 0) do
  begin
    if ((cornerFlags[0] and Byte(DT_STRAIGHTPATH_OFFMESH_CONNECTION)) <> 0) or
      (dtVdist2DSqr(@cornerVerts[0], @m_pos[0]) > Sqr(MIN_TARGET_DIST)) then
      break;
    Dec(ncorners);
    if (ncorners <> 0) then
    begin
      Move(cornerFlags[1], cornerFlags^, sizeof(Byte)*ncorners);
      Move(cornerPolys[1], cornerPolys^, sizeof(TdtPolyRef)*ncorners);
      Move(cornerVerts[3], cornerVerts^, sizeof(Single)*3*ncorners);
    end;
  end;
  
  // Prune points after an off-mesh connection.
  for i := 0 to ncorners - 1 do
  begin
    if (cornerFlags[i] and Byte(DT_STRAIGHTPATH_OFFMESH_CONNECTION)) <> 0 then
    begin
      ncorners := i+1;
      break;
    end;
  end;
  
  Result := ncorners;
end;

(**
@par

Inaccurate locomotion or dynamic obstacle avoidance can force the argent position significantly outside the 
original corridor. Over time this can result in the formation of a non-optimal corridor. Non-optimal paths can 
also form near the corners of tiles.

This function uses an efficient local visibility search to try to optimize the corridor 
between the current position and @p next.

The corridor will change only if @p next is visible from the current position and moving directly toward the point
is better than following the existing path.

The more inaccurate the agent movement, the more beneficial this function becomes. Simply adjust the frequency
of the call to match the needs to the agent.

This function is not suitable for long distance searches.
*)
procedure TdtPathCorridor.optimizePathVisibility(const next: PSingle; const pathOptimizationRange: Single;
                  navquery: TdtNavMeshQuery; const filter: TdtQueryFilter);
const MAX_RES = 32;
var goal, delta, norm: array [0..2] of Single; dist, t: Single; res: array [0..MAX_RES-1] of TdtPolyRef; nres: Integer;
begin
  Assert(m_path <> nil);

  // Clamp the ray to max distance.
  dtVcopy(@goal[0], next);
  dist := dtVdist2D(@m_pos[0], @goal[0]);

  // If too close to the goal, do not try to optimize.
  if (dist < 0.01) then
    Exit;

  // Overshoot a little. This helps to optimize open fields in tiled meshes.
  dist := dtMin(dist+0.01, pathOptimizationRange);

  // Adjust ray length.
  dtVsub(@delta[0], @goal[0], @m_pos[0]);
  dtVmad(@goal[0], @m_pos[0], @delta[0], pathOptimizationRange/dist);

  nres := 0;
  navquery.raycast(m_path[0], @m_pos[0], @goal[0], filter, @t, @norm[0], @res[0], @nres, MAX_RES);
  if (nres > 1) and (t > 0.99) then
  begin
    m_npath := dtMergeCorridorStartShortcut(m_path, m_npath, m_maxPath, @res[0], nres);
  end;
end;

(**
@par

Inaccurate locomotion or dynamic obstacle avoidance can force the agent position significantly outside the
original corridor. Over time this can result in the formation of a non-optimal corridor. This function will use a
local area path search to try to re-optimize the corridor.

The more inaccurate the agent movement, the more beneficial this function becomes. Simply adjust the frequency of 
the call to match the needs to the agent.
*)
function TdtPathCorridor.optimizePathTopology(navquery: TdtNavMeshQuery; const filter: TdtQueryFilter): Boolean;
const MAX_ITER = 32;
const MAX_RES = 32;
var res: array [0..MAX_RES-1] of TdtPolyRef; nres: Integer; status: TdtStatus;
begin
  Assert(navquery <> nil);
  Assert(filter <> nil);
  Assert(m_path <> nil);

  if (m_npath < 3) then
    Exit(false);

  nres := 0;
  navquery.initSlicedFindPath(m_path[0], m_path[m_npath-1], @m_pos[0], @m_target[0], filter);
  navquery.updateSlicedFindPath(MAX_ITER, nil);
  status := navquery.finalizeSlicedFindPathPartial(m_path, m_npath, @res[0], @nres, MAX_RES);

  if dtStatusSucceed(status) and (nres > 0) then
  begin
    m_npath := dtMergeCorridorStartShortcut(m_path, m_npath, m_maxPath, @res[0], nres);
    Exit(true);
  end;

  Result := false;
end;

function TdtPathCorridor.moveOverOffmeshConnection(offMeshConRef: TdtPolyRef; refs: PdtPolyRef;
                     startPos, endPos: PSingle;
                     navquery: TdtNavMeshQuery): Boolean;
var prevRef, polyRef: TdtPolyRef; npos, i: Integer; nav: TdtNavMesh; status: TdtStatus;
begin
  Assert(navquery <> nil);
  Assert(m_path <> nil);
  Assert(m_npath <> 0);

  // Advance the path up to and over the off-mesh connection.
  prevRef := 0; polyRef := m_path[0];
  npos := 0;
  while (npos < m_npath) and (polyRef <> offMeshConRef) do
  begin
    prevRef := polyRef;
    polyRef := m_path[npos];
    Inc(npos);
  end;
  if (npos = m_npath) then
  begin
    // Could not find offMeshConRef
    Exit(false);
  end;

  // Prune path
  for i := npos to m_npath - 1 do
    m_path[i-npos] := m_path[i];
  Dec(m_npath, npos);

  refs[0] := prevRef;
  refs[1] := polyRef;

  nav := navquery.getAttachedNavMesh;
  Assert(nav <> nil);

  status := nav.getOffMeshConnectionPolyEndPoints(refs[0], refs[1], startPos, endPos);
  if (dtStatusSucceed(status)) then
  begin
    dtVcopy(@m_pos[0], endPos);
    Exit(true);
  end;

  Result := false;
end;

(**
@par

Behavior:

- The movement is constrained to the surface of the navigation mesh.
- The corridor is automatically adjusted (shorted or lengthened) in order to remain valid.
- The new position will be located in the adjusted corridor's first polygon.

The expected use case is that the desired position will be 'near' the current corridor. What is considered 'near'
depends on local polygon density, query search extents, etc.

The resulting position will differ from the desired position if the desired position is not on the navigation mesh,
or it can't be reached using a local search.
*)
function TdtPathCorridor.movePosition(const npos: PSingle; navquery: TdtNavMeshQuery; const filter: TdtQueryFilter): Boolean;
const MAX_VISITED = 16;
var reslt: array [0..2] of Single; visited: array [0..MAX_VISITED-1] of TdtPolyRef; nvisited: Integer; status: TdtStatus;
h: Single;
begin
  Assert(m_path <> nil);
  Assert(m_npath <> 0);

  // Move along navmesh and update new position.
  nvisited := 0;
  status := navquery.moveAlongSurface(m_path[0], @m_pos[0], npos, filter,
                         @reslt[0], @visited[0], @nvisited, MAX_VISITED);
  if (dtStatusSucceed(status)) then
  begin
    m_npath := dtMergeCorridorStartMoved(m_path, m_npath, m_maxPath, @visited[0], nvisited);

    // Adjust the position to stay on top of the navmesh.
    h := m_pos[1];
    navquery.getPolyHeight(m_path[0], @reslt[0], @h);
    reslt[1] := h;
    dtVcopy(@m_pos[0], @reslt[0]);
    Exit(true);
  end;
  Result := false;
end;

(**
@par

Behavior:

- The movement is constrained to the surface of the navigation mesh. 
- The corridor is automatically adjusted (shorted or lengthened) in order to remain valid. 
- The new target will be located in the adjusted corridor's last polygon.

The expected use case is that the desired target will be 'near' the current corridor. What is considered 'near' depends on local polygon density, query search extents, etc.

The resulting target will differ from the desired target if the desired target is not on the navigation mesh, or it can't be reached using a local search.
*)
function TdtPathCorridor.moveTargetPosition(const npos: PSingle; navquery: TdtNavMeshQuery; const filter: TdtQueryFilter): Boolean;
const MAX_VISITED = 16;
var reslt: array [0..2] of Single; visited: array [0..MAX_VISITED-1] of TdtPolyRef; nvisited: Integer; status: TdtStatus;
begin
  Assert(m_path <> nil);
  Assert(m_npath <> 0);

  // Move along navmesh and update new position.
  nvisited := 0;
  status := navquery.moveAlongSurface(m_path[m_npath-1], @m_target[0], npos, filter,
                         @reslt[0], @visited[0], @nvisited, MAX_VISITED);
  if (dtStatusSucceed(status)) then
  begin
    m_npath := dtMergeCorridorEndMoved(m_path, m_npath, m_maxPath, @visited[0], nvisited);
    // TODO: should we do that?
    // Adjust the position to stay on top of the navmesh.
    (*  float h := m_target[1];
     navquery.getPolyHeight(m_path[m_npath-1], reslt, &h);
     reslt[1] := h;*)

    dtVcopy(@m_target[0], @reslt[0]);

    Exit(true);
  end;
  Result := false;
end;

/// @par
///
/// The current corridor position is expected to be within the first polygon in the path. The target 
/// is expected to be in the last polygon. 
/// 
/// @warning The size of the path must not exceed the size of corridor's path buffer set during #init().
procedure TdtPathCorridor.setCorridor(const target: PSingle; const path: PdtPolyRef; const npath: Integer);
begin
  Assert(m_path <> nil);
  Assert(npath > 0);
  Assert(npath < m_maxPath);
  
  dtVcopy(@m_target[0], target);
  Move(path^, m_path^, sizeof(TdtPolyRef)*npath);
  m_npath := npath;
end;

function TdtPathCorridor.fixPathStart(safeRef: TdtPolyRef; const safePos: PSingle): Boolean;
begin
  Assert(m_path <> nil);

  dtVcopy(@m_pos[0], safePos);
  if (m_npath < 3) and (m_npath > 0) then
  begin
    m_path[2] := m_path[m_npath-1];
    m_path[0] := safeRef;
    m_path[1] := 0;
    m_npath := 3;
  end
  else
  begin
    m_path[0] := safeRef;
    m_path[1] := 0;
  end;

  Result := true;
end;

function TdtPathCorridor.trimInvalidPath(safeRef: TdtPolyRef; const safePos: PSingle;
               navquery: TdtNavMeshQuery; const filter: TdtQueryFilter): Boolean;
var n: Integer; tgt: array [0..2] of Single;
begin
  Assert(navquery <> nil);
  Assert(filter <> nil);
  Assert(m_path <> nil);

  // Keep valid path as far as possible.
  n := 0;
  while (n < m_npath) and (navquery.isValidPolyRef(m_path[n], filter)) do
  begin
    Inc(n);
  end;

  if (n = m_npath) then
  begin
    // All valid, no need to fix.
    Exit(true);
  end
  else if (n = 0) then
  begin
    // The first polyref is bad, use current safe values.
    dtVcopy(@m_pos[0], safePos);
    m_path[0] := safeRef;
    m_npath := 1;
  end
  else
  begin
    // The path is partially usable.
    m_npath := n;
  end;

  // Clamp target pos to last poly
  dtVcopy(@tgt[0], @m_target[0]);
  navquery.closestPointOnPolyBoundary(m_path[m_npath-1], @tgt[0], @m_target[0]);

  Result := true;
end;

/// @par
///
/// The path can be invalidated if there are structural changes to the underlying navigation mesh, or the state of
/// a polygon within the path changes resulting in it being filtered out. (E.g. An exclusion or inclusion flag changes.)
function TdtPathCorridor.isValid(const maxLookAhead: Integer; navquery: TdtNavMeshQuery; const filter: TdtQueryFilter): Boolean;
var n, i: Integer;
begin
  // Check that all polygons still pass query filter.
  n := dtMin(m_npath, maxLookAhead);
  for i := 0 to n - 1 do
  begin
    if (not navquery.isValidPolyRef(m_path[i], filter)) then
      Exit(false);
  end;

  Result := true;
end;

function TdtPathCorridor.getPos(): PSingle; begin Result := @m_pos[0]; end;
function TdtPathCorridor.getTarget(): PSingle; begin Result := @m_target[0]; end;
function TdtPathCorridor.getFirstPoly(): TdtPolyRef; begin if m_npath <> 0 then Result := m_path[0] else Result := 0; end;
function TdtPathCorridor.getLastPoly(): TdtPolyRef; begin if m_npath <> 0 then Result := m_path[m_npath-1] else Result := 0; end;
function TdtPathCorridor.getPath(): PdtPolyRef; begin Result := m_path; end;
function TdtPathCorridor.getPathCount(): Integer; begin Result := m_npath; end;

end.
