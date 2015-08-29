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

unit RN_DetourCrowd;
interface
uses Math, SysUtils, RN_DetourNavMesh, RN_DetourNavMeshHelper, RN_DetourNavMeshQuery, RN_DetourObstacleAvoidance,
  RN_DetourLocalBoundary, RN_DetourPathCorridor, RN_DetourProximityGrid, RN_DetourPathQueue;

/// The maximum number of neighbors that a crowd agent can take into account
/// for steering decisions.
/// @ingroup crowd
const DT_CROWDAGENT_MAX_NEIGHBOURS = 6;

/// The maximum number of corners a crowd agent will look ahead in the path.
/// This value is used for sizing the crowd agent corner buffers.
/// Due to the behavior of the crowd manager, the actual number of useful
/// corners will be one less than this number.
/// @ingroup crowd
const DT_CROWDAGENT_MAX_CORNERS = 4;

/// The maximum number of crowd avoidance configurations supported by the
/// crowd manager.
/// @ingroup crowd
/// @see dtObstacleAvoidanceParams, dtCrowd::setObstacleAvoidanceParams(), dtCrowd::getObstacleAvoidanceParams(),
///     dtCrowdAgentParams::obstacleAvoidanceType
const DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS = 8;

/// The maximum number of query filter types supported by the crowd manager.
/// @ingroup crowd
/// @see dtQueryFilter, dtCrowd::getFilter() dtCrowd::getEditableFilter(),
///    dtCrowdAgentParams::queryFilterType
const DT_CROWD_MAX_QUERY_FILTER_TYPE = 16;

/// Provides neighbor data for agents managed by the crowd.
/// @ingroup crowd
/// @see dtCrowdAgent::neis, dtCrowd
type
  PdtCrowdNeighbour = ^TdtCrowdNeighbour;
  TdtCrowdNeighbour = record
    idx: Integer;    ///< The index of the neighbor in the crowd.
    dist: Single;    ///< The distance between the current agent and the neighbor.
  end;

  /// The type of navigation mesh polygon the agent is currently traversing.
  /// @ingroup crowd
  TCrowdAgentState =
  (
    DT_CROWDAGENT_STATE_INVALID,    ///< The agent is not in a valid state.
    DT_CROWDAGENT_STATE_WALKING,    ///< The agent is traversing a normal navigation mesh polygon.
    DT_CROWDAGENT_STATE_OFFMESH    ///< The agent is traversing an off-mesh connection.
  );

  /// Configuration parameters for a crowd agent.
  /// @ingroup crowd
  PdtCrowdAgentParams = ^TdtCrowdAgentParams;
  TdtCrowdAgentParams = record
    radius: Single;            ///< Agent radius. [Limit: >= 0]
    height: Single;            ///< Agent height. [Limit: > 0]
    maxAcceleration: Single;        ///< Maximum allowed acceleration. [Limit: >= 0]
    maxSpeed: Single;            ///< Maximum allowed speed. [Limit: >= 0]

    /// Defines how close a collision element must be before it is considered for steering behaviors. [Limits: > 0]
    collisionQueryRange: Single;

    pathOptimizationRange: Single;    ///< The path visibility optimization range. [Limit: > 0]

    /// How aggresive the agent manager should be at avoiding collisions with this agent. [Limit: >= 0]
    separationWeight: Single;

    /// Flags that impact steering behavior. (See: #UpdateFlags)
    updateFlags: Byte;

    /// The index of the avoidance configuration to use for the agent.
    /// [Limits: 0 <= value <= #DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS]
    obstacleAvoidanceType: Byte;

    /// The index of the query filter used by this agent.
    queryFilterType: Byte;

    /// User defined data attached to the agent.
    userData: Pointer;
  end;

  TMoveRequestState =
  (
    DT_CROWDAGENT_TARGET_NONE = 0,
    DT_CROWDAGENT_TARGET_FAILED,
    DT_CROWDAGENT_TARGET_VALID,
    DT_CROWDAGENT_TARGET_REQUESTING,
    DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE,
    DT_CROWDAGENT_TARGET_WAITING_FOR_PATH,
    DT_CROWDAGENT_TARGET_VELOCITY
  );

  /// Represents an agent managed by a #dtCrowd object.
  /// @ingroup crowd
  PPdtCrowdAgent = ^PdtCrowdAgent;
  PdtCrowdAgent = ^TdtCrowdAgent;
  TdtCrowdAgent = record
    /// True if the agent is active, false if the agent is in an unused slot in the agent pool.
    active: Boolean;

    /// The type of mesh polygon the agent is traversing. (See: #CrowdAgentState)
    state: TCrowdAgentState;

    /// True if the agent has valid path (targetState == DT_CROWDAGENT_TARGET_VALID) and the path does not lead to the requested position, else false.
    partial: Boolean;

    /// The path corridor the agent is using.
    corridor: TdtPathCorridor;

    /// The local boundary data for the agent.
    boundary: TdtLocalBoundary;

    /// Time since the agent's path corridor was optimized.
    topologyOptTime: Single;

    /// The known neighbors of the agent.
    neis: array [0..DT_CROWDAGENT_MAX_NEIGHBOURS-1] of TdtCrowdNeighbour;

    /// The number of neighbors.
    nneis: Integer;
  
    /// The desired speed.
    desiredSpeed: Single;

    npos: array [0..2] of Single;    ///< The current agent position. [(x, y, z)]
    disp: array [0..2] of Single;
    dvel: array [0..2] of Single;    ///< The desired velocity of the agent. [(x, y, z)]
    nvel: array [0..2] of Single;
    vel: array [0..2] of Single;    ///< The actual velocity of the agent. [(x, y, z)]

    /// The agent's configuration parameters.
    params: TdtCrowdAgentParams;

    /// The local path corridor corners for the agent. (Staight path.) [(x, y, z) * #ncorners]
    cornerVerts: array [0..DT_CROWDAGENT_MAX_CORNERS*3-1] of Single;

    /// The local path corridor corner flags. (See: #dtStraightPathFlags) [(flags) * #ncorners]
    cornerFlags: array [0..DT_CROWDAGENT_MAX_CORNERS-1] of Byte;

    /// The reference id of the polygon being entered at the corner. [(polyRef) * #ncorners]
    cornerPolys: array [0..DT_CROWDAGENT_MAX_CORNERS-1] of TdtPolyRef;

    /// The number of corners.
    ncorners: Integer;
  
    targetState: TMoveRequestState;      ///< State of the movement request.
    targetRef: TdtPolyRef;        ///< Target polyref of the movement request.
    targetPos: array [0..2] of Single;          ///< Target position of the movement request (or velocity in case of DT_CROWDAGENT_TARGET_VELOCITY).
    targetPathqRef: TdtPathQueueRef;    ///< Path finder ref.
    targetReplan: Boolean;          ///< Flag indicating that the current path is being replanned.
    targetReplanTime: Single;        /// <Time since the agent's target was replanned.
  end;

  PdtCrowdAgentAnimation = ^TdtCrowdAgentAnimation;
  TdtCrowdAgentAnimation = record
    active: Boolean;
    initPos, startPos, endPos: array [0..2] of Single;
    polyRef: TdtPolyRef;
    t, tmax: Single;
  end;

  /// Crowd agent update flags.
  /// @ingroup crowd
  /// @see dtCrowdAgentParams::updateFlags
const
    DT_CROWD_ANTICIPATE_TURNS = 1;
    DT_CROWD_OBSTACLE_AVOIDANCE = 2;
    DT_CROWD_SEPARATION = 4;
    DT_CROWD_OPTIMIZE_VIS = 8;      ///< Use #dtPathCorridor::optimizePathVisibility() to optimize the agent path.
    DT_CROWD_OPTIMIZE_TOPO = 16;    ///< Use dtPathCorridor::optimizePathTopology() to optimize the agent path.

type
  PdtCrowdAgentDebugInfo = ^TdtCrowdAgentDebugInfo;
  TdtCrowdAgentDebugInfo = record
    idx: Integer;
    optStart, optEnd: array [0..2] of Single;
    vod: TdtObstacleAvoidanceDebugData;
  end;

  /// Provides local steering behaviors for a group of agents.
  /// @ingroup crowd
  TdtCrowd = class
  private
    m_maxAgents: Integer;
    m_agents: PdtCrowdAgent;
    m_activeAgents: PPdtCrowdAgent;
    m_agentAnims: PdtCrowdAgentAnimation;

    m_pathq: TdtPathQueue;

    m_obstacleQueryParams: array [0..DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS-1] of TdtObstacleAvoidanceParams;
    m_obstacleQuery: TdtObstacleAvoidanceQuery;

    m_grid: TdtProximityGrid;

    m_pathResult: PdtPolyRef;
    m_maxPathResult: Integer;

    m_ext: array [0..2] of Single;

    m_filters: array [0..DT_CROWD_MAX_QUERY_FILTER_TYPE-1] of TdtQueryFilter;

    m_maxAgentRadius: Single;

    m_velocitySampleCount: Integer;

    m_navquery: TdtNavMeshQuery;

    procedure updateTopologyOptimization(agents: PPdtCrowdAgent; const nagents: Integer; const dt: Single);
    procedure updateMoveRequest(const dt: Single);
    procedure checkPathValidity(agents: PPdtCrowdAgent; const nagents: Integer; const dt: Single);

    function getAgentIndex(agent: PdtCrowdAgent): Integer; { return (int)(agent - m_agents); }

    function requestMoveTargetReplan(const idx: Integer; ref: TdtPolyRef; const pos: PSingle): Boolean;

    procedure purge();

  public
    constructor Create;
    destructor Destroy; override;

    /// Initializes the crowd.
    ///  @param[in]    maxAgents    The maximum number of agents the crowd can manage. [Limit: >= 1]
    ///  @param[in]    maxAgentRadius  The maximum radius of any agent that will be added to the crowd. [Limit: > 0]
    ///  @param[in]    nav        The navigation mesh to use for planning.
    /// @return True if the initialization succeeded.
    function init(const maxAgents: Integer; const maxAgentRadius: Single; nav: TdtNavMesh): Boolean;
  
    /// Sets the shared avoidance configuration for the specified index.
    ///  @param[in]    idx    The index. [Limits: 0 <= value < #DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS]
    ///  @param[in]    params  The new configuration.
    procedure setObstacleAvoidanceParams(const idx: Integer; const params: PdtObstacleAvoidanceParams);

    /// Gets the shared avoidance configuration for the specified index.
    ///  @param[in]    idx    The index of the configuration to retreive.
    ///              [Limits:  0 <= value < #DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS]
    /// @return The requested configuration.
    function getObstacleAvoidanceParams(const idx: Integer): PdtObstacleAvoidanceParams;

    /// Gets the specified agent from the pool.
    ///   @param[in]    idx    The agent index. [Limits: 0 <= value < #getAgentCount()]
    /// @return The requested agent.
    function getAgent(const idx: Integer): PdtCrowdAgent;

    /// Gets the specified agent from the pool.
    ///   @param[in]    idx    The agent index. [Limits: 0 <= value < #getAgentCount()]
    /// @return The requested agent.
    function getEditableAgent(const idx: Integer): PdtCrowdAgent;

    /// The maximum number of agents that can be managed by the object.
    /// @return The maximum number of agents.
    function getAgentCount(): Integer;
  
    /// Adds a new agent to the crowd.
    ///  @param[in]    pos    The requested position of the agent. [(x, y, z)]
    ///  @param[in]    params  The configutation of the agent.
    /// @return The index of the agent in the agent pool. Or -1 if the agent could not be added.
    function addAgent(const pos: PSingle; const params: PdtCrowdAgentParams): Integer;

    /// Updates the specified agent's configuration.
    ///  @param[in]    idx    The agent index. [Limits: 0 <= value < #getAgentCount()]
    ///  @param[in]    params  The new agent configuration.
    procedure updateAgentParameters(const idx: Integer; const params: PdtCrowdAgentParams);

    /// Removes the agent from the crowd.
    ///  @param[in]    idx    The agent index. [Limits: 0 <= value < #getAgentCount()]
    procedure removeAgent(const idx: Integer);
  
    /// Submits a new move request for the specified agent.
    ///  @param[in]    idx    The agent index. [Limits: 0 <= value < #getAgentCount()]
    ///  @param[in]    ref    The position's polygon reference.
    ///  @param[in]    pos    The position within the polygon. [(x, y, z)]
    /// @return True if the request was successfully submitted.
    function requestMoveTarget(const idx: Integer; ref: TdtPolyRef; const pos: PSingle): Boolean;

    /// Submits a new move request for the specified agent.
    ///  @param[in]    idx    The agent index. [Limits: 0 <= value < #getAgentCount()]
    ///  @param[in]    vel    The movement velocity. [(x, y, z)]
    /// @return True if the request was successfully submitted.
    function requestMoveVelocity(const idx: Integer; const vel: PSingle): Boolean;

    /// Resets any request for the specified agent.
    ///  @param[in]    idx    The agent index. [Limits: 0 <= value < #getAgentCount()]
    /// @return True if the request was successfully reseted.
    function resetMoveTarget(const idx: Integer): Boolean;

    /// Gets the active agents int the agent pool.
    ///  @param[out]  agents    An array of agent pointers. [(#dtCrowdAgent *) * maxAgents]
    ///  @param[in]    maxAgents  The size of the crowd agent array.
    /// @return The number of agents returned in @p agents.
    function getActiveAgents(agents: PPdtCrowdAgent; const maxAgents: Integer): Integer;

    /// Updates the steering and positions of all agents.
    ///  @param[in]    dt    The time, in seconds, to update the simulation. [Limit: > 0]
    ///  @param[out]  debug  A debug object to load with debug information. [Opt]
    procedure update(const dt: Single; debug: PdtCrowdAgentDebugInfo);

    /// Gets the filter used by the crowd.
    /// @return The filter used by the crowd.
    function getFilter(const i: Integer): TdtQueryFilter; { return (i >= 0 && i < DT_CROWD_MAX_QUERY_FILTER_TYPE) ? &m_filters[i] : 0; }

    /// Gets the filter used by the crowd.
    /// @return The filter used by the crowd.
    function getEditableFilter(const i: Integer): TdtQueryFilter; { return (i >= 0 && i < DT_CROWD_MAX_QUERY_FILTER_TYPE) ? &m_filters[i] : 0; }

    /// Gets the search extents [(x, y, z)] used by the crowd for query operations.
    /// @return The search extents used by the crowd. [(x, y, z)]
    function getQueryExtents(): PSingle; { return m_ext; }

    /// Gets the velocity sample count.
    /// @return The velocity sample count.
    property getVelocitySampleCount: Integer read m_velocitySampleCount;

    /// Gets the crowd's proximity grid.
    /// @return The crowd's proximity grid.
    property getGrid: TdtProximityGrid read m_grid;

    /// Gets the crowd's path request queue.
    /// @return The crowd's path request queue.
    property getPathQueue: TdtPathQueue read m_pathq;

    /// Gets the query object used by the crowd.
    property getNavMeshQuery: TdtNavMeshQuery read m_navquery;
  end;

  /// Allocates a crowd object using the Detour allocator.
  /// @return A crowd object that is ready for initialization, or null on failure.
  ///  @ingroup crowd
  function dtAllocCrowd(): TdtCrowd;

  /// Frees the specified crowd object using the Detour allocator.
  ///  @param[in]    ptr    A crowd object allocated using #dtAllocCrowd
  ///  @ingroup crowd
  procedure dtFreeCrowd(var ptr: TdtCrowd);


  //Delphi. Added to be accessible from outside this unit, for debug render
  procedure calcSmoothSteerDirection(const ag: PdtCrowdAgent; dir: PSingle);

///////////////////////////////////////////////////////////////////////////

// This section contains detailed documentation for members that don't have
// a source file. It reduces clutter in the main section of the header.

(**

@defgroup crowd Crowd

Members in this module implement local steering and dynamic avoidance features.

The crowd is the big beast of the navigation features. It not only handles a 
lot of the path management for you, but also local steering and dynamic 
avoidance between members of the crowd. I.e. It can keep your agents from 
running into each other.

Main class: #dtCrowd

The #dtNavMeshQuery and #dtPathCorridor classes provide perfectly good, easy 
to use path planning features. But in the end they only give you points that 
your navigation client should be moving toward. When it comes to deciding things
like agent velocity and steering to avoid other agents, that is up to you to 
implement. Unless, of course, you decide to use #dtCrowd.

Basically, you add an agent to the crowd, providing various configuration
settings such as maximum speed and acceleration. You also provide a local
target to more toward. The crowd manager then provides, with every update, the
new agent position and velocity for the frame. The movement will be
constrained to the navigation mesh, and steering will be applied to ensure
agents managed by the crowd do not collide with each other.

This is very powerful feature set. But it comes with limitations.

The biggest limitation is that you must give control of the agent's position
completely over to the crowd manager. You can update things like maximum speed
and acceleration. But in order for the crowd manager to do its thing, it can't
allow you to constantly be giving it overrides to position and velocity. So
you give up direct control of the agent's movement. It belongs to the crowd.

The second biggest limitation revolves around the fact that the crowd manager
deals with local planning. So the agent's target should never be more than
256 polygons aways from its current position. If it is, you risk
your agent failing to reach its target. So you may still need to do long
distance planning and provide the crowd manager with intermediate targets.

Other significant limitations:

- All agents using the crowd manager will use the same #dtQueryFilter.
- Crowd management is relatively expensive. The maximum agents under crowd
  management at any one time is between 20 and 30.  A good place to start
  is a maximum of 25 agents for 0.5ms per frame.

@note This is a summary list of members.  Use the index or search
feature to find minor members.

@struct dtCrowdAgentParams
@see dtCrowdAgent, dtCrowd::addAgent(), dtCrowd::updateAgentParameters()

@var dtCrowdAgentParams::obstacleAvoidanceType
@par

#dtCrowd permits agents to use different avoidance configurations.  This value
is the index of the #dtObstacleAvoidanceParams within the crowd.

@see dtObstacleAvoidanceParams, dtCrowd::setObstacleAvoidanceParams(),
   dtCrowd::getObstacleAvoidanceParams()

@var dtCrowdAgentParams::collisionQueryRange
@par

Collision elements include other agents and navigation mesh boundaries.

This value is often based on the agent radius and/or maximum speed. E.g. radius * 8

@var dtCrowdAgentParams::pathOptimizationRange
@par

Only applicalbe if #updateFlags includes the #DT_CROWD_OPTIMIZE_VIS flag.

This value is often based on the agent radius. E.g. radius * 30

@see dtPathCorridor::optimizePathVisibility()

@var dtCrowdAgentParams::separationWeight
@par

A higher value will result in agents trying to stay farther away from each other at
the cost of more difficult steering in tight spaces.

*)

implementation
uses RN_DetourCommon, RN_DetourStatus;


function dtAllocCrowd(): TdtCrowd;
begin
  Result := TdtCrowd.Create;
end;

procedure dtFreeCrowd(var ptr: TdtCrowd);
begin
  FreeAndNil(ptr);
end;


const MAX_ITERS_PER_UPDATE = 100;

const MAX_PATHQUEUE_NODES = 4096;
const MAX_COMMON_NODES = 512;

function tween(const t, t0, t1: Single): Single;
begin
  Result := dtClamp((t-t0) / (t1-t0), 0.0, 1.0);
end;

procedure integrate(ag: PdtCrowdAgent; const dt: Single);
var maxDelta, ds: Single; dv: array [0..2] of Single;
begin
  // Fake dynamic constraint.
  maxDelta := ag.params.maxAcceleration * dt;
  dtVsub(@dv[0], @ag.nvel[0], @ag.vel[0]);
  ds := dtVlen(@dv[0]);
  if (ds > maxDelta) then
    dtVscale(@dv[0], @dv[0], maxDelta/ds);
  dtVadd(@ag.vel[0], @ag.vel[0], @dv[0]);

  // Integrate
  if (dtVlen(@ag.vel[0]) > 0.0001) then
    dtVmad(@ag.npos[0], @ag.npos[0], @ag.vel[0], dt)
  else
    dtVset(@ag.vel[0],0,0,0);
end;

function overOffmeshConnection(const ag: PdtCrowdAgent; const radius: Single): Boolean;
var offMeshConnection: Boolean; distSq: Single;
begin
  if (ag.ncorners = 0) then
    Exit(false);

  offMeshConnection := (ag.cornerFlags[ag.ncorners-1] and Byte(DT_STRAIGHTPATH_OFFMESH_CONNECTION)) <> 0;
  if (offMeshConnection) then
  begin
    distSq := dtVdist2DSqr(@ag.npos[0], @ag.cornerVerts[(ag.ncorners-1)*3]);
    if (distSq < radius*radius) then
      Exit(true);
  end;

  Result := false;
end;

function getDistanceToGoal(const ag: PdtCrowdAgent; const range: Single): Single;
var endOfPath: Boolean;
begin
  if (ag.ncorners = 0) then
    Exit(range);

  endOfPath := (ag.cornerFlags[ag.ncorners-1] and Byte(DT_STRAIGHTPATH_END)) <> 0;
  if (endOfPath) then
    Exit(dtMin(dtVdist2D(@ag.npos[0], @ag.cornerVerts[(ag.ncorners-1)*3]), range));

  Result := range;
end;

procedure calcSmoothSteerDirection(const ag: PdtCrowdAgent; dir: PSingle);
var ip0, ip1: Integer; p0, p1: PSingle; dir0, dir1: array [0..2] of Single; len0, len1: Single;
begin
  if (ag.ncorners = 0) then
  begin
    dtVset(dir, 0,0,0);
    Exit;
  end;

  ip0 := 0;
  ip1 := dtMin(1, ag.ncorners-1);
  p0 := @ag.cornerVerts[ip0*3];
  p1 := @ag.cornerVerts[ip1*3];

  dtVsub(@dir0[0], p0, @ag.npos[0]);
  dtVsub(@dir1[0], p1, @ag.npos[0]);
  dir0[1] := 0;
  dir1[1] := 0;

  len0 := dtVlen(@dir0[0]);
  len1 := dtVlen(@dir1[0]);
  if (len1 > 0.001) then
    dtVscale(@dir1[0],@dir1[0],1.0/len1);
  
  dir[0] := dir0[0] - dir1[0]*len0*0.5;
  dir[1] := 0;
  dir[2] := dir0[2] - dir1[2]*len0*0.5;
  
  dtVnormalize(dir);
end;

procedure calcStraightSteerDirection(const ag: PdtCrowdAgent; dir: PSingle);
begin
  if (ag.ncorners = 0) then
  begin
    dtVset(dir, 0,0,0);
    Exit;
  end;
  dtVsub(dir, @ag.cornerVerts[0], @ag.npos[0]);
  dir[1] := 0;
  dtVnormalize(dir);
end;

function addNeighbour(const idx: Integer; const dist: Single;
            neis: PdtCrowdNeighbour; const nneis, maxNeis: Integer): Integer;
var nei: PdtCrowdNeighbour; i, tgt, n: Integer;
begin
  // Insert neighbour based on the distance.
  if (nneis = 0) then
  begin
    nei := @neis[nneis];
  end
  else if (dist >= neis[nneis-1].dist) then
  begin
    if (nneis >= maxNeis) then
      Exit(nneis);
    nei := @neis[nneis];
  end
  else
  begin
    for i := 0 to nneis - 1 do
      if (dist <= neis[i].dist) then
        break;

    tgt := i+1;
    n := dtMin(nneis-i, maxNeis-tgt);
    
    Assert(tgt+n <= maxNeis);

    if (n > 0) then
      Move(neis[i], neis[tgt], sizeof(TdtCrowdNeighbour)*n);
    nei := @neis[i];
  end;
  
  FillChar(nei^, sizeof(TdtCrowdNeighbour), 0);
  
  nei.idx := idx;
  nei.dist := dist;
  
  Result := dtMin(nneis+1, maxNeis);
end;

function getNeighbours(const pos: PSingle; const height, range: Single;
             const skip: PdtCrowdAgent; reslt: PdtCrowdNeighbour; const maxResult: Integer;
             agents: PPdtCrowdAgent; const nagents: Integer; grid: TdtProximityGrid): Integer;
const MAX_NEIS = 32;
var n, nids, i: Integer; ids: array [0..MAX_NEIS-1] of Word; ag: PdtCrowdAgent; diff: array [0..2] of Single; distSqr: Single;
begin
  n := 0;

  nids := grid.queryItems(pos[0]-range, pos[2]-range,
                pos[0]+range, pos[2]+range,
                @ids[0], MAX_NEIS);

  for i := 0 to nids - 1 do
  begin
    ag := agents[ids[i]];

    if (ag = skip) then continue;

    // Check for overlap.
    dtVsub(@diff[0], pos, @ag.npos[0]);
    if (Abs(diff[1]) >= (height+ag.params.height)/2.0) then
      continue;
    diff[1] := 0;
    distSqr := dtVlenSqr(@diff[0]);
    if (distSqr > Sqr(range)) then
      continue;

    n := addNeighbour(ids[i], distSqr, reslt, n, maxResult);
  end;
  Result := n;
end;

function addToOptQueue(newag: PdtCrowdAgent; agents: PPdtCrowdAgent; const nagents, maxAgents: Integer): Integer;
var slot, i, tgt, n: Integer;
begin
  // Insert neighbour based on greatest time.
  if (nagents = 0) then
  begin
    slot := nagents;
  end
  else if (newag.topologyOptTime <= agents[nagents-1].topologyOptTime) then
  begin
    if (nagents >= maxAgents) then
      Exit(nagents);
    slot := nagents;
  end
  else
  begin
    for i := 0 to nagents - 1 do
      if (newag.topologyOptTime >= agents[i].topologyOptTime) then
        break;
    
    tgt := i+1;
    n := dtMin(nagents-i, maxAgents-tgt);
    
    Assert(tgt+n <= maxAgents);
    
    if (n > 0) then
      Move(agents[i], agents[tgt], sizeof(PdtCrowdAgent)*n);
    slot := i;
  end;
  
  agents[slot] := newag;
  
  Result := dtMin(nagents+1, maxAgents);
end;

function addToPathQueue(newag: PdtCrowdAgent; agents: PPdtCrowdAgent; const nagents, maxAgents: Integer): Integer;
var slot, i, tgt, n: Integer;
begin
  // Insert neighbour based on greatest time.
  if (nagents = 0) then
  begin
    slot := nagents;
  end
  else if (newag.targetReplanTime <= agents[nagents-1].targetReplanTime) then
  begin
    if (nagents >= maxAgents) then
      Exit(nagents);
    slot := nagents;
  end
  else
  begin
    for i := 0 to nagents - 1 do
      if (newag.targetReplanTime >= agents[i].targetReplanTime) then
        break;

    tgt := i+1;
    n := dtMin(nagents-i, maxAgents-tgt);

    Assert(tgt+n <= maxAgents);

    if (n > 0) then
      Move(agents[i], agents[tgt], sizeof(PdtCrowdAgent)*n);
    slot := i;
  end;

  agents[slot] := newag;

  Result := dtMin(nagents+1, maxAgents);
end;


(**
@class dtCrowd
@par

This is the core class of the @ref crowd module.  See the @ref crowd documentation for a summary
of the crowd features.

A common method for setting up the crowd is as follows:

-# Allocate the crowd using #dtAllocCrowd.
-# Initialize the crowd using #init().
-# Set the avoidance configurations using #setObstacleAvoidanceParams().
-# Add agents using #addAgent() and make an initial movement request using #requestMoveTarget().

A common process for managing the crowd is as follows:

-# Call #update() to allow the crowd to manage its agents.
-# Retrieve agent information using #getActiveAgents().
-# Make movement requests using #requestMoveTarget() when movement goal changes.
-# Repeat every frame.

Some agent configuration settings can be updated using #updateAgentParameters().  But the crowd owns the
agent position.  So it is not possible to update an active agent's position.  If agent position
must be fed back into the crowd, the agent must be removed and re-added.

Notes: 

- Path related information is available for newly added agents only after an #update() has been
  performed.
- Agent objects are kept in a pool and re-used.  So it is important when using agent objects to check the value of
  #dtCrowdAgent::active to determine if the agent is actually in use or not.
- This class is meant to provide 'local' movement. There is a limit of 256 polygons in the path corridor.  
  So it is not meant to provide automatic pathfinding services over long distances.

@see dtAllocCrowd(), dtFreeCrowd(), init(), dtCrowdAgent

*)

constructor TdtCrowd.Create;
var i: Integer;
begin
  m_maxAgents := 0;
  m_agents := nil;
  m_activeAgents := nil;
  m_agentAnims := nil;
  m_obstacleQuery := nil;
  m_grid := nil;
  m_pathResult := nil;
  m_maxPathResult := 0;
  m_maxAgentRadius := 0;
  m_velocitySampleCount := 0;
  m_navquery := nil;

  m_pathq := TdtPathQueue.Create; // Delphi: Assume C++ invokes constructor
  for i := 0 to DT_CROWD_MAX_QUERY_FILTER_TYPE-1 do
    m_filters[i] := TdtQueryFilter.Create;
end;

destructor TdtCrowd.Destroy;
var i: Integer;
begin
  purge();
  m_pathq.Free; // Delphi: Assume C++ invokes destructor
  for i := 0 to DT_CROWD_MAX_QUERY_FILTER_TYPE-1 do
    m_filters[i].Free;
  inherited;
end;

procedure TdtCrowd.purge();
var i: Integer;
begin
  // Delphi: Release objects we have created in Init (see m_agents comments there)
  for i := 0 to m_maxAgents - 1 do
  begin
    FreeAndNil(m_agents[i].corridor);
    FreeAndNil(m_agents[i].boundary);
  end;
  FreeMem(m_agents);

  m_agents := nil;
  m_maxAgents := 0;

  FreeMem(m_activeAgents);
  m_activeAgents := nil;

  FreeMem(m_agentAnims);
  m_agentAnims := nil;

  FreeMem(m_pathResult);
  m_pathResult := nil;

  dtFreeProximityGrid(m_grid);
  m_grid := nil;

  dtFreeObstacleAvoidanceQuery(m_obstacleQuery);
  m_obstacleQuery := nil;

  dtFreeNavMeshQuery(m_navquery);
  m_navquery := nil;
end;

/// @par
///
/// May be called more than once to purge and re-initialize the crowd.
function TdtCrowd.init(const maxAgents: Integer; const maxAgentRadius: Single; nav: TdtNavMesh): Boolean;
var i: Integer; params: PdtObstacleAvoidanceParams;
begin
  purge();

  m_maxAgents := maxAgents;
  m_maxAgentRadius := maxAgentRadius;

  dtVset(@m_ext[0], m_maxAgentRadius*2.0,m_maxAgentRadius*1.5,m_maxAgentRadius*2.0);

  m_grid := dtAllocProximityGrid();
  if (m_grid = nil) then
    Exit(false);
  if (not m_grid.init(m_maxAgents*4, maxAgentRadius*3)) then
    Exit(false);

  m_obstacleQuery := dtAllocObstacleAvoidanceQuery();
  if (m_obstacleQuery = nil) then
    Exit(false);
  if (not m_obstacleQuery.init(6, 8)) then
    Exit(false);

  // Init obstacle query params.
  FillChar(m_obstacleQueryParams[0], sizeof(m_obstacleQueryParams), 0);
  for i := 0 to DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS - 1 do
  begin
    params := @m_obstacleQueryParams[i];
    params.velBias := 0.4;
    params.weightDesVel := 2.0;
    params.weightCurVel := 0.75;
    params.weightSide := 0.75;
    params.weightToi := 2.5;
    params.horizTime := 2.5;
    params.gridSize := 33;
    params.adaptiveDivs := 7;
    params.adaptiveRings := 2;
    params.adaptiveDepth := 5;
  end;

  // Allocate temp buffer for merging paths.
  m_maxPathResult := 256;
  GetMem(m_pathResult, sizeof(TdtPolyRef)*m_maxPathResult);

  if (not m_pathq.init(m_maxPathResult, MAX_PATHQUEUE_NODES, nav)) then
    Exit(false);

  GetMem(m_agents, sizeof(TdtCrowdAgent)*m_maxAgents);

  GetMem(m_activeAgents, sizeof(PdtCrowdAgent)*m_maxAgents);

  GetMem(m_agentAnims, sizeof(TdtCrowdAgentAnimation)*m_maxAgents);

  for i := 0 to m_maxAgents - 1 do
  begin
    // Delphi: Objects are auto-created in C++ struct, so I've been told and so does the code below expects
    m_agents[i].corridor := TdtPathCorridor.Create;
    m_agents[i].boundary := TdtLocalBoundary.Create;

    m_agents[i].active := false;
    if (not m_agents[i].corridor.init(m_maxPathResult)) then
      Exit(false);
  end;

  for i := 0 to m_maxAgents - 1 do
  begin
    m_agentAnims[i].active := false;
  end;

  // The navquery is mostly used for local searches, no need for large node pool.
  m_navquery := dtAllocNavMeshQuery();

  if (dtStatusFailed(m_navquery.init(nav, MAX_COMMON_NODES))) then
    Exit(false);

  Result := true;
end;

procedure TdtCrowd.setObstacleAvoidanceParams(const idx: Integer; const params: PdtObstacleAvoidanceParams);
begin
  if (idx >= 0) and (idx < DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS)then
    Move(params^, m_obstacleQueryParams[idx], sizeof(TdtObstacleAvoidanceParams));
end;

function TdtCrowd.getObstacleAvoidanceParams(const idx: Integer): PdtObstacleAvoidanceParams;
begin
  if (idx >= 0) and (idx < DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS) then
    Exit(@m_obstacleQueryParams[idx]);
  Result := nil;
end;

function TdtCrowd.getAgentCount(): Integer;
begin
  Result := m_maxAgents;
end;

/// @par
///
/// Agents in the pool may not be in use.  Check #dtCrowdAgent.active before using the returned object.
function TdtCrowd.getAgent(const idx: Integer): PdtCrowdAgent;
begin
  if (idx < 0) or (idx >= m_maxAgents) then
    Exit(nil);
  Result := @m_agents[idx];
end;

///
/// Agents in the pool may not be in use.  Check #dtCrowdAgent.active before using the returned object.
function TdtCrowd.getEditableAgent(const idx: Integer): PdtCrowdAgent;
begin
  if (idx < 0) or (idx >= m_maxAgents) then
    Exit(nil);
  Result := @m_agents[idx];
end;

procedure TdtCrowd.updateAgentParameters(const idx: Integer; const params: PdtCrowdAgentParams);
begin
  if (idx < 0) or (idx >= m_maxAgents) then
    Exit;
  Move(params^, m_agents[idx].params, sizeof(TdtCrowdAgentParams));
end;

/// @par
///
/// The agent's position will be constrained to the surface of the navigation mesh.
function TdtCrowd.addAgent(const pos: PSingle; const params: PdtCrowdAgentParams): Integer;
var idx, i: Integer; ag: PdtCrowdAgent; nearest: array [0..2] of Single; ref: TdtPolyRef; status: TdtStatus;
begin
  // Find empty slot.
  idx := -1;
  for i := 0 to m_maxAgents - 1 do
  begin
    if (not m_agents[i].active) then
    begin
      idx := i;
      break;
    end;
  end;
  if (idx = -1) then
    Exit(-1);

  ag := @m_agents[idx];

  updateAgentParameters(idx, params);

  // Find nearest position on navmesh and place the agent there.
  ref := 0;
  dtVcopy(@nearest[0], pos);
  status := m_navquery.findNearestPoly(pos, @m_ext[0], m_filters[ag.params.queryFilterType], @ref, @nearest[0]);
  if (dtStatusFailed(status)) then
  begin
    dtVcopy(@nearest[0], pos);
    ref := 0;
  end;
  
  ag.corridor.reset(ref, @nearest[0]);
  ag.boundary.reset();
  ag.partial := false;

  ag.topologyOptTime := 0;
  ag.targetReplanTime := 0;
  ag.nneis := 0;
  
  dtVset(@ag.dvel[0], 0,0,0);
  dtVset(@ag.nvel[0], 0,0,0);
  dtVset(@ag.vel[0], 0,0,0);
  dtVcopy(@ag.npos[0], @nearest[0]);

  ag.desiredSpeed := 0;

  if (ref <> 0) then
    ag.state := DT_CROWDAGENT_STATE_WALKING
  else
    ag.state := DT_CROWDAGENT_STATE_INVALID;

  ag.targetState := DT_CROWDAGENT_TARGET_NONE;
  
  ag.active := true;

  Result := idx;
end;

/// @par
///
/// The agent is deactivated and will no longer be processed.  Its #dtCrowdAgent object
/// is not removed from the pool.  It is marked as inactive so that it is available for reuse.
procedure TdtCrowd.removeAgent(const idx: Integer);
begin
  if (idx >= 0) and (idx < m_maxAgents) then
  begin
    m_agents[idx].active := false;
  end;
end;

function TdtCrowd.requestMoveTargetReplan(const idx: Integer; ref: TdtPolyRef; const pos: PSingle): Boolean;
var ag: PdtCrowdAgent;
begin
  if (idx < 0) or (idx >= m_maxAgents) then
    Exit(false);

  ag := @m_agents[idx];

  // Initialize request.
  ag.targetRef := ref;
  dtVcopy(@ag.targetPos[0], pos);
  ag.targetPathqRef := DT_PATHQ_INVALID;
  ag.targetReplan := true;
  if (ag.targetRef <> 0) then
    ag.targetState := DT_CROWDAGENT_TARGET_REQUESTING
  else
    ag.targetState := DT_CROWDAGENT_TARGET_FAILED;
  
  Result := true;
end;

/// @par
/// 
/// This method is used when a new target is set.
/// 
/// The position will be constrained to the surface of the navigation mesh.
///
/// The request will be processed during the next #update().
function TdtCrowd.requestMoveTarget(const idx: Integer; ref: TdtPolyRef; const pos: PSingle): Boolean;
var ag: PdtCrowdAgent;
begin
  if (idx < 0) or (idx >= m_maxAgents) then
    Exit(false);
  if (ref = 0) then
    Exit(false);

  ag := @m_agents[idx];
  
  // Initialize request.
  ag.targetRef := ref;
  dtVcopy(@ag.targetPos[0], pos);
  ag.targetPathqRef := DT_PATHQ_INVALID;
  ag.targetReplan := false;
  if (ag.targetRef <> 0) then
    ag.targetState := DT_CROWDAGENT_TARGET_REQUESTING
  else
    ag.targetState := DT_CROWDAGENT_TARGET_FAILED;

  Result := true;
end;

function TdtCrowd.requestMoveVelocity(const idx: Integer; const vel: PSingle): Boolean;
var ag: PdtCrowdAgent;
begin
  if (idx < 0) or (idx >= m_maxAgents) then
    Exit(false);

  ag := @m_agents[idx];
  
  // Initialize request.
  ag.targetRef := 0;
  dtVcopy(@ag.targetPos[0], vel);
  ag.targetPathqRef := DT_PATHQ_INVALID;
  ag.targetReplan := false;
  ag.targetState := DT_CROWDAGENT_TARGET_VELOCITY;
  
  Result := true;
end;

function TdtCrowd.resetMoveTarget(const idx: Integer): Boolean;
var ag: PdtCrowdAgent;
begin
  if (idx < 0) or (idx >= m_maxAgents) then
    Exit(false);

  ag := @m_agents[idx];
  
  // Initialize request.
  ag.targetRef := 0;
  dtVset(@ag.targetPos[0], 0,0,0);
  ag.targetPathqRef := DT_PATHQ_INVALID;
  ag.targetReplan := false;
  ag.targetState := DT_CROWDAGENT_TARGET_NONE;
  
  Result := true;
end;

function TdtCrowd.getActiveAgents(agents: PPdtCrowdAgent; const maxAgents: Integer): Integer;
var n, i: Integer;
begin
  n := 0;
  for i := 0 to m_maxAgents - 1 do
  begin
    if (not m_agents[i].active) then continue;
    if (n < maxAgents) then
    begin
      agents[n] := @m_agents[i];
      Inc(n);
    end;
  end;
  Result := n;
end;


procedure TdtCrowd.updateMoveRequest(const dt: Single);
const PATH_MAX_AGENTS = 8;
const MAX_RES = 32;
const MAX_ITER = 20;
var queue: array [0..PATH_MAX_AGENTS-1] of PdtCrowdAgent; nqueue, i, j, npath: Integer; ag: PdtCrowdAgent; path, res: PdtPolyRef;
reqPos, targetPos, nearest: array [0..2] of Single; reqPath: array [0..MAX_RES-1] of TdtPolyRef; reqPathCount: Integer; status: TdtStatus;
valid: Boolean; nres: Integer;
begin
  nqueue := 0;

  // Fire off new requests.
  for i := 0 to m_maxAgents - 1 do
  begin
    ag := @m_agents[i];
    if (not ag.active) then
      continue;
    if (ag.state = DT_CROWDAGENT_STATE_INVALID) then
      continue;
    if (ag.targetState = DT_CROWDAGENT_TARGET_NONE) or (ag.targetState = DT_CROWDAGENT_TARGET_VELOCITY) then
      continue;

    if (ag.targetState = DT_CROWDAGENT_TARGET_REQUESTING) then
    begin
      path := ag.corridor.getPath();
      npath := ag.corridor.getPathCount();
      Assert(npath <> 0);

      reqPathCount := 0;

      // Quick search towards the goal.
      m_navquery.initSlicedFindPath(path[0], ag.targetRef, @ag.npos[0], @ag.targetPos[0], m_filters[ag.params.queryFilterType]);
      m_navquery.updateSlicedFindPath(MAX_ITER, nil);

      if (ag.targetReplan) then // && npath > 10)
      begin
        // Try to use existing steady path during replan if possible.
        status := m_navquery.finalizeSlicedFindPathPartial(path, npath, @reqPath[0], @reqPathCount, MAX_RES);
      end
      else
      begin
        // Try to move towards target when goal changes.
        status := m_navquery.finalizeSlicedFindPath(@reqPath[0], @reqPathCount, MAX_RES);
      end;

      if (not dtStatusFailed(status)) and (reqPathCount > 0) then
      begin
        // In progress or succeed.
        if (reqPath[reqPathCount-1] <> ag.targetRef) then
        begin
          // Partial path, constrain target position inside the last polygon.
          status := m_navquery.closestPointOnPoly(reqPath[reqPathCount-1], @ag.targetPos[0], @reqPos[0], nil);
          if (dtStatusFailed(status)) then
            reqPathCount := 0;
        end
        else
        begin
          dtVcopy(@reqPos[0], @ag.targetPos[0]);
        end;
      end
      else
      begin
        reqPathCount := 0;
      end;
        
      if (reqPathCount = 0) then
      begin
        // Could not find path, start the request from current location.
        dtVcopy(@reqPos[0], @ag.npos[0]);
        reqPath[0] := path[0];
        reqPathCount := 1;
      end;

      ag.corridor.setCorridor(@reqPos[0], @reqPath[0], reqPathCount);
      ag.boundary.reset();
      ag.partial := false;

      if (reqPath[reqPathCount-1] = ag.targetRef) then
      begin
        ag.targetState := DT_CROWDAGENT_TARGET_VALID;
        ag.targetReplanTime := 0.0;
      end
      else
      begin
        // The path is longer or potentially unreachable, full plan.
        ag.targetState := DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE;
      end;
    end;

    if (ag.targetState = DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE) then
    begin
      nqueue := addToPathQueue(ag, @queue[0], nqueue, PATH_MAX_AGENTS);
    end;
  end;

  for i := 0 to nqueue - 1 do
  begin
    ag := queue[i];
    ag.targetPathqRef := m_pathq.request(ag.corridor.getLastPoly(), ag.targetRef,
                       ag.corridor.getTarget(), @ag.targetPos[0], m_filters[ag.params.queryFilterType]);
    if (ag.targetPathqRef <> DT_PATHQ_INVALID) then
      ag.targetState := DT_CROWDAGENT_TARGET_WAITING_FOR_PATH;
  end;


  // Update requests.
  m_pathq.update(MAX_ITERS_PER_UPDATE);

  // Process path results.
  for i := 0 to m_maxAgents - 1 do
  begin
    ag := @m_agents[i];
    if (not ag.active) then
      continue;
    if (ag.targetState = DT_CROWDAGENT_TARGET_NONE) or (ag.targetState = DT_CROWDAGENT_TARGET_VELOCITY) then
      continue;

    if (ag.targetState = DT_CROWDAGENT_TARGET_WAITING_FOR_PATH) then
    begin
      // Poll path queue.
      status := m_pathq.getRequestStatus(ag.targetPathqRef);
      if (dtStatusFailed(status)) then
      begin
        // Path find failed, retry if the target location is still valid.
        ag.targetPathqRef := DT_PATHQ_INVALID;
        if (ag.targetRef <> 0) then
          ag.targetState := DT_CROWDAGENT_TARGET_REQUESTING
        else
          ag.targetState := DT_CROWDAGENT_TARGET_FAILED;
        ag.targetReplanTime := 0.0;
      end
      else if (dtStatusSucceed(status)) then
      begin
        path := ag.corridor.getPath();
        npath := ag.corridor.getPathCount();
        Assert(npath <> 0);

        // Apply results.
        dtVcopy(@targetPos[0], @ag.targetPos[0]);

        res := m_pathResult;
        valid := true;
        nres := 0;
        status := m_pathq.getPathResult(ag.targetPathqRef, res, @nres, m_maxPathResult);
        if dtStatusFailed(status) or (nres = 0) then
          valid := false;

        if (dtStatusDetail(status, DT_PARTIAL_RESULT)) then
          ag.partial := true
        else
          ag.partial := false;

        // Merge result and existing path.
        // The agent might have moved whilst the request is
        // being processed, so the path may have changed.
        // We assume that the end of the path is at the same location
        // where the request was issued.

        // The last ref in the old path should be the same as
        // the location where the request was issued..
        if valid and (path[npath-1] <> res[0]) then
          valid := false;

        if (valid) then
        begin
          // Put the old path infront of the old path.
          if (npath > 1) then
          begin
            // Make space for the old path.
            if ((npath-1)+nres > m_maxPathResult) then
              nres := m_maxPathResult - (npath-1);

            Move(res^, (res+npath-1)^, sizeof(TdtPolyRef)*nres);
            // Copy old path in the beginning.
            Move(path^, res^, sizeof(TdtPolyRef)*(npath-1));
            Inc(nres, npath-1);

            // Remove trackbacks
            j := 0;
            while (j < nres) do
            begin
              if (j-1 >= 0) and (j+1 < nres) then
              begin
                if (res[j-1] = res[j+1]) then
                begin
                  Move((res+(j+1))^, (res+(j-1))^, sizeof(TdtPolyRef)*(nres-(j+1)));
                  Dec(nres, 2);
                  Dec(j, 2);
                end;
              end;
              Inc(j);
            end;

          end;
          
          // Check for partial path.
          if (res[nres-1] <> ag.targetRef) then
          begin
            // Partial path, constrain target position inside the last polygon.
            status := m_navquery.closestPointOnPoly(res[nres-1], @targetPos[0], @nearest[0], nil);
            if (dtStatusSucceed(status)) then
              dtVcopy(@targetPos[0], @nearest[0])
            else
              valid := false;
          end;
        end;
        
        if (valid) then
        begin
          // Set current corridor.
          ag.corridor.setCorridor(@targetPos[0], res, nres);
          // Force to update boundary.
          ag.boundary.reset();
          ag.targetState := DT_CROWDAGENT_TARGET_VALID;
        end
        else
        begin
          // Something went wrong.
          ag.targetState := DT_CROWDAGENT_TARGET_FAILED;
        end;

        ag.targetReplanTime := 0.0;
      end;
    end;
  end;
  
end;


procedure TdtCrowd.updateTopologyOptimization(agents: PPdtCrowdAgent; const nagents: Integer; const dt: Single);
const OPT_TIME_THR = 0.5; // seconds
const OPT_MAX_AGENTS = 1;
var ag: PdtCrowdAgent; queue: array [0..OPT_MAX_AGENTS-1] of PdtCrowdAgent; nqueue, i: Integer;
begin
  if (nagents = 0) then
    Exit;

  nqueue := 0;
  
  for i := 0 to nagents - 1 do
  begin
    ag := agents[i];
    if (ag.state <> DT_CROWDAGENT_STATE_WALKING) then
      continue;
    if (ag.targetState = DT_CROWDAGENT_TARGET_NONE) or (ag.targetState = DT_CROWDAGENT_TARGET_VELOCITY) then
      continue;
    if ((ag.params.updateFlags and DT_CROWD_OPTIMIZE_TOPO) = 0) then
      continue;
    ag.topologyOptTime := ag.topologyOptTime + dt;
    if (ag.topologyOptTime >= OPT_TIME_THR) then
      nqueue := addToOptQueue(ag, @queue[0], nqueue, OPT_MAX_AGENTS);
  end;

  for i := 0 to nqueue - 1 do
  begin
    ag := queue[i];
    ag.corridor.optimizePathTopology(m_navquery, m_filters[ag.params.queryFilterType]);
    ag.topologyOptTime := 0;
  end;

end;

procedure TdtCrowd.checkPathValidity(agents: PPdtCrowdAgent; const nagents: Integer; const dt: Single);
const CHECK_LOOKAHEAD = 10;
const TARGET_REPLAN_DELAY = 1.0; // seconds
var ag: PdtCrowdAgent; i: Integer; replan: Boolean; idx: Integer; agentPos, nearest: array [0..2] of Single; agentRef: TdtPolyRef;
begin

  for i := 0 to nagents - 1 do
  begin
    ag := agents[i];

    if (ag.state <> DT_CROWDAGENT_STATE_WALKING) then
      continue;

    ag.targetReplanTime := ag.targetReplanTime + dt;

    replan := false;

    // First check that the current location is valid.
    idx := getAgentIndex(ag);
    agentRef := ag.corridor.getFirstPoly();
    dtVcopy(@agentPos[0], @ag.npos[0]);
    if (not m_navquery.isValidPolyRef(agentRef, m_filters[ag.params.queryFilterType])) then
    begin
      // Current location is not valid, try to reposition.
      // TODO: this can snap agents, how to handle that?
      dtVcopy(@nearest[0], @agentPos[0]);
      agentRef := 0;
      m_navquery.findNearestPoly(@ag.npos[0], @m_ext[0], m_filters[ag.params.queryFilterType], @agentRef, @nearest[0]);
      dtVcopy(@agentPos[0], @nearest[0]);

      if (agentRef = 0) then
      begin
        // Could not find location in navmesh, set state to invalid.
        ag.corridor.reset(0, @agentPos[0]);
        ag.partial := false;
        ag.boundary.reset();
        ag.state := DT_CROWDAGENT_STATE_INVALID;
        continue;
      end;

      // Make sure the first polygon is valid, but leave other valid
      // polygons in the path so that replanner can adjust the path better.
      ag.corridor.fixPathStart(agentRef, @agentPos[0]);
//      ag.corridor.trimInvalidPath(agentRef, agentPos, m_navquery, &m_filter);
      ag.boundary.reset();
      dtVcopy(@ag.npos[0], @agentPos[0]);

      replan := true;
    end;

    // If the agent does not have move target or is controlled by velocity, no need to recover the target nor replan.
    if (ag.targetState = DT_CROWDAGENT_TARGET_NONE) or (ag.targetState = DT_CROWDAGENT_TARGET_VELOCITY) then
      continue;

    // Try to recover move request position.
    if (ag.targetState <> DT_CROWDAGENT_TARGET_NONE) and (ag.targetState <> DT_CROWDAGENT_TARGET_FAILED) then
    begin
      if (not m_navquery.isValidPolyRef(ag.targetRef, m_filters[ag.params.queryFilterType])) then
      begin
        // Current target is not valid, try to reposition.
        dtVcopy(@nearest[0], @ag.targetPos[0]);
        ag.targetRef := 0;
        m_navquery.findNearestPoly(@ag.targetPos[0], @m_ext[0], m_filters[ag.params.queryFilterType], @ag.targetRef, @nearest[0]);
        dtVcopy(@ag.targetPos[0], @nearest[0]);
        replan := true;
      end;
      if (ag.targetRef = 0) then
      begin
        // Failed to reposition target, fail moverequest.
        ag.corridor.reset(agentRef, @agentPos[0]);
        ag.partial := false;
        ag.targetState := DT_CROWDAGENT_TARGET_NONE;
      end;
    end;

    // If nearby corridor is not valid, replan.
    if (not ag.corridor.isValid(CHECK_LOOKAHEAD, m_navquery, m_filters[ag.params.queryFilterType])) then
    begin
      // Fix current path.
//      ag.corridor.trimInvalidPath(agentRef, agentPos, m_navquery, &m_filter);
//      ag.boundary.reset();
      replan := true;
    end;

    // If the end of the path is near and it is not the requested location, replan.
    if (ag.targetState = DT_CROWDAGENT_TARGET_VALID) then
    begin
      if (ag.targetReplanTime > TARGET_REPLAN_DELAY) and
        (ag.corridor.getPathCount() < CHECK_LOOKAHEAD) and
        (ag.corridor.getLastPoly() <> ag.targetRef) then
        replan := true;
    end;

    // Try to replan path to goal.
    if (replan) then
    begin
      if (ag.targetState <> DT_CROWDAGENT_TARGET_NONE) then
      begin
        requestMoveTargetReplan(idx, ag.targetRef, @ag.targetPos[0]);
      end;
    end;
  end;
end;

procedure TdtCrowd.update(const dt: Single; debug: PdtCrowdAgentDebugInfo);
const COLLISION_RESOLVE_FACTOR = 0.7;
var debugIdx: Integer; agents: PPdtCrowdAgent; nagents, i, j, idx, ns, iter, idx0, idx1: Integer; ag, nei: PdtCrowdAgent; p, target, s: PSingle;
r, updateThr, triggerRadius: Single; anim: PdtCrowdAgentAnimation; refs: array [0..1] of TdtPolyRef;
dvel, disp, diff: array [0..2] of Single; vod: TdtObstacleAvoidanceDebugData; adaptive: Boolean; params: PdtObstacleAvoidanceParams;
slowDownRadius, speedScale, separationDist, invSeparationDist, separationWeight, w, distSqr, dist, weight, speedSqr, desiredSqr, pen, iw, ta, tb, u: Single;
begin
  m_velocitySampleCount := 0;

  if debug <> nil then debugIdx := debug.idx else debugIdx := -1;

  agents := m_activeAgents;
  nagents := getActiveAgents(agents, m_maxAgents);

  // Check that all agents still have valid paths.
  checkPathValidity(agents, nagents, dt);

  // Update async move request and path finder.
  updateMoveRequest(dt);

  // Optimize path topology.
  updateTopologyOptimization(agents, nagents, dt);

  // Register agents to proximity grid.
  m_grid.clear();
  for i := 0 to nagents - 1 do
  begin
    ag := agents[i];
    p := @ag.npos[0];
    r := ag.params.radius;
    m_grid.addItem(Word(i), p[0]-r, p[2]-r, p[0]+r, p[2]+r);
  end;

  // Get nearby navmesh segments and agents to collide with.
  for i := 0 to nagents - 1 do
  begin
    ag := agents[i];
    if (ag.state <> DT_CROWDAGENT_STATE_WALKING) then
      continue;

    // Update the collision boundary after certain distance has been passed or
    // if it has become invalid.
    updateThr := ag.params.collisionQueryRange*0.25;
    if (dtVdist2DSqr(@ag.npos[0], ag.boundary.getCenter()) > Sqr(updateThr)) or
      (not ag.boundary.isValid(m_navquery, m_filters[ag.params.queryFilterType])) then
    begin
      ag.boundary.update(ag.corridor.getFirstPoly(), @ag.npos[0], ag.params.collisionQueryRange,
                m_navquery, m_filters[ag.params.queryFilterType]);
    end;
    // Query neighbour agents
    ag.nneis := getNeighbours(@ag.npos[0], ag.params.height, ag.params.collisionQueryRange,
                  ag, @ag.neis[0], DT_CROWDAGENT_MAX_NEIGHBOURS,
                  agents, nagents, m_grid);
    for j := 0 to ag.nneis - 1 do
      ag.neis[j].idx := getAgentIndex(agents[ag.neis[j].idx]);
  end;

  // Find next corner to steer to.
  for i := 0 to nagents - 1 do
  begin
    ag := agents[i];

    if (ag.state <> DT_CROWDAGENT_STATE_WALKING) then
      continue;
    if (ag.targetState = DT_CROWDAGENT_TARGET_NONE) or (ag.targetState = DT_CROWDAGENT_TARGET_VELOCITY) then
      continue;

    // Find corners for steering
    ag.ncorners := ag.corridor.findCorners(@ag.cornerVerts[0], @ag.cornerFlags[0], @ag.cornerPolys[0],
                        DT_CROWDAGENT_MAX_CORNERS, m_navquery, m_filters[ag.params.queryFilterType]);

    // Check to see if the corner after the next corner is directly visible,
    // and short cut to there.
    if ((ag.params.updateFlags and DT_CROWD_OPTIMIZE_VIS) <> 0) and (ag.ncorners > 0) then
    begin
      target := @ag.cornerVerts[dtMin(1,ag.ncorners-1)*3];
      ag.corridor.optimizePathVisibility(target, ag.params.pathOptimizationRange, m_navquery, m_filters[ag.params.queryFilterType]);

      // Copy data for debug purposes.
      if (debugIdx = i) then
      begin
        dtVcopy(@debug.optStart[0], ag.corridor.getPos());
        dtVcopy(@debug.optEnd[0], target);
      end;
    end
    else
    begin
      // Copy data for debug purposes.
      if (debugIdx = i) then
      begin
        dtVset(@debug.optStart[0], 0,0,0);
        dtVset(@debug.optEnd[0], 0,0,0);
      end;
    end;
  end;

  // Trigger off-mesh connections (depends on corners).
  for i := 0 to nagents - 1 do
  begin
    ag := agents[i];

    if (ag.state <> DT_CROWDAGENT_STATE_WALKING) then
      continue;
    if (ag.targetState = DT_CROWDAGENT_TARGET_NONE) or (ag.targetState = DT_CROWDAGENT_TARGET_VELOCITY) then
      continue;

    // Check
    triggerRadius := ag.params.radius*2.25;
    if (overOffmeshConnection(ag, triggerRadius)) then
    begin
      // Prepare to off-mesh connection.
      idx := Integer(ag - m_agents);
      anim := @m_agentAnims[idx];

      // Adjust the path over the off-mesh connection.
      if (ag.corridor.moveOverOffmeshConnection(ag.cornerPolys[ag.ncorners-1], @refs[0],
                             @anim.startPos[0], @anim.endPos[0], m_navquery)) then
      begin
        dtVcopy(@anim.initPos[0], @ag.npos[0]);
        anim.polyRef := refs[1];
        anim.active := true;
        anim.t := 0.0;
        anim.tmax := (dtVdist2D(@anim.startPos[0], @anim.endPos[0]) / ag.params.maxSpeed) * 0.5;
        
        ag.state := DT_CROWDAGENT_STATE_OFFMESH;
        ag.ncorners := 0;
        ag.nneis := 0;
        continue;
      end
      else
      begin
        // Path validity check will ensure that bad/blocked connections will be replanned.
      end;
    end;
  end;

  // Calculate steering.
  for i := 0 to nagents - 1 do
  begin
    ag := agents[i];

    if (ag.state <> DT_CROWDAGENT_STATE_WALKING) then
      continue;
    if (ag.targetState = DT_CROWDAGENT_TARGET_NONE) then
      continue;

    dvel[0] := 0; dvel[1] := 0; dvel[2] := 0;

    if (ag.targetState = DT_CROWDAGENT_TARGET_VELOCITY) then
    begin
      dtVcopy(@dvel[0], @ag.targetPos[0]);
      ag.desiredSpeed := dtVlen(@ag.targetPos[0]);
    end
    else
    begin
      // Calculate steering direction.
      if (ag.params.updateFlags and DT_CROWD_ANTICIPATE_TURNS) <> 0 then
        calcSmoothSteerDirection(ag, @dvel[0])
      else
        calcStraightSteerDirection(ag, @dvel[0]);

      // Calculate speed scale, which tells the agent to slowdown at the end of the path.
      slowDownRadius := 0.3; // Delphi. Don't slowdown // ag.params.radius*2;  // TODO: make less hacky.
      speedScale := getDistanceToGoal(ag, slowDownRadius) / slowDownRadius;

      ag.desiredSpeed := ag.params.maxSpeed;
      dtVscale(@dvel[0], @dvel[0], ag.desiredSpeed * speedScale);
    end;

    // Separation
    if (ag.params.updateFlags and DT_CROWD_SEPARATION) <> 0 then
    begin
      separationDist := ag.params.collisionQueryRange;
      invSeparationDist := 1.0 / separationDist;
      separationWeight := ag.params.separationWeight;

      w := 0;
      disp[0] := 0; disp[1] := 0; disp[2] := 0;

      for j := 0 to ag.nneis - 1 do
      begin
        nei := @m_agents[ag.neis[j].idx];

        dtVsub(@diff[0], @ag.npos[0], @nei.npos[0]);
        diff[1] := 0;
        
        distSqr := dtVlenSqr(@diff[0]);
        if (distSqr < 0.00001) then
          continue;
        if (distSqr > Sqr(separationDist)) then
          continue;
        dist := Sqrt(distSqr);
        weight := separationWeight * (1.0 - Sqr(dist*invSeparationDist));
        
        dtVmad(@disp[0], @disp[0], @diff[0], weight/dist);
        w := w + 1.0;
      end;

      if (w > 0.0001) then
      begin
        // Adjust desired velocity.
        dtVmad(@dvel[0], @dvel[0], @disp[0], 1.0/w);
        // Clamp desired velocity to desired speed.
        speedSqr := dtVlenSqr(@dvel[0]);
        desiredSqr := Sqr(ag.desiredSpeed);
        if (speedSqr > desiredSqr) then
          dtVscale(@dvel[0], @dvel[0], desiredSqr/speedSqr);
      end;
    end;
    
    // Set the desired velocity.
    dtVcopy(@ag.dvel[0], @dvel[0]);
  end;
  
  // Velocity planning.  
  for i := 0 to nagents - 1 do
  begin
    ag := agents[i];

    if (ag.state <> DT_CROWDAGENT_STATE_WALKING) then
      continue;
    
    if (ag.params.updateFlags and DT_CROWD_OBSTACLE_AVOIDANCE) <> 0 then
    begin
      m_obstacleQuery.reset();
      
      // Add neighbours as obstacles.
      for j := 0 to ag.nneis - 1 do
      begin
        nei := @m_agents[ag.neis[j].idx];
        m_obstacleQuery.addCircle(@nei.npos[0], nei.params.radius, @nei.vel[0], @nei.dvel[0]);
      end;

      // Append neighbour segments as obstacles.
      for j := 0 to ag.boundary.getSegmentCount - 1 do
      begin
        s := ag.boundary.getSegment(j);
        if (dtTriArea2D(@ag.npos[0], s, s+3) < 0.0) then
          continue;
        m_obstacleQuery.addSegment(s, s+3);
      end;

      vod := nil;
      if (debugIdx = i) then
        vod := debug.vod;
      
      // Sample new safe velocity.
      adaptive :=  true;
      ns := 0;

      params := @m_obstacleQueryParams[ag.params.obstacleAvoidanceType];

      if (adaptive) then
      begin
        ns := m_obstacleQuery.sampleVelocityAdaptive(@ag.npos[0], ag.params.radius, ag.desiredSpeed,
                               @ag.vel[0], @ag.dvel[0], @ag.nvel[0], params, vod);
      end
      else
      begin
        ns := m_obstacleQuery.sampleVelocityGrid(@ag.npos[0], ag.params.radius, ag.desiredSpeed,
                             @ag.vel[0], @ag.dvel[0], @ag.nvel[0], params, vod);
      end;
      m_velocitySampleCount := m_velocitySampleCount + ns;
    end
    else
    begin
      // If not using velocity planning, new velocity is directly the desired velocity.
      dtVcopy(@ag.nvel[0], @ag.dvel[0]);
    end;
  end;

  // Integrate.
  for i := 0 to nagents - 1 do
  begin
    ag := agents[i];
    if (ag.state <> DT_CROWDAGENT_STATE_WALKING) then
      continue;
    integrate(ag, dt);
  end;

  // Handle collisions.

  for iter := 0 to 3 do
  begin
    for i := 0 to nagents - 1 do
    begin
      ag := agents[i];
      idx0 := getAgentIndex(ag);

      if (ag.state <> DT_CROWDAGENT_STATE_WALKING) then
        continue;

      dtVset(@ag.disp[0], 0,0,0);
      
      w := 0;

      for j := 0 to ag.nneis - 1 do
      begin
        nei := @m_agents[ag.neis[j].idx];
        idx1 := getAgentIndex(nei);

        dtVsub(@diff[0], @ag.npos[0], @nei.npos[0]);
        diff[1] := 0;
        
        dist := dtVlenSqr(@diff[0]);
        if (dist > Sqr(ag.params.radius + nei.params.radius)) then
          continue;
        dist := Sqrt(dist);
        pen := (ag.params.radius + nei.params.radius) - dist;
        if (dist < 0.0001) then
        begin
          // Agents on top of each other, try to choose diverging separation directions.
          if (idx0 > idx1) then
            dtVset(@diff[0], -ag.dvel[2],0,ag.dvel[0])
          else
            dtVset(@diff[0], ag.dvel[2],0,-ag.dvel[0]);
          pen := 0.01;
        end
        else
        begin
          pen := (1.0/dist) * (pen*0.5) * COLLISION_RESOLVE_FACTOR;
        end;

        dtVmad(@ag.disp[0], @ag.disp[0], @diff[0], pen);

        w := w + 1.0;
      end;
      
      if (w > 0.0001) then
      begin
        iw := 1.0 / w;
        dtVscale(@ag.disp[0], @ag.disp[0], iw);
      end;
    end;
    
    for i := 0 to nagents - 1 do
    begin
      ag := agents[i];
      if (ag.state <> DT_CROWDAGENT_STATE_WALKING) then
        continue;

      dtVadd(@ag.npos[0], @ag.npos[0], @ag.disp[0]);
    end;
  end;

  for i := 0 to nagents - 1 do
  begin
    ag := agents[i];
    if (ag.state <> DT_CROWDAGENT_STATE_WALKING) then
      continue;
    
    // Move along navmesh.
    ag.corridor.movePosition(@ag.npos[0], m_navquery, m_filters[ag.params.queryFilterType]);
    // Get valid constrained position back.
    dtVcopy(@ag.npos[0], ag.corridor.getPos());

    // If not using path, truncate the corridor to just one poly.
    if (ag.targetState = DT_CROWDAGENT_TARGET_NONE) or (ag.targetState = DT_CROWDAGENT_TARGET_VELOCITY) then
    begin
      ag.corridor.reset(ag.corridor.getFirstPoly(), @ag.npos[0]);
      ag.partial := false;
    end;

  end;
  
  // Update agents using off-mesh connection.
  for i := 0 to m_maxAgents - 1 do
  begin
    anim := @m_agentAnims[i];
    if (not anim.active) then
      continue;
    ag := agents[i];

    anim.t := anim.t + dt;
    if (anim.t > anim.tmax) then
    begin
      // Reset animation
      anim.active := false;
      // Prepare agent for walking.
      ag.state := DT_CROWDAGENT_STATE_WALKING;
      continue;
    end;
    
    // Update position
    ta := anim.tmax*0.15;
    tb := anim.tmax;
    if (anim.t < ta) then
    begin
      u := tween(anim.t, 0.0, ta);
      dtVlerp(@ag.npos[0], @anim.initPos[0], @anim.startPos[0], u);
    end
    else
    begin
      u := tween(anim.t, ta, tb);
      dtVlerp(@ag.npos[0], @anim.startPos[0], @anim.endPos[0], u);
    end;
      
    // Update velocity.
    dtVset(@ag.vel[0], 0,0,0);
    dtVset(@ag.dvel[0], 0,0,0);
  end;
  
end;


function TdtCrowd.getAgentIndex(agent: PdtCrowdAgent): Integer; begin Result := Integer(agent - m_agents); end;
function TdtCrowd.getFilter(const i: Integer): TdtQueryFilter; begin if (i >= 0) and (i < DT_CROWD_MAX_QUERY_FILTER_TYPE) then Result := m_filters[i] else Result := nil; end;
function TdtCrowd.getEditableFilter(const i: Integer): TdtQueryFilter; begin if (i >= 0) and (i < DT_CROWD_MAX_QUERY_FILTER_TYPE) then Result := m_filters[i] else Result := nil; end;
function TdtCrowd.getQueryExtents(): PSingle; begin Result := @m_ext[0]; end;

end.
