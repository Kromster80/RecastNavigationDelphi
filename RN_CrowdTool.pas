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

unit RN_CrowdTool;
interface
uses
  Classes, Controls, StdCtrls, Unit_FrameCrowdTool,
  RN_Sample, RN_DetourNavMesh, RN_DetourNavMeshHelper, RN_DetourObstacleAvoidance, RN_ValueHistory, RN_DetourCrowd;

// Tool to create crowds.
type
  PCrowdToolParams = ^TCrowdToolParams;
  TCrowdToolParams = record
    m_showCorners: Boolean;
    m_showCollisionSegments: Boolean;
    m_showPath: Boolean;
    m_showVO: Boolean;
    m_showOpt: Boolean;
    m_showNeis: Boolean;

    m_showLabels: Boolean;
    m_showGrid: Boolean;
    m_showNodes: Boolean;
    m_showPerfGraph: Boolean;
    m_showDetailAll: Boolean;

    m_anticipateTurns: Boolean;
    m_optimizeVis: Boolean;
    m_optimizeTopo: Boolean;
    m_obstacleAvoidance: Boolean;
    m_obstacleAvoidanceType: Single;
    m_separation: Boolean;
    m_separationWeight: Single;
  end;

  TCrowdToolState = class (TSampleToolState)
  private
    m_sample: TSample;
    m_nav: TdtNavMesh;
    m_crowd: TdtCrowd;

    m_targetPos: array [0..2] of Single;
    m_targetRef: TdtPolyRef;

    m_agentDebug: TdtCrowdAgentDebugInfo;
    m_vod: TdtObstacleAvoidanceDebugData;
  
    const AGENT_MAX_TRAIL = 64;
    const MAX_AGENTS = 128;
    type
    PAgentTrail = ^TAgentTrail;
    TAgentTrail = record
      trail: array [0..AGENT_MAX_TRAIL*3-1] of Single;
      htrail: Integer;
    end;
    var
    m_trails: array [0..MAX_AGENTS-1] of TAgentTrail;
  
    m_crowdTotalTime: TValueHistory;
    m_crowdSampleCount: TValueHistory;

    m_toolParams: TCrowdToolParams;

    m_run: Boolean;

  public
    constructor Create;
    destructor Destroy; override;

    procedure init(sample: TSample); override;
    procedure reset; override;
    procedure handleRender; override;
    procedure handleRenderOverlay(proj, model: PDouble; view: PInteger); override;
    procedure handleUpdate(dt: Single); override;

    property isRunning: Boolean read m_run;
    procedure setRunning(const s: Boolean); { m_run = s; }

    procedure addAgent(const pos: PSingle);
    procedure removeAgent(const idx: Integer);
    procedure hilightAgent(const idx: Integer);
    procedure updateAgentParams();
    function hitTestAgents(const s, p: PSingle): Integer;
    procedure setMoveTarget(const p: PSingle; adjust: Boolean);
    procedure updateTick(const dt: Single);

    function getToolParams: PCrowdToolParams; { return &m_toolParams; }
  end;

  TCrowdTool = class (TSampleTool)
  private
    fFrame: TFrameCrowdTool;
    fUpdateUI: Boolean;

    m_sample: TSample;
    m_state: TCrowdToolState;

    type TToolMode =
    (
      TOOLMODE_CREATE,
      TOOLMODE_MOVE_TARGET,
      TOOLMODE_SELECT,
      TOOLMODE_TOGGLE_POLYS
    );
    var
    m_mode: TToolMode;

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
  RN_InputGeom, RN_SampleInterfaces, RN_DebugDraw, RN_PerfTimer,
  RN_Recast, RN_RecastHelper, RN_RecastDebugDraw,
  RN_DetourNavMeshBuilder, RN_DetourProximityGrid, RN_DetourNavMeshQuery, RN_DetourStatus, RN_DetourDebugDraw, RN_DetourCommon;


function isectSegAABB(const sp, sq, amin, amax: PSingle;
             tmin, tmax: PSingle): Boolean;
const EPS = 0.000001;
var d: array [0..2] of Single; i: Integer; ood,t1,t2: Single;
begin
  dtVsub(@d[0], sq, sp);
  tmin^ := 0;  // set to -FLT_MAX to get first hit on line
  tmax^ := MaxSingle;    // set to max distance ray can travel (for segment)

  // For all three slabs
  for i := 0 to 2 do
  begin
    if (Abs(d[i]) < EPS) then
    begin
      // Ray is parallel to slab. No hit if origin not within slab
      if (sp[i] < amin[i]) or (sp[i] > amax[i]) then
        Exit(false);
    end
    else
    begin
      // Compute intersection t value of ray with near and far plane of slab
      ood := 1.0 / d[i];
      t1 := (amin[i] - sp[i]) * ood;
      t2 := (amax[i] - sp[i]) * ood;
      // Make t1 be intersection with near plane, t2 with far plane
      if (t1 > t2) then dtSwap(t1, t2);
      // Compute the intersection of slab intersections intervals
      if (t1 > tmin^) then tmin^ := t1;
      if (t2 < tmax^) then tmax^ := t2;
      // Exit with no collision as soon as slab intersection becomes empty
      if (tmin^ > tmax^) then Exit(false);
    end;
  end;

  Result := true;
end;

procedure getAgentBounds(const ag: PdtCrowdAgent; bmin, bmax: PSingle);
var p: PSingle; r, h: Single;
begin
  p := @ag.npos[0];
  r := ag.params.radius;
  h := ag.params.height;
  bmin[0] := p[0] - r;
  bmin[1] := p[1];
  bmin[2] := p[2] - r;
  bmax[0] := p[0] + r;
  bmax[1] := p[1] + h;
  bmax[2] := p[2] + r;
end;

constructor TCrowdToolState.Create();
begin
  inherited;

  m_run := true;
  m_toolParams.m_showCorners := false;
  m_toolParams.m_showCollisionSegments := false;
  m_toolParams.m_showPath := false;
  m_toolParams.m_showVO := false;
  m_toolParams.m_showOpt := false;
  m_toolParams.m_showNeis := false;

  m_toolParams.m_showLabels := false;
  m_toolParams.m_showGrid := false;
  m_toolParams.m_showNodes := false;
  m_toolParams.m_showPerfGraph := false;
  m_toolParams.m_showDetailAll := false;

  m_toolParams.m_anticipateTurns := true;
  m_toolParams.m_optimizeVis := true;
  m_toolParams.m_optimizeTopo := true;
  m_toolParams.m_obstacleAvoidance := true;
  m_toolParams.m_obstacleAvoidanceType := 3.0;
  m_toolParams.m_separation := false;
  m_toolParams.m_separationWeight := 2.0;
  
  FillChar(m_trails[0], sizeof(m_trails), 0);

  m_vod := dtAllocObstacleAvoidanceDebugData();
  m_vod.init(2048);

  FillChar(m_agentDebug, sizeof(m_agentDebug), 0);
  m_agentDebug.idx := -1;
  m_agentDebug.vod := m_vod;

  // Delphi: Assume C++ invokes constructor
  m_crowdTotalTime := TValueHistory.Create;
  m_crowdSampleCount := TValueHistory.Create;
end;

destructor TCrowdToolState.Destroy;
begin
  dtFreeObstacleAvoidanceDebugData(m_vod);

  // Delphi: Assume C++ invokes destructor
  m_crowdTotalTime.Free;
  m_crowdSampleCount.Free;

  inherited;
end;

procedure TCrowdToolState.init(sample: TSample);
var nav: TdtNavMesh; crowd: TdtCrowd; params: TdtObstacleAvoidanceParams;
begin
  if (m_sample <> sample) then
  begin
    m_sample := sample;
//    m_oldFlags := m_sample.getNavMeshDrawFlags();
//    m_sample.setNavMeshDrawFlags(m_oldFlags & ~DU_DRAWNAVMESH_CLOSEDLIST);
  end;

  nav := m_sample.getNavMesh;
  crowd := m_sample.getCrowd;

  if (nav <> nil) and (crowd <> nil) and ((m_nav <> nav) or (m_crowd <> crowd)) then
  begin
    m_nav := nav;
    m_crowd := crowd;

    crowd.init(MAX_AGENTS, m_sample.getAgentRadius, nav);

    // Make polygons with 'disabled' flag invalid.
    crowd.getEditableFilter(0).setExcludeFlags(SAMPLE_POLYFLAGS_DISABLED);

    // Setup local avoidance params to different qualities.

    // Use mostly default settings, copy from dtCrowd.
    Move(crowd.getObstacleAvoidanceParams(0)^, params, sizeof(TdtObstacleAvoidanceParams));

    // Low (11)
    params.velBias := 0.5;
    params.adaptiveDivs := 5;
    params.adaptiveRings := 2;
    params.adaptiveDepth := 1;
    crowd.setObstacleAvoidanceParams(0, @params);

    // Medium (22)
    params.velBias := 0.5;
    params.adaptiveDivs := 5;
    params.adaptiveRings := 2;
    params.adaptiveDepth := 2;
    crowd.setObstacleAvoidanceParams(1, @params);

    // Good (45)
    params.velBias := 0.5;
    params.adaptiveDivs := 7;
    params.adaptiveRings := 2;
    params.adaptiveDepth := 3;
    crowd.setObstacleAvoidanceParams(2, @params);

    // High (66)
    params.velBias := 0.5;
    params.adaptiveDivs := 7;
    params.adaptiveRings := 3;
    params.adaptiveDepth := 3;

    crowd.setObstacleAvoidanceParams(3, @params);
  end;
end;

procedure TCrowdToolState.reset();
begin
end;

procedure TCrowdToolState.handleRender();
var dd: TDebugDrawGL; rad: Single; nav: TdtNavMesh; crowd: TdtCrowd; navquery: TdtNavMeshQuery; i, j: Integer; ag: PdtCrowdAgent;
path: PdtPolyRef; npath: Integer; gridy, cs: Single; grid: TdtProximityGrid; pos, vel, dvel, v, p: PSingle; bounds: PInteger; x, y, count: Integer;
col: Cardinal; trail: PAgentTrail; prev: array [0..2] of Single; preva, a, radius, height: Single; idx: Integer; va, vb, center, s: PSingle;
nei: PdtCrowdAgent; vod: TdtObstacleAvoidanceDebugData; dx,dy,dz, sr, pen, pen2: Single;
begin
  dd := TDebugDrawGL.Create;
  rad := m_sample.getAgentRadius;

  nav := m_sample.getNavMesh;
  crowd := m_sample.getCrowd;
  if (nav = nil) or (crowd = nil) then
    Exit;

  if (m_toolParams.m_showNodes and (crowd.getPathQueue <> nil)) then
  begin
    navquery := crowd.getPathQueue.getNavQuery();
    if (navquery <> nil) then
      duDebugDrawNavMeshNodes(dd, navquery);
  end;

  dd.depthMask(false);

  // Draw paths
  if (m_toolParams.m_showPath) then
  begin
    for i := 0 to crowd.getAgentCount - 1 do
    begin
      if (m_toolParams.m_showDetailAll = false) and (i <> m_agentDebug.idx) then
        continue;
      ag := crowd.getAgent(i);
      if (not ag.active) then
        continue;
      path := ag.corridor.getPath();
      npath := ag.corridor.getPathCount();
      for j := 0 to npath - 1 do
        duDebugDrawNavMeshPoly(dd, nav, path[j], duRGBA(255,255,255,24));
    end;
  end;

  if (m_targetRef <> 0) then
    duDebugDrawCross(dd, m_targetPos[0],m_targetPos[1]+0.1,m_targetPos[2], rad, duRGBA(255,255,255,192), 2.0);

  // Occupancy grid.
  if (m_toolParams.m_showGrid) then
  begin
    gridy := -MaxSingle;
    for i := 0 to crowd.getAgentCount - 1 do
    begin
      ag := crowd.getAgent(i);
      if (not ag.active) then continue;
      pos := ag.corridor.getPos();
      gridy := dtMax(gridy, pos[1]);
    end;
    gridy := gridy + 1.0;

    dd.&begin(DU_DRAW_QUADS);
    grid := crowd.getGrid;
    bounds := grid.getBounds();
    cs := grid.getCellSize();
    for y := bounds[1] to bounds[3] do
    begin
      for x := bounds[0] to bounds[2] do
      begin
        count := grid.getItemCountAt(x,y);
        if (count = 0) then continue;
        col := duRGBA(128,0,0,dtMin(count*40,255));
        dd.vertex(x*cs, gridy, y*cs, col);
        dd.vertex(x*cs, gridy, y*cs+cs, col);
        dd.vertex(x*cs+cs, gridy, y*cs+cs, col);
        dd.vertex(x*cs+cs, gridy, y*cs, col);
      end;
    end;
    dd.&end();
  end;

  // Trail
  for i := 0 to crowd.getAgentCount - 1 do
  begin
    ag := crowd.getAgent(i);
    if (not ag.active) then continue;

    trail := @m_trails[i];
    pos := @ag.npos[0];

    dd.&begin(DU_DRAW_LINES,3.0);
    preva := 1;
    dtVcopy(@prev[0], pos);
    for j := 0 to AGENT_MAX_TRAIL-1 - 1 do
    begin
      idx := (trail.htrail + AGENT_MAX_TRAIL-j) mod AGENT_MAX_TRAIL;
      v := @trail.trail[idx*3];
      a := 1 - j/AGENT_MAX_TRAIL;
      dd.vertex(prev[0],prev[1]+0.1,prev[2], duRGBA(0,0,0,Trunc(128*preva)));
      dd.vertex(v[0],v[1]+0.1,v[2], duRGBA(0,0,0,Trunc(128*a)));
      preva := a;
      dtVcopy(@prev[0], v);
    end;
    dd.&end();

  end;

  // Corners & co
  for i := 0 to crowd.getAgentCount - 1 do
  begin
    if (m_toolParams.m_showDetailAll = false) and (i <> m_agentDebug.idx) then
      continue;
    ag := crowd.getAgent(i);
    if (not ag.active) then
      continue;

    radius := ag.params.radius;
    pos := @ag.npos[0];
    
    if (m_toolParams.m_showCorners) then
    begin
      if (ag.ncorners <> 0) then
      begin
        dd.&begin(DU_DRAW_LINES, 2.0);
        for j := 0 to ag.ncorners - 1 do
        begin
          if j = 0 then va := pos else va := @ag.cornerVerts[(j-1)*3];
          vb := @ag.cornerVerts[j*3];
          dd.vertex(va[0],va[1]+radius,va[2], duRGBA(128,0,0,192));
          dd.vertex(vb[0],vb[1]+radius,vb[2], duRGBA(128,0,0,192));
        end;
        if (ag.ncorners <> 0) and ((ag.cornerFlags[ag.ncorners-1] and Byte(DT_STRAIGHTPATH_OFFMESH_CONNECTION)) <> 0) then
        begin
          v := @ag.cornerVerts[(ag.ncorners-1)*3];
          dd.vertex(v[0],v[1],v[2], duRGBA(192,0,0,192));
          dd.vertex(v[0],v[1]+radius*2,v[2], duRGBA(192,0,0,192));
        end;

        dd.&end();
        
        
        if (m_toolParams.m_anticipateTurns) then
        begin
          (*          float dvel[3], pos[3];
           calcSmoothSteerDirection(ag.pos, ag.cornerVerts, ag.ncorners, dvel);
           pos[0] := ag.pos[0] + dvel[0];
           pos[1] := ag.pos[1] + dvel[1];
           pos[2] := ag.pos[2] + dvel[2];

           const float off := ag.radius+0.1f;
           const float* tgt := &ag.cornerVerts[0];
           const float y := ag.pos[1]+off;

           dd.begin(DU_DRAW_LINES, 2.0f);

           dd.vertex(ag.pos[0],y,ag.pos[2], duRGBA(255,0,0,192));
           dd.vertex(pos[0],y,pos[2], duRGBA(255,0,0,192));

           dd.vertex(pos[0],y,pos[2], duRGBA(255,0,0,192));
           dd.vertex(tgt[0],y,tgt[2], duRGBA(255,0,0,192));

           dd.end();*)
        end;
      end;
    end;

    if (m_toolParams.m_showCollisionSegments) then
    begin
      center := ag.boundary.getCenter();
      duDebugDrawCross(dd, center[0],center[1]+radius,center[2], 0.2, duRGBA(192,0,128,255), 2.0);
      duDebugDrawCircle(dd, center[0],center[1]+radius,center[2], ag.params.collisionQueryRange,
                duRGBA(192,0,128,128), 2.0);

      dd.&begin(DU_DRAW_LINES, 3.0);
      for j := 0 to ag.boundary.getSegmentCount - 1 do
      begin
        s := ag.boundary.getSegment(j);
        col := duRGBA(192,0,128,192);
        if (dtTriArea2D(pos, s, s+3) < 0.0) then
          col := duDarkenCol(col);

        duAppendArrow(dd, s[0],s[1]+0.2,s[2], s[3],s[4]+0.2,s[5], 0.0, 0.3, col);
      end;
      dd.&end();
    end;

    if (m_toolParams.m_showNeis) then
    begin
      duDebugDrawCircle(dd, pos[0],pos[1]+radius,pos[2], ag.params.collisionQueryRange,
                duRGBA(0,192,128,128), 2.0);

      dd.&begin(DU_DRAW_LINES, 2.0);
      for j := 0 to ag.nneis - 1 do
      begin
        // Get 'n'th active agent.
        // TODO: fix this properly.
        nei := crowd.getAgent(ag.neis[j].idx);
        if (nei <> nil) then
        begin
          dd.vertex(pos[0],pos[1]+radius,pos[2], duRGBA(0,192,128,128));
          dd.vertex(nei.npos[0],nei.npos[1]+radius,nei.npos[2], duRGBA(0,192,128,128));
        end;
      end;
      dd.&end();
    end;

    if (m_toolParams.m_showOpt) then
    begin
      dd.&begin(DU_DRAW_LINES, 2.0);
      dd.vertex(m_agentDebug.optStart[0],m_agentDebug.optStart[1]+0.3,m_agentDebug.optStart[2], duRGBA(0,128,0,192));
      dd.vertex(m_agentDebug.optEnd[0],m_agentDebug.optEnd[1]+0.3,m_agentDebug.optEnd[2], duRGBA(0,128,0,192));
      dd.&end();
    end;
  end;

  // Agent cylinders.
  for i := 0 to crowd.getAgentCount - 1 do
  begin
    ag := crowd.getAgent(i);
    if (not ag.active) then continue;

    radius := ag.params.radius;
    pos := @ag.npos[0];

    col := duRGBA(0,0,0,32);
    if (m_agentDebug.idx = i) then
      col := duRGBA(255,0,0,128);

    duDebugDrawCircle(dd, pos[0], pos[1], pos[2], radius, col, 2.0);
  end;

  for i := 0 to crowd.getAgentCount - 1 do
  begin
    ag := crowd.getAgent(i);
    if (not ag.active) then continue;

    height := ag.params.height;
    radius := ag.params.radius;
    pos := @ag.npos[0];
    
    col := duRGBA(220,220,220,128);
    if (ag.targetState = DT_CROWDAGENT_TARGET_REQUESTING) or (ag.targetState = DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE) then
      col := duLerpCol(col, duRGBA(128,0,255,128), 32)
    else if (ag.targetState = DT_CROWDAGENT_TARGET_WAITING_FOR_PATH) then
      col := duLerpCol(col, duRGBA(128,0,255,128), 128)
    else if (ag.targetState = DT_CROWDAGENT_TARGET_FAILED) then
      col := duRGBA(255,32,16,128)
    else if (ag.targetState = DT_CROWDAGENT_TARGET_VELOCITY) then
      col := duLerpCol(col, duRGBA(64,255,0,128), 128);

    duDebugDrawCylinder(dd, pos[0]-radius, pos[1]+radius*0.1, pos[2]-radius,
              pos[0]+radius, pos[1]+height, pos[2]+radius, col);
  end;


  if (m_toolParams.m_showVO) then
  begin
    for i := 0 to crowd.getAgentCount - 1 do
    begin
      if (m_toolParams.m_showDetailAll = false) and (i <> m_agentDebug.idx) then
        continue;
      ag := crowd.getAgent(i);
      if (not ag.active) then
        continue;
    
      // Draw detail about agent sela
      vod := m_agentDebug.vod;

      dx := ag.npos[0];
      dy := ag.npos[1]+ag.params.height;
      dz := ag.npos[2];

      duDebugDrawCircle(dd, dx,dy,dz, ag.params.maxSpeed, duRGBA(255,255,255,64), 2.0);
      
      dd.&begin(DU_DRAW_QUADS);
      for j := 0 to vod.getSampleCount - 1 do
      begin
        p := vod.getSampleVelocity(j);
        sr := vod.getSampleSize(j);
        pen := vod.getSamplePenalty(j);
        pen2 := vod.getSamplePreferredSidePenalty(j);
        col := duLerpCol(duRGBA(255,255,255,220), duRGBA(128,96,0,220), Round(pen*255));
        col := duLerpCol(col, duRGBA(128,0,0,220), Round(pen2*128));
        dd.vertex(dx+p[0]-sr, dy, dz+p[2]-sr, col);
        dd.vertex(dx+p[0]-sr, dy, dz+p[2]+sr, col);
        dd.vertex(dx+p[0]+sr, dy, dz+p[2]+sr, col);
        dd.vertex(dx+p[0]+sr, dy, dz+p[2]-sr, col);
      end;
      dd.&end();
    end;
  end;
  
  // Velocity stuff.
  for i := 0 to crowd.getAgentCount - 1 do
  begin
    ag := crowd.getAgent(i);
    if (not ag.active) then continue;

    radius := ag.params.radius;
    height := ag.params.height;
    pos := @ag.npos[0];
    vel := @ag.vel[0];
    dvel := @ag.dvel[0];
    
    col := duRGBA(220,220,220,192);
    if (ag.targetState = DT_CROWDAGENT_TARGET_REQUESTING) or (ag.targetState = DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE) then
      col := duLerpCol(col, duRGBA(128,0,255,192), 32)
    else if (ag.targetState = DT_CROWDAGENT_TARGET_WAITING_FOR_PATH) then
      col := duLerpCol(col, duRGBA(128,0,255,192), 128)
    else if (ag.targetState = DT_CROWDAGENT_TARGET_FAILED) then
      col := duRGBA(255,32,16,192)
    else if (ag.targetState = DT_CROWDAGENT_TARGET_VELOCITY) then
      col := duLerpCol(col, duRGBA(64,255,0,192), 128);
    
    duDebugDrawCircle(dd, pos[0], pos[1]+height, pos[2], radius, col, 2.0);

    duDebugDrawArrow(dd, pos[0],pos[1]+height,pos[2],
             pos[0]+dvel[0],pos[1]+height+dvel[1],pos[2]+dvel[2],
             0.0, 0.4, duRGBA(0,192,255,192), IfThen(m_agentDebug.idx = i, 2.0, 1.0));

    duDebugDrawArrow(dd, pos[0],pos[1]+height,pos[2],
             pos[0]+vel[0],pos[1]+height+vel[1],pos[2]+vel[2],
             0.0, 0.4, duRGBA(0,0,0,160), 2.0);
  end;

  dd.depthMask(true);

  dd.Free;
end;


procedure TCrowdToolState.handleRenderOverlay(proj, model: PDouble; view: PInteger);
//var crowd: TdtCrowd;
begin
  {GLdouble x, y, z;

  // Draw start and end point labels
  if (m_targetRef && gluProject((GLdouble)m_targetPos[0], (GLdouble)m_targetPos[1], (GLdouble)m_targetPos[2],
                  model, proj, view, &x, &y, &z))
  begin
    imguiDrawText((int)x, (int)(y+25), IMGUI_ALIGN_CENTER, "TARGET", imguiRGBA(0,0,0,220));
  end;

  char label[32];

  if (m_toolParams.m_showNodes)
  begin
    crowd := m_sample.getCrowd;
    if (crowd && crowd.getPathQueue())
    begin
      const dtNavMeshQuery* navquery := crowd.getPathQueue().getNavQuery();
      const dtNodePool* pool := navquery.getNodePool();
      if (pool)
      begin
        const float off := 0.5f;
        for (int i := 0; i < pool.getHashSize(); ++i)
        begin
          for (dtNodeIndex j := pool.getFirst(i); j != DT_NULL_IDX; j := pool.getNext(j))
          begin
            const dtNode* node := pool.getNodeAtIdx(j+1);
            if (!node) continue;

            if (gluProject((GLdouble)node.pos[0],(GLdouble)node.pos[1]+off,(GLdouble)node.pos[2],
                     model, proj, view, &x, &y, &z))
            begin
              const float heuristic := node.total;// - node.cost;
              snprintf(label, 32, "%.2f", heuristic);
              imguiDrawText((int)x, (int)y+15, IMGUI_ALIGN_CENTER, label, imguiRGBA(0,0,0,220));
            end;
          end;
        end;
      end;
    end;
  end;

  if (m_toolParams.m_showLabels)
  begin
    crowd := m_sample.getCrowd;
    if (crowd)
    begin
      for i := 0 to crowd.getAgentCount - 1 do
      begin
        const dtCrowdAgent* ag := crowd.getAgent(i);
        if (not ag.active) then continue;
        const float* pos := ag.npos;
        const float h := ag.params.height;
        if (gluProject((GLdouble)pos[0], (GLdouble)pos[1]+h, (GLdouble)pos[2],
                 model, proj, view, &x, &y, &z))
        begin
          snprintf(label, 32, "%d", i);
          imguiDrawText((int)x, (int)y+15, IMGUI_ALIGN_CENTER, label, imguiRGBA(0,0,0,220));
        end;
      end;
    end;
  end;
  if (m_agentDebug.idx <> -1)
  begin
    crowd := m_sample.getCrowd;
    if (crowd)
    begin
      for i := 0 to crowd.getAgentCount - 1 do
      begin
        if (m_toolParams.m_showDetailAll = false) and (i <> m_agentDebug.idx) then
          continue;
        ag := crowd.getAgent(i);
        if (not ag.active) then
          continue;
        radius := ag.params.radius;
        if (m_toolParams.m_showNeis) then
        begin
          for j := 0 to ag.nneis - 1 do
          begin
            nei := crowd.getAgent(ag.neis[j].idx);
            if (not nei.active) then continue;
            
            if (gluProject((GLdouble)nei.npos[0], (GLdouble)nei.npos[1]+radius, (GLdouble)nei.npos[2],
                     model, proj, view, &x, &y, &z))
            begin
              snprintf(label, 32, '%.3f', ag.neis[j].dist);
              imguiDrawText((int)x, (int)y+15, IMGUI_ALIGN_CENTER, label, imguiRGBA(255,255,255,220));
            end;
          end;
        end;
      end;
    end;
  end;


  if (m_toolParams.m_showPerfGraph)
  begin
    GraphParams gp;
    gp.setRect(300, 10, 500, 200, 8);
    gp.setValueRange(0.0f, 2.0f, 4, "ms");

    drawGraphBackground(&gp);
    drawGraph(&gp, &m_crowdTotalTime, 1, "Total", duRGBA(255,128,0,255));

    gp.setRect(300, 10, 500, 50, 8);
    gp.setValueRange(0.0f, 2000.0f, 1, "");
    drawGraph(&gp, &m_crowdSampleCount, 0, "Sample Count", duRGBA(96,96,96,128));
  end;
  }
end;

procedure TCrowdToolState.handleUpdate(dt: Single);
begin
  if (m_run) then
    updateTick(dt);
end;

procedure TCrowdToolState.addAgent(const pos: PSingle);
var crowd: TdtCrowd; ap: TdtCrowdAgentParams; idx, i: Integer; trail: PAgentTrail;
begin
  if (m_sample = nil) then Exit;
  crowd := m_sample.getCrowd;

  FillChar(ap, sizeof(TdtCrowdAgentParams), 0);
  ap.radius := m_sample.getAgentRadius;
  ap.height := m_sample.getAgentHeight;
  ap.maxAcceleration := 8.0;
  ap.maxSpeed := 3.5;
  ap.collisionQueryRange := ap.radius * 12.0;
  ap.pathOptimizationRange := ap.radius * 30.0;
  ap.updateFlags := 0;
  if (m_toolParams.m_anticipateTurns) then
    ap.updateFlags := ap.updateFlags or DT_CROWD_ANTICIPATE_TURNS;
  if (m_toolParams.m_optimizeVis) then
    ap.updateFlags := ap.updateFlags or DT_CROWD_OPTIMIZE_VIS;
  if (m_toolParams.m_optimizeTopo) then
    ap.updateFlags := ap.updateFlags or DT_CROWD_OPTIMIZE_TOPO;
  if (m_toolParams.m_obstacleAvoidance) then
    ap.updateFlags := ap.updateFlags or DT_CROWD_OBSTACLE_AVOIDANCE;
  if (m_toolParams.m_separation) then
    ap.updateFlags := ap.updateFlags or DT_CROWD_SEPARATION;
  ap.obstacleAvoidanceType := Trunc(m_toolParams.m_obstacleAvoidanceType);
  ap.separationWeight := m_toolParams.m_separationWeight;

  idx := crowd.addAgent(pos, @ap);
  if (idx <> -1) then
  begin
    if (m_targetRef <> 0) then
      crowd.requestMoveTarget(idx, m_targetRef, @m_targetPos[0]);

    // Init trail
    trail := @m_trails[idx];
    for i := 0 to AGENT_MAX_TRAIL - 1 do
      dtVcopy(@trail.trail[i*3], pos);
    trail.htrail := 0;
  end;
end;

procedure TCrowdToolState.removeAgent(const idx: Integer);
var crowd: TdtCrowd;
begin
  if (m_sample = nil) then Exit;
  crowd := m_sample.getCrowd;

  crowd.removeAgent(idx);
  
  if (idx = m_agentDebug.idx) then
    m_agentDebug.idx := -1;
end;

procedure TCrowdToolState.hilightAgent(const idx: Integer);
begin
  m_agentDebug.idx := idx;
end;

procedure calcVel(vel, pos, tgt: PSingle; const speed: Single);
begin
  dtVsub(vel, tgt, pos);
  vel[1] := 0.0;
  dtVnormalize(vel);
  dtVscale(vel, vel, speed);
end;

procedure TCrowdToolState.setMoveTarget(const p: PSingle; adjust: Boolean);
var crowd: TdtCrowd; navquery: TdtNavMeshQuery; filter: TdtQueryFilter; ext: PSingle; vel: array [0..2] of Single;
ag: PdtCrowdAgent; i: Integer;
begin
  if (m_sample = nil) then Exit;

  // Find nearest point on navmesh and set move request to that location.
  navquery := m_sample.getNavMeshQuery;
  crowd := m_sample.getCrowd;
  filter := crowd.getFilter(0);
  ext := crowd.getQueryExtents();

  if (adjust) then
  begin
    // Request velocity
    if (m_agentDebug.idx <> -1) then
    begin
      ag := crowd.getAgent(m_agentDebug.idx);
      if (ag <> nil) and ag.active then
      begin
        calcVel(@vel[0], @ag.npos[0], p, ag.params.maxSpeed);
        crowd.requestMoveVelocity(m_agentDebug.idx, @vel[0]);
      end;
    end
    else
    begin
      for i := 0 to crowd.getAgentCount - 1 do
      begin
        ag := crowd.getAgent(i);
        if (not ag.active) then continue;
        calcVel(@vel[0], @ag.npos[0], p, ag.params.maxSpeed);
        crowd.requestMoveVelocity(i, @vel[0]);
      end;
    end;
  end
  else
  begin
    navquery.findNearestPoly(p, ext, filter, @m_targetRef, @m_targetPos[0]);
    
    if (m_agentDebug.idx <> -1) then
    begin
      ag := crowd.getAgent(m_agentDebug.idx);
      if (ag <> nil) and ag.active then
        crowd.requestMoveTarget(m_agentDebug.idx, m_targetRef, @m_targetPos[0]);
    end
    else
    begin
      for i := 0 to crowd.getAgentCount - 1 do
      begin
        ag := crowd.getAgent(i);
        if (not ag.active) then continue;
        crowd.requestMoveTarget(i, m_targetRef, @m_targetPos[0]);
      end;
    end;
  end;
end;

function TCrowdToolState.hitTestAgents(const s, p: PSingle): Integer;
var crowd: TdtCrowd; isel, i: Integer; tsel, tmin, tmax: Single; ag: PdtCrowdAgent; bmin, bmax: array [0..2] of Single;
begin
  if (m_sample = nil) then Exit(-1);
  crowd := m_sample.getCrowd;

  isel := -1;
  tsel := MaxSingle;

  for i := 0 to crowd.getAgentCount - 1 do
  begin
    ag := crowd.getAgent(i);
    if (not ag.active) then continue;
    getAgentBounds(ag, @bmin[0], @bmax[0]);

    if (isectSegAABB(s, p, @bmin[0],@bmax[0], @tmin, @tmax)) then
    begin
      if (tmin > 0) and (tmin < tsel) then
      begin
        isel := i;
        tsel := tmin;
      end;
    end;
  end;

  Result := isel;
end;

procedure TCrowdToolState.updateAgentParams();
var crowd: TdtCrowd; updateFlags, obstacleAvoidanceType: Byte; params: TdtCrowdAgentParams; i: Integer; ag: PdtCrowdAgent;
begin
  if (m_sample = nil) then Exit;
  crowd := m_sample.getCrowd;
  if (crowd = nil) then Exit;

  updateFlags := 0;
  obstacleAvoidanceType := 0;

  if (m_toolParams.m_anticipateTurns) then
    updateFlags := updateFlags or DT_CROWD_ANTICIPATE_TURNS;
  if (m_toolParams.m_optimizeVis) then
    updateFlags := updateFlags or DT_CROWD_OPTIMIZE_VIS;
  if (m_toolParams.m_optimizeTopo) then
    updateFlags := updateFlags or DT_CROWD_OPTIMIZE_TOPO;
  if (m_toolParams.m_obstacleAvoidance) then
    updateFlags := updateFlags or DT_CROWD_OBSTACLE_AVOIDANCE;
  if (m_toolParams.m_separation) then
    updateFlags := updateFlags or DT_CROWD_SEPARATION;

  obstacleAvoidanceType := Trunc(m_toolParams.m_obstacleAvoidanceType);

  for i := 0 to crowd.getAgentCount - 1 do
  begin
    ag := crowd.getAgent(i);
    if (not ag.active) then continue;
    Move(ag.params, params, sizeof(TdtCrowdAgentParams));
    params.updateFlags := updateFlags;
    params.obstacleAvoidanceType := obstacleAvoidanceType;
    params.separationWeight := m_toolParams.m_separationWeight;
    crowd.updateAgentParameters(i, @params);
  end;
end;

procedure TCrowdToolState.updateTick(const dt: Single);
var crowd: TdtCrowd; nav: TdtNavMesh; startTime, endTime: Int64; i: Integer; ag: PdtCrowdAgent; trail: PAgentTrail;
begin
  if (m_sample = nil) then Exit;
  nav := m_sample.getNavMesh;
  crowd := m_sample.getCrowd;
  if (nav = nil) or (crowd = nil) then Exit;

  startTime := getPerfTime();

  crowd.update(dt, @m_agentDebug);

  endTime := getPerfTime();

  // Update agent trails
  for i := 0 to crowd.getAgentCount - 1 do
  begin
    ag := crowd.getAgent(i);
    trail := @m_trails[i];
    if (not ag.active) then
      continue;
    // Update agent movement trail.
    trail.htrail := (trail.htrail + 1) mod AGENT_MAX_TRAIL;
    dtVcopy(@trail.trail[trail.htrail*3], @ag.npos[0]);
  end;

  m_agentDebug.vod.normalizeSamples();

  m_crowdSampleCount.addSample(crowd.getVelocitySampleCount);
  m_crowdTotalTime.addSample(getPerfDeltaTimeUsec(startTime, endTime) / 1000.0);
end;

procedure TCrowdToolState.setRunning(const s: Boolean); begin m_run := s; end;
function TCrowdToolState.getToolParams: PCrowdToolParams; begin Result := @m_toolParams; end;



constructor TCrowdTool.Create(aOwner: TWinControl);
begin
  inherited Create;
  m_sample := nil;
  m_state := nil;
  m_mode := TOOLMODE_CREATE;
  &type := TOOL_CROWD;

  fFrame := TFrameCrowdTool.Create(aOwner);
  fFrame.Align := alClient;
  fFrame.Parent := aOwner;
  fFrame.Visible := True;
  fFrame.rgCrowdToolMode.OnClick := handleMenu;
  fFrame.cbOptimizeVisibility.OnClick := handleMenu;
  fFrame.cbOptimizeTopology.OnClick := handleMenu;
  fFrame.cbAnticipateTurns.OnClick := handleMenu;
  fFrame.cbObstacleAvoidance.OnClick := handleMenu;
  fFrame.cbSeparation.OnClick := handleMenu;
end;

destructor TCrowdTool.Destroy;
begin
  fFrame.Free;
  m_state.Free;

  inherited;
end;

procedure TCrowdTool.init(sample: TSample);
begin
  if (m_sample <> sample) then
  begin
    m_sample := sample;
  end;

  if (sample = nil) then
    Exit;

  m_state := TCrowdToolState(sample.getToolState(&type));
  if (m_state = nil) then
  begin
    m_state := TCrowdToolState.Create;
    sample.setToolState(&type, m_state);
  end;
  m_state.init(sample);
end;

procedure TCrowdTool.reset();
begin
end;

procedure TCrowdTool.handleMenu(Sender: TObject);
var params: PCrowdToolParams;
begin
  if fUpdateUI then Exit;

  if (m_state = nil) then
    Exit;
  params := m_state.getToolParams;

  // Delphi: When the Sender is NIL, we fill the controls with current state values
  if Sender = nil then
  begin
    fUpdateUI := True;
    fFrame.rgCrowdToolMode.ItemIndex := Byte(m_mode);

    fFrame.cbOptimizeVisibility.Checked := params.m_optimizeVis;
    fFrame.cbOptimizeTopology.Checked := params.m_optimizeTopo;
    fFrame.cbAnticipateTurns.Checked := params.m_anticipateTurns;
    fFrame.cbObstacleAvoidance.Checked := params.m_obstacleAvoidance;
    fFrame.cbSeparation.Checked := params.m_separation;
    fUpdateUI := False;
  end;

  if Sender = fFrame.rgCrowdToolMode then
  begin
    m_mode := TToolMode(fFrame.rgCrowdToolMode.ItemIndex);

    params.m_optimizeVis := fFrame.cbOptimizeVisibility.Checked;
    params.m_optimizeTopo := fFrame.cbOptimizeTopology.Checked;
    params.m_anticipateTurns := fFrame.cbAnticipateTurns.Checked;
    params.m_obstacleAvoidance := fFrame.cbObstacleAvoidance.Checked;
    params.m_separation := fFrame.cbSeparation.Checked;
    m_state.updateAgentParams();

  end;

  {
  if (params.m_expandOptions)
  begin
    imguiIndent();
    if (imguiCheck("Optimize Visibility", params.m_optimizeVis))
    begin
      params.m_optimizeVis := !params.m_optimizeVis;
      m_state.updateAgentParams();
    end;
    if (imguiCheck("Optimize Topology", params.m_optimizeTopo))
    begin
      params.m_optimizeTopo := !params.m_optimizeTopo;
      m_state.updateAgentParams();
    end;
    if (imguiCheck("Anticipate Turns", params.m_anticipateTurns))
    begin
      params.m_anticipateTurns := !params.m_anticipateTurns;
      m_state.updateAgentParams();
    end;
    if (imguiCheck("Obstacle Avoidance", params.m_obstacleAvoidance))
    begin
      params.m_obstacleAvoidance := !params.m_obstacleAvoidance;
      m_state.updateAgentParams();
    end;
    if (imguiSlider("Avoidance Quality", &params.m_obstacleAvoidanceType, 0.0f, 3.0f, 1.0f))
    begin
      m_state.updateAgentParams();
    end;
    if (imguiCheck("Separation", params.m_separation))
    begin
      params.m_separation := !params.m_separation;
      m_state.updateAgentParams();
    end;
    if (imguiSlider("Separation Weight", &params.m_separationWeight, 0.0f, 20.0f, 0.01f))
    begin
      m_state.updateAgentParams();
    end;
    
    imguiUnindent();
  end;

  if (imguiCollapse("Selected Debug Draw", 0, params.m_expandSelectedDebugDraw))
    params.m_expandSelectedDebugDraw := !params.m_expandSelectedDebugDraw;
    
  if (params.m_expandSelectedDebugDraw)
  begin
    imguiIndent();
    if (imguiCheck("Show Corners", params.m_showCorners))
      params.m_showCorners := !params.m_showCorners;
    if (imguiCheck("Show Collision Segs", params.m_showCollisionSegments))
      params.m_showCollisionSegments := !params.m_showCollisionSegments;
    if (imguiCheck("Show Path", params.m_showPath))
      params.m_showPath := !params.m_showPath;
    if (imguiCheck("Show VO", params.m_showVO))
      params.m_showVO := !params.m_showVO;
    if (imguiCheck("Show Path Optimization", params.m_showOpt))
      params.m_showOpt := !params.m_showOpt;
    if (imguiCheck("Show Neighbours", params.m_showNeis))
      params.m_showNeis := !params.m_showNeis;
    imguiUnindent();
  end;
    
  if (imguiCollapse("Debug Draw", 0, params.m_expandDebugDraw))
    params.m_expandDebugDraw := !params.m_expandDebugDraw;
  
  if (params.m_expandDebugDraw)
  begin
    imguiIndent();
    if (imguiCheck("Show Labels", params.m_showLabels))
      params.m_showLabels := !params.m_showLabels;
    if (imguiCheck("Show Prox Grid", params.m_showGrid))
      params.m_showGrid := !params.m_showGrid;
    if (imguiCheck("Show Nodes", params.m_showNodes))
      params.m_showNodes := !params.m_showNodes;
    if (imguiCheck("Show Perf Graph", params.m_showPerfGraph))
      params.m_showPerfGraph := !params.m_showPerfGraph;
    if (imguiCheck("Show Detail All", params.m_showDetailAll))
      params.m_showDetailAll := !params.m_showDetailAll;
    imguiUnindent();
  end;}
end;

procedure TCrowdTool.handleClick(s,p: PSingle; shift: Boolean);
var crowd: TdtCrowd; geom: TInputGeom; ahit: Integer; nav: TdtNavMesh; navquery: TdtNavMeshQuery; filter: TdtQueryFilter;
ext: PSingle; tgt: array [0..2] of Single; ref: TdtPolyRef; flags: Word;
begin
  if (m_sample = nil) then Exit;
  if (m_state = nil) then Exit;
  geom := m_sample.getInputGeom;
  if (geom = nil) then Exit;
  crowd := m_sample.getCrowd;
  if (crowd = nil) then Exit;

  if (m_mode = TOOLMODE_CREATE) then
  begin
    if (shift) then
    begin
      // Delete
      ahit := m_state.hitTestAgents(s,p);
      if (ahit <> -1) then
        m_state.removeAgent(ahit);
    end
    else
    begin
      // Add
      m_state.addAgent(p);
    end;
  end
  else if (m_mode = TOOLMODE_MOVE_TARGET) then
  begin
    m_state.setMoveTarget(p, shift);
  end
  else if (m_mode = TOOLMODE_SELECT) then
  begin
    // Highlight
    ahit := m_state.hitTestAgents(s,p);
    m_state.hilightAgent(ahit);
  end
  else if (m_mode = TOOLMODE_TOGGLE_POLYS) then
  begin
    nav := m_sample.getNavMesh;
    navquery := m_sample.getNavMeshQuery;
    if (nav <> nil) and (navquery <> nil) then
    begin
      filter := TdtQueryFilter.Create; // Delphi: Assume c++ creates a dummy here
      ext := crowd.getQueryExtents();
      navquery.findNearestPoly(p, ext, filter, @ref, @tgt[0]);
      if (ref <> 0) then
      begin
        flags := 0;
        if (dtStatusSucceed(nav.getPolyFlags(ref, @flags))) then
        begin
          flags := flags xor SAMPLE_POLYFLAGS_DISABLED;
          nav.setPolyFlags(ref, flags);
        end;
      end;
      filter.Free;
    end;
  end;

end;

procedure TCrowdTool.handleStep();
var dt: Single;
begin
  if (m_state = nil) then Exit;

  dt := 1.0/20.0;
  m_state.updateTick(dt);

  m_state.setRunning(false);
end;

procedure TCrowdTool.handleToggle();
begin
  if (m_state = nil) then Exit;
  m_state.setRunning(not m_state.isRunning);
end;

procedure TCrowdTool.handleUpdate(dt: Single);
begin
  //rcIgnoreUnused(dt);
end;

procedure TCrowdTool.handleRender();
begin
end;

procedure TCrowdTool.handleRenderOverlay(proj, model: PDouble; view: PInteger);
begin
{  rcIgnoreUnused(model);
  rcIgnoreUnused(proj);

  // Tool help
  const int h := view[3];
  int ty := h-40;

  if (m_mode == TOOLMODE_CREATE)
  begin
    imguiDrawText(280, ty, IMGUI_ALIGN_LEFT, "LMB: add agent.  Shift+LMB: remove agent.", imguiRGBA(255,255,255,192));  
  end;
  else if (m_mode == TOOLMODE_MOVE_TARGET)
  begin
    imguiDrawText(280, ty, IMGUI_ALIGN_LEFT, "LMB: set move target.  Shift+LMB: adjust set velocity.", imguiRGBA(255,255,255,192));  
    ty -= 20;
    imguiDrawText(280, ty, IMGUI_ALIGN_LEFT, "Setting velocity will move the agents without pathfinder.", imguiRGBA(255,255,255,192));  
  end;
  else if (m_mode == TOOLMODE_SELECT)
  begin
    imguiDrawText(280, ty, IMGUI_ALIGN_LEFT, "LMB: select agent.", imguiRGBA(255,255,255,192));  
  end;
  ty -= 20;
  imguiDrawText(280, ty, IMGUI_ALIGN_LEFT, "SPACE: Run/Pause simulation.  1: Step simulation.", imguiRGBA(255,255,255,192));  
  ty -= 20;

  if (m_state && m_state.isRunning())
    imguiDrawText(280, ty, IMGUI_ALIGN_LEFT, "- RUNNING -", imguiRGBA(255,32,16,255));  
  else 
    imguiDrawText(280, ty, IMGUI_ALIGN_LEFT, "- PAUSED -", imguiRGBA(255,255,255,128));  }
end;


end.