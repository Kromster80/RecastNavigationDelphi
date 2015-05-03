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

unit RN_DetourObstacleAvoidance;
interface
uses Math, SysUtils;

type
  PdtObstacleCircle = ^TdtObstacleCircle;
  TdtObstacleCircle = record
    p: array [0..2] of Single;        ///< Position of the obstacle
    vel: array [0..2] of Single;      ///< Velocity of the obstacle
    dvel: array [0..2] of Single;      ///< Velocity of the obstacle
    rad: Single;        ///< Radius of the obstacle
    dp, np: array [0..2] of Single;    ///< Use for side selection during sampling.
  end;

  PdtObstacleSegment = ^TdtObstacleSegment;
  TdtObstacleSegment = record
    p, q: array [0..2] of Single;    ///< End points of the obstacle segment
    touch: Boolean;
  end;


  TdtObstacleAvoidanceDebugData = class
  public
    constructor Create;
    destructor Destroy; override;

    function init(const maxSamples: Integer): Boolean;
    procedure reset();
    procedure addSample(const vel: PSingle; const ssize, pen, vpen, vcpen, spen, tpen: Single);

    procedure normalizeSamples();

    function getSampleCount(): Integer;
    function getSampleVelocity(i: Integer): PSingle;
    function getSampleSize(i: Integer): Single;
    function getSamplePenalty(i: Integer): Single;
    function getSampleDesiredVelocityPenalty(i: Integer): Single;
    function getSampleCurrentVelocityPenalty(i: Integer): Single;
    function getSamplePreferredSidePenalty(i: Integer): Single;
    function getSampleCollisionTimePenalty(i: Integer): Single;

  private
    m_nsamples: Integer;
    m_maxSamples: Integer;
    m_vel: PSingle;
    m_ssize: PSingle;
    m_pen: PSingle;
    m_vpen: PSingle;
    m_vcpen: PSingle;
    m_spen: PSingle;
    m_tpen: PSingle;
  end;

  function dtAllocObstacleAvoidanceDebugData(): TdtObstacleAvoidanceDebugData;
  procedure dtFreeObstacleAvoidanceDebugData(var ptr: TdtObstacleAvoidanceDebugData);

const
  DT_MAX_PATTERN_DIVS = 32;  ///< Max numver of adaptive divs.
  DT_MAX_PATTERN_RINGS = 4;  ///< Max number of adaptive rings.

type
  PdtObstacleAvoidanceParams = ^TdtObstacleAvoidanceParams;
  TdtObstacleAvoidanceParams = record
    velBias: Single;
    weightDesVel: Single;
    weightCurVel: Single;
    weightSide: Single;
    weightToi: Single;
    horizTime: Single;
    gridSize: Byte;  ///< grid
    adaptiveDivs: Byte;  ///< adaptive
    adaptiveRings: Byte;  ///< adaptive
    adaptiveDepth: Byte;  ///< adaptive
  end;

  TdtObstacleAvoidanceQuery = class
  public
    constructor Create;
    destructor Destroy; override;

    function init(const maxCircles, maxSegments: Integer): Boolean;

    procedure reset();

    procedure addCircle(const pos: PSingle; const rad: Single;
             const vel, dvel: PSingle);

    procedure addSegment(const p, q: PSingle);

    function sampleVelocityGrid(const pos: PSingle; const rad, vmax: Single;
                 const vel, dvel, nvel: PSingle;
                 const params: PdtObstacleAvoidanceParams;
                 debug: TdtObstacleAvoidanceDebugData = nil): Integer;

    function sampleVelocityAdaptive(const pos: PSingle; const rad, vmax: Single;
                   const vel, dvel, nvel: PSingle;
                   const params: PdtObstacleAvoidanceParams;
                   debug: TdtObstacleAvoidanceDebugData = nil): Integer;

    function getObstacleCircleCount(): Integer; { return m_ncircles; }
    function getObstacleCircle(i: Integer): PdtObstacleCircle; { return &m_circles[i]; }

    function getObstacleSegmentCount(): Integer; { return m_nsegments; }
    function getObstacleSegment(i: Integer): PdtObstacleSegment; { return &m_segments[i]; }

  private

    procedure prepare(const pos, dvel: PSingle);

    function processSample(const vcand: PSingle; const cs: Single;
              const pos: PSingle; const rad: Single;
              const vel, dvel: PSingle;
              const minPenalty: Single;
              debug: TdtObstacleAvoidanceDebugData): Single;

// Delphi: Unused ever?    function insertCircle(const dist: Single): PdtObstacleCircle;
// Delphi: Unused ever?    function insertSegment(const dist: Single): PdtObstacleSegment;
  private
    m_params: TdtObstacleAvoidanceParams;
    m_invHorizTime: Single;
    m_vmax: Single;
    m_invVmax: Single;

    m_maxCircles: Integer;
    m_circles: PdtObstacleCircle;
    m_ncircles: Integer;

    m_maxSegments: Integer;
    m_segments: PdtObstacleSegment;
    m_nsegments: Integer;
  end;

  function dtAllocObstacleAvoidanceQuery(): TdtObstacleAvoidanceQuery;
  procedure dtFreeObstacleAvoidanceQuery(var ptr: TdtObstacleAvoidanceQuery);

implementation
uses RN_DetourCommon;


function sweepCircleCircle(const c0: PSingle; const r0: Single; const v: PSingle;
               const c1: PSingle; const r1: Single;
               tmin, tmax: PSingle): Integer;
const EPS = 0.0001;
var s: array [0..2] of Single; r,c,a,b,d,rd: Single;
begin
  dtVsub(@s[0],c1,c0);
  r := r0+r1;
  c := dtVdot2D(@s[0],@s[0]) - r*r;
  a := dtVdot2D(v,v);
  if (a < EPS) then Exit(0);  // not moving

  // Overlap, calc time to exit.
  b := dtVdot2D(v,@s[0]);
  d := b*b - a*c;
  if (d < 0.0) then Exit(0); // no intersection.
  a := 1.0 / a;
  rd := Sqrt(d);
  tmin^ := (b - rd) * a;
  tmax^ := (b + rd) * a;
  Result := 1;
end;

function isectRaySeg(const ap, u, bp, bq: PSingle;
             t: PSingle): Integer;
var v,w: array [0..2] of Single; d,s: Single;
begin
  dtVsub(@v[0],bq,bp);
  dtVsub(@w[0],ap,bp);
  d := dtVperp2D(u,@v[0]);
  if (abs(d) < 0.000001) then Exit(0);
  d := 1.0/d;
  t^ := dtVperp2D(@v[0],@w[0]) * d;
  if (t^ < 0) or (t^ > 1) then Exit(0);
  s := dtVperp2D(u,@w[0]) * d;
  if (s < 0) or (s > 1) then Exit(0);
  Result := 1;
end;



function dtAllocObstacleAvoidanceDebugData(): TdtObstacleAvoidanceDebugData;
begin
  Result := TdtObstacleAvoidanceDebugData.Create;
end;

procedure dtFreeObstacleAvoidanceDebugData(var ptr: TdtObstacleAvoidanceDebugData);
begin
  FreeAndNil(ptr);
end;


constructor TdtObstacleAvoidanceDebugData.Create();
begin
  inherited;

  m_nsamples := 0;
  m_maxSamples := 0;
  m_vel := nil;
  m_ssize := nil;
  m_pen := nil;
  m_vpen := nil;
  m_vcpen := nil;
  m_spen := nil;
  m_tpen := nil;
end;

destructor TdtObstacleAvoidanceDebugData.Destroy;
begin
  if m_vel <> nil then FreeMem(m_vel);
  if m_ssize <> nil then FreeMem(m_ssize);
  if m_pen <> nil then FreeMem(m_pen);
  if m_vpen <> nil then FreeMem(m_vpen);
  if m_vcpen <> nil then FreeMem(m_vcpen);
  if m_spen <> nil then FreeMem(m_spen);
  if m_tpen <> nil then FreeMem(m_tpen);

  inherited;
end;

function TdtObstacleAvoidanceDebugData.init(const maxSamples: Integer): Boolean;
begin
  Assert(maxSamples <> 0);
  m_maxSamples := maxSamples;

  GetMem(m_vel, sizeof(Single)*3*m_maxSamples);
  GetMem(m_pen, sizeof(Single)*m_maxSamples);
  GetMem(m_ssize, sizeof(Single)*m_maxSamples);
  GetMem(m_vpen, sizeof(Single)*m_maxSamples);
  GetMem(m_vcpen, sizeof(Single)*m_maxSamples);
  GetMem(m_spen, sizeof(Single)*m_maxSamples);
  GetMem(m_tpen, sizeof(Single)*m_maxSamples);

  Result := true;
end;

procedure TdtObstacleAvoidanceDebugData.reset();
begin
  m_nsamples := 0;
end;

procedure TdtObstacleAvoidanceDebugData.addSample(const vel: PSingle; const ssize, pen, vpen, vcpen, spen, tpen: Single);
begin
  if (m_nsamples >= m_maxSamples) then
    Exit;
  Assert(m_vel <> nil);
  Assert(m_ssize <> nil);
  Assert(m_pen <> nil);
  Assert(m_vpen <> nil);
  Assert(m_vcpen <> nil);
  Assert(m_spen <> nil);
  Assert(m_tpen <> nil);
  dtVcopy(@m_vel[m_nsamples*3], vel);
  m_ssize[m_nsamples] := ssize;
  m_pen[m_nsamples] := pen;
  m_vpen[m_nsamples] := vpen;
  m_vcpen[m_nsamples] := vcpen;
  m_spen[m_nsamples] := spen;
  m_tpen[m_nsamples] := tpen;
  Inc(m_nsamples);
end;

procedure normalizeArray(arr: PSingle; const n: Integer);
var minPen, maxPen, penRange, s: Single; i: Integer;
begin
  // Normalize penaly range.
  minPen := MaxSingle;
  maxPen := -MaxSingle;
  for i := 0 to n - 1 do
  begin
    minPen := dtMin(minPen, arr[i]);
    maxPen := dtMax(maxPen, arr[i]);
  end;
  penRange := maxPen-minPen;
  if penRange > 0.001 then s := 1.0 / penRange else s := 1;
  for i := 0 to n - 1 do
    arr[i] := dtClamp((arr[i]-minPen)*s, 0.0, 1.0);
end;

procedure TdtObstacleAvoidanceDebugData.normalizeSamples();
begin
  normalizeArray(m_pen, m_nsamples);
  normalizeArray(m_vpen, m_nsamples);
  normalizeArray(m_vcpen, m_nsamples);
  normalizeArray(m_spen, m_nsamples);
  normalizeArray(m_tpen, m_nsamples);
end;

function TdtObstacleAvoidanceDebugData.getSampleCount(): Integer; begin Result := m_nsamples; end;

function TdtObstacleAvoidanceDebugData.getSampleVelocity(i: Integer): PSingle; begin Result := @m_vel[i*3]; end;
function TdtObstacleAvoidanceDebugData.getSampleSize(i: Integer): Single; begin Result := m_ssize[i]; end;
function TdtObstacleAvoidanceDebugData.getSamplePenalty(i: Integer): Single; begin Result := m_pen[i]; end;
function TdtObstacleAvoidanceDebugData.getSampleDesiredVelocityPenalty(i: Integer): Single; begin Result := m_vpen[i]; end;
function TdtObstacleAvoidanceDebugData.getSampleCurrentVelocityPenalty(i: Integer): Single; begin Result := m_vcpen[i]; end;
function TdtObstacleAvoidanceDebugData.getSamplePreferredSidePenalty(i: Integer): Single; begin Result := m_spen[i]; end;
function TdtObstacleAvoidanceDebugData.getSampleCollisionTimePenalty(i: Integer): Single; begin Result := m_tpen[i]; end;

function dtAllocObstacleAvoidanceQuery(): TdtObstacleAvoidanceQuery;
begin
  Result := TdtObstacleAvoidanceQuery.Create;
end;

procedure dtFreeObstacleAvoidanceQuery(var ptr: TdtObstacleAvoidanceQuery);
begin
  FreeAndNil(ptr);
end;


constructor TdtObstacleAvoidanceQuery.Create();
begin
  inherited;

  m_maxCircles := 0;
  m_circles := nil;
  m_ncircles := 0;
  m_maxSegments := 0;
  m_segments := nil;
  m_nsegments := 0;
end;

destructor TdtObstacleAvoidanceQuery.Destroy;
begin
  FreeMem(m_circles);
  FreeMem(m_segments);

  inherited;
end;

function TdtObstacleAvoidanceQuery.init(const maxCircles, maxSegments: Integer): Boolean;
begin
  m_maxCircles := maxCircles;
  m_ncircles := 0;
  GetMem(m_circles, sizeof(TdtObstacleCircle)*m_maxCircles);
  FillChar(m_circles[0], sizeof(TdtObstacleCircle)*m_maxCircles, 0);

  m_maxSegments := maxSegments;
  m_nsegments := 0;
  GetMem(m_segments, sizeof(TdtObstacleSegment)*m_maxSegments);
  FillChar(m_segments[0], sizeof(TdtObstacleSegment)*m_maxSegments, 0);

  Result := true;
end;

procedure TdtObstacleAvoidanceQuery.reset();
begin
  m_ncircles := 0;
  m_nsegments := 0;
end;

procedure TdtObstacleAvoidanceQuery.addCircle(const pos: PSingle; const rad: Single;
             const vel, dvel: PSingle);
var cir: PdtObstacleCircle;
begin
  if (m_ncircles >= m_maxCircles) then
    Exit;

  cir := @m_circles[m_ncircles];
  Inc(m_ncircles);
  dtVcopy(@cir.p, pos);
  cir.rad := rad;
  dtVcopy(@cir.vel[0], vel);
  dtVcopy(@cir.dvel[0], dvel);
end;

procedure TdtObstacleAvoidanceQuery.addSegment(const p, q: PSingle);
var seg: PdtObstacleSegment;
begin
  if (m_nsegments >= m_maxSegments) then
    Exit;

  seg := @m_segments[m_nsegments];
  Inc(m_nsegments);
  dtVcopy(@seg.p, p);
  dtVcopy(@seg.q[0], q);
end;

procedure TdtObstacleAvoidanceQuery.prepare(const pos, dvel: PSingle);
var i: Integer; cir: PdtObstacleCircle; pa,pb: PSingle; orig,dv: array [0..2] of Single; a,r,t: Single; seg: PdtObstacleSegment;
begin
  // Prepare obstacles
  for i := 0 to m_ncircles - 1 do
  begin
    cir := @m_circles[i];

    // Side
    pa := pos;
    pb := @cir.p;

    orig[0] := 0; orig[1] := 0; orig[2] := 0;
    dtVsub(@cir.dp[0],pb,pa);
    dtVnormalize(@cir.dp[0]);
    dtVsub(@dv[0], @cir.dvel[0], dvel);

    a := dtTriArea2D(@orig[0], @cir.dp[0], @dv[0]);
    if (a < 0.01) then
    begin
      cir.np[0] := -cir.dp[2];
      cir.np[2] := cir.dp[0];
    end
    else
    begin
      cir.np[0] := cir.dp[2];
      cir.np[2] := -cir.dp[0];
    end;
  end;

  for i := 0 to m_nsegments - 1 do
  begin
    seg := @m_segments[i];

    // Precalc if the agent is really close to the segment.
    r := 0.01;
    seg.touch := dtDistancePtSegSqr2D(pos, @seg.p, @seg.q[0], @t) < Sqr(r);
  end;
end;


(* Calculate the collision penalty for a given velocity vector
 *
 * @param vcand sampled velocity
 * @param dvel desired velocity
 * @param minPenalty threshold penalty for early out
 *)
function TdtObstacleAvoidanceQuery.processSample(const vcand: PSingle; const cs: Single;
              const pos: PSingle; const rad: Single;
              const vel, dvel: PSingle;
              const minPenalty: Single;
              debug: TdtObstacleAvoidanceDebugData): Single;
const FLT_EPSILON = 1.19209290E-07; // decimal constant
var vpen, vcpen, minPen, tmin, side: Single; tThresold: Double; nside,i: Integer; cir: PdtObstacleCircle; vab, sdir, snorm: array [0..2] of Single;
htmin, htmax,spen, tpen, penalty: Single; seg: PdtObstacleSegment;
begin
  // penalty for straying away from the desired and current velocities
  vpen := m_params.weightDesVel * (dtVdist2D(vcand, dvel) * m_invVmax);
  vcpen := m_params.weightCurVel * (dtVdist2D(vcand, vel) * m_invVmax);

  // find the threshold hit time to bail out based on the early out penalty
  // (see how the penalty is calculated below to understnad)
  minPen := minPenalty - vpen - vcpen;
  tThresold := (m_params.weightToi/minPen - 0.1) * m_params.horizTime;
  if (tThresold - m_params.horizTime > -FLT_EPSILON) then
    Exit(minPenalty); // already too much

  // Find min time of impact and exit amongst all obstacles.
  tmin := m_params.horizTime;
  side := 0;
  nside := 0;

  for i := 0 to m_ncircles - 1 do
  begin
    cir := @m_circles[i];

    // RVO
    dtVscale(@vab[0], vcand, 2);
    dtVsub(@vab[0], @vab[0], vel);
    dtVsub(@vab[0], @vab[0], @cir.vel[0]);

    // Side
    side := side + dtClamp(dtMin(dtVdot2D(@cir.dp[0],@vab[0])*0.5+0.5, dtVdot2D(@cir.np[0],@vab[0])*2), 0.0, 1.0);
    Inc(nside);

    htmin := 0; htmax := 0;
    if (sweepCircleCircle(pos,rad, @vab[0], @cir.p,cir.rad, @htmin, @htmax) = 0) then
      continue;

    // Handle overlapping obstacles.
    if (htmin < 0.0) and (htmax > 0.0) then
    begin
      // Avoid more when overlapped.
      htmin := -htmin * 0.5;
    end;

    if (htmin >= 0.0) then
    begin
      // The closest obstacle is somewhere ahead of us, keep track of nearest obstacle.
      if (htmin < tmin) then
      begin
        tmin := htmin;
        if (tmin < tThresold) then
          Exit(minPenalty);
      end;
    end;
  end;

  for i := 0 to m_nsegments - 1 do
  begin
    seg := @m_segments[i];
    htmin := 0;

    if (seg.touch) then
    begin
      // Special case when the agent is very close to the segment.
      dtVsub(@sdir[0], @seg.q[0], @seg.p[0]);
      snorm[0] := -sdir[2];
      snorm[2] := sdir[0];
      // If the velocity is pointing towards the segment, no collision.
      if (dtVdot2D(@snorm[0], vcand) < 0.0) then
        continue;
      // Else immediate collision.
      htmin := 0.0;
    end
    else
    begin
      if (isectRaySeg(pos, vcand, @seg.p[0], @seg.q[0], @htmin) = 0) then
        continue;
    end;

    // Avoid less when facing walls.
    htmin := htmin * 2.0;
    
    // The closest obstacle is somewhere ahead of us, keep track of nearest obstacle.
    if (htmin < tmin) then
    begin
      tmin := htmin;
      if (tmin < tThresold) then
        Exit(minPenalty);
    end;
  end;
  
  // Normalize side bias, to prevent it dominating too much.
  if (nside <> 0) then
    side := side / nside;
  
  spen := m_params.weightSide * side;
  tpen := m_params.weightToi * (1.0 / (0.1 + tmin*m_invHorizTime));

  penalty := vpen + vcpen + spen + tpen;
  
  // Store different penalties for debug viewing
  if (debug <> nil) then
    debug.addSample(vcand, cs, penalty, vpen, vcpen, spen, tpen);
  
  Result := penalty;
end;

function TdtObstacleAvoidanceQuery.sampleVelocityGrid(const pos: PSingle; const rad, vmax: Single;
                 const vel, dvel, nvel: PSingle;
                 const params: PdtObstacleAvoidanceParams;
                 debug: TdtObstacleAvoidanceDebugData = nil): Integer;
var cvx,cvz,cs,half,minPenalty,penalty: Single; ns,y,x: Integer; vcand: array [0..2] of Single;
begin
  prepare(pos, dvel);

  Move(params^, m_params, sizeof(TdtObstacleAvoidanceParams));
  m_invHorizTime := 1.0 / m_params.horizTime;
  m_vmax := vmax;
  if vmax > 0 then m_invVmax := 1.0 / vmax else m_invVmax := MaxSingle;

  dtVset(nvel, 0,0,0);

  if (debug <> nil) then
    debug.reset();

  cvx := dvel[0] * m_params.velBias;
  cvz := dvel[2] * m_params.velBias;
  cs := vmax * 2 * (1 - m_params.velBias) / (m_params.gridSize-1);
  half := (m_params.gridSize-1)*cs*0.5;

  minPenalty := MaxSingle;
  ns := 0;

  for y := 0 to m_params.gridSize - 1 do
  begin
    for x := 0 to m_params.gridSize - 1 do
    begin
      vcand[0] := cvx + x*cs - half;
      vcand[1] := 0;
      vcand[2] := cvz + y*cs - half;

      if (Sqr(vcand[0])+Sqr(vcand[2]) > Sqr(vmax+cs/2)) then continue;

      penalty := processSample(@vcand[0], cs, pos,rad,vel,dvel, minPenalty, debug);
      Inc(ns);
      if (penalty < minPenalty) then
      begin
        minPenalty := penalty;
        dtVcopy(nvel, @vcand[0]);
      end;
    end;
  end;

  Result := ns;
end;


// vector normalization that ignores the y-component.
procedure dtNormalize2D(v: PSingle);
var d: Single;
begin
  d := Sqrt(v[0]*v[0]+v[2]*v[2]);
  if (d=0) then
    Exit;
  d := 1.0 / d;
  v[0] := v[0] * d;
  v[2] := v[2] * d;
end;

// vector normalization that ignores the y-component.
procedure dtRorate2D(dest, v: PSingle; ang: Single);
var c,s: Single;
begin
  c := cos(ang);
  s := sin(ang);
  dest[0] := v[0]*c - v[2]*s;
  dest[2] := v[0]*s + v[2]*c;
  dest[1] := v[1];
end;


function TdtObstacleAvoidanceQuery.sampleVelocityAdaptive(const pos: PSingle; const rad, vmax: Single;
                   const vel, dvel, nvel: PSingle;
                   const params: PdtObstacleAvoidanceParams;
                   debug: TdtObstacleAvoidanceDebugData = nil): Integer;
var pat: array [0..(DT_MAX_PATTERN_DIVS*DT_MAX_PATTERN_RINGS+1)*2-1] of Single; npat: Integer; ndivs, nrings, depth, nd, nr, nd2: Integer;
da,ca,sa,r,cr,minPenalty,penalty: Single; ddir: array [0..5] of Single; j,i,k,ns: Integer; last1, last2: PSingle;
res,bvel,vcand: array [0..2] of Single;
begin
  prepare(pos, dvel);

  Move(params^, m_params, sizeof(TdtObstacleAvoidanceParams));
  m_invHorizTime := 1.0 / m_params.horizTime;
  m_vmax := vmax;
  if vmax > 0 then m_invVmax := 1.0 / vmax else m_invVmax := MaxSingle;

  dtVset(nvel, 0,0,0);

  if (debug <> nil) then
    debug.reset();

  // Build sampling pattern aligned to desired velocity.
  npat := 0;

  ndivs := m_params.adaptiveDivs;
  nrings := m_params.adaptiveRings;
  depth := m_params.adaptiveDepth;

  nd := dtClamp(ndivs, 1, DT_MAX_PATTERN_DIVS);
  nr := dtClamp(nrings, 1, DT_MAX_PATTERN_RINGS);
  nd2 := nd div 2;
  da := (1.0/nd) * PI*2;
  ca := cos(da);
  sa := sin(da);

  // desired direction
  dtVcopy(@ddir[0], dvel);
  dtNormalize2D(@ddir[0]);
  dtRorate2D(@ddir[3], @ddir[0], da*0.5); // rotated by da/2

  // Always add sample at zero
  pat[npat*2+0] := 0;
  pat[npat*2+1] := 0;
  Inc(npat);

  for j := 0 to nr - 1 do
  begin
    r := (nr-j)/nr;
    pat[npat*2+0] := ddir[(j mod 1)*3] * r;
    pat[npat*2+1] := ddir[(j mod 1)*3+2] * r;
    last1 := @pat[npat*2];
    last2 := last1;
    Inc(npat);

    i := 1;
    while (i < nd-1) do
    begin
      // get next point on the "right" (rotate CW)
      pat[npat*2+0] := last1[0]*ca + last1[1]*sa;
      pat[npat*2+1] := -last1[0]*sa + last1[1]*ca;
      // get next point on the "left" (rotate CCW)
      pat[npat*2+2] := last2[0]*ca - last2[1]*sa;
      pat[npat*2+3] := last2[0]*sa + last2[1]*ca;

      last1 := @pat[npat*2];
      last2 := last1 + 2;
      Inc(npat, 2);

      Inc(i, 2);
    end;

    if ((nd and 1) = 0) then
    begin
      pat[npat*2+2] := last2[0]*ca - last2[1]*sa;
      pat[npat*2+3] := last2[0]*sa + last2[1]*ca;
      Inc(npat);
    end;
  end;


  // Start sampling.
  cr := vmax * (1.0 - m_params.velBias);
  dtVset(@res[0], dvel[0] * m_params.velBias, 0, dvel[2] * m_params.velBias);
  ns := 0;

  for k := 0 to depth - 1 do
  begin
    minPenalty := MaxSingle;
    dtVset(@bvel[0], 0,0,0);
    
    for i := 0 to npat - 1 do
    begin
      vcand[0] := res[0] + pat[i*2+0]*cr;
      vcand[1] := 0;
      vcand[2] := res[2] + pat[i*2+1]*cr;
      
      if (Sqr(vcand[0])+Sqr(vcand[2]) > Sqr(vmax+0.001)) then continue;

      penalty := processSample(@vcand[0],cr/10, pos,rad,vel,dvel, minPenalty, debug);
      Inc(ns);
      if (penalty < minPenalty) then
      begin
        minPenalty := penalty;
        dtVcopy(@bvel[0], @vcand[0]);
      end;
    end;

    dtVcopy(@res[0], @bvel[0]);

    cr := cr * 0.5;
  end;

  dtVcopy(nvel, @res[0]);

  Result := ns;
end;

function TdtObstacleAvoidanceQuery.getObstacleCircleCount(): Integer; begin Result := m_ncircles; end;
function TdtObstacleAvoidanceQuery.getObstacleCircle(i: Integer): PdtObstacleCircle; begin Result := @m_circles[i]; end;

function TdtObstacleAvoidanceQuery.getObstacleSegmentCount(): Integer; begin Result := m_nsegments; end;
function TdtObstacleAvoidanceQuery.getObstacleSegment(i: Integer): PdtObstacleSegment; begin Result := @m_segments[i]; end;


end.
