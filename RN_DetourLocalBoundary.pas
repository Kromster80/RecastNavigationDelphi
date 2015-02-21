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
unit RN_DetourLocalBoundary;
interface
uses RN_DetourNavMeshQuery, RN_DetourNavMeshHelper;

type
  TdtLocalBoundary = class
  private
    const
    MAX_LOCAL_SEGS = 8;
    MAX_LOCAL_POLYS = 16;

    type
    PSegment = ^TSegment;
    TSegment = record
      s: array [0..5] of Single; ///< Segment start/end
      d: Single; ///< Distance for pruning.
    end;

    var
    m_center: array [0..2] of Single;
    m_segs: array [0..MAX_LOCAL_SEGS-1] of TSegment;
    m_nsegs: Integer;

    m_polys: array [0..MAX_LOCAL_POLYS-1] of TdtPolyRef;
    m_npolys: Integer;

    procedure addSegment(const dist: Single; const s: PSingle);

  public
    constructor Create;
    destructor Destroy; override;

    procedure reset();

    procedure update(ref: TdtPolyRef; const pos: PSingle; const collisionQueryRange: Single;
          navquery: TdtNavMeshQuery; const filter: TdtQueryFilter);

    function isValid(navquery: TdtNavMeshQuery; const filter: TdtQueryFilter): Boolean;

    function getCenter(): PSingle; { return m_center; }
    function getSegmentCount(): Integer; { return m_nsegs; }
    function getSegment(i: Integer): PSingle; { return m_segs[i].s; }
  end;

implementation
uses Math, RN_DetourCommon, RN_DetourNavMesh;


constructor TdtLocalBoundary.Create;
begin
  inherited;

  dtVset(@m_center[0], MaxSingle,MaxSingle,MaxSingle);
end;

destructor TdtLocalBoundary.Destroy;
begin
  inherited;
end;

procedure TdtLocalBoundary.reset();
begin
  dtVset(@m_center[0], MaxSingle,MaxSingle,MaxSingle);
  m_npolys := 0;
  m_nsegs := 0;
end;

procedure TdtLocalBoundary.addSegment(const dist: Single; const s: PSingle);
var seg: PSegment; i,tgt,n: Integer;
begin
  // Insert neighbour based on the distance.
  seg := nil;
  if (m_nsegs = 0) then
  begin
    // First, trivial accept.
    seg := @m_segs[0];
  end
  else if (dist >= m_segs[m_nsegs-1].d) then
  begin
    // Further than the last segment, skip.
    if (m_nsegs >= MAX_LOCAL_SEGS) then
      Exit;
    // Last, trivial accept.
    seg := @m_segs[m_nsegs];
  end
  else
  begin
    // Insert inbetween.
    for i := 0 to m_nsegs - 1 do
      if (dist <= m_segs[i].d) then
        break;
    tgt := i+1;
    n := dtMin(m_nsegs-i, MAX_LOCAL_SEGS-tgt);
    Assert(tgt+n <= MAX_LOCAL_SEGS);
    if (n > 0) then
      Move(m_segs[i], m_segs[tgt], sizeof(TSegment)*n);
    seg := @m_segs[i];
  end;

  seg.d := dist;
  Move(s^, seg.s[0], sizeof(Single)*6);

  if (m_nsegs < MAX_LOCAL_SEGS) then
    Inc(m_nsegs);
end;

procedure TdtLocalBoundary.update(ref: TdtPolyRef; const pos: PSingle; const collisionQueryRange: Single;
          navquery: TdtNavMeshQuery; const filter: TdtQueryFilter);
const MAX_SEGS_PER_POLY = DT_VERTS_PER_POLYGON*3;
var segs: array [0..MAX_SEGS_PER_POLY*6-1] of Single; nsegs,j,k: Integer; s: PSingle; tseg, distSqr: Single;
begin

  if (ref = 0) then
  begin
    dtVset(@m_center[0], MaxSingle,MaxSingle,MaxSingle);
    m_nsegs := 0;
    m_npolys := 0;
    Exit;
  end;

  dtVcopy(@m_center[0], pos);

  // First query non-overlapping polygons.
  navquery.findLocalNeighbourhood(ref, pos, collisionQueryRange,
                   filter, @m_polys[0], nil, @m_npolys, MAX_LOCAL_POLYS);

  // Secondly, store all polygon edges.
  m_nsegs := 0;
  nsegs := 0;
  for j := 0 to m_npolys - 1 do
  begin
    navquery.getPolyWallSegments(m_polys[j], filter, @segs[0], nil, @nsegs, MAX_SEGS_PER_POLY);
    for k := 0 to nsegs - 1 do
    begin
      s := @segs[k*6];
      // Skip too distant segments.
      distSqr := dtDistancePtSegSqr2D(pos, s, s+3, @tseg);
      if (distSqr > Sqr(collisionQueryRange)) then
        continue;
      addSegment(distSqr, s);
    end;
  end;
end;

function TdtLocalBoundary.isValid(navquery: TdtNavMeshQuery; const filter: TdtQueryFilter): Boolean;
var i: Integer;
begin
  if (m_npolys = 0) then
    Exit(false);

  // Check that all polygons still pass query filter.
  for i := 0 to m_npolys - 1 do
  begin
    if (not navquery.isValidPolyRef(m_polys[i], filter)) then
      Exit(false);
  end;

  Result := true;
end;

function TdtLocalBoundary.getCenter(): PSingle; begin Result := @m_center[0]; end;
function TdtLocalBoundary.getSegmentCount(): Integer; begin Result := m_nsegs; end;
function TdtLocalBoundary.getSegment(i: Integer): PSingle; begin Result := @m_segs[i].s[0]; end;

end.
