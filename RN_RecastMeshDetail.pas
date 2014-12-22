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

unit RN_RecastMeshDetail;
interface
uses
  Math, SysUtils, RN_Helper, RN_Recast, RN_RecastMesh;


/// Builds a detail mesh from the provided polygon mesh.
///  @ingroup recast
///  @param[in,out]  ctx        The build context to use during the operation.
///  @param[in]    mesh      A fully built polygon mesh.
///  @param[in]    chf        The compact heightfield used to build the polygon mesh.
///  @param[in]    sampleDist    Sets the distance to use when samping the heightfield. [Limit: >=0] [Units: wu]
///  @param[in]    sampleMaxError  The maximum distance the detail mesh surface should deviate from
///                  heightfield data. [Limit: >=0] [Units: wu]
///  @param[out]  dmesh      The resulting detail mesh.  (Must be pre-allocated.)
///  @returns True if the operation completed successfully.
function rcBuildPolyMeshDetail(ctx: TrcContext; const mesh: PrcPolyMesh; const chf: PrcCompactHeightfield;
               const sampleDist, sampleMaxError: Single;
               dmesh: PrcPolyMeshDetail): Boolean;

const RC_UNSET_HEIGHT = $ffff;

type
  PrcHeightPatch = ^TrcHeightPatch;
  TrcHeightPatch = record
    data: PWord;
    xmin, ymin, width, height: Integer;
  end;

implementation
uses RN_RecastAlloc, RN_RecastHelper;

function vdot2(const a, b: PSingle): Single;
begin
  Result := a[0]*b[0] + a[2]*b[2];
end;

function vdistSq2(const p,q: PSingle): Single;
var dx,dy: Single;
begin
  dx := q[0] - p[0];
  dy := q[2] - p[2];
  Result := dx*dx + dy*dy;
end;

function vdist2(const p,q: PSingle): Single;
begin
  Result := sqrt(vdistSq2(p,q));
end;

function vcross2(const p1,p2,p3: PSingle): Single;
var u1,v1,u2,v2: Single;
begin
  u1 := p2[0] - p1[0];
  v1 := p2[2] - p1[2];
  u2 := p3[0] - p1[0];
  v2 := p3[2] - p1[2];
  Result := u1 * v2 - v1 * u2;
end;

function circumCircle(const p1,p2,p3: PSingle;
             c: PSingle; r: PSingle): Boolean;
const EPS = 0.000001;
var v1,v2,v3: array [0..2] of Single; cp,v1Sq,v2Sq,v3Sq: Single;
begin
  v1[0] := 0; v1[1] := 0; v1[2] := 0;

  // Calculate the circle relative to p1, to avoid some precision issues.
  rcVsub(@v2[0], p2, p1);
  rcVsub(@v3[0], p3, p1);

  cp := vcross2(@v1[0], @v2[0], @v3[0]);
  if (abs(cp) > EPS) then
  begin
    v1Sq := vdot2(@v1[0],@v1[0]);
    v2Sq := vdot2(@v2[0],@v2[0]);
    v3Sq := vdot2(@v3[0],@v3[0]);
    c[0] := (v1Sq*(v2[2]-v3[2]) + v2Sq*(v3[2]-v1[2]) + v3Sq*(v1[2]-v2[2])) / (2*cp);
    c[1] := 0;
    c[2] := (v1Sq*(v3[0]-v2[0]) + v2Sq*(v1[0]-v3[0]) + v3Sq*(v2[0]-v1[0])) / (2*cp);
    r^ := vdist2(c, @v1[0]);
    rcVadd(c, c, p1);
    Exit(true);
  end;

  rcVcopy(c, p1);
  r^ := 0;
  Result := False;
end;

function distPtTri(const p,a,b,c: PSingle): Single;
const EPS = 0.0001;
var v0,v1,v2: array [0..2] of Single; dot00,dot01,dot02,dot11,dot12: Single; invDenom,u,v,y: Single;
begin
  rcVsub(@v0[0], c,a);
  rcVsub(@v1[0], b,a);
  rcVsub(@v2[0], p,a);

  dot00 := vdot2(@v0[0], @v0[0]);
  dot01 := vdot2(@v0[0], @v1[0]);
  dot02 := vdot2(@v0[0], @v2[0]);
  dot11 := vdot2(@v1[0], @v1[0]);
  dot12 := vdot2(@v1[0], @v2[0]);

  // Compute barycentric coordinates
  invDenom := 1.0 / (dot00 * dot11 - dot01 * dot01);
  u := (dot11 * dot02 - dot01 * dot12) * invDenom;
  v := (dot00 * dot12 - dot01 * dot02) * invDenom;

  // If point lies inside the triangle, return interpolated y-coord.
  if (u >= -EPS) and (v >= -EPS) and ((u+v) <= 1+EPS) then
  begin
    y := a[1] + v0[1]*u + v1[1]*v;
    Exit(abs(y-p[1]));
  end;
  Result := MaxSingle;
end;

function distancePtSeg(const pt,p,q: PSingle): Single;
var pqx, pqy, pqz, dx, dy, dz, d, t: Single;
begin
  pqx := q[0] - p[0];
  pqy := q[1] - p[1];
  pqz := q[2] - p[2];
  dx := pt[0] - p[0];
  dy := pt[1] - p[1];
  dz := pt[2] - p[2];
  d := pqx*pqx + pqy*pqy + pqz*pqz;
  t := pqx*dx + pqy*dy + pqz*dz;
  if (d > 0) then
    t := t/d;
  if (t < 0) then
    t := 0
  else if (t > 1) then
    t := 1;

  dx := p[0] + t*pqx - pt[0];
  dy := p[1] + t*pqy - pt[1];
  dz := p[2] + t*pqz - pt[2];

  Result := dx*dx + dy*dy + dz*dz;
end;

function distancePtSeg2d(const pt,p,q: PSingle): Single;
var pqx, pqz, dx, dz, d, t: Single;
begin
  pqx := q[0] - p[0];
  pqz := q[2] - p[2];
  dx := pt[0] - p[0];
  dz := pt[2] - p[2];
  d := pqx*pqx + pqz*pqz;
  t := pqx*dx + pqz*dz;
  if (d > 0) then
    t := t/d;
  if (t < 0) then
    t := 0
  else if (t > 1) then
    t := 1;

  dx := p[0] + t*pqx - pt[0];
  dz := p[2] + t*pqz - pt[2];

  Result := dx*dx + dz*dz;
end;

function distToTriMesh(const p: PSingle; const verts: PSingle; const nverts: Integer; const tris: PInteger; const ntris: Integer): Single;
var dmin,d: Single; i: Integer; va,vb,vc: PSingle;
begin
  dmin := MaxSingle;
  for i := 0 to ntris - 1 do
  begin
    va := @verts[tris[i*4+0]*3];
    vb := @verts[tris[i*4+1]*3];
    vc := @verts[tris[i*4+2]*3];
    d := distPtTri(p, va,vb,vc);
    if (d < dmin) then
      dmin := d;
  end;
  if (dmin = MaxSingle) then Exit(-1);
  Result := dmin;
end;

function distToPoly(nvert: Integer; verts: PSingle; p: PSingle): Single;
var dmin: Single; i,j,c: Integer; vi,vj: PSingle;
begin
  dmin := MaxSingle;
  c := 0;
  i := 0;
  j := nvert-1;
  while (i < nvert) do
  begin
    vi := @verts[i*3];
    vj := @verts[j*3];
    if (((vi[2] > p[2]) <> (vj[2] > p[2])) and
      (p[0] < (vj[0]-vi[0]) * (p[2]-vi[2]) / (vj[2]-vi[2]) + vi[0]) ) then
      c := not c;
    dmin := rcMin(dmin, distancePtSeg2d(p, vj, vi));

    j := i;
    Inc(i);
  end;
  Result := IfThen(c <> 0, -dmin, dmin);
end;


function getHeight(const fx, fy, fz: Single;
                const cs, ics, ch: Single;
                const hp: PrcHeightPatch): Word;
const off: array [0..15] of Integer = (-1,0, -1,-1, 0,-1, 1,-1, 1,0, 1,1, 0,1, -1,1);
var ix,iz,nx,nz: Integer; h,nh: Word; i: Integer; dmin,d: Single;
begin
  ix := floor(fx*ics + 0.01);
  iz := floor(fz*ics + 0.01);
  ix := rcClamp(ix-hp.xmin, 0, hp.width - 1);
  iz := rcClamp(iz-hp.ymin, 0, hp.height - 1);
  h := hp.data[ix+iz*hp.width];
  if (h = RC_UNSET_HEIGHT) then
  begin
    // Special case when data might be bad.
    // Find nearest neighbour pixel which has valid height.
    dmin := MaxSingle;
    for i := 0 to 7 do
    begin
      nx := ix+off[i*2+0];
      nz := iz+off[i*2+1];
      if (nx < 0) or (nz < 0) or (nx >= hp.width) or (nz >= hp.height) then continue;
      nh := hp.data[nx+nz*hp.width];
      if (nh = RC_UNSET_HEIGHT) then continue;

      d := abs(nh*ch - fy);
      if (d < dmin) then
      begin
        h := nh;
        dmin := d;
      end;
    end;
  end;
  Result := h;
end;


//enum EdgeValues
const
  EV_UNDEF = -1;
  EV_HULL = -2;

function findEdge(edges: PInteger; nedges, s, t: Integer): Integer;
var i: Integer; e: PInteger;
begin
  for i := 0 to nedges - 1 do
  begin
    e := @edges[i*4];
    if ((e[0] = s) and (e[1] = t)) or ((e[0] = t) and (e[1] = s)) then
      Exit(i);
  end;
  Result := EV_UNDEF;
end;

function addEdge(ctx: TrcContext; edges: PInteger; nedges: PInteger; const maxEdges, s, t, l, r: Integer): Integer;
var e: Integer; edge: PInteger;
begin
  if (nedges^ >= maxEdges) then
  begin
    ctx.log(RC_LOG_ERROR, Format('addEdge: Too many edges (%d/%d).', [nedges^, maxEdges]));
    Exit(EV_UNDEF);
  end;

  // Add edge if not already in the triangulation.
  e := findEdge(edges, nedges^, s, t);
  if (e = EV_UNDEF) then
  begin
    edge := @edges[nedges^*4];
    edge[0] := s;
    edge[1] := t;
    edge[2] := l;
    edge[3] := r;
    Result := nedges^;
    Inc(nedges^);
  end
  else
  begin
    Result := EV_UNDEF;
  end;
end;

procedure updateLeftFace(e: PInteger; s, t, f: Integer);
begin
  if (e[0] = s) and (e[1] = t) and (e[2] = EV_UNDEF) then
    e[2] := f
  else if (e[1] = s) and (e[0] = t) and (e[3] = EV_UNDEF) then
    e[3] := f;
end;

function overlapSegSeg2d(a,b,c,d: PSingle): Integer;
var a1,a2,a3,a4: Single;
begin
  a1 := vcross2(a, b, d);
  a2 := vcross2(a, b, c);
  if (a1*a2 < 0.0) then
  begin
    a3 := vcross2(c, d, a);
    a4 := a3 + a2 - a1;
    if (a3 * a4 < 0.0) then
      Exit(1);
  end;
  Result := 0;
end;

function overlapEdges(const pts: PSingle; const edges: PInteger; nedges, s1, t1: Integer): Boolean;
var i,s0,t0: Integer;
begin
  for i := 0 to nedges - 1 do
  begin
    s0 := edges[i*4+0];
    t0 := edges[i*4+1];
    // Same or connected edges do not overlap.
    if (s0 = s1) or (s0 = t1) or (t0 = s1) or (t0 = t1) then
      continue;
    if (overlapSegSeg2d(@pts[s0*3], @pts[t0*3], @pts[s1*3], @pts[t1*3]) <> 0) then
      Exit(true);
  end;
  Result := false;
end;

procedure completeFacet(ctx: TrcContext; const pts: PSingle; npts: Integer; edges: PInteger; nedges: PInteger; const maxEdges: Integer; nfaces: PInteger; e: Integer);
const EPS = 0.00001;
var edge: PInteger; s,t,pt,u: Integer; c: array [0..2] of Single; r,d,tol: Single;
begin

  edge := @edges[e*4];

  // Cache s and t.
  //int s,t;
  if (edge[2] = EV_UNDEF) then
  begin
    s := edge[0];
    t := edge[1];
  end
  else if (edge[3] = EV_UNDEF) then
  begin
    s := edge[1];
    t := edge[0];
  end
  else
  begin
    // Edge already completed.
    Exit;
  end;

  // Find best point on left of edge.
  pt := npts;
  c[0] := 0; c[1] := 0; c[2] := 0;
  r := -1;
  for u := 0 to npts - 1 do
  begin
    if (u = s) or (u = t) then continue;
    if (vcross2(@pts[s*3], @pts[t*3], @pts[u*3]) > EPS) then
    begin
      if (r < 0) then
      begin
        // The circle is not updated yet, do it now.
        pt := u;
        circumCircle(@pts[s*3], @pts[t*3], @pts[u*3], @c[0], @r);
        continue;
      end;
      d := vdist2(@c[0], @pts[u*3]);
      tol := 0.001;
      if (d > r*(1+tol)) then
      begin
        // Outside current circumcircle, skip.
        continue;
      end
      else if (d < r*(1-tol)) then
      begin
        // Inside safe circumcircle, update circle.
        pt := u;
        circumCircle(@pts[s*3], @pts[t*3], @pts[u*3], @c[0], @r);
      end
      else
      begin
        // Inside epsilon circum circle, do extra tests to make sure the edge is valid.
        // s-u and t-u cannot overlap with s-pt nor t-pt if they exists.
        if (overlapEdges(pts, edges, nedges^, s,u)) then
          continue;
        if (overlapEdges(pts, edges, nedges^, t,u)) then
          continue;
        // Edge is valid.
        pt := u;
        circumCircle(@pts[s*3], @pts[t*3], @pts[u*3], @c[0], @r);
      end;
    end;
  end;

  // Add new triangle or update edge info if s-t is on hull.
  if (pt < npts) then
  begin
    // Update face information of edge being completed.
    updateLeftFace(@edges[e*4], s, t, nfaces^);

    // Add new edge or update face info of old edge.
    e := findEdge(edges, nedges^, pt, s);
    if (e = EV_UNDEF) then
      addEdge(ctx, edges, nedges, maxEdges, pt, s, nfaces^, EV_UNDEF)
    else
      updateLeftFace(@edges[e*4], pt, s, nfaces^);

    // Add new edge or update face info of old edge.
    e := findEdge(edges, nedges^, t, pt);
    if (e = EV_UNDEF) then
      addEdge(ctx, edges, nedges, maxEdges, t, pt, nfaces^, EV_UNDEF)
    else
      updateLeftFace(@edges[e*4], t, pt, nfaces^);

    Inc(nfaces^);
  end
  else
  begin
    updateLeftFace(@edges[e*4], s, t, EV_HULL);
  end;
end;

procedure delaunayHull(ctx: TrcContext; const npts: Integer; const pts: PSingle;
             const nhull: Integer; const hull: PInteger;
             tris, edges: PrcIntArray);
var i,j,nfaces,nedges,maxEdges,currentEdge: Integer; e,t: PInteger;
begin
  nfaces := 0;
  nedges := 0;
  maxEdges := npts*10;
  edges.resize(maxEdges*4);

  i := 0;
  j := nhull-1;
  while (i < nhull) do
  begin
    addEdge(ctx, @edges.m_data[0], @nedges, maxEdges, hull[j], hull[i], EV_HULL, EV_UNDEF);
    j := i;
    Inc(i);
  end;

  currentEdge := 0;
  while (currentEdge < nedges) do
  begin
    if (edges^[currentEdge*4+2] = EV_UNDEF) then
      completeFacet(ctx, pts, npts, @edges.m_data[0], @nedges, maxEdges, @nfaces, currentEdge);
    if (edges^[currentEdge*4+3] = EV_UNDEF) then
      completeFacet(ctx, pts, npts, @edges.m_data[0], @nedges, maxEdges, @nfaces, currentEdge);
    Inc(currentEdge);
  end;

  // Create tris
  tris.resize(nfaces*4);
  for i := 0 to nfaces*4 - 1 do
    tris^[i] := -1;

  for i := 0 to nedges - 1 do
  begin
    e := @edges.m_data[i*4];
    if (e[3] >= 0) then
    begin
      // Left face
      t := @tris.m_data[e[3]*4];
      if (t[0] = -1) then
      begin
        t[0] := e[0];
        t[1] := e[1];
      end
      else if (t[0] = e[1]) then
        t[2] := e[0]
      else if (t[1] = e[0]) then
        t[2] := e[1];
    end;
    if (e[2] >= 0) then
    begin
      // Right
      t := @tris.m_data[e[2]*4];
      if (t[0] = -1) then
      begin
        t[0] := e[1];
        t[1] := e[0];
      end
      else if (t[0] = e[0]) then
        t[2] := e[1]
      else if (t[1] = e[1]) then
        t[2] := e[0];
    end;
  end;

  i := 0;
  while(i < tris.size div 4) do
  begin
    t := @tris.m_data[i*4];
    if (t[0] = -1) or (t[1] = -1) or (t[2] = -1) then
    begin
      ctx.log(RC_LOG_WARNING, Format('delaunayHull: Removing dangling face %d [%d,%d,%d].', [i, t[0], t[1], t[2]]));
      t[0] := tris^[tris.size-4];
      t[1] := tris^[tris.size-3];
      t[2] := tris^[tris.size-2];
      t[3] := tris^[tris.size-1];
      tris.resize(tris.size-4);
      Dec(i);
    end;
    Inc(i);
  end;
end;

// Calculate minimum extend of the polygon.
function polyMinExtent(verts: PSingle; nverts: Integer): Single;
var minDist,maxEdgeDist,d: Single; i,j,ni: Integer; p1,p2: PSingle;
begin
  minDist := MaxSingle;
  for i := 0 to nverts - 1 do
  begin
    ni := (i+1) mod nverts;
    p1 := @verts[i*3];
    p2 := @verts[ni*3];
    maxEdgeDist := 0;
    for j := 0 to nverts - 1 do
    begin
      if (j = i) or (j = ni) then continue;
      d := distancePtSeg2d(@verts[j*3], p1, p2);
      maxEdgeDist := rcMax(maxEdgeDist, d);
    end;
    minDist := rcMin(minDist, maxEdgeDist);
  end;
  Result := Sqrt(minDist);
end;

function next(i, n: Integer): Integer; begin Result := (i+1) mod n; end;
function prev(i, n: Integer): Integer; begin Result := (i+n-1) mod n; end;

procedure triangulateHull(const nverts: Integer; const verts: PSingle; const nhull: Integer; const hull: PInteger; tris: PrcIntArray);
var start,left,right: Integer; dmin,d: Single; i,pi,ni,nleft,nright: Integer; pv,cv,nv: PSingle; cvleft,nvleft,cvright,nvright: PSingle;
dleft,dright: Single;
begin
  start := 0; left := 1; right := nhull-1;

  // Start from an ear with shortest perimeter.
  // This tends to favor well formed triangles as starting point.
  dmin := 0;
  for i := 0 to nhull - 1 do
  begin
    pi := prev(i, nhull);
    ni := next(i, nhull);
    pv := @verts[hull[pi]*3];
    cv := @verts[hull[i]*3];
    nv := @verts[hull[ni]*3];
    d := vdist2(pv,cv) + vdist2(cv,nv) + vdist2(nv,pv);
    if (d < dmin) then
    begin
      start := i;
      left := ni;
      right := pi;
      dmin := d;
    end;
  end;

  // Add first triangle
  tris.push(hull[start]);
  tris.push(hull[left]);
  tris.push(hull[right]);
  tris.push(0);

  // Triangulate the polygon by moving left or right,
  // depending on which triangle has shorter perimeter.
  // This heuristic was chose emprically, since it seems
  // handle tesselated straight edges well.
  while (next(left, nhull) <> right) do
  begin
    // Check to see if se should advance left or right.
    nleft := next(left, nhull);
    nright := prev(right, nhull);

    cvleft := @verts[hull[left]*3];
    nvleft := @verts[hull[nleft]*3];
    cvright := @verts[hull[right]*3];
    nvright := @verts[hull[nright]*3];
    dleft := vdist2(cvleft, nvleft) + vdist2(nvleft, cvright);
    dright := vdist2(cvright, nvright) + vdist2(cvleft, nvright);

    if (dleft < dright) then
    begin
      tris.push(hull[left]);
      tris.push(hull[nleft]);
      tris.push(hull[right]);
      tris.push(0);
      left := nleft;
    end
    else
    begin
      tris.push(hull[left]);
      tris.push(hull[nright]);
      tris.push(hull[right]);
      tris.push(0);
      right := nright;
    end;
  end;
end;


function getJitterX(const i: Integer): Single;
begin
  Result := (((i * $8da6b343) and $ffff) / 65535.0 * 2.0) - 1.0;
end;

function getJitterY(const i: Integer): Single;
begin
  Result := (((i * $d8163841) and $ffff) / 65535.0 * 2.0) - 1.0;
end;

function buildPolyDetail(ctx: TrcContext; &in: PSingle; nin: Integer;
              const sampleDist, sampleMaxError: Single;
              const chf: PrcCompactHeightfield; const hp: PrcHeightPatch;
              verts: PSingle; nverts: PInteger; tris: PrcIntArray;
              edges, samples: PrcIntArray): Boolean;
const MAX_VERTS = 127;
  MAX_TRIS = 255;  // Max tris for delaunay is 2n-2-k (n=num verts, k=num hull verts).
  MAX_VERTS_PER_EDGE = 32;
var edge: array [0..(MAX_VERTS_PER_EDGE+1)*3-1] of Single; hull: array [0..MAX_VERTS] of Integer; nhull: Integer;
i,j,k,m: Integer; cs,ics,minExtent: Single; vi,vj: PSingle; swapped: Boolean; dx,dy,dz,d: Single; nn: Integer; u: Single;
pos: PSingle; idx: array [0..MAX_VERTS_PER_EDGE-1] of Integer; nidx,a,b: Integer; va,vb: PSingle; maxd: Single; maxi: Integer;
dev: Single; bmin, bmax,pt,bestpt: array [0..2] of Single; x0,x1,z0,z1,x,z: Integer; nsamples,iter: Integer; bestd: Single;
besti,ntris: Integer; s: PInteger;
begin
  nhull := 0;
  nverts^ := 0;

  for i := 0 to nin - 1 do
    rcVcopy(@verts[i*3], @&in[i*3]);
  nverts^ := nin;

  edges.resize(0);
  tris.resize(0);

  cs := chf.cs;
  ics := 1.0/cs;

  // Calculate minimum extents of the polygon based on input data.
  minExtent := polyMinExtent(verts, nverts^);

  // Tessellate outlines.
  // This is done in separate pass in order to ensure
  // seamless height values across the ply boundaries.
  if (sampleDist > 0) then
  begin
    i := 0; j := nin-1;
    while (i < nin) do
    begin
      vj := @&in[j*3];
      vi := @&in[i*3];
      swapped := false;
      // Make sure the segments are always handled in same order
      // using lexological sort or else there will be seams.
      if (abs(vj[0]-vi[0]) < 0.000001) then
      begin
        if (vj[2] > vi[2]) then
        begin
          rcSwap(vj,vi);
          swapped := true;
        end;
      end
      else
      begin
        if (vj[0] > vi[0]) then
        begin
          rcSwap(vj,vi);
          swapped := true;
        end;
      end;
      // Create samples along the edge.
      dx := vi[0] - vj[0];
      dy := vi[1] - vj[1];
      dz := vi[2] - vj[2];
      d := sqrt(dx*dx + dz*dz);
      nn := 1 + floor(d/sampleDist);
      if (nn >= MAX_VERTS_PER_EDGE) then nn := MAX_VERTS_PER_EDGE-1;
      if (nverts^+nn >= MAX_VERTS) then
        nn := MAX_VERTS-1-nverts^;

      for k := 0 to nn do
      begin
        u := k/nn;
        pos := @edge[k*3];
        pos[0] := vj[0] + dx*u;
        pos[1] := vj[1] + dy*u;
        pos[2] := vj[2] + dz*u;
        pos[1] := getHeight(pos[0],pos[1],pos[2], cs, ics, chf.ch, hp)*chf.ch;
      end;
      // Simplify samples.
      FillChar(idx[0], SizeOf(Integer) * MAX_VERTS_PER_EDGE, #0);
      idx[0] := 0; idx[1] := nn;
      nidx := 2;
      k := 0;
      while (k < nidx-1) do
      begin
        a := idx[k];
        b := idx[k+1];
        va := @edge[a*3];
        vb := @edge[b*3];
        // Find maximum deviation along the segment.
        maxd := 0;
        maxi := -1;
        for m := a+1 to b - 1 do
        begin
          dev := distancePtSeg(@edge[m*3],va,vb);
          if (dev > maxd) then
          begin
            maxd := dev;
            maxi := m;
          end;
        end;
        // If the max deviation is larger than accepted error,
        // add new point, else continue to next segment.
        if (maxi <> -1) and (maxd > Sqr(sampleMaxError)) then
        begin
          for m := nidx downto k + 1 do
            idx[m] := idx[m-1];
          idx[k+1] := maxi;
          Inc(nidx);
        end
        else
        begin
          Inc(k);
        end;
      end;

      hull[nhull] := j;
      Inc(nhull);
      // Add new vertices.
      if (swapped) then
      begin
        for k := nidx-2 downto 1 do
        begin
          rcVcopy(@verts[nverts^*3], @edge[idx[k]*3]);
          hull[nhull] := nverts^;
          Inc(nhull);
          Inc(nverts^);
        end;
      end
      else
      begin
        for k := 1 to nidx-1 - 1 do
        begin
          rcVcopy(@verts[nverts^*3], @edge[idx[k]*3]);
          hull[nhull] := nverts^;
          Inc(nhull);
          Inc(nverts^);
        end;
      end;

      j := i;
      Inc(i);
    end;
  end;

  // If the polygon minimum extent is small (sliver or small triangle), do not try to add internal points.
  if (minExtent < sampleDist*2) then
  begin
    triangulateHull(nverts^, verts, nhull, @hull[0], tris);
    Exit(true);
  end;

  // Tessellate the base mesh.
  // We're using the triangulateHull instead of delaunayHull as it tends to
  // create a bit better triangulation for long thing triangles when there
  // are no internal points.
  triangulateHull(nverts^, verts, nhull, @hull[0], tris);

  if (tris.size = 0) then
  begin
    // Could not triangulate the poly, make sure there is some valid data there.
    ctx.log(RC_LOG_WARNING, Format('buildPolyDetail: Could not triangulate polygon (%d verts).', [nverts^]));
    Exit(true);
  end;

  if (sampleDist > 0) then
  begin
    // Create sample locations in a grid.
    rcVcopy(@bmin[0], @&in[0]);
    rcVcopy(@bmax[0], @&in[0]);
    for i := 1 to nin - 1 do
    begin
      rcVmin(@bmin[0], @&in[i*3]);
      rcVmax(@bmax[0], @&in[i*3]);
    end;
    x0 := floor(bmin[0]/sampleDist);
    x1 := ceil(bmax[0]/sampleDist);
    z0 := floor(bmin[2]/sampleDist);
    z1 := ceil(bmax[2]/sampleDist);
    samples.resize(0);
    for z := z0 to z1 - 1 do
    begin
      for x := x0 to x1 - 1 do
      begin
        pt[0] := x*sampleDist;
        pt[1] := (bmax[1]+bmin[1])*0.5;
        pt[2] := z*sampleDist;
        // Make sure the samples are not too close to the edges.
        if (distToPoly(nin,&in,@pt[0]) > -sampleDist/2) then continue;
        samples.push(x);
        samples.push(getHeight(pt[0], pt[1], pt[2], cs, ics, chf.ch, hp));
        samples.push(z);
        samples.push(0); // Not added
      end;
    end;

    // Add the samples starting from the one that has the most
    // error. The procedure stops when all samples are added
    // or when the max error is within treshold.
    nsamples := samples.size div 4;
    for iter := 0 to nsamples - 1 do
    begin
      if (nverts^ >= MAX_VERTS) then
        break;

      // Find sample with most error.
      bestpt[0] := 0; bestpt[1] := 0; bestpt[2] := 0;
      bestd := 0;
      besti := -1;
      for i := 0 to nsamples - 1 do
      begin
        s := @samples.m_data[i*4];
        if (s[3] <> 0) then continue; // skip added.

        // The sample location is jittered to get rid of some bad triangulations
        // which are cause by symmetrical data from the grid structure.
        pt[0] := s[0]*sampleDist + getJitterX(i)*cs*0.1;
        pt[1] := s[1]*chf.ch;
        pt[2] := s[2]*sampleDist + getJitterY(i)*cs*0.1;
        d := distToTriMesh(@pt[0], verts, nverts^, @tris.m_data[0], tris.size div 4);
        if (d < 0) then continue; // did not hit the mesh.
        if (d > bestd) then
        begin
          bestd := d;
          besti := i;
          rcVcopy(@bestpt[0],@pt[0]);
        end;
      end;
      // If the max error is within accepted threshold, stop tesselating.
      if (bestd <= sampleMaxError) or (besti = -1) then
        break;
      // Mark sample as added.
      samples^[besti*4+3] := 1;
      // Add the new sample point.
      rcVcopy(@verts[nverts^*3], @bestpt[0]);
      Inc(nverts^);

      // Create new triangulation.
      // TODO: Incremental add instead of full rebuild.
      edges.resize(0);
      tris.resize(0);
      delaunayHull(ctx, nverts^, verts, nhull, @hull[0], tris, edges);
    end;
  end;

  ntris := tris.size div 4;
  if (ntris > MAX_TRIS) then
  begin
    tris.resize(MAX_TRIS*4);
    ctx.log(RC_LOG_ERROR, Format('rcBuildPolyMeshDetail: Shrinking triangle count from %d to max %d.', [ntris, MAX_TRIS]));
  end;

  Result := true;
end;


procedure getHeightDataSeedsFromVertices(const chf: PrcCompactHeightfield;
                       const poly: PWord; const npoly: Integer;
                       const verts: PWord; const bs: Integer;
                       hp: PrcHeightPatch; stack: PrcIntArray);
const offset: array [0..9*2-1] of Integer = (0,0, -1,-1, 0,-1, 1,-1, 1,0, 1,1, 0,1, -1,1, -1,0);
var i,j,k,cx,cy,cz,ci,dmin,ax,ay,az,d,idx: Integer; c: PrcCompactCell; s,cs: PrcCompactSpan; pcx,pcz,dir,ai: Integer;
begin
  // Floodfill the heightfield to get 2D height data,
  // starting at vertex locations as seeds.

  // Note: Reads to the compact heightfield are offset by border size (bs)
  // since border size offset is already removed from the polymesh vertices.

  FillChar(hp.data[0], sizeof(Word)*hp.width*hp.height, 0);

  stack.resize(0);

  // Use poly vertices as seed points for the flood fill.
  for j := 0 to npoly - 1 do
  begin
    cx := 0; cz := 0; ci := -1;
    dmin := RC_UNSET_HEIGHT;
    for k := 0 to 8 do
    begin
      ax := verts[poly[j]*3+0] + offset[k*2+0];
      ay := verts[poly[j]*3+1];
      az := verts[poly[j]*3+2] + offset[k*2+1];
      if (ax < hp.xmin) or (ax >= hp.xmin+hp.width) or
        (az < hp.ymin) or (az >= hp.ymin+hp.height) then
        continue;

      c := @chf.cells[(ax+bs)+(az+bs)*chf.width];
      for i := c.index to Integer(c.index+c.count) - 1 do
      begin
        s := @chf.spans[i];
        d := Abs(ay - s.y);
        if (d < dmin) then
        begin
          cx := ax;
          cz := az;
          ci := i;
          dmin := d;
        end;
      end;
    end;
    if (ci <> -1) then
    begin
      stack.push(cx);
      stack.push(cz);
      stack.push(ci);
    end;
  end;

  // Find center of the polygon using flood fill.
  pcx := 0; pcz := 0;
  for j := 0 to npoly - 1 do
  begin
    Inc(pcx, verts[poly[j]*3+0]);
    Inc(pcz, verts[poly[j]*3+2]);
  end;
  pcx := pcx div npoly;
  pcz := pcz div npoly;

  for i := 0 to stack.size div 3 - 1 do
  begin
    cx := stack^[i*3+0];
    cy := stack^[i*3+1];
    idx := cx-hp.xmin+(cy-hp.ymin)*hp.width;
    hp.data[idx] := 1;
  end;

  while (stack.size > 0) do
  begin
    ci := stack.pop();
    cy := stack.pop();
    cx := stack.pop();

    // Check if close to center of the polygon.
    if (Abs(cx-pcx) <= 1) and (Abs(cy-pcz) <= 1) then
    begin
      stack.resize(0);
      stack.push(cx);
      stack.push(cy);
      stack.push(ci);
      break;
    end;

    cs := @chf.spans[ci];

    for dir := 0 to 3 do
    begin
      if (rcGetCon(cs, dir) = RC_NOT_CONNECTED) then continue;

      ax := cx + rcGetDirOffsetX(dir);
      ay := cy + rcGetDirOffsetY(dir);

      if (ax < hp.xmin) or (ax >= (hp.xmin+hp.width)) or
        (ay < hp.ymin) or (ay >= (hp.ymin+hp.height)) then
        continue;

      if (hp.data[ax-hp.xmin+(ay-hp.ymin)*hp.width] <> 0) then
        continue;

      ai := chf.cells[(ax+bs)+(ay+bs)*chf.width].index + rcGetCon(cs, dir);

      idx := ax-hp.xmin+(ay-hp.ymin)*hp.width;
      hp.data[idx] := 1;

      stack.push(ax);
      stack.push(ay);
      stack.push(ai);
    end;
  end;

  FillChar(hp.data[0], sizeof(Word)*hp.width*hp.height, $ff);

  // Mark start locations.
  for i := 0 to stack.size div 3 - 1 do
  begin
    cx := stack^[i*3+0];
    cy := stack^[i*3+1];
    ci := stack^[i*3+2];
    idx := cx-hp.xmin+(cy-hp.ymin)*hp.width;
    cs := @chf.spans[ci];
    hp.data[idx] := cs.y;

    // getHeightData seeds are given in coordinates with borders
    stack^[i+0] := stack^[i+0] + bs;
    stack^[i+1] := stack^[i+1] + bs;
  end;
end;


procedure getHeightData(chf: PrcCompactHeightfield;
              const poly: PWord; const npoly: Integer;
              const verts: PWord; const bs: Integer;
              hp: PrcHeightPatch; stack: PrcIntArray;
              region: Integer);
const RETRACT_SIZE = 256;
var empty: Boolean; hy,hx,y,x,i,dir: Integer; c: PrcCompactCell; s: PrcCompactSpan; border: Boolean; ax,ay,ai: Integer;
&as,cs: PrcCompactSpan; head: Integer; cx,cy,ci: Integer;
begin
  // Note: Reads to the compact heightfield are offset by border size (bs)
  // since border size offset is already removed from the polymesh vertices.

  stack.resize(0);
  FillChar(hp.data[0], sizeof(Word)*hp.width*hp.height, $ff);

  empty := true;

  // Copy the height from the same region, and mark region borders
  // as seed points to fill the rest.
  for hy := 0 to hp.height - 1 do
  begin
    y := hp.ymin + hy + bs;
    for hx := 0 to hp.width - 1 do
    begin
      x := hp.xmin + hx + bs;
      c := @chf.cells[x+y*chf.width];
      for i := c.index to Integer(c.index+c.count) - 1 do
      begin
        s := @chf.spans[i];
        if (s.reg = region) then
        begin
          // Store height
          hp.data[hx + hy*hp.width] := s.y;
          empty := false;

          // If any of the neighbours is not in same region,
          // add the current location as flood fill start
          border := false;
          for dir := 0 to 3 do
          begin
            if (rcGetCon(s, dir) <> RC_NOT_CONNECTED) then
            begin
              ax := x + rcGetDirOffsetX(dir);
              ay := y + rcGetDirOffsetY(dir);
              ai := chf.cells[ax+ay*chf.width].index + rcGetCon(s, dir);
              &as := @chf.spans[ai];
              if (&as.reg <> region) then
              begin
                border := true;
                break;
              end;
            end;
          end;
          if (border) then
          begin
            stack.push(x);
            stack.push(y);
            stack.push(i);
          end;
          break;
        end;
      end;
    end;
  end;

  // if the polygon does not contian any points from the current region (rare, but happens)
  // then use the cells closest to the polygon vertices as seeds to fill the height field
  if (empty) then
    getHeightDataSeedsFromVertices(chf, poly, npoly, verts, bs, hp, stack);

  //static const int RETRACT_SIZE := 256;
  head := 0;

  while (head*3 < stack.size) do
  begin
    cx := stack^[head*3+0];
    cy := stack^[head*3+1];
    ci := stack^[head*3+2];
    Inc(head);
    if (head >= RETRACT_SIZE) then
    begin
      head := 0;
      if (stack.size > RETRACT_SIZE*3) then
        Move(stack^.m_data[RETRACT_SIZE*3], stack^.m_data[0], sizeof(integer)*(stack.size-RETRACT_SIZE*3));
      stack.resize(stack.size-RETRACT_SIZE*3);
    end;

    cs := @chf.spans[ci];
    for dir := 0 to 3 do
    begin
      if (rcGetCon(cs, dir) = RC_NOT_CONNECTED) then continue;

      ax := cx + rcGetDirOffsetX(dir);
      ay := cy + rcGetDirOffsetY(dir);
      hx := ax - hp.xmin - bs;
      hy := ay - hp.ymin - bs;

      if (hx < 0) or (hx >= hp.width) or (hy < 0) or (hy >= hp.height) then
        continue;

      if (hp.data[hx + hy*hp.width] <> RC_UNSET_HEIGHT) then
        continue;

      ai := chf.cells[ax + ay*chf.width].index + rcGetCon(cs, dir);
      &as := @chf.spans[ai];

      hp.data[hx + hy*hp.width] := &as.y;

      stack.push(ax);
      stack.push(ay);
      stack.push(ai);
    end;
  end;
end;

function getEdgeFlags(const va, vb: PSingle;
                  const vpoly: PSingle; const npoly: Integer): Byte;
const thrSqr = 0.000001;
var i,j: Integer;
begin
  // Return true if edge (va,vb) is part of the polygon.
  i := 0;
  j := npoly-1;
  while(i < npoly) do
  begin
    if (distancePtSeg2d(va, @vpoly[j*3], @vpoly[i*3]) < thrSqr) and
      (distancePtSeg2d(vb, @vpoly[j*3], @vpoly[i*3]) < thrSqr) then
      Exit(1);

    j := i;
    Inc(i);
  end;
  Result := 0;
end;

function getTriFlags(const va, vb, vc: PSingle;
                 const vpoly: PSingle; const npoly: Integer): Byte;
var flags: Byte;
begin
  flags := 0;
  flags := flags or getEdgeFlags(va,vb,vpoly,npoly) shl 0;
  flags := flags or getEdgeFlags(vb,vc,vpoly,npoly) shl 2;
  flags := flags or getEdgeFlags(vc,va,vpoly,npoly) shl 4;
  Result := flags;
end;

/// @par
///
/// See the #rcConfig documentation for more information on the configuration parameters.
///
/// @see rcAllocPolyMeshDetail, rcPolyMesh, rcCompactHeightfield, rcPolyMeshDetail, rcConfig
function rcBuildPolyMeshDetail(ctx: TrcContext; const mesh: PrcPolyMesh; const chf: PrcCompactHeightfield;
               const sampleDist, sampleMaxError: Single;
               dmesh: PrcPolyMeshDetail): Boolean;
var nvp,borderSize: Integer; cs,ch: Single; orig: PSingle; edges,tris,stack,samples: TrcIntArray; verts: array [0..256*3-1] of Single;
hp: TrcHeightPatch; nPolyVerts,maxhw,maxhh: Integer; bounds: PInteger; poly: PSingle; i,j: Integer; xmin,xmax,ymin,ymax: PInteger;
p,v: PWord; vcap,tcap,npoly,nverts,ntris: Integer; newv: PSingle; newt: PByte; t: PInteger;
begin
  //rcAssert(ctx);

  ctx.startTimer(RC_TIMER_BUILD_POLYMESHDETAIL);

  if (mesh.nverts = 0) or (mesh.npolys = 0) then
  begin
    Result :=true;
    Exit;
  end;

  nvp := mesh.nvp;
  cs := mesh.cs;
  ch := mesh.ch;
  orig := @mesh.bmin[0];
  borderSize := mesh.borderSize;

  edges.Create(64);
  tris.Create(512);
  stack.Create(512);
  samples.Create(512);
  //float verts[256*3];
  //rcHeightPatch hp;
  nPolyVerts := 0;
  maxhw := 0; maxhh := 0;

  GetMem(bounds, sizeof(integer)*mesh.npolys*4);
  GetMem(poly, sizeof(Single)*nvp*3);

  // Find max size for a polygon area.
  for i := 0 to mesh.npolys - 1 do
  begin
    p := @mesh.polys[i*nvp*2];
    xmin := @bounds[i*4+0];
    xmax := @bounds[i*4+1];
    ymin := @bounds[i*4+2];
    ymax := @bounds[i*4+3];
    xmin^ := chf.width;
    xmax^ := 0;
    ymin^ := chf.height;
    ymax^ := 0;
    for j := 0 to nvp - 1 do
    begin
      if(p[j] = RC_MESH_NULL_IDX) then break;
      v := @mesh.verts[p[j]*3];
      xmin^ := rcMin(xmin^, v[0]);
      xmax^ := rcMax(xmax^, v[0]);
      ymin^ := rcMin(ymin^, v[2]);
      ymax^ := rcMax(ymax^, v[2]);
      Inc(nPolyVerts);
    end;
    xmin^ := rcMax(0,xmin^-1);
    xmax^ := rcMin(chf.width,xmax^+1);
    ymin^ := rcMax(0,ymin^-1);
    ymax^ := rcMin(chf.height,ymax^+1);
    if (xmin^ >= xmax^) or (ymin^ >= ymax^) then continue;
    maxhw := rcMax(maxhw, xmax^-xmin^);
    maxhh := rcMax(maxhh, ymax^-ymin^);
  end;

  GetMem(hp.data, sizeof(Word)*maxhw*maxhh);

  dmesh.nmeshes := mesh.npolys;
  dmesh.nverts := 0;
  dmesh.ntris := 0;
  GetMem(dmesh.meshes, sizeof(Cardinal)*dmesh.nmeshes*4);

  vcap := nPolyVerts+nPolyVerts div 2;
  tcap := vcap*2;

  dmesh.nverts := 0;
  GetMem(dmesh.verts, sizeof(Single)*vcap*3);

  dmesh.ntris := 0;
  GetMem(dmesh.tris, sizeof(Byte)*tcap*4);

  for i := 0 to mesh.npolys - 1 do
  begin
    p := @mesh.polys[i*nvp*2];

    // Store polygon vertices for processing.
    npoly := 0;
    for j := 0 to nvp - 1 do
    begin
      if(p[j] = RC_MESH_NULL_IDX) then break;
      v := @mesh.verts[p[j]*3];
      poly[j*3+0] := v[0]*cs;
      poly[j*3+1] := v[1]*ch;
      poly[j*3+2] := v[2]*cs;
      Inc(npoly);
    end;

    // Get the height data from the area of the polygon.
    hp.xmin := bounds[i*4+0];
    hp.ymin := bounds[i*4+2];
    hp.width := bounds[i*4+1]-bounds[i*4+0];
    hp.height := bounds[i*4+3]-bounds[i*4+2];
    getHeightData(chf, p, npoly, mesh.verts, borderSize, @hp, @stack, mesh.regs[i]);

    // Build detail mesh.
    nverts := 0;
    if (not buildPolyDetail(ctx, poly, npoly,
               sampleDist, sampleMaxError,
               chf, @hp, @verts[0], @nverts, @tris,
               @edges, @samples)) then
    begin
      Exit(false);
    end;

    // Move detail verts to world space.
    for j := 0 to nverts - 1 do
    begin
      verts[j*3+0] := verts[j*3+0] + orig[0];
      verts[j*3+1] := verts[j*3+1] + orig[1] + chf.ch; // Is this offset necessary?
      verts[j*3+2] := verts[j*3+2] + orig[2];
    end;
    // Offset poly too, will be used to flag checking.
    for j := 0 to npoly - 1 do
    begin
      poly[j*3+0] := poly[j*3+0] + orig[0];
      poly[j*3+1] := poly[j*3+1] + orig[1];
      poly[j*3+2] := poly[j*3+2] + orig[2];
    end;

    // Store detail submesh.
    ntris := tris.size div 4;

    dmesh.meshes[i*4+0] := dmesh.nverts;
    dmesh.meshes[i*4+1] := nverts;
    dmesh.meshes[i*4+2] := dmesh.ntris;
    dmesh.meshes[i*4+3] := ntris;

    // Store vertices, allocate more memory if necessary.
    if (dmesh.nverts+nverts > vcap) then
    begin
      while (dmesh.nverts+nverts > vcap) do
        Inc(vcap, 256);

      GetMem(newv, sizeof(Single)*vcap*3);
      if (dmesh.nverts <> 0) then
        Move(dmesh.verts^, newv^, sizeof(Single)*3*dmesh.nverts);
      FreeMem(dmesh.verts);
      dmesh.verts := newv;
    end;
    for j := 0 to nverts - 1 do
    begin
      dmesh.verts[dmesh.nverts*3+0] := verts[j*3+0];
      dmesh.verts[dmesh.nverts*3+1] := verts[j*3+1];
      dmesh.verts[dmesh.nverts*3+2] := verts[j*3+2];
      Inc(dmesh.nverts);
    end;

    // Store triangles, allocate more memory if necessary.
    if (dmesh.ntris+ntris > tcap) then
    begin
      while (dmesh.ntris+ntris > tcap) do
        Inc(tcap, 256);
      GetMem(newt, sizeof(Byte)*tcap*4);
      if (dmesh.ntris <> 0) then
        Move(dmesh.tris^, newt^, sizeof(Byte)*4*dmesh.ntris);
      FreeMem(dmesh.tris);
      dmesh.tris := newt;
    end;
    for j := 0 to ntris - 1 do
    begin
      t := @tris.m_data[j*4];
      dmesh.tris[dmesh.ntris*4+0] := Byte(t[0]);
      dmesh.tris[dmesh.ntris*4+1] := Byte(t[1]);
      dmesh.tris[dmesh.ntris*4+2] := Byte(t[2]);
      dmesh.tris[dmesh.ntris*4+3] := getTriFlags(@verts[t[0]*3], @verts[t[1]*3], @verts[t[2]*3], poly, npoly);
      Inc(dmesh.ntris);
    end;
  end;

  FreeMem(bounds);
  FreeMem(poly);

  // Delphi: Manually release record and buffer it holds within
  edges.Free;
  tris.Free;
  stack.Free;
  samples.Free;

  ctx.stopTimer(RC_TIMER_BUILD_POLYMESHDETAIL);

  Result := true;
end;

{/// @see rcAllocPolyMeshDetail, rcPolyMeshDetail
bool rcMergePolyMeshDetails(ctx: TrcContext; rcPolyMeshDetail** meshes, const int nmeshes, rcPolyMeshDetail& mesh)
begin
  rcAssert(ctx);

  ctx.startTimer(RC_TIMER_MERGE_POLYMESHDETAIL);

  int maxVerts := 0;
  int maxTris := 0;
  int maxMeshes := 0;

  for (int i := 0; i < nmeshes; ++i)
  begin
    if (!meshes[i]) continue;
    maxVerts += meshes[i].nverts;
    maxTris += meshes[i].ntris;
    maxMeshes += meshes[i].nmeshes;
  end;

  mesh.nmeshes := 0;
  mesh.meshes := (unsigned int*)rcAlloc(sizeof(unsigned int)*maxMeshes*4, RC_ALLOC_PERM);
  if (!mesh.meshes)
  begin
    ctx.log(RC_LOG_ERROR, 'rcBuildPolyMeshDetail: Out of memory 'pmdtl.meshes' (%d).', maxMeshes*4);
    return false;
  end;

  mesh.ntris := 0;
  mesh.tris := (unsigned char*)rcAlloc(sizeof(unsigned char)*maxTris*4, RC_ALLOC_PERM);
  if (!mesh.tris)
  begin
    ctx.log(RC_LOG_ERROR, 'rcBuildPolyMeshDetail: Out of memory 'dmesh.tris' (%d).', maxTris*4);
    return false;
  end;

  mesh.nverts := 0;
  mesh.verts := (float*)rcAlloc(sizeof(float)*maxVerts*3, RC_ALLOC_PERM);
  if (!mesh.verts)
  begin
    ctx.log(RC_LOG_ERROR, 'rcBuildPolyMeshDetail: Out of memory 'dmesh.verts' (%d).', maxVerts*3);
    return false;
  end;

  // Merge datas.
  for (int i := 0; i < nmeshes; ++i)
  begin
    rcPolyMeshDetail* dm := meshes[i];
    if (!dm) continue;
    for (int j := 0; j < dm.nmeshes; ++j)
    begin
      unsigned int* dst := &mesh.meshes[mesh.nmeshes*4];
      unsigned int* src := &dm.meshes[j*4];
      dst[0] := (unsigned int)mesh.nverts+src[0];
      dst[1] := src[1];
      dst[2] := (unsigned int)mesh.ntris+src[2];
      dst[3] := src[3];
      mesh.nmeshes++;
    end;

    for (int k := 0; k < dm.nverts; ++k)
    begin
      rcVcopy(&mesh.verts[mesh.nverts*3], &dm.verts[k*3]);
      mesh.nverts++;
    end;
    for (int k := 0; k < dm.ntris; ++k)
    begin
      mesh.tris[mesh.ntris*4+0] := dm.tris[k*4+0];
      mesh.tris[mesh.ntris*4+1] := dm.tris[k*4+1];
      mesh.tris[mesh.ntris*4+2] := dm.tris[k*4+2];
      mesh.tris[mesh.ntris*4+3] := dm.tris[k*4+3];
      mesh.ntris++;
    end;
  end;

  ctx.stopTimer(RC_TIMER_MERGE_POLYMESHDETAIL);

  return true;
end;}

end.