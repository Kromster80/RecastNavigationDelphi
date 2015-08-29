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

unit RN_RecastMesh;
interface
uses
  Math, SysUtils, RN_Helper, RN_Recast;

/// Builds a polygon mesh from the provided contours.
///  @ingroup recast
///  @param[in,out]  ctx    The build context to use during the operation.
///  @param[in]    cset  A fully built contour set.
///  @param[in]    nvp    The maximum number of vertices allowed for polygons generated during the
///              contour to polygon conversion process. [Limit: >= 3]
///  @param[out]  mesh  The resulting polygon mesh. (Must be re-allocated.)
///  @returns True if the operation completed successfully.
function rcBuildPolyMesh(ctx: TrcContext; cset: PrcContourSet; const nvp: Integer; mesh: PrcPolyMesh): Boolean;

/// Merges multiple polygon meshes into a single mesh.
///  @ingroup recast
///  @param[in,out]  ctx    The build context to use during the operation.
///  @param[in]    meshes  An array of polygon meshes to merge. [Size: @p nmeshes]
///  @param[in]    nmeshes  The number of polygon meshes in the meshes array.
///  @param[in]    mesh  The resulting polygon mesh. (Must be pre-allocated.)
///  @returns True if the operation completed successfully.
function rcMergePolyMeshes(ctx: TrcContext; meshes: array of TrcPolyMesh; const nmeshes: Integer; mesh: PrcPolyMesh): Boolean;

implementation
uses RN_RecastHelper;

type
  PrcEdge = ^TrcEdge;
  TrcEdge = record
    vert: array [0..1] of Word;
    polyEdge: array [0..1] of Word;
    poly: array [0..1] of Word;
  end;

function buildMeshAdjacency(polys: PWord; const npolys: Integer;
                 const nverts: Integer; const vertsPerPoly: Integer): Boolean;
var maxEdgeCount,edgeCount: Integer; firstEdge,nextEdge: PWord; edges: PrcEdge;
i,j: Integer; t: PWord; v0,v1: Word; edge,e1: PrcEdge; e: Word; p0,p1: PWord;
begin
  // Based on code by Eric Lengyel from:
  // http://www.terathon.com/code/edges.php

  maxEdgeCount := npolys*vertsPerPoly;
  GetMem(firstEdge, sizeof(Word)*(nverts + maxEdgeCount));

  nextEdge := firstEdge + nverts;
  edgeCount := 0;

  GetMem(edges, sizeof(TrcEdge)*maxEdgeCount);

  for i := 0 to nverts - 1 do
    firstEdge[i] := RC_MESH_NULL_IDX;

  for i := 0 to npolys - 1 do
  begin
    t := @polys[i*vertsPerPoly*2];
    for j := 0 to vertsPerPoly - 1 do
    begin
      if (t[j] = RC_MESH_NULL_IDX) then break;
      v0 := t[j];
      v1 := IfThen((j+1 >= vertsPerPoly) or (t[j+1] = RC_MESH_NULL_IDX), t[0], t[j+1]);
      if (v0 < v1) then
      begin
        edge := @edges[edgeCount];
        edge.vert[0] := v0;
        edge.vert[1] := v1;
        edge.poly[0] := i;
        edge.polyEdge[0] := j;
        edge.poly[1] := i;
        edge.polyEdge[1] := 0;
        // Insert edge
        nextEdge[edgeCount] := firstEdge[v0];
        firstEdge[v0] := edgeCount;
        Inc(edgeCount);
      end;
    end;
  end;

  for i := 0 to npolys - 1 do
  begin
    t := @polys[i*vertsPerPoly*2];
    for j := 0 to vertsPerPoly - 1 do
    begin
      if (t[j] = RC_MESH_NULL_IDX) then break;
      v0 := t[j];
      v1 := IfThen((j+1 >= vertsPerPoly) or (t[j+1] = RC_MESH_NULL_IDX), t[0], t[j+1]);
      if (v0 > v1) then
      begin
        //for (unsigned short e := firstEdge[v1]; e != RC_MESH_NULL_IDX; e := nextEdge[e])
        e := firstEdge[v1];
        while (e <> RC_MESH_NULL_IDX) do
        begin
          edge := @edges[e];
          if (edge.vert[1] = v0) and (edge.poly[0] = edge.poly[1]) then
          begin
            edge.poly[1] := i;
            edge.polyEdge[1] := j;
            break;
          end;

          e := nextEdge[e];
        end;
      end;
    end;
  end;

  // Store adjacency
  for i := 0 to edgeCount - 1 do
  begin
    e1 := @edges[i];
    if (e1.poly[0] <> e1.poly[1]) then
    begin
      p0 := @polys[e1.poly[0]*vertsPerPoly*2];
      p1 := @polys[e1.poly[1]*vertsPerPoly*2];
      p0[vertsPerPoly + e1.polyEdge[0]] := e1.poly[1];
      p1[vertsPerPoly + e1.polyEdge[1]] := e1.poly[0];
    end;
  end;

  FreeMem(firstEdge);
  FreeMem(edges);

  Result := true;
end;


const VERTEX_BUCKET_COUNT = (1 shl 12);

function computeVertexHash(x,y,z: Integer): Integer;
const h1: Cardinal = $8da6b343; // Large multiplicative constants;
const h2: Cardinal = $d8163841; // here arbitrarily chosen primes
const h3: Cardinal = $cb1ab31f;
var n: Cardinal;
begin
  n := Cardinal(h1 * x + h2 * y + h3 * z);
  Result := Integer(n and (VERTEX_BUCKET_COUNT-1));
end;

function addVertex(x,y,z: Word;
                verts: PWord; firstVert: PInteger; nextVert: PInteger; nv: PInteger): Word;
var bucket,i: Integer; v: PWord;
begin
  bucket := computeVertexHash(x, 0, z);
  i := firstVert[bucket];

  while (i <> -1) do
  begin
    v := @verts[i*3];
    if (v[0] = x) and (Abs(v[1] - y) <= 2) and (v[2] = z) then
      Exit(i);
    i := nextVert[i]; // next
  end;

  // Could not find, create new.
  i := nv^; Inc(nv^);
  v := @verts[i*3];
  v[0] := x;
  v[1] := y;
  v[2] := z;
  nextVert[i] := firstVert[bucket];
  firstVert[bucket] := i;

  Exit(i);
end;

function prev(i, n: Integer): Integer; begin Result := (i+n-1) mod n; end;
function next(i, n: Integer): Integer; begin Result := (i+1) mod n; end;

function area2(const a,b,c: PInteger): Integer;
begin
  Result := (b[0] - a[0]) * (c[2] - a[2]) - (c[0] - a[0]) * (b[2] - a[2]);
end;

//  Exclusive or: true iff exactly one argument is true.
//  The arguments are negated to ensure that they are 0/1
//  values.  Then the bitwise Xor operator may apply.
//  (This idea is due to Michael Baldwin.)
function xorb(x, y: Boolean): Boolean;
begin
  Result := not x xor not y;
end;

// Returns true iff c is strictly to the left of the directed
// line through a to b.
function left(const a,b,c: PInteger): Boolean;
begin
  Result := area2(a, b, c) < 0;
end;

function leftOn(const a,b,c: PInteger): Boolean;
begin
  Result := area2(a, b, c) <= 0;
end;

function collinear(const a,b,c: PInteger): Boolean;
begin
  Result := area2(a, b, c) = 0;
end;

//  Returns true iff ab properly intersects cd: they share
//  a point interior to both segments.  The properness of the
//  intersection is ensured by using strict leftness.
function intersectProp(const a,b,c,d: PInteger): Boolean;
begin
  // Eliminate improper cases.
  if (collinear(a,b,c) or collinear(a,b,d) or
    collinear(c,d,a) or collinear(c,d,b)) then
    Exit(false);

  Result := xorb(left(a,b,c), left(a,b,d)) and xorb(left(c,d,a), left(c,d,b));
end;

// Returns T iff (a,b,c) are collinear and point c lies
// on the closed segement ab.
function between(const a,b,c: PInteger): Boolean;
begin
  if (not collinear(a, b, c)) then
    Exit(false);
  // If ab not vertical, check betweenness on x; else on y.
  if (a[0] <> b[0]) then
    Result :=  ((a[0] <= c[0]) and (c[0] <= b[0])) or ((a[0] >= c[0]) and (c[0] >= b[0]))
  else
    Result :=  ((a[2] <= c[2]) and (c[2] <= b[2])) or ((a[2] >= c[2]) and (c[2] >= b[2]));
end;

// Returns true iff segments ab and cd intersect, properly or improperly.
function intersect(const a,b,c,d: PInteger): Boolean;
begin
  if (intersectProp(a, b, c, d)) then
    Result := true
  else if (between(a, b, c) or between(a, b, d) or
       between(c, d, a) or between(c, d, b)) then
    Result := true
  else
    Result := false;
end;

function vequal(const a,b: PInteger): Boolean;
begin
  Result := (a[0] = b[0]) and (a[2] = b[2]);
end;

// Returns T iff (v_i, v_j) is a proper internal *or* external
// diagonal of P, *ignoring edges incident to v_i and v_j*.
function diagonalie(i, j, n: Integer; const verts: PInteger; indices: PInteger): Boolean;
var d0,d1,p0,p1: PInteger; k,k1: Integer;
begin
  d0 := @verts[(indices[i] and $0fffffff) * 4];
  d1 := @verts[(indices[j] and $0fffffff) * 4];

  // For each edge (k,k+1) of P
  for k := 0 to n - 1 do
  begin
    k1 := next(k, n);
    // Skip edges incident to i or j
    if (not ((k = i) or (k1 = i) or (k = j) or (k1 = j))) then
    begin
      p0 := @verts[(indices[k] and $0fffffff) * 4];
      p1 := @verts[(indices[k1] and $0fffffff) * 4];

      if (vequal(d0, p0) or vequal(d1, p0) or vequal(d0, p1) or vequal(d1, p1)) then
        continue;

      if (intersect(d0, d1, p0, p1)) then
        Exit(false);
    end;
  end;
  Result := true;
end;

// Returns true iff the diagonal (i,j) is strictly internal to the
// polygon P in the neighborhood of the i endpoint.
function inCone(i, j, n: Integer; const verts: PInteger; indices: PInteger): Boolean;
var pi,pj,pi1,pin1: PInteger;
begin
  pi := @verts[(indices[i] and $0fffffff) * 4];
  pj := @verts[(indices[j] and $0fffffff) * 4];
  pi1 := @verts[(indices[next(i, n)] and $0fffffff) * 4];
  pin1 := @verts[(indices[prev(i, n)] and $0fffffff) * 4];

  // If P[i] is a convex vertex [ i+1 left or on (i-1,i) ].
  if (leftOn(pin1, pi, pi1)) then
    Exit(left(pi, pj, pin1) and left(pj, pi, pi1));
  // Assume (i-1,i,i+1) not collinear.
  // else P[i] is reflex.
  Result := not (leftOn(pi, pj, pi1) and leftOn(pj, pi, pin1));
end;

// Returns T iff (v_i, v_j) is a proper internal
// diagonal of P.
function diagonal(i, j, n: Integer; const verts: PInteger; indices: PInteger): Boolean;
begin
  Result := inCone(i, j, n, verts, indices) and diagonalie(i, j, n, verts, indices);
end;


function diagonalieLoose(i, j, n: Integer; const verts: PInteger; indices: PInteger): Boolean;
var d0,d1,p0,p1: PInteger; k,k1: Integer;
begin
  d0 := @verts[(indices[i] and $0fffffff) * 4];
  d1 := @verts[(indices[j] and $0fffffff) * 4];

  // For each edge (k,k+1) of P
  for k := 0 to n - 1 do
  begin
    k1 := next(k, n);
    // Skip edges incident to i or j
    if (not ((k = i) or (k1 = i) or (k = j) or (k1 = j))) then
    begin
      p0 := @verts[(indices[k] and $0fffffff) * 4];
      p1 := @verts[(indices[k1] and $0fffffff) * 4];

      if (vequal(d0, p0) or vequal(d1, p0) or vequal(d0, p1) or vequal(d1, p1)) then
        continue;

      if (intersectProp(d0, d1, p0, p1)) then
        Exit(false);
    end;
  end;
  Result := true;
end;

function inConeLoose(i, j, n: Integer; const verts: PInteger; indices: PInteger): Boolean;
var pi,pj,pi1,pin1: PInteger;
begin
  pi := @verts[(indices[i] and $0fffffff) * 4];
  pj := @verts[(indices[j] and $0fffffff) * 4];
  pi1 := @verts[(indices[next(i, n)] and $0fffffff) * 4];
  pin1 := @verts[(indices[prev(i, n)] and $0fffffff) * 4];

  // If P[i] is a convex vertex [ i+1 left or on (i-1,i) ].
  if (leftOn(pin1, pi, pi1)) then
    Exit(leftOn(pi, pj, pin1) and leftOn(pj, pi, pi1));
  // Assume (i-1,i,i+1) not collinear.
  // else P[i] is reflex.
  Result := not (leftOn(pi, pj, pi1) and leftOn(pj, pi, pin1));
end;

function diagonalLoose(i, j, n: Integer; const verts: PInteger; indices: PInteger): Boolean;
begin
  Result := inConeLoose(i, j, n, verts, indices) and diagonalieLoose(i, j, n, verts, indices);
end;


function triangulate(n: Integer; const verts: PInteger; indices: PInteger; tris: PInteger): Integer;
var ntris,i,minLen,mini,i1,i2,dx,dy,len: Integer; dst,p0,p2: PInteger; k: Integer;
begin
  ntris := 0;
  dst := tris;

  // The last bit of the index is used to indicate if the vertex can be removed.
  for i := 0 to n - 1 do
  begin
    i1 := next(i, n);
    i2 := next(i1, n);
    if (diagonal(i, i2, n, verts, indices)) then
      indices[i1] := Integer(indices[i1] or $80000000);
  end;

  while (n > 3) do
  begin
    minLen := -1;
    mini := -1;
    for i := 0 to n - 1 do
    begin
      i1 := next(i, n);
      if (indices[i1] and $80000000 <> 0) then
      begin
        p0 := @verts[(indices[i] and $0fffffff) * 4];
        p2 := @verts[(indices[next(i1, n)] and $0fffffff) * 4];

        dx := p2[0] - p0[0];
        dy := p2[2] - p0[2];
        len := dx*dx + dy*dy;

        if (minLen < 0) or (len < minLen) then
        begin
          minLen := len;
          mini := i;
        end;
      end;
    end;

    if (mini = -1) then
    begin
      // We might get here because the contour has overlapping segments, like this:
      //
      //  A o-o=====o---o B
      //   /  |C   D|    \
      //  o   o     o     o
      //  :   :     :     :
      // We'll try to recover by loosing up the inCone test a bit so that a diagonal
      // like A-B or C-D can be found and we can continue.
      minLen := -1;
      mini := -1;
      for i := 0 to n - 1 do
      begin
        i1 := next(i, n);
        i2 := next(i1, n);
        if (diagonalLoose(i, i2, n, verts, indices)) then
        begin
          p0 := @verts[(indices[i] and $0fffffff) * 4];
          p2 := @verts[(indices[next(i2, n)] and $0fffffff) * 4];
          dx := p2[0] - p0[0];
          dy := p2[2] - p0[2];
          len := dx*dx + dy*dy;

          if (minLen < 0) or (len < minLen) then
          begin
            minLen := len;
            mini := i;
          end;
        end;
      end;
      if (mini = -1) then
      begin
        // The contour is messed up. This sometimes happens
        // if the contour simplification is too aggressive.
        Exit(-ntris);
      end;
    end;

    i := mini;
    i1 := next(i, n);
    i2 := next(i1, n);

    //*dst++ := indices[i] and $0fffffff;
    //*dst++ := indices[i1] and $0fffffff;
    //*dst++ := indices[i2] and $0fffffff;
    dst^ := indices[i] and $0fffffff; Inc(dst);
    dst^ := indices[i1] and $0fffffff; Inc(dst);
    dst^ := indices[i2] and $0fffffff; Inc(dst);
    Inc(ntris);

    // Removes P[i1] by copying P[i+1]...P[n-1] left one index.
    Dec(n);
    for k := i1 to n - 1 do
      indices[k] := indices[k+1];

    if (i1 >= n) then i1 := 0;
    i := prev(i1,n);
    // Update diagonal flags.
    if (diagonal(prev(i, n), i1, n, verts, indices)) then
      indices[i] := Integer(indices[i] or $80000000)
    else
      indices[i] := indices[i] and $0fffffff;

    if (diagonal(i, next(i1, n), n, verts, indices)) then
      indices[i1] := Integer(indices[i1] or $80000000)
    else
      indices[i1] := indices[i1] and $0fffffff;
  end;

  // Append the remaining triangle.
  //*dst++ := indices[0] and $0fffffff;
  //*dst++ := indices[1] and $0fffffff;
  //*dst++ := indices[2] and $0fffffff;
  dst^ := indices[0] and $0fffffff; Inc(dst);
  dst^ := indices[1] and $0fffffff; Inc(dst);
  dst^ := indices[2] and $0fffffff; //Inc(dst);
  Inc(ntris);

  Result := ntris;
end;

function countPolyVerts(const p: PWord; const nvp: Integer): Integer;
var i: Integer;
begin
  for i := 0 to nvp - 1 do
    if (p[i] = RC_MESH_NULL_IDX) then
      Exit(i);
  Result := nvp;
end;

function uleft(const a,b,c: PWord): Boolean;
begin
  Result := (Integer(b[0]) - Integer(a[0])) * (Integer(c[2]) - Integer(a[2])) -
       (Integer(c[0]) - Integer(a[0])) * (Integer(b[2]) - Integer(a[2])) < 0;
end;

function getPolyMergeValue(pa, pb: PWord;
               const verts: PWord; ea, eb: PInteger;
               const nvp: Integer): Integer;
var na,nb,i,j: Integer; va0,va1,vb0,vb1,va,vb,vc: Word; dx,dy: Integer;
begin
  na := countPolyVerts(pa, nvp);
  nb := countPolyVerts(pb, nvp);

  // If the merged polygon would be too big, do not merge.
  if (na+nb-2 > nvp) then
    Exit(-1);

  // Check if the polygons share an edge.
  ea^ := -1;
  eb^ := -1;

  for i := 0 to na - 1 do
  begin
    va0 := pa[i];
    va1 := pa[(i+1) mod na];
    if (va0 > va1) then
      rcSwap(va0, va1);
    for j := 0 to nb - 1 do
    begin
      vb0 := pb[j];
      vb1 := pb[(j+1) mod nb];
      if (vb0 > vb1) then
        rcSwap(vb0, vb1);
      if (va0 = vb0) and (va1 = vb1) then
      begin
        ea^ := i;
        eb^ := j;
        break;
      end;
    end;
  end;

  // No common edge, cannot merge.
  if (ea^ = -1) or (eb^ = -1) then
    Exit(-1);

  // Check to see if the merged polygon would be convex.
  //unsigned short va, vb, vc;

  va := pa[(ea^+na-1) mod na];
  vb := pa[ea^];
  vc := pb[(eb^+2) mod nb];
  if (not uleft(@verts[va*3], @verts[vb*3], @verts[vc*3])) then
    Exit(-1);

  va := pb[(eb^+nb-1) mod nb];
  vb := pb[eb^];
  vc := pa[(ea^+2) mod na];
  if (not uleft(@verts[va*3], @verts[vb*3], @verts[vc*3])) then
    Exit(-1);

  va := pa[ea^];
  vb := pa[(ea^+1) mod na];

  dx := Integer(verts[va*3+0]) - Integer(verts[vb*3+0]);
  dy := Integer(verts[va*3+2]) - Integer(verts[vb*3+2]);

  Result := dx*dx + dy*dy;
end;

procedure mergePolys(pa, pb: PWord; ea, eb: Integer;
             tmp: PWord; const nvp: Integer);
var na,nb,i,n: Integer;
begin
  na := countPolyVerts(pa, nvp);
  nb := countPolyVerts(pb, nvp);

  // Merge polygons.
  FillChar(tmp[0], sizeof(Word)*nvp, $ff);
  n := 0;
  // Add pa
  for i := 0 to na-1 - 1 do
  begin
    tmp[n] := pa[(ea+1+i) mod na];
    Inc(n);
  end;
  // Add pb
  for i := 0 to nb-1 - 1 do
  begin
    tmp[n] := pb[(eb+1+i) mod nb];
    Inc(n);
  end;

  Move(tmp^, pa^, sizeof(Word) * nvp);
end;


procedure pushFront(v: Integer; arr: PInteger; an: PInteger);
var i: Integer;
begin
  Inc(an^);
  for i := an^ - 1 downto 1 do arr[i] := arr[i-1];
  arr[0] := v;
end;

procedure pushBack(v: Integer; arr: PInteger; an: PInteger);
begin
  arr[an^] := v;
  Inc(an^);
end;

function canRemoveVertex(ctx: TrcContext; mesh: PrcPolyMesh; const rem: Word): Boolean;
var nvp,numRemovedVerts,numTouchedVerts,numRemainingEdges,i,j,k,m,nv,numRemoved,numVerts,maxEdges,nedges: Integer; p: PWord;
edges: PInteger; a,b: Integer; e: PInteger; exists: Boolean; numOpenEdges: Integer;
begin
  nvp := mesh.nvp;

  // Count number of polygons to remove.
  numRemovedVerts := 0;
  numTouchedVerts := 0;
  numRemainingEdges := 0;
  for i := 0 to mesh.npolys - 1 do
  begin
    p := @mesh.polys[i*nvp*2];
    nv := countPolyVerts(p, nvp);
    numRemoved := 0;
    numVerts := 0;
    for j := 0 to nv - 1 do
    begin
      if (p[j] = rem) then
      begin
        Inc(numTouchedVerts);
        Inc(numRemoved);
      end;
      Inc(numVerts);
    end;
    if (numRemoved <> 0) then
    begin
      Inc(numRemovedVerts, numRemoved);
      Inc(numRemainingEdges, numVerts-(numRemoved+1));
    end;
  end;

  // There would be too few edges remaining to create a polygon.
  // This can happen for example when a tip of a triangle is marked
  // as deletion, but there are no other polys that share the vertex.
  // In this case, the vertex should not be removed.
  if (numRemainingEdges <= 2) then
    Exit(false);

  // Find edges which share the removed vertex.
  maxEdges := numTouchedVerts*2;
  nedges := 0;
  GetMem(edges, SizeOf(Integer)*maxEdges*3);

  for i := 0 to mesh.npolys - 1 do
  begin
    p := @mesh.polys[i*nvp*2];
    nv := countPolyVerts(p, nvp);

    // Collect edges which touches the removed vertex.
    //for (int j := 0, k := nv-1; j < nv; k := j++)
    j := 0;
    k := nv-1;
    while (j < nv) do
    begin
      if (p[j] = rem) or (p[k] = rem) then
      begin
        // Arrange edge so that a=rem.
        a := p[j]; b := p[k];
        if (b = rem) then
          rcSwap(a,b);

        // Check if the edge exists
        exists := false;
        for m := 0 to nedges - 1 do
        begin
          e := @edges[m*3];
          if (e[1] = b) then
          begin
            // Exists, increment vertex share count.
            Inc(e[2]);
            exists := true;
          end;
        end;
        // Add new edge.
        if (not exists) then
        begin
          e := @edges[nedges*3];
          e[0] := a;
          e[1] := b;
          e[2] := 1;
          Inc(nedges);
        end;
      end;

      k := j;
      Inc(j);
    end;
  end;

  // There should be no more than 2 open edges.
  // This catches the case that two non-adjacent polygons
  // share the removed vertex. In that case, do not remove the vertex.
  numOpenEdges := 0;
  for i := 0 to nedges - 1 do
  begin
    if (edges[i*3+2] < 2) then
      Inc(numOpenEdges);
  end;

  FreeMem(edges);

  if (numOpenEdges > 2) then
    Exit(false);

  Result := true;
end;

function removeVertex(ctx: TrcContext; mesh: PrcPolyMesh; const rem: Word; const maxTris: Integer): Boolean;
var nvp,numRemovedVerts,i,j,k,nv,nedges,nhole,nhreg,nharea: Integer; p,p2: PWord; edges,hole,hreg,harea: PInteger;
hasRem,match,add: Boolean; e: PInteger; ea,eb,r,a: Integer; tris,tverts,thole: PInteger; pi: Integer; ntris: Integer;
polys,pregs,pareas: PWord; tmpPoly: PWord; npolys: Integer; t: PInteger; bestMergeVal,bestPa,bestPb,bestEa,bestEb: Integer;
pj,pk,pa,pb,last: PWord; v: Integer;
begin
  nvp := mesh.nvp;

  // Count number of polygons to remove.
  numRemovedVerts := 0;
  for i := 0 to mesh.npolys - 1 do
  begin
    p := @mesh.polys[i*nvp*2];
    nv := countPolyVerts(p, nvp);
    for j := 0 to nv - 1 do
    begin
      if (p[j] = rem) then
        Inc(numRemovedVerts);
    end;
  end;

  nedges := 0;
  GetMem(edges, SizeOf(Integer)*numRemovedVerts*nvp*4);

  nhole := 0;
  GetMem(hole, SizeOf(Integer)*numRemovedVerts*nvp);

  nhreg := 0;
  GetMem(hreg, SizeOf(Integer)*numRemovedVerts*nvp);

  nharea := 0;
  GetMem(harea, SizeOf(Integer)*numRemovedVerts*nvp);

  i := 0;
  while (i < mesh.npolys) do
  begin
    p := @mesh.polys[i*nvp*2];
    nv := countPolyVerts(p, nvp);
    hasRem := false;
    for j := 0 to nv - 1 do
      if (p[j] = rem) then hasRem := true;
    if (hasRem) then
    begin
      // Collect edges which does not touch the removed vertex.
      j := 0;
      k := nv-1;
      while (j < nv) do
      begin
        if (p[j] <> rem) and (p[k] <> rem) then
        begin
          e := @edges[nedges*4];
          e[0] := p[k];
          e[1] := p[j];
          e[2] := mesh.regs[i];
          e[3] := mesh.areas[i];
          Inc(nedges);
        end;
        k := j;
        Inc(j);
      end;
      // Remove the polygon.
      p2 := @mesh.polys[(mesh.npolys-1)*nvp*2];
      if (p <> p2) then
        Move(p2^,p^,sizeof(Word)*nvp);
      //memset(p+nvp,0xff,sizeof(unsigned short)*nvp);
      FillChar(p[nvp], sizeof(Word)*nvp, $ff);
      mesh.regs[i] := mesh.regs[mesh.npolys-1];
      mesh.areas[i] := mesh.areas[mesh.npolys-1];
      Dec(mesh.npolys);
      Dec(i);
    end;

    Inc(i);
  end;

  // Remove vertex.
  for i := Integer(rem) to mesh.nverts - 2 do
  begin
    mesh.verts[i*3+0] := mesh.verts[(i+1)*3+0];
    mesh.verts[i*3+1] := mesh.verts[(i+1)*3+1];
    mesh.verts[i*3+2] := mesh.verts[(i+1)*3+2];
  end;
  Dec(mesh.nverts);

  // Adjust indices to match the removed vertex layout.
  for i := 0 to mesh.npolys - 1 do
  begin
    p := @mesh.polys[i*nvp*2];
    nv := countPolyVerts(p, nvp);
    for j := 0 to nv - 1 do
      if (p[j] > rem) then Dec(p[j]);
  end;
  for i := 0 to nedges - 1 do
  begin
    if (edges[i*4+0] > rem) then Dec(edges[i*4+0]);
    if (edges[i*4+1] > rem) then Dec(edges[i*4+1]);
  end;

  if (nedges = 0) then
    Exit(true);

  // Start with one vertex, keep appending connected
  // segments to the start and end of the hole.
  pushBack(edges[0], hole, @nhole);
  pushBack(edges[2], hreg, @nhreg);
  pushBack(edges[3], harea, @nharea);

  while (nedges <> 0) do
  begin
    match := false;

    i := 0;
    while i < nedges do
    begin
      ea := edges[i*4+0];
      eb := edges[i*4+1];
      r := edges[i*4+2];
      a := edges[i*4+3];
      add := false;
      if (hole[0] = eb) then
      begin
        // The segment matches the beginning of the hole boundary.
        pushFront(ea, hole, @nhole);
        pushFront(r, hreg, @nhreg);
        pushFront(a, harea, @nharea);
        add := true;
      end
      else if (hole[nhole-1] = ea) then
      begin
        // The segment matches the end of the hole boundary.
        pushBack(eb, hole, @nhole);
        pushBack(r, hreg, @nhreg);
        pushBack(a, harea, @nharea);
        add := true;
      end;
      if (add) then
      begin
        // The edge segment was added, remove it.
        edges[i*4+0] := edges[(nedges-1)*4+0];
        edges[i*4+1] := edges[(nedges-1)*4+1];
        edges[i*4+2] := edges[(nedges-1)*4+2];
        edges[i*4+3] := edges[(nedges-1)*4+3];
        Dec(nedges);
        match := true;
        Dec(i);
      end;

      Inc(i);
    end;

    if (not match) then
      break;
  end;

  GetMem(tris, SizeOf(Integer)*nhole*3);
  GetMem(tverts, SizeOf(Integer)*nhole*4);
  GetMem(thole, SizeOf(Integer)*nhole);

  // Generate temp vertex array for triangulation.
  for i := 0 to nhole - 1 do
  begin
    pi := hole[i];
    tverts[i*4+0] := mesh.verts[pi*3+0];
    tverts[i*4+1] := mesh.verts[pi*3+1];
    tverts[i*4+2] := mesh.verts[pi*3+2];
    tverts[i*4+3] := 0;
    thole[i] := i;
  end;

  // Triangulate the hole.
  ntris := triangulate(nhole, @tverts[0], @thole[0], tris);
  if (ntris < 0) then
  begin
    ntris := -ntris;
    ctx.log(RC_LOG_WARNING, 'removeVertex: triangulate() returned bad results.');
  end;

  // Merge the hole triangles back to polygons.
  GetMem(polys, SizeOf(Word)*(ntris+1)*nvp);
  GetMem(pregs, SizeOf(Word)*ntris);
  GetMem(pareas, SizeOf(Word)*ntris);

  tmpPoly := @polys[ntris*nvp];

  // Build initial polygons.
  npolys := 0;
  FillChar(polys[0], ntris*nvp*sizeof(Word), $ff);
  for j := 0 to ntris - 1 do
  begin
    t := @tris[j*3];
    if (t[0] <> t[1]) and (t[0] <> t[2]) and (t[1] <> t[2]) then
    begin
      polys[npolys*nvp+0] := hole[t[0]];
      polys[npolys*nvp+1] := hole[t[1]];
      polys[npolys*nvp+2] := hole[t[2]];
      pregs[npolys] := hreg[t[0]];
      pareas[npolys] := harea[t[0]];
      Inc(npolys);
    end;
  end;
  if (npolys = 0) then
    Exit(true);

  // Merge polygons.
  if (nvp > 3) then
  begin
    while (true) do
    begin
      // Find best polygons to merge.
      bestMergeVal := 0;
      bestPa := 0; bestPb := 0; bestEa := 0; bestEb := 0;

      for j := 0 to npolys-1 - 1 do
      begin
        pj := @polys[j*nvp];
        for k := j+1 to npolys - 1 do
        begin
          pk := @polys[k*nvp];

          v := getPolyMergeValue(pj, pk, mesh.verts, @ea, @eb, nvp);
          if (v > bestMergeVal) then
          begin
            bestMergeVal := v;
            bestPa := j;
            bestPb := k;
            bestEa := ea;
            bestEb := eb;
          end;
        end;
      end;

      if (bestMergeVal > 0) then
      begin
        // Found best, merge.
        pa := @polys[bestPa*nvp];
        pb := @polys[bestPb*nvp];
        mergePolys(pa, pb, bestEa, bestEb, tmpPoly, nvp);
        last := @polys[(npolys-1)*nvp];
        if (pb <> last) then
          Move(last^, pb^, sizeof(Word)*nvp);
        pregs[bestPb] := pregs[npolys-1];
        pareas[bestPb] := pareas[npolys-1];
        Dec(npolys);
      end
      else
      begin
        // Could not merge any polygons, stop.
        break;
      end;
    end;
  end;

  // Store polygons.
  for i := 0 to npolys - 1 do
  begin
    if (mesh.npolys >= maxTris) then break;
    p := @mesh.polys[mesh.npolys*nvp*2];
    FillChar(p[0],sizeof(Word)*nvp*2,$ff);
    for j := 0 to nvp - 1 do
      p[j] := polys[i*nvp+j];
    mesh.regs[mesh.npolys] := pregs[i];
    mesh.areas[mesh.npolys] := pareas[i];
    Inc(mesh.npolys);
    if (mesh.npolys > maxTris) then
    begin
      ctx.log(RC_LOG_ERROR, Format('removeVertex: Too many polygons %d (max:%d).', [mesh.npolys, maxTris]));
      Exit(false);
    end;
  end;

  FreeMem(edges);
  FreeMem(hole);
  FreeMem(hreg);
  FreeMem(harea);

  FreeMem(tris);
  FreeMem(tverts);
  FreeMem(thole);

  FreeMem(polys);
  FreeMem(pregs);
  FreeMem(pareas);

  Result := true;
end;

/// @par
///
/// @note If the mesh data is to be used to construct a Detour navigation mesh, then the upper
/// limit must be retricted to <= #DT_VERTS_PER_POLYGON.
///
/// @see rcAllocPolyMesh, rcContourSet, rcPolyMesh, rcConfig
function rcBuildPolyMesh(ctx: TrcContext; cset: PrcContourSet; const nvp: Integer; mesh: PrcPolyMesh): Boolean;
var maxVertices, maxTris, maxVertsPerCont: Integer; i,j,k: Integer; vflags: PByte; firstVert,nextVert: PInteger;
indices,tris: PInteger; polys: PWord; tmpPoly: PWord; cont: PrcContour; ntris: Integer; v,t: PInteger; npolys: Integer;
bestMergeVal,bestPa,bestPb,bestEa,bestEb: Integer; pj,pk: PWord; ea,eb: Integer; pa,pb,lastPoly,p,q,va,vb: PWord; v1,nj: Integer;
w,h: Integer;
begin
  Assert(ctx <> nil);

  ctx.startTimer(RC_TIMER_BUILD_POLYMESH);

  rcVcopy(@mesh.bmin[0], @cset.bmin[0]);
  rcVcopy(@mesh.bmax[0], @cset.bmax[0]);
  mesh.cs := cset.cs;
  mesh.ch := cset.ch;
  mesh.borderSize := cset.borderSize;

  maxVertices := 0;
  maxTris := 0;
  maxVertsPerCont := 0;
  for i := 0 to cset.nconts - 1 do
  begin
    // Skip null contours.
    if (cset.conts[i].nverts < 3) then continue;
    Inc(maxVertices, cset.conts[i].nverts);
    Inc(maxTris, cset.conts[i].nverts - 2);
    maxVertsPerCont := rcMax(maxVertsPerCont, cset.conts[i].nverts);
  end;

  if (maxVertices >= $fffe) then
  begin
    ctx.log(RC_LOG_ERROR, Format('rcBuildPolyMesh: Too many vertices %d.', [maxVertices]));
    Exit(false);
  end;

  GetMem(vflags, SizeOf(Byte)*maxVertices);
  FillChar(vflags[0], SizeOf(Byte)*maxVertices, 0);
  GetMem(mesh.verts, SizeOf(Word)*maxVertices*3);
  GetMem(mesh.polys, SizeOf(Word)*maxTris*nvp*2);
  GetMem(mesh.regs, SizeOf(Word)*maxTris);
  GetMem(mesh.areas, SizeOf(Byte)*maxTris);

  mesh.nverts := 0;
  mesh.npolys := 0;
  mesh.nvp := nvp;
  mesh.maxpolys := maxTris;

  FillChar(mesh.verts[0], sizeof(Word)*maxVertices*3, 0);
  FillChar(mesh.polys[0], sizeof(Word)*maxTris*nvp*2, $ff);
  FillChar(mesh.regs[0], sizeof(Word)*maxTris, 0);
  FillChar(mesh.areas[0], sizeof(Byte)*maxTris, 0);

  GetMem(nextVert, SizeOf(Integer)*maxVertices);
  FillChar(nextVert[0], sizeof(integer)*maxVertices, 0);
  GetMem(firstVert, SizeOf(Integer)*VERTEX_BUCKET_COUNT);
  for i := 0 to VERTEX_BUCKET_COUNT - 1 do
    firstVert[i] := -1;

  GetMem(indices, SizeOf(Integer)*maxVertsPerCont);
  GetMem(tris, SizeOf(Integer)*maxVertsPerCont*3);
  GetMem(polys, SizeOf(Word)*(maxVertsPerCont+1)*nvp);
  tmpPoly := @polys[maxVertsPerCont*nvp];

  for i := 0 to cset.nconts - 1 do
  begin
    cont := @cset.conts[i];

    // Skip null contours.
    if (cont.nverts < 3) then
      continue;

    // Triangulate contour
    for j := 0 to cont.nverts - 1 do
      indices[j] := j;

    ntris := triangulate(cont.nverts, cont.verts, @indices[0], @tris[0]);
    if (ntris <= 0) then
    begin
      // Bad triangulation, should not happen.
(*      printf('\tconst float bmin[3] := begin%ff,%ff,%ffend;;\n', cset.bmin[0], cset.bmin[1], cset.bmin[2]);
      printf('\tconst float cs := %ff;\n', cset.cs);
      printf('\tconst float ch := %ff;\n', cset.ch);
      printf('\tconst int verts[] := begin\n');
      for (int k := 0; k < cont.nverts; ++k)
      begin
        const int* v := &cont.verts[k*4];
        printf('\t\t%d,%d,%d,%d,\n', v[0], v[1], v[2], v[3]);
      end;
      printf('\tend;;\n\tconst int nverts := sizeof(verts)/(sizeof(int)*4);\n');*)
      ctx.log(RC_LOG_WARNING, Format('rcBuildPolyMesh: Bad triangulation Contour %d.', [i]));
      ntris := -ntris;
    end;

    // Add and merge vertices.
    for j := 0 to cont.nverts - 1 do
    begin
      v := @cont.verts[j*4];
      indices[j] := addVertex(v[0], v[1], v[2],
                   mesh.verts, firstVert, nextVert, @mesh.nverts);
      if (v[3] and RC_BORDER_VERTEX <> 0) then
      begin
        // This vertex should be removed.
  //TEMP, otherwise polys on Tile edges are missing!!!!!      vflags[indices[j]] := 1;
      end;
    end;

    // Build initial polygons.
    npolys := 0;
    FillChar(polys[0], maxVertsPerCont*nvp*sizeof(Word), $ff);
    for j := 0 to ntris - 1 do
    begin
      t := @tris[j*3];
      if (t[0] <> t[1]) and (t[0] <> t[2]) and (t[1] <> t[2]) then
      begin
        polys[npolys*nvp+0] := indices[t[0]];
        polys[npolys*nvp+1] := indices[t[1]];
        polys[npolys*nvp+2] := indices[t[2]];
        Inc(npolys);
      end;
    end;
    if (npolys = 0) then
      continue;

    // Merge polygons.
    if (nvp > 3) then
    begin
      while True do
      begin
        // Find best polygons to merge.
        bestMergeVal := 0;
        bestPa := 0; bestPb := 0; bestEa := 0; bestEb := 0;

        for j := 0 to npolys-1 - 1 do
        begin
          pj := @polys[j*nvp];
          for k := j+1 to npolys - 1 do
          begin
            pk := @polys[k*nvp];

            v1 := getPolyMergeValue(pj, pk, mesh.verts, @ea, @eb, nvp);
            if (v1 > bestMergeVal) then
            begin
              bestMergeVal := v1;
              bestPa := j;
              bestPb := k;
              bestEa := ea;
              bestEb := eb;
            end;
          end;
        end;

        if (bestMergeVal > 0) then
        begin
          // Found best, merge.
          pa := @polys[bestPa*nvp];
          pb := @polys[bestPb*nvp];
          mergePolys(pa, pb, bestEa, bestEb, tmpPoly, nvp);
          lastPoly := @polys[(npolys-1)*nvp];
          if (pb <> lastPoly) then
            Move(lastPoly^, pb^, sizeof(Word)*nvp);
          Dec(npolys);
        end
        else
        begin
          // Could not merge any polygons, stop.
          break;
        end;
      end;
    end;

    // Store polygons.
    for j := 0 to npolys - 1 do
    begin
      p := @mesh.polys[mesh.npolys*nvp*2];
      q := @polys[j*nvp];
      for k := 0 to nvp - 1 do
        p[k] := q[k];
      mesh.regs[mesh.npolys] := cont.reg;
      mesh.areas[mesh.npolys] := cont.area;
      Inc(mesh.npolys);
      if (mesh.npolys > maxTris) then
      begin
        ctx.log(RC_LOG_ERROR, Format('rcBuildPolyMesh: Too many polygons %d (max:%d).', [mesh.npolys, maxTris]));
        Exit(false);
      end;
    end;
  end;


  // Remove edge vertices.
  i := 0;
  while (i < mesh.nverts) do
  begin
    if (vflags[i] <> 0) then
    begin
      if (not canRemoveVertex(ctx, mesh, i)) then
      begin
        //C++ seems to be doing loop increase, so do we
        Inc(i);
        Continue;
      end;
      if (not removeVertex(ctx, mesh, i, maxTris)) then
      begin
        // Failed to remove vertex
        ctx.log(RC_LOG_ERROR, Format('rcBuildPolyMesh: Failed to remove edge vertex %d.', [i]));
        Exit(false);
      end;
      // Remove vertex
      // Note: mesh.nverts is already decremented inside removeVertex()!
      // Fixup vertex flags
      for j := i to mesh.nverts - 1 do
        vflags[j] := vflags[j+1];
      Dec(i);
    end;
    Inc(i);
  end;

  // Calculate adjacency.
  if (not buildMeshAdjacency(mesh.polys, mesh.npolys, mesh.nverts, nvp)) then
  begin
    ctx.log(RC_LOG_ERROR, 'rcBuildPolyMesh: Adjacency failed.');
    Exit(false);
  end;

  // Find portal edges
  if (mesh.borderSize > 0) then
  begin
    w := cset.width;
    h := cset.height;
    for i := 0 to mesh.npolys - 1 do
    begin
      p := @mesh.polys[i*2*nvp];
      for j := 0 to nvp - 1 do
      begin
        if (p[j] = RC_MESH_NULL_IDX) then break;
        // Skip connected edges.
        if (p[nvp+j] <> RC_MESH_NULL_IDX) then
          continue;
        nj := j+1;
        if (nj >= nvp) or (p[nj] = RC_MESH_NULL_IDX) then nj := 0;
        va := @mesh.verts[p[j]*3];
        vb := @mesh.verts[p[nj]*3];

        if (va[0] = 0) and (vb[0] = 0) then
          p[nvp+j] := $8000 or 0
        else if (va[2] = h) and (vb[2] = h) then
          p[nvp+j] := $8000 or 1
        else if (va[0] = w) and (vb[0] = w) then
          p[nvp+j] := $8000 or 2
        else if (va[2] = 0) and (vb[2] = 0) then
          p[nvp+j] := $8000 or 3;
      end;
    end;
  end;

  // Just allocate the mesh flags array. The user is resposible to fill it.
  GetMem(mesh.flags, sizeof(Word)*mesh.npolys);
  FillChar(mesh.flags[0], sizeof(Word) * mesh.npolys, $0);

  if (mesh.nverts > $ffff) then
  begin
    ctx.log(RC_LOG_ERROR, Format('rcBuildPolyMesh: The resulting mesh has too many vertices %d (max %d). Data can be corrupted.', [mesh.nverts, $ffff]));
  end;
  if (mesh.npolys > $ffff) then
  begin
    ctx.log(RC_LOG_ERROR, Format('rcBuildPolyMesh: The resulting mesh has too many polygons %d (max %d). Data can be corrupted.', [mesh.npolys, $ffff]));
  end;

  FreeMem(vflags);
  FreeMem(nextVert);
  FreeMem(firstVert);
  FreeMem(indices);
  FreeMem(tris);
  FreeMem(polys);

  ctx.stopTimer(RC_TIMER_BUILD_POLYMESH);

  Result := true;
end;

/// @see rcAllocPolyMesh, rcPolyMesh
function rcMergePolyMeshes(ctx: TrcContext; meshes: array of TrcPolyMesh; const nmeshes: Integer; mesh: PrcPolyMesh): Boolean;
var maxVerts,maxPolys,maxVertsPerMesh,i,j,k: Integer; nextVert,firstVert: PInteger; vremap: PWord; pmesh: PrcPolyMesh; ox,oz: Word;
isMinX, isMinZ, isMaxX, isMaxZ, isOnBorder: Boolean; v,tgt,src: PWord; dir: Word;
begin
  //rcAssert(ctx);

  //if (!nmeshes || !meshes)
  if (nmeshes = 0) {or !meshes)} then
    Exit(true);

  ctx.startTimer(RC_TIMER_MERGE_POLYMESH);

  mesh.nvp := meshes[0].nvp;
  mesh.cs := meshes[0].cs;
  mesh.ch := meshes[0].ch;
  rcVcopy(@mesh.bmin[0], @meshes[0].bmin[0]);
  rcVcopy(@mesh.bmax[0], @meshes[0].bmax[0]);

  maxVerts := 0;
  maxPolys := 0;
  maxVertsPerMesh := 0;
  for i := 0 to nmeshes - 1 do
  begin
    rcVmin(@mesh.bmin[0], @meshes[i].bmin[0]);
    rcVmax(@mesh.bmax[0], @meshes[i].bmax[0]);
    maxVertsPerMesh := rcMax(maxVertsPerMesh, meshes[i].nverts);
    Inc(maxVerts, meshes[i].nverts);
    Inc(maxPolys, meshes[i].npolys);
  end;

  mesh.nverts := 0;
  GetMem(mesh.verts, sizeof(Word)*maxVerts*3);

  mesh.npolys := 0;
  GetMem(mesh.polys, sizeof(Word)*maxPolys*2*mesh.nvp);
  FillChar(mesh.polys[0], sizeof(Word)*maxPolys*2*mesh.nvp, $ff);

  GetMem(mesh.regs, sizeof(Word)*maxPolys);
  FillChar(mesh.regs[0], sizeof(Word)*maxPolys, 0);

  GetMem(mesh.areas, sizeof(Byte)*maxPolys);
  FillChar(mesh.areas[0], sizeof(Byte)*maxPolys, 0);

  GetMem(mesh.flags, sizeof(Word)*maxPolys);
  FillChar(mesh.flags[0], sizeof(Word)*maxPolys, 0);

  GetMem(nextVert, sizeof(integer)*maxVerts);
  FillChar(nextVert[0], sizeof(integer)*maxVerts, 0);

  GetMem(firstVert, sizeof(integer)*VERTEX_BUCKET_COUNT);
  for i := 0 to VERTEX_BUCKET_COUNT - 1 do
    firstVert[i] := -1;

  GetMem(vremap, sizeof(Word)*maxVertsPerMesh);
  FillChar(vremap[0], sizeof(Word)*maxVertsPerMesh, 0);

  for i := 0 to nmeshes - 1 do
  begin
    pmesh := @meshes[i];

    ox := floor((pmesh.bmin[0]-mesh.bmin[0])/mesh.cs+0.5);
    oz := floor((pmesh.bmin[2]-mesh.bmin[2])/mesh.cs+0.5);

    isMinX := (ox = 0);
    isMinZ := (oz = 0);
    isMaxX := (floor((mesh.bmax[0] - pmesh.bmax[0]) / mesh.cs + 0.5)) = 0;
    isMaxZ := (floor((mesh.bmax[2] - pmesh.bmax[2]) / mesh.cs + 0.5)) = 0;
    isOnBorder := (isMinX or isMinZ or isMaxX or isMaxZ);

    for j := 0 to pmesh.nverts - 1 do
    begin
      v := @pmesh.verts[j*3];
      vremap[j] := addVertex(v[0]+ox, v[1], v[2]+oz,
                  @mesh.verts, firstVert, nextVert, @mesh.nverts);
    end;

    for j := 0 to pmesh.npolys - 1 do
    begin
      tgt := @mesh.polys[mesh.npolys*2*mesh.nvp];
      src := @pmesh.polys[j*2*mesh.nvp];
      mesh.regs[mesh.npolys] := pmesh.regs[j];
      mesh.areas[mesh.npolys] := pmesh.areas[j];
      mesh.flags[mesh.npolys] := pmesh.flags[j];
      Inc(mesh.npolys);
      for k := 0 to mesh.nvp - 1 do
      begin
        if (src[k] = RC_MESH_NULL_IDX) then break;
        tgt[k] := vremap[src[k]];
      end;

      if (isOnBorder) then
      begin
        for k := mesh.nvp to mesh.nvp * 2 - 1 do
        begin
          if (src[k] and $8000 <> 0) and (src[k] <> $ffff) then
          begin
            dir := src[k] and $f;
            case dir of
              0: // Portal x-
                if (isMinX) then
                  tgt[k] := src[k];
              1: // Portal z+
                if (isMaxZ) then
                  tgt[k] := src[k];
              2: // Portal x+
                if (isMaxX) then
                  tgt[k] := src[k];
              3: // Portal z-
                if (isMinZ) then
                  tgt[k] := src[k];
            end;
          end;
        end;
      end;
    end;
  end;

  // Calculate adjacency.
  if (not buildMeshAdjacency(mesh.polys, mesh.npolys, mesh.nverts, mesh.nvp)) then
  begin
    ctx.log(RC_LOG_ERROR, 'rcMergePolyMeshes: Adjacency failed.');
    Exit(false);
  end;

  if (mesh.nverts > $ffff) then
  begin
    ctx.log(RC_LOG_ERROR, Format('rcMergePolyMeshes: The resulting mesh has too many vertices %d (max %d). Data can be corrupted.', [mesh.nverts, $ffff]));
  end;
  if (mesh.npolys > $ffff) then
  begin
    ctx.log(RC_LOG_ERROR, Format('rcMergePolyMeshes: The resulting mesh has too many polygons %d (max %d). Data can be corrupted.', [mesh.npolys, $ffff]));
  end;

  FreeMem(nextVert);
  FreeMem(firstVert);
  FreeMem(vremap);

  ctx.stopTimer(RC_TIMER_MERGE_POLYMESH);

  Result := true;
end;

{bool rcCopyPolyMesh(ctx: TrcContext; const rcPolyMesh& src, rcPolyMesh& dst)
begin
  rcAssert(ctx);

  // Destination must be empty.
  rcAssert(dst.verts == 0);
  rcAssert(dst.polys == 0);
  rcAssert(dst.regs == 0);
  rcAssert(dst.areas == 0);
  rcAssert(dst.flags == 0);

  dst.nverts := src.nverts;
  dst.npolys := src.npolys;
  dst.maxpolys := src.npolys;
  dst.nvp := src.nvp;
  rcVcopy(dst.bmin, src.bmin);
  rcVcopy(dst.bmax, src.bmax);
  dst.cs := src.cs;
  dst.ch := src.ch;
  dst.borderSize := src.borderSize;

  dst.verts := (unsigned short*)rcAlloc(sizeof(unsigned short)*src.nverts*3, RC_ALLOC_PERM);
  if (!dst.verts)
  begin
    ctx.log(RC_LOG_ERROR, 'rcCopyPolyMesh: Out of memory 'dst.verts' (%d).', src.nverts*3);
    return false;
  end;
  memcpy(dst.verts, src.verts, sizeof(unsigned short)*src.nverts*3);

  dst.polys := (unsigned short*)rcAlloc(sizeof(unsigned short)*src.npolys*2*src.nvp, RC_ALLOC_PERM);
  if (!dst.polys)
  begin
    ctx.log(RC_LOG_ERROR, 'rcCopyPolyMesh: Out of memory 'dst.polys' (%d).', src.npolys*2*src.nvp);
    return false;
  end;
  memcpy(dst.polys, src.polys, sizeof(unsigned short)*src.npolys*2*src.nvp);

  dst.regs := (unsigned short*)rcAlloc(sizeof(unsigned short)*src.npolys, RC_ALLOC_PERM);
  if (!dst.regs)
  begin
    ctx.log(RC_LOG_ERROR, 'rcCopyPolyMesh: Out of memory 'dst.regs' (%d).', src.npolys);
    return false;
  end;
  memcpy(dst.regs, src.regs, sizeof(unsigned short)*src.npolys);

  dst.areas := (unsigned char*)rcAlloc(sizeof(unsigned char)*src.npolys, RC_ALLOC_PERM);
  if (!dst.areas)
  begin
    ctx.log(RC_LOG_ERROR, 'rcCopyPolyMesh: Out of memory 'dst.areas' (%d).', src.npolys);
    return false;
  end;
  memcpy(dst.areas, src.areas, sizeof(unsigned char)*src.npolys);

  dst.flags := (unsigned short*)rcAlloc(sizeof(unsigned short)*src.npolys, RC_ALLOC_PERM);
  if (!dst.flags)
  begin
    ctx.log(RC_LOG_ERROR, 'rcCopyPolyMesh: Out of memory 'dst.flags' (%d).', src.npolys);
    return false;
  end;
  memcpy(dst.flags, src.flags, sizeof(unsigned short)*src.npolys);

  return true;
end;}

end.