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

unit RN_ChunkyTriMesh;
interface
uses Math, RN_Helper;

type
  TrcChunkyTriMeshNode = record
    bmin, bmax: array [0..1] of Single;
    i, n: Integer;
  end;
  PrcChunkyTriMeshNode = ^TrcChunkyTriMeshNode;

  TrcChunkyTriMesh = record
    nodes: array of TrcChunkyTriMeshNode;
    nnodes: Integer;
    tris: PInteger;
    ntris: Integer;
    maxTrisPerChunk: Integer;
  end;
  PrcChunkyTriMesh = ^TrcChunkyTriMesh;

  /// Creates partitioned triangle mesh (AABB tree),
  /// where each node contains at max trisPerChunk triangles.
  function rcCreateChunkyTriMesh(const verts: PSingle; const tris: PInteger; ntris: Integer; trisPerChunk: Integer; cm: PrcChunkyTriMesh): Boolean;

  /// Returns the chunk indices which overlap the input rectable.
  function rcGetChunksOverlappingRect(const cm: PrcChunkyTriMesh; bmin, bmax: PSingle; ids: PInteger; const maxIds: Integer): Integer;

  /// Returns the chunk indices which overlap the input segment.
  function rcGetChunksOverlappingSegment(const cm: PrcChunkyTriMesh; p, q: PSingle; ids: PInteger; const maxIds: Integer): Integer;


implementation


type
  TBoundsItem = record
    bmin: array [0..1] of Single;
    bmax: array [0..1] of Single;
    i: Integer;
  end;
  PBoundsItem = ^TBoundsItem;

function compareItemX(const va,vb: Pointer): Integer;
var a,b: PBoundsItem;
begin
  a := va;
  b := vb;
  if (a.bmin[0] < b.bmin[0]) then
    Result := -1 else
  if (a.bmin[0] > b.bmin[0]) then
    Result := 1 else
  Result := 0;
end;

function compareItemY(const va,vb: Pointer): Integer;
var a,b: PBoundsItem;
begin
  a := va;
  b := vb;
  if (a.bmin[1] < b.bmin[1]) then
    Result := -1 else
  if (a.bmin[1] > b.bmin[1]) then
    Result := 1 else
  Result := 0;
end;

procedure calcExtends(const items: PBoundsItem; const nitems: Integer;
            const imin, imax: Integer;
            bmin, bmax: PSingle);
var i: Integer; it: PBoundsItem;
begin
  bmin[0] := items[imin].bmin[0];
  bmin[1] := items[imin].bmin[1];

  bmax[0] := items[imin].bmax[0];
  bmax[1] := items[imin].bmax[1];

  for i := imin+1 to imax - 1 do
  begin
    it := @items[i];
    if (it.bmin[0] < bmin[0]) then bmin[0] := it.bmin[0];
    if (it.bmin[1] < bmin[1]) then bmin[1] := it.bmin[1];

    if (it.bmax[0] > bmax[0]) then bmax[0] := it.bmax[0];
    if (it.bmax[1] > bmax[1]) then bmax[1] := it.bmax[1];
  end;
end;

function longestAxis(x, y: Single): Integer;
begin
  Result := Byte(y > x);
end;

procedure subdivide(items: PBoundsItem; nitems, imin, imax, trisPerChunk: Integer;
            curNode: PInteger; nodes: PrcChunkyTriMeshNode; const maxNodes: Integer;
            curTri: PInteger; outTris: PInteger; const inTris: PInteger);
var inum,icur,i: Integer; node: PrcChunkyTriMeshNode; src,dst: PInteger; axis,isplit,iescape: Integer;
begin
  inum := imax - imin;
  icur := curNode^;

  if (curNode^ > maxNodes) then
    Exit;

  node := @nodes[curNode^];
  Inc(curNode^);

  if (inum <= trisPerChunk) then
  begin
    // Leaf
    calcExtends(items, nitems, imin, imax, @node.bmin[0], @node.bmax[0]);

    // Copy triangles.
    node.i := curTri^;
    node.n := inum;

    for i := imin to imax -1 do
    begin
      src := @inTris[items[i].i*3];
      dst := @outTris[curTri^*3];
      Inc(curTri^);
      dst[0] := src[0];
      dst[1] := src[1];
      dst[2] := src[2];
    end;
  end
  else
  begin
    // Split
    calcExtends(items, nitems, imin, imax, @node.bmin[0], @node.bmax[0]);

    axis := longestAxis(node.bmax[0] - node.bmin[0],
                 node.bmax[1] - node.bmin[1]);

    if (axis = 0) then
    begin
      // Sort along x-axis
      qsort(@items[imin], inum, sizeof(TBoundsItem), compareItemX);
    end
    else if (axis = 1) then
    begin
      // Sort along y-axis
      qsort(@items[imin], inum, sizeof(TBoundsItem), compareItemY);
    end;

    isplit := imin+inum div 2;

    // Left
    subdivide(items, nitems, imin, isplit, trisPerChunk, curNode, nodes, maxNodes, curTri, outTris, inTris);
    // Right
    subdivide(items, nitems, isplit, imax, trisPerChunk, curNode, nodes, maxNodes, curTri, outTris, inTris);

    iescape := curNode^ - icur;
    // Negative index means escape.
    node.i := -iescape;
  end;
end;

function rcCreateChunkyTriMesh(const verts: PSingle; const tris: PInteger; ntris: Integer; trisPerChunk: Integer; cm: PrcChunkyTriMesh): Boolean;
var nchunks,i,j: Integer; items: array of TBoundsItem; t: PInteger; it: PBoundsItem; v: PSingle; curTri,curNode: Integer;
node: PrcChunkyTriMeshNode; isLeaf: Boolean;
begin
  nchunks := (ntris + trisPerChunk-1) div trisPerChunk;

  SetLength(cm.nodes, nchunks*4);

  GetMem(cm.tris, SizeOf(Integer)*ntris*3);

  cm.ntris := ntris;

  // Build tree
  SetLength(items, ntris);

  for i := 0 to ntris -1 do
  begin
    t := @tris[i*3];
    it := @items[i];
    it.i := i;
    // Calc triangle XZ bounds.
    it.bmin[0] := verts[t[0]*3+0];
    it.bmax[0] := verts[t[0]*3+0];
    it.bmin[1] := verts[t[0]*3+2];
    it.bmax[1] := verts[t[0]*3+2];
    for j := 1 to 2 do
    begin
      v := @verts[t[j]*3];
      if (v[0] < it.bmin[0]) then it.bmin[0] := v[0];
      if (v[2] < it.bmin[1]) then it.bmin[1] := v[2];

      if (v[0] > it.bmax[0]) then it.bmax[0] := v[0];
      if (v[2] > it.bmax[1]) then it.bmax[1] := v[2];
    end;
  end;

  curTri := 0;
  curNode := 0;
  subdivide(@items[0], ntris, 0, ntris, trisPerChunk, @curNode, @cm.nodes[0], nchunks*4, @curTri, cm.tris, tris);

  SetLength(items, 0);

  cm.nnodes := curNode;

  // Calc max tris per node.
  cm.maxTrisPerChunk := 0;
  for i := 0 to cm.nnodes -1 do
  begin
    node := @cm.nodes[i];
    isLeaf := node.i >= 0;
    if (not isLeaf) then continue;
    if (node.n > cm.maxTrisPerChunk) then
      cm.maxTrisPerChunk := node.n;
  end;

  Result := true;
end;

function checkOverlapRect(amin, amax, bmin, bmax: PSingle): Boolean;
var overlap: Boolean;
begin
  overlap := true;
  if (amin[0] > bmax[0]) or (amax[0] < bmin[0]) then overlap := false;
  if (amin[1] > bmax[1]) or (amax[1] < bmin[1]) then overlap := false;
  Result := overlap;
end;

function rcGetChunksOverlappingRect(const cm: PrcChunkyTriMesh;
    bmin, bmax: PSingle;
    ids: PInteger; const maxIds: Integer): Integer;
var i,n,escapeIndex: Integer; node: PrcChunkyTriMeshNode; overlap,isLeafNode: Boolean;
begin
  // Traverse tree
  i := 0;
  n := 0;
  while (i < cm.nnodes) do
  begin
    node := @cm.nodes[i];
    overlap := checkOverlapRect(bmin, bmax, @node.bmin[0], @node.bmax[0]);
    isLeafNode := node.i >= 0;

    if (isLeafNode and overlap) then
    begin
      if (n < maxIds) then
      begin
        ids[n] := i;
        Inc(n);
      end;
    end;

    if (overlap or isLeafNode) then
      Inc(i)
    else
    begin
      escapeIndex := -node.i;
      i := i + escapeIndex;
    end;
  end;

  Result := n;
end;

function checkOverlapSegment(const p,q,bmin,bmax: PSingle): Boolean;
const EPSILON = 0.000001;
var tmin,tmax,ood,t1,t2,tmp: Single; d: array [0..1] of Single; i: Integer;
begin
  tmin := 0;
  tmax := 1;

  d[0] := q[0] - p[0];
  d[1] := q[1] - p[1];

  for i := 0 to 1 do
  begin
    if (Abs(d[i]) < EPSILON) then
    begin
      // Ray is parallel to slab. No hit if origin not within slab
      if (p[i] < bmin[i]) or (p[i] > bmax[i]) then
        Exit(false);
    end
    else
    begin
      // Compute intersection t value of ray with near and far plane of slab
      ood := 1.0 / d[i];
      t1 := (bmin[i] - p[i]) * ood;
      t2 := (bmax[i] - p[i]) * ood;
      if (t1 > t2) then begin tmp := t1; t1 := t2; t2 := tmp; end;
      if (t1 > tmin) then tmin := t1;
      if (t2 < tmax) then tmax := t2;
      if (tmin > tmax) then Exit(false);
    end;
  end;
  Result := true;
end;

function rcGetChunksOverlappingSegment(const cm: PrcChunkyTriMesh; p, q: PSingle; ids: PInteger; const maxIds: Integer): Integer;
var i,n: Integer; node: PrcChunkyTriMeshNode; overlap,isLeafNode: Boolean; escapeIndex: Integer;
begin
  // Traverse tree
  i := 0;
  n := 0;
  while (i < cm.nnodes) do
  begin
    node := @cm.nodes[i];
    overlap := checkOverlapSegment(p, q, @node.bmin[0], @node.bmax[0]);
    isLeafNode := node.i >= 0;

    if (isLeafNode and overlap) then
    begin
      if (n < maxIds) then
      begin
        ids[n] := i;
        Inc(n);
      end;
    end;

    if (overlap or isLeafNode) then
      Inc(i)
    else
    begin
      escapeIndex := -node.i;
      Inc(i, escapeIndex);
    end;
  end;

  Result := n;
end;

end.
