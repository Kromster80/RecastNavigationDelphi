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

unit RN_RecastContour;
interface
uses
  Math, SysUtils, RN_Helper, RN_Recast;


/// Builds a contour set from the region outlines in the provided compact heightfield.
///  @ingroup recast
///  @param[in,out]  ctx      The build context to use during the operation.
///  @param[in]    chf      A fully built compact heightfield.
///  @param[in]    maxError  The maximum distance a simplfied contour's border edges should deviate
///                the original raw contour. [Limit: >=0] [Units: wu]
///  @param[in]    maxEdgeLen  The maximum allowed length for contour edges along the border of the mesh.
///                [Limit: >=0] [Units: vx]
///  @param[out]  cset    The resulting contour set. (Must be pre-allocated.)
///  @param[in]    buildFlags  The build flags. (See: #rcBuildContoursFlags)
///  @returns True if the operation completed successfully.
function rcBuildContours(ctx: TrcContext; chf: PrcCompactHeightfield;
           const maxError: Single; const maxEdgeLen: Integer;
           cset: PrcContourSet; const buildFlags: Integer = RC_CONTOUR_TESS_WALL_EDGES): Boolean;


implementation
uses RN_RecastAlloc, RN_RecastHelper, RN_RecastContourHelper;

function getCornerHeight(x, y, i, dir: Integer;
               chf: PrcCompactHeightfield;
               isBorderVertex: PBoolean): Integer;
var s,as1,as2: PrcCompactSpan; ch,dirp: Integer; regs: array [0..3] of Integer; ax,ay,ai,ax2,ay2,ai2: Integer;
j,a,b,c,d: Integer; twoSameExts, twoInts, intsSameArea, noZeros: Boolean;
begin
  s := @chf.spans[i];
  ch := s.y;
  dirp := (dir+1) and $3;

  // Combine region and area codes in order to prevent
  // border vertices which are in between two areas to be removed.
  regs[0] := chf.spans[i].reg or (chf.areas[i] shl 16);

  if (rcGetCon(s, dir) <> RC_NOT_CONNECTED) then
  begin
    ax := x + rcGetDirOffsetX(dir);
    ay := y + rcGetDirOffsetY(dir);
    ai := chf.cells[ax+ay*chf.width].index + rcGetCon(s, dir);
    as1 := @chf.spans[ai];
    ch := rcMax(ch, as1.y);
    regs[1] := chf.spans[ai].reg or (chf.areas[ai] shl 16);
    if (rcGetCon(as1, dirp) <> RC_NOT_CONNECTED) then
    begin
      ax2 := ax + rcGetDirOffsetX(dirp);
      ay2 := ay + rcGetDirOffsetY(dirp);
      ai2 := chf.cells[ax2+ay2*chf.width].index + rcGetCon(as1, dirp);
      as2 := @chf.spans[ai2];
      ch := rcMax(ch, as2.y);
      regs[2] := chf.spans[ai2].reg or (chf.areas[ai2] shl 16);
    end;
  end;
  if (rcGetCon(s, dirp) <> RC_NOT_CONNECTED) then
  begin
    ax := x + rcGetDirOffsetX(dirp);
    ay := y + rcGetDirOffsetY(dirp);
    ai := chf.cells[ax+ay*chf.width].index + rcGetCon(s, dirp);
    as1 := @chf.spans[ai];
    ch := rcMax(ch, as1.y);
    regs[3] := chf.spans[ai].reg or (chf.areas[ai] shl 16);
    if (rcGetCon(as1, dir) <> RC_NOT_CONNECTED) then
    begin
      ax2 := ax + rcGetDirOffsetX(dir);
      ay2 := ay + rcGetDirOffsetY(dir);
      ai2 := chf.cells[ax2+ay2*chf.width].index + rcGetCon(as1, dir);
      as2 := @chf.spans[ai2];
      ch := rcMax(ch, as2.y);
      regs[2] := chf.spans[ai2].reg or (chf.areas[ai2] shl 16);
    end;
  end;

  // Check if the vertex is special edge vertex, these vertices will be removed later.
  for j := 0 to 3 do
  begin
    a := j;
    b := (j+1) and $3;
    c := (j+2) and $3;
    d := (j+3) and $3;

    // The vertex is a border vertex there are two same exterior cells in a row,
    // followed by two interior cells and none of the regions are out of bounds.
    twoSameExts := ((regs[a] and regs[b] and RC_BORDER_REG) <> 0) and (regs[a] = regs[b]);
    twoInts := ((regs[c] or regs[d]) and RC_BORDER_REG) = 0;
    intsSameArea := (regs[c] shr 16) = (regs[d] shr 16);
    noZeros := (regs[a] <> 0) and (regs[b] <> 0) and (regs[c] <> 0) and (regs[d] <> 0);
    if (twoSameExts and twoInts and intsSameArea and noZeros) then
    begin
      isBorderVertex^ := true;
      Break;
    end;
  end;

  Result := ch;
end;

procedure walkContour(x,y,i: Integer;
            chf: PrcCompactHeightfield;
            flags: PByte; points: PrcIntArray);
var dir,startDir: Byte; starti: Integer; area: Byte; iter: Integer; isBorderVertex,isAreaBorder: Boolean; px,py,pz: Integer;
r: Integer; s: PrcCompactSpan; ax,ay,ai: Integer; ni,nx,ny: Integer; nc: PrcCompactCell;
begin
  // Choose the first non-connected edge
  dir := 0;
  //while ((flags[i] & (1 << dir)) == 0)
  while ((flags[i] and (1 shl dir)) = 0) do
    Inc(dir);

  startDir := dir;
  starti := i;

  area := chf.areas[i];

  iter := 0;
  while (iter < 40000) do
  begin
    if (flags[i] and (1 shl dir) <> 0) then
    begin
      // Choose the edge corner
      isBorderVertex := false;
      isAreaBorder := false;
      px := x;
      py := getCornerHeight(x, y, i, dir, chf, @isBorderVertex);
      pz := y;
      case dir of
        0: Inc(pz);
        1: begin Inc(px); Inc(pz); end;
        2: Inc(px);
      end;
      r := 0;
      s := @chf.spans[i];
      if (rcGetCon(s, dir) <> RC_NOT_CONNECTED) then
      begin
        ax := x + rcGetDirOffsetX(dir);
        ay := y + rcGetDirOffsetY(dir);
        ai := chf.cells[ax+ay*chf.width].index + rcGetCon(s, dir);
        r := chf.spans[ai].reg;
        if (area <> chf.areas[ai]) then
          isAreaBorder := true;
      end;
      if (isBorderVertex) then
        r := r or RC_BORDER_VERTEX;
      if (isAreaBorder) then
        r := r or RC_AREA_BORDER;

      points.push(px);
      points.push(py);
      points.push(pz);
      points.push(r);

      flags[i] := flags[i] and not (1 shl dir); // Remove visited edges
      dir := (dir+1) and $3;  // Rotate CW
    end
    else
    begin
      ni := -1;
      nx := x + rcGetDirOffsetX(dir);
      ny := y + rcGetDirOffsetY(dir);
      s := @chf.spans[i];
      if (rcGetCon(s, dir) <> RC_NOT_CONNECTED) then
      begin
        nc := @chf.cells[nx+ny*chf.width];
        ni := nc.index + rcGetCon(s, dir);
      end;
      if (ni = -1) then
      begin
        // Should not happen.
        Exit;
      end;
      x := nx;
      y := ny;
      i := ni;
      dir := (dir+3) and $3;  // Rotate CCW
    end;

    if (starti = i) and (startDir = dir) then
    begin
      break;
    end;

    Inc(Iter);
  end;
end;

function distancePtSeg(const x,z: Integer;
                 px,pz: Integer;
                 qx,qz: Integer): Single;
var pqx,pqz,dx,dz,d,t: Single;
begin
  pqx := (qx - px);
  pqz := (qz - pz);
  dx := (x - px);
  dz := (z - pz);
  d := pqx*pqx + pqz*pqz;
  t := pqx*dx + pqz*dz;
  if (d > 0) then
    t := t/d;
  if (t < 0) then
    t := 0
  else if (t > 1) then
    t := 1;

  dx := px + t*pqx - x;
  dz := pz + t*pqz - z;

  Result := dx*dx + dz*dz;
end;

procedure simplifyContour(points, simplified: PrcIntArray;
              const maxError: Single; const maxEdgeLen: Integer; buildFlags: Integer);
var hasConnections: Boolean; i,ni,ii: Integer; differentRegs,areaBorders: Boolean; llx,lly,llz,lli,urx,ury,urz,uri,x,y,z: Integer;
pn: Integer; ax,az,ai,bx,bz,bi: Integer; maxd: Single; maxi,ci,cinc,endi: Integer; d: Single; n,j: Integer; tess: Boolean;
dx,dz: Integer;
begin
  // Add initial points.
  hasConnections := false;
  for i := 0 to points.size div 4 - 1 do
  begin
    if ((points^[i*4+3] and RC_CONTOUR_REG_MASK) <> 0) then
    begin
      hasConnections := true;
      break;
    end;
  end;

  if (hasConnections) then
  begin
    // The contour has some portals to other regions.
    // Add a new point to every location where the region changes.
    ni := points.size div 4;
    for i := 0 to ni - 1 do
    begin
      ii := (i+1) mod ni;
      differentRegs := (points^[i*4+3] and RC_CONTOUR_REG_MASK) <> (points^[ii*4+3] and RC_CONTOUR_REG_MASK);
      areaBorders := (points^[i*4+3] and RC_AREA_BORDER) <> (points^[ii*4+3] and RC_AREA_BORDER);
      if (differentRegs or areaBorders) then
      begin
        simplified.push(points^[i*4+0]);
        simplified.push(points^[i*4+1]);
        simplified.push(points^[i*4+2]);
        simplified.push(i);
      end;
    end;
  end;

  if (simplified.size = 0) then
  begin
    // If there is no connections at all,
    // create some initial points for the simplification process.
    // Find lower-left and upper-right vertices of the contour.
    llx := points^[0];
    lly := points^[1];
    llz := points^[2];
    lli := 0;
    urx := points^[0];
    ury := points^[1];
    urz := points^[2];
    uri := 0;
    for i := 0 to points.size div 4 - 1 do
    begin
      x := points^[i*4+0];
      y := points^[i*4+1];
      z := points^[i*4+2];
      if (x < llx) or ((x = llx) and (z < llz)) then
      begin
        llx := x;
        lly := y;
        llz := z;
        lli := i;
      end;
      if (x > urx) or ((x = urx) and (z > urz)) then
      begin
        urx := x;
        ury := y;
        urz := z;
        uri := i;
      end;
    end;
    simplified.push(llx);
    simplified.push(lly);
    simplified.push(llz);
    simplified.push(lli);
    
    simplified.push(urx);
    simplified.push(ury);
    simplified.push(urz);
    simplified.push(uri);
  end;

  // Add points until all raw points are within
  // error tolerance to the simplified shape.
  pn := points.size div 4;
  i := 0;
  while(i < simplified.size div 4) do
  begin
    ii := (i+1) mod (simplified.size div 4);

    ax := simplified^[i*4+0];
    az := simplified^[i*4+2];
    ai := simplified^[i*4+3];

    bx := simplified^[ii*4+0];
    bz := simplified^[ii*4+2];
    bi := simplified^[ii*4+3];

    // Find maximum deviation from the segment.
    maxd := 0;
    maxi := -1;

    // Traverse the segment in lexilogical order so that the
    // max deviation is calculated similarly when traversing
    // opposite segments.
    if (bx > ax) or ((bx = ax) and (bz > az)) then
    begin
      cinc := 1;
      ci := (ai+cinc) mod pn;
      endi := bi;
    end
    else
    begin
      cinc := pn-1;
      ci := (bi+cinc) mod pn;
      endi := ai;
      rcSwap(ax, bx);
      rcSwap(az, bz);
    end;

    // Tessellate only outer edges or edges between areas.
    if ((points^[ci*4+3] and RC_CONTOUR_REG_MASK) = 0) or
      (points^[ci*4+3] and RC_AREA_BORDER <> 0) then
    begin
      while (ci <> endi) do
      begin
        d := distancePtSeg(points^[ci*4+0], points^[ci*4+2], ax, az, bx, bz);
        if (d > maxd) then
        begin
          maxd := d;
          maxi := ci;
        end;
        ci := (ci+cinc) mod pn;
      end;
    end;


    // If the max deviation is larger than accepted error,
    // add new point, else continue to next segment.
    if (maxi <> -1) and (maxd > (maxError*maxError)) then
    begin
      // Add space for the new point.
      simplified.resize(simplified.size+4);
      n := simplified.size div 4;
      for j := n-1 downto i + 1 do
      begin
        simplified^[j*4+0] := simplified^[(j-1)*4+0];
        simplified^[j*4+1] := simplified^[(j-1)*4+1];
        simplified^[j*4+2] := simplified^[(j-1)*4+2];
        simplified^[j*4+3] := simplified^[(j-1)*4+3];
      end;
      // Add the point.
      simplified^[(i+1)*4+0] := points^[maxi*4+0];
      simplified^[(i+1)*4+1] := points^[maxi*4+1];
      simplified^[(i+1)*4+2] := points^[maxi*4+2];
      simplified^[(i+1)*4+3] := maxi;
    end
    else
    begin
      Inc(i);
    end;
  end;

  // Split too long edges.
  if (maxEdgeLen > 0) and ((buildFlags and (RC_CONTOUR_TESS_WALL_EDGES or RC_CONTOUR_TESS_AREA_EDGES)) <> 0) then
  begin
    i := 0;
    while (i < simplified.size div 4) do
    begin
      ii := (i+1) mod (simplified.size div 4);

      ax := simplified^[i*4+0];
      az := simplified^[i*4+2];
      ai := simplified^[i*4+3];

      bx := simplified^[ii*4+0];
      bz := simplified^[ii*4+2];
      bi := simplified^[ii*4+3];

      // Find maximum deviation from the segment.
      maxi := -1;
      ci := (ai+1) mod pn;

      // Tessellate only outer edges or edges between areas.
      tess := false;
      // Wall edges.
      if ((buildFlags and RC_CONTOUR_TESS_WALL_EDGES <> 0) and (points^[ci*4+3] and RC_CONTOUR_REG_MASK = 0)) then
        tess := true;
      // Edges between areas.
      if ((buildFlags and RC_CONTOUR_TESS_AREA_EDGES <> 0) and (points^[ci*4+3] and RC_AREA_BORDER <> 0)) then
        tess := true;

      if (tess) then
      begin
        dx := bx - ax;
        dz := bz - az;
        if (dx*dx + dz*dz > maxEdgeLen*maxEdgeLen) then
        begin
          // Round based on the segments in lexilogical order so that the
          // max tesselation is consistent regardles in which direction
          // segments are traversed.
          n := IfThen(bi < ai, (bi+pn - ai), (bi - ai));
          if (n > 1) then
          begin
            if (bx > ax) or ((bx = ax) and (bz > az)) then
              maxi := (ai + n div 2) mod pn
            else
              maxi := (ai + (n+1) div 2) mod pn;
          end;
        end;
      end;

      // If the max deviation is larger than accepted error,
      // add new point, else continue to next segment.
      if (maxi <> -1) then
      begin
        // Add space for the new point.
        simplified.resize(simplified.size + 4);
        n := simplified.size div 4;
        for j := n-1 downto i + 1 do
        begin
          simplified^[j*4+0] := simplified^[(j-1)*4+0];
          simplified^[j*4+1] := simplified^[(j-1)*4+1];
          simplified^[j*4+2] := simplified^[(j-1)*4+2];
          simplified^[j*4+3] := simplified^[(j-1)*4+3];
        end;
        // Add the point.
        simplified^[(i+1)*4+0] := points^[maxi*4+0];
        simplified^[(i+1)*4+1] := points^[maxi*4+1];
        simplified^[(i+1)*4+2] := points^[maxi*4+2];
        simplified^[(i+1)*4+3] := maxi;
      end
      else
      begin
        Inc(i);
      end;
    end;
  end;

  for i := 0 to simplified.size div 4 - 1 do
  begin
    // The edge vertex flag is take from the current raw point,
    // and the neighbour region is take from the next raw point.
    ai := (simplified^[i*4+3]+1) mod pn;
    bi := simplified^[i*4+3];
    simplified^[i*4+3] := (points^[ai*4+3] and (RC_CONTOUR_REG_MASK or RC_AREA_BORDER)) or (points^[bi*4+3] and RC_BORDER_VERTEX);
  end;

end;

function calcAreaOfPolygon2D(const verts: PInteger; const nverts: Integer): Integer;
var area: Integer; i,j: Integer; vi,vj: PInteger;
begin
  area := 0;
  i := 0;
  j := nverts-1;
  while (i < nverts) do
  begin
    vi := @verts[i * 4];
    vj := @verts[j * 4];
    Inc(area, vi[0] * vj[2] - vj[0] * vi[2]);

    j := i;
    Inc(i);
  end;
  Result := (area+1) div 2;
end;

// TODO: these are the same as in RecastMesh.cpp, consider using the same.

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
    Result := ((a[0] <= c[0]) and (c[0] <= b[0])) or ((a[0] >= c[0]) and (c[0] >= b[0]))
  else
    Result := ((a[2] <= c[2]) and (c[2] <= b[2])) or ((a[2] >= c[2]) and (c[2] >= b[2]));
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

function intersectSegCountour(const d0, d1: PInteger; i, n: Integer; const verts: PInteger): Boolean;
var k,k1: Integer; p0,p1: PInteger;
begin
  // For each edge (k,k+1) of P
  for k := 0 to n - 1 do
  begin
    k1 := next(k, n);
    // Skip edges incident to i.
    if (i = k) or (i = k1) then
      Continue;
    p0 := @verts[k * 4];
    p1 := @verts[k1 * 4];
    if (vequal(d0, p0) or vequal(d1, p0) or vequal(d0, p1) or vequal(d1, p1)) then
      Continue;

    if (intersect(d0, d1, p0, p1)) then
      Exit(true);
  end;
  Result := false;
end;

function inCone(i, n: Integer; const verts: PInteger; const pj: PInteger): Boolean;
var pi,pi1,pin1: PInteger;
begin
  pi := @verts[i * 4];
  pi1 := @verts[next(i, n) * 4];
  pin1 := @verts[prev(i, n) * 4];

  // If P[i] is a convex vertex [ i+1 left or on (i-1,i) ].
  if (leftOn(pin1, pi, pi1)) then
    Exit(left(pi, pj, pin1) and left(pj, pi, pi1));
  // Assume (i-1,i,i+1) not collinear.
  // else P[i] is reflex.
  Result := not (leftOn(pi, pj, pi1) and leftOn(pj, pi, pin1));
end;


procedure removeDegenerateSegments(simplified: PrcIntArray);
var npts,i,ni,j: Integer; a,b: Integer;
begin
  // Remove adjacent vertices which are equal on xz-plane,
  // or else the triangulator will get confused.
  npts := simplified.size div 4;
  for i := 0 to npts - 1 do
  begin
    ni := next(i, npts);

    a := simplified^[i*4]; b := simplified^[ni*4]; //Needs special treatment to get PInteger
    if (vequal(@a, @b)) then
    begin
      // Degenerate segment, remove.
      for j := i to simplified.size div 4-1 - 1 do
      begin
        simplified^[j*4+0] := simplified^[(j+1)*4+0];
        simplified^[j*4+1] := simplified^[(j+1)*4+1];
        simplified^[j*4+2] := simplified^[(j+1)*4+2];
        simplified^[j*4+3] := simplified^[(j+1)*4+3];
      end;
      simplified.resize(simplified.size-4);
      Dec(npts);
    end;
  end;
end;


function mergeContours(ca, cb: PrcContour; ia, ib: Integer): Boolean;
var maxVerts: Integer; verts: PInteger; nv,i: Integer; dst,src: PInteger;
begin
  maxVerts := ca.nverts + cb.nverts + 2;
  GetMem(verts, SizeOf(Integer)*maxVerts*4);

  nv := 0;

  // Copy contour A.
  for i := 0 to ca.nverts do
  begin
    dst := @verts[nv*4];
    src := @ca.verts[((ia+i) mod ca.nverts)*4];
    dst[0] := src[0];
    dst[1] := src[1];
    dst[2] := src[2];
    dst[3] := src[3];
    Inc(nv);
  end;

  // Copy contour B
  for i := 0 to cb.nverts do
  begin
    dst := @verts[nv*4];
    src := @cb.verts[((ib+i) mod cb.nverts)*4];
    dst[0] := src[0];
    dst[1] := src[1];
    dst[2] := src[2];
    dst[3] := src[3];
    Inc(nv);
  end;

  FreeMem(ca.verts);
  ca.verts := verts;
  ca.nverts := nv;

  FreeMem(cb.verts);
  cb.verts := nil;
  cb.nverts := 0;

  Result := True;
end;

// Finds the lowest leftmost vertex of a contour.
procedure findLeftMostVertex(const contour: PrcContour; minx, minz, leftmost: PInteger);
var i,x,z: Integer;
begin
  minx^ := contour.verts[0];
  minz^ := contour.verts[2];
  leftmost^ := 0;
  for i := 1 to contour.nverts - 1 do
  begin
    x := contour.verts[i*4+0];
    z := contour.verts[i*4+2];
    if (x < minx^) or ((x = minx^) and (z < minz^)) then
    begin
      minx^ := x;
      minz^ := z;
      leftmost^ := i;
    end;
  end;
end;

function compareHoles(const va,vb: Pointer): Integer;
var a,b: PrcContourHole;
begin
  Result := 0;

  a := PrcContourHole(va);
  b := PrcContourHole(vb);
  if (a.minx = b.minx) then
  begin
    if (a.minz < b.minz) then
      Result := -1
    else
    if (a.minz > b.minz) then
      Result := 1;
  end
  else
  begin
    if (a.minx < b.minx) then
      Result := -1
    else
    if (a.minx > b.minx) then
      Result := 1;
  end;
end;


function compareDiagDist(const va,vb: Pointer): Integer;
var a,b: PrcPotentialDiagonal;
begin
  a := PrcPotentialDiagonal(va);
  b := PrcPotentialDiagonal(vb);
  if (a.dist < b.dist) then
    Result := -1
  else
  if (a.dist > b.dist) then
    Result := 1
  else
  Result := 0;
end;


procedure mergeRegionHoles(ctx: TrcContext; region: PrcContourRegion);
var i,j,k,maxVerts,index,bestVertex,iter: Integer; diags: PrcPotentialDiagonal; outline: PrcContour; hole: PrcContour;
ndiags: Integer; corner,pt: PInteger; intersect: Boolean; dx,dz: Integer;
begin
  // Sort holes from left to right.
  for i := 0 to region.nholes - 1 do
    findLeftMostVertex(region.holes[i].contour, @region.holes[i].minx, @region.holes[i].minz, @region.holes[i].leftmost);

  qsort(region.holes, region.nholes, sizeof(TrcContourHole), compareHoles);

  maxVerts := region.outline.nverts;
  for i := 0 to region.nholes - 1 do
    Inc(maxVerts, region.holes[i].contour.nverts);

  GetMem(diags, sizeof(TrcPotentialDiagonal)*maxVerts);

  outline := region.outline;

  // Merge holes into the outline one by one.
  for i := 0 to region.nholes - 1 do
  begin
    hole := region.holes[i].contour;

    index := -1;
    bestVertex := region.holes[i].leftmost;
    for iter := 0 to hole.nverts - 1 do
    begin
      // Find potential diagonals.
      // The 'best' vertex must be in the cone described by 3 cosequtive vertices of the outline.
      // ..o j-1
      //   |
      //   |   * best
      //   |
      // j o-----o j+1
      //         :
      ndiags := 0;
      corner := @hole.verts[bestVertex*4];
      for j := 0 to outline.nverts - 1 do
      begin
        if (inCone(j, outline.nverts, outline.verts, corner)) then
        begin
          dx := outline.verts[j*4+0] - corner[0];
          dz := outline.verts[j*4+2] - corner[2];
          diags[ndiags].vert := j;
          diags[ndiags].dist := dx*dx + dz*dz;
          Inc(ndiags);
        end;
      end;
      // Sort potential diagonals by distance, we want to make the connection as short as possible.
      qsort(diags, ndiags, sizeof(TrcPotentialDiagonal), compareDiagDist);

      // Find a diagonal that is not intersecting the outline not the remaining holes.
      index := -1;
      for j := 0 to ndiags - 1 do
      begin
        pt := @outline.verts[diags[j].vert*4];
        intersect := intersectSegCountour(pt, corner, diags[i].vert, outline.nverts, outline.verts);
        k := i;
        while ((k < region.nholes) and not intersect) do
        begin
          intersect := intersect or intersectSegCountour(pt, corner, -1, region.holes[k].contour.nverts, region.holes[k].contour.verts);
          Inc(k);
        end;
        if (not intersect) then
        begin
          index := diags[j].vert;
          break;
        end;
      end;
      // If found non-intersecting diagonal, stop looking.
      if (index <> -1) then
        break;
      // All the potential diagonals for the current vertex were intersecting, try next vertex.
      bestVertex := (bestVertex + 1) mod hole.nverts;
    end;

    if (index = -1) then
    begin
      ctx.log(RC_LOG_WARNING, Format('mergeHoles: Failed to find merge points for %p and %p.', [region.outline, hole]));
      Continue;
    end;
    if (not mergeContours(region.outline, hole, index, bestVertex)) then
    begin
      ctx.log(RC_LOG_WARNING, Format('mergeHoles: Failed to merge contours %p and %p.', [region.outline, hole]));
      Continue;
    end;
  end;

  FreeMem(diags);
end;


/// @par
///
/// The raw contours will match the region outlines exactly. The @p maxError and @p maxEdgeLen
/// parameters control how closely the simplified contours will match the raw contours.
///
/// Simplified contours are generated such that the vertices for portals between areas match up.
/// (They are considered mandatory vertices.)
///
/// Setting @p maxEdgeLength to zero will disabled the edge length feature.
///
/// See the #rcConfig documentation for more information on the configuration parameters.
///
/// @see rcAllocContourSet, rcCompactHeightfield, rcContourSet, rcConfig
function rcBuildContours(ctx: TrcContext; chf: PrcCompactHeightfield;
           const maxError: Single; const maxEdgeLen: Integer;
           cset: PrcContourSet; const buildFlags: Integer = RC_CONTOUR_TESS_WALL_EDGES): Boolean;
var w,h,borderSize: Integer; pad: Single; maxContours: Integer; flags: PByte; y,x: Integer; c: PrcCompactCell; s: PrcCompactSpan;
i,j,dir: Integer; res,area: Byte; r,reg: Word; ax,ay,ai: Integer; verts,simplified: TrcIntArray; cont: PrcContour; oldMax: Integer;
newConts: array of TrcContour; v: PInteger; winding: PShortInt; nholes,nregions: Integer;
regions: PrcContourRegion; holes: PrcContourHole; index: Integer; reg1: PrcContourRegion;
begin
  //rcAssert(ctx);

  w := chf.width;
  h := chf.height;
  borderSize := chf.borderSize;

  ctx.startTimer(RC_TIMER_BUILD_CONTOURS);

  rcVcopy(@cset.bmin, @chf.bmin);
  rcVcopy(@cset.bmax, @chf.bmax);
  if (borderSize > 0) then
  begin
    // If the heightfield was build with bordersize, remove the offset.
    pad := borderSize*chf.cs;
    cset.bmin[0] := cset.bmin[0] + pad;
    cset.bmin[2] := cset.bmin[2] + pad;
    cset.bmax[0] := cset.bmax[0] - pad;
    cset.bmax[2] := cset.bmax[2] - pad;
  end;
  cset.cs := chf.cs;
  cset.ch := chf.ch;
  cset.width := chf.width - chf.borderSize*2;
  cset.height := chf.height - chf.borderSize*2;
  cset.borderSize := chf.borderSize;

  maxContours := rcMax(chf.maxRegions, 8);
  SetLength(cset.conts, maxContours);
  cset.nconts := 0;

  GetMem(flags, sizeof(Byte)*chf.spanCount);

  ctx.startTimer(RC_TIMER_BUILD_CONTOURS_TRACE);

  // Mark boundaries.
  for y := 0 to h - 1 do
  begin
    for x := 0 to w - 1 do
    begin
      c := @chf.cells[x+y*w];

      for i := c.index to Integer(c.index+c.count) - 1 do
      begin
        res := 0;
        s := @chf.spans[i];
        if (chf.spans[i].reg = 0) or (chf.spans[i].reg and RC_BORDER_REG <> 0) then
        begin
          flags[i] := 0;
          continue;
        end;
        for dir := 0 to 3 do
        begin
          r := 0;
          if (rcGetCon(s, dir) <> RC_NOT_CONNECTED) then
          begin
            ax := x + rcGetDirOffsetX(dir);
            ay := y + rcGetDirOffsetY(dir);
            ai := chf.cells[ax+ay*w].index + rcGetCon(s, dir);
            r := chf.spans[ai].reg;
          end;
          if (r = chf.spans[i].reg) then
            res := res or (1 shl dir);
        end;
        flags[i] := res xor $f; // Inverse, mark non connected edges.
      end;
    end;
  end;

  ctx.stopTimer(RC_TIMER_BUILD_CONTOURS_TRACE);

  verts.Create(256);
  simplified.Create(64);

  for y := 0 to h - 1 do
  begin
    for x := 0 to w - 1 do
    begin
      c := @chf.cells[x+y*w];
      for i := c.index to Integer(c.index+c.count) - 1 do
      begin
        if (flags[i] = 0) or (flags[i] = $f) then
        begin
          flags[i] := 0;
          continue;
        end;
        reg := chf.spans[i].reg;
        if (reg = 0) or (reg and RC_BORDER_REG <> 0) then
          continue;
        area := chf.areas[i];

        verts.resize(0);
        simplified.resize(0);

        ctx.startTimer(RC_TIMER_BUILD_CONTOURS_TRACE);
        walkContour(x, y, i, chf, flags, @verts);
        ctx.stopTimer(RC_TIMER_BUILD_CONTOURS_TRACE);

        ctx.startTimer(RC_TIMER_BUILD_CONTOURS_SIMPLIFY);
        simplifyContour(@verts, @simplified, maxError, maxEdgeLen, buildFlags);
        removeDegenerateSegments(@simplified);
        ctx.stopTimer(RC_TIMER_BUILD_CONTOURS_SIMPLIFY);


        // Store region.contour remap info.
        // Create contour.
        if simplified.size div 4 >= 3 then
        begin
          if (cset.nconts >= maxContours) then
          begin
            // Allocate more contours.
            // This happens when a region has holes.
            oldMax := maxContours;
            maxContours := maxContours * 2;
            SetLength(cset.conts, maxContours);
            //for (int j = 0; j < cset.nconts; ++j)
            //{
            //  newConts[j] = cset.conts[j];
            //  // Reset source pointers to prevent data deletion.
            //  cset.conts[j].verts = 0;
            //  cset.conts[j].rverts = 0;
            //}
            //rcFree(cset.conts);
            //cset.conts = newConts;

            ctx.log(RC_LOG_WARNING, Format('rcBuildContours: Expanding max contours from %d to %d.', [oldMax, maxContours]));
          end;

          cont := @cset.conts[cset.nconts];
          Inc(cset.nconts);

          cont.nverts := simplified.size div 4;
          GetMem(cont.verts, SizeOf(Integer)*cont.nverts*4);
          Move(simplified.m_data^, cont.verts^, sizeof(integer)*cont.nverts*4);
          if (borderSize > 0) then
          begin
            // If the heightfield was build with bordersize, remove the offset.
            for j := 0 to cont.nverts - 1 do
            begin
              v := @cont.verts[j*4];
              v[0] := v[0] - borderSize;
              v[2] := v[2] - borderSize;
            end;
          end;

          cont.nrverts := verts.size div 4;
          GetMem(cont.rverts, SizeOf(Integer)*cont.nrverts*4);
          Move(verts.m_data^, cont.rverts^, sizeof(integer)*cont.nrverts*4);

          if (borderSize > 0) then
          begin
            // If the heightfield was build with bordersize, remove the offset.
            for j := 0 to cont.nrverts - 1 do
            begin
              v := @cont.rverts[j*4];
              v[0] := v[0] - borderSize;
              v[2] := v[2] - borderSize;
            end;
          end;

          cont.reg := reg;
          cont.area := area;
        end;
      end;
    end;
  end;

  // Delphi: Manually release record and buffer it holds within
  verts.Free;
  simplified.Free;

  // Merge holes if needed.
  if (cset.nconts > 0) then
  begin
    // Calculate winding of all polygons.
    GetMem(winding, SizeOf(ShortInt)*cset.nconts);
    nholes := 0;
    for i := 0 to cset.nconts - 1 do
    begin
      cont := @cset.conts[i];
      // If the contour is wound backwards, it is a hole.
      winding[i] := IfThen(calcAreaOfPolygon2D(cont.verts, cont.nverts) < 0, -1, 1);
      if (winding[i] < 0) then
        Inc(nholes);
    end;

    if (nholes > 0) then
    begin
      // Collect outline contour and holes contours per region.
      // We assume that there is one outline and multiple holes.
      nregions := chf.maxRegions+1;
      GetMem(regions, sizeof(TrcContourRegion)*nregions);
      FillChar(regions[0], sizeof(TrcContourRegion)*nregions, 0);

      GetMem(holes, sizeof(TrcContourHole)*cset.nconts);
      FillChar(holes[0], sizeof(TrcContourHole)*cset.nconts, 0);

      for i := 0 to cset.nconts - 1 do
      begin
        cont := @cset.conts[i];
        // Positively would contours are outlines, negative holes.
        if (winding[i] > 0) then
        begin
          if (regions[cont.reg].outline <> nil) then
            ctx.log(RC_LOG_ERROR, Format('rcBuildContours: Multiple outlines for region %d.', [cont.reg]));
          regions[cont.reg].outline := cont;
        end
        else
        begin
          Inc(regions[cont.reg].nholes);
        end;
      end;
      index := 0;
      for i := 0 to nregions - 1 do
      begin
        if (regions[i].nholes > 0) then
        begin
          regions[i].holes := @holes[index];
          Inc(index, regions[i].nholes);
          regions[i].nholes := 0;
        end;
      end;
      for i := 0 to cset.nconts - 1 do
      begin
        cont := @cset.conts[i];
        reg1 := @regions[cont.reg];
        if (winding[i] < 0) then
        begin
          reg1.holes[reg1.nholes].contour := cont;
          Inc(reg1.nholes);
        end;
      end;

      // Finally merge each regions holes into the outline.
      for i := 0 to nregions - 1 do
      begin
        reg1 := @regions[i];
        if (reg1.nholes = 0) then continue;

        if (reg1.outline <> nil) then
        begin
          mergeRegionHoles(ctx, reg1);
        end
        else
        begin
          // The region does not have an outline.
          // This can happen if the contour becaomes selfoverlapping because of
          // too aggressive simplification settings.
          ctx.log(RC_LOG_ERROR, Format('rcBuildContours: Bad outline for region %d, contour simplification is likely too aggressive.', [i]));
        end;
      end;

      FreeMem(regions);
      FreeMem(holes);
    end;

    FreeMem(winding);
  end;

  FreeMem(flags);

  ctx.stopTimer(RC_TIMER_BUILD_CONTOURS);

  Result := true;
end;

end.
