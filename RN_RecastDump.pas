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

unit RN_RecastDump;
interface
uses Classes, Math, SysUtils, RN_Recast;


function duDumpPolyMeshToObj(pmesh: PrcPolyMesh; aFilename: string): boolean;
function duDumpPolyMeshDetailToObj(dmesh: PrcPolyMeshDetail; aFilename: string): Boolean;
function duDumpContourSet(cset: PrcContourSet; aFilename: string): Boolean;
//function duReadContourSet(xset: PrcContourSet; aFilename: string): Boolean;
function duDumpHeightfield(hf: PrcHeightfield; aFilename: string): Boolean;
function duDumpCompactHeightfield(chf: PrcCompactHeightfield; aFilename: string): Boolean;
//function duReadCompactHeightfield(chf: PrcCompactHeightfield; aFilename: string): Boolean;
procedure duLogBuildTimes(ctx: TrcContext; totalTimeUsec: Integer);


implementation

{duFileIO::~duFileIO()
begin
  // Empty
end;

static void ioprintf(duFileIO* io, const char* format, ...)
begin
  char line[256];
  va_list ap;
  va_start(ap, format);
  const int n = vsnprintf(line, sizeof(line), format, ap);
  va_end(ap);
  if (n > 0)
    io->write(line, sizeof(char)*n);
end;}

function duDumpPolyMeshToObj(pmesh: PrcPolyMesh; aFilename: string): boolean;
var io: TStringList;
nvp: Integer; cs,ch: Single; orig: PSingle; i,j: Integer; x,y,z: Single; v,p: PWord;
begin
  io := TStringList.Create;

  nvp := pmesh.nvp;
  cs := pmesh.cs;
  ch := pmesh.ch;
  orig := @pmesh.bmin;

  io.Append('# Recast Navmesh');
  io.Append('o NavMesh');

  io.Append('');

  for i := 0 to pmesh.nverts - 1 do
  begin
    v := @pmesh.verts[i*3];
    x := orig[0] + v[0]*cs;
    y := orig[1] + (v[1]+1)*ch + 0.1;
    z := orig[2] + v[2]*cs;
    io.Append(Format('v %.6f %.6f %.6f', [x,y,z]));
  end;

  io.Append('');

  for i := 0 to pmesh.npolys - 1 do
  begin
    p := @pmesh.polys[i*nvp*2];
    for j := 2 to nvp - 1 do
    begin
      if (p[j] = RC_MESH_NULL_IDX) then break;
      io.Append(Format('f %d %d %d', [p[0]+1, p[j-1]+1, p[j]+1]));
    end;
    //io.Append('');
  end;

  io.SaveToFile(aFilename);
  io.Free;

  Result := true;
end;

function duDumpPolyMeshDetailToObj(dmesh: PrcPolyMeshDetail; aFilename: string): Boolean;
var io: TStringList;
i,j: Integer; v: PSingle; m: PInteger; bverts,btris,ntris: Cardinal; tris: PByte;
begin
  io := TStringList.Create;

  io.Append('# Recast Navmesh');
  io.Append('o NavMesh');

  io.Append('');

  for i := 0 to dmesh.nverts - 1 do
  begin
    v := @dmesh.verts[i*3];
    io.Append(Format('v %.6f %.6f %.6f', [v[0],v[1],v[2]]));
  end;

  io.Append('');

  for i := 0 to dmesh.nmeshes - 1 do
  begin
    m := @dmesh.meshes[i*4];
    bverts := m[0];
    btris := m[2];
    ntris := m[3];
    tris := @dmesh.tris[btris*4];
    for j := 0 to ntris - 1 do
    begin
      io.Append(Format('f %d %d %d',
          [(bverts+tris[j*4+0])+1,
          (bverts+tris[j*4+1])+1,
          (bverts+tris[j*4+2])+1]));
    end;
    //io.Append('');
  end;

  io.SaveToFile(aFilename);
  io.Free;

  Result := true;
end;

const CSET_MAGIC: Cardinal = Ord('c') shl 24 or Ord('s') shl 16 or Ord('e') shl 8 or Ord('t');
const CSET_VERSION: Cardinal = 2;

function duDumpContourSet(cset: PrcContourSet; aFilename: string): boolean;
var io: TMemoryStream; i: Integer; cont: PrcContour;
begin
  io := TMemoryStream.Create;

  io.write(CSET_MAGIC, sizeof(CSET_MAGIC));
  io.write(CSET_VERSION, sizeof(CSET_VERSION));

  io.write(cset.nconts, sizeof(cset.nconts));

  io.write(cset.bmin[0], sizeof(cset.bmin));
  io.write(cset.bmax[0], sizeof(cset.bmax));

  io.write(cset.cs, sizeof(cset.cs));
  io.write(cset.ch, sizeof(cset.ch));

  io.write(cset.width, sizeof(cset.width));
  io.write(cset.height, sizeof(cset.height));
  io.write(cset.borderSize, sizeof(cset.borderSize));

  for i := 0 to cset.nconts - 1 do
  begin
    cont := @cset.conts[i];
    io.write(cont.nverts, sizeof(cont.nverts));
    io.write(cont.nrverts, sizeof(cont.nrverts));
    io.write(cont.reg, sizeof(cont.reg));
    io.write(cont.area, sizeof(cont.area));
    io.write(cont.verts[0], sizeof(integer)*4*cont.nverts);
    io.write(cont.rverts[0], sizeof(integer)*4*cont.nrverts);
  end;

  io.SaveToFile(aFilename);
  io.Free;

  Result := true;
end;

{function duReadContourSet(xset: PrcContourSet; aFilename: string): Boolean;
begin
  if (!io)
  begin
    printf('duReadContourSet: input IO is null.\n');
    return false;
  end;
  if (!io.isReading())
  begin
    printf('duReadContourSet: input IO not reading.\n');
    return false;
  end;

  int magic = 0;
  int version = 0;

  io.read(&magic, sizeof(magic));
  io.read(&version, sizeof(version));

  if (magic != CSET_MAGIC)
  begin
    printf('duReadContourSet: Bad voodoo.\n');
    return false;
  end;
  if (version != CSET_VERSION)
  begin
    printf('duReadContourSet: Bad version.\n');
    return false;
  end;

  io.read(&cset.nconts, sizeof(cset.nconts));

  cset.conts = (rcContour*)rcAlloc(sizeof(rcContour)*cset.nconts, RC_ALLOC_PERM);
  if (!cset.conts)
  begin
    printf('duReadContourSet: Could not alloc contours (%d)\n', cset.nconts);
    return false;
  end;
  memset(cset.conts, 0, sizeof(rcContour)*cset.nconts);

  io.read(cset.bmin, sizeof(cset.bmin));
  io.read(cset.bmax, sizeof(cset.bmax));

  io.read(&cset.cs, sizeof(cset.cs));
  io.read(&cset.ch, sizeof(cset.ch));

  io.read(&cset.width, sizeof(cset.width));
  io.read(&cset.height, sizeof(cset.height));
  io.read(&cset.borderSize, sizeof(cset.borderSize));

  for (int i = 0; i < cset.nconts; ++i)
  begin
    rcContour& cont = cset.conts[i];
    io.read(&cont.nverts, sizeof(cont.nverts));
    io.read(&cont.nrverts, sizeof(cont.nrverts));
    io.read(&cont.reg, sizeof(cont.reg));
    io.read(&cont.area, sizeof(cont.area));

    cont.verts = (int*)rcAlloc(sizeof(int)*4*cont.nverts, RC_ALLOC_PERM);
    if (!cont.verts)
    begin
      printf('duReadContourSet: Could not alloc contour verts (%d)\n', cont.nverts);
      return false;
    end;
    cont.rverts = (int*)rcAlloc(sizeof(int)*4*cont.nrverts, RC_ALLOC_PERM);
    if (!cont.rverts)
    begin
      printf('duReadContourSet: Could not alloc contour rverts (%d)\n', cont.nrverts);
      return false;
    end;

    io.read(cont.verts, sizeof(int)*4*cont.nverts);
    io.read(cont.rverts, sizeof(int)*4*cont.nrverts);
  end;

  return true;
end;}


const CHF_MAGIC: Cardinal = Ord('r') shl 24 or Ord('c') shl 16 or Ord('h') shl 8 or Ord('f');
const CHF_VERSION: Cardinal = 3;

function duDumpHeightfield(hf: PrcHeightfield; aFilename: string): Boolean;
var io: TMemoryStream; tmp: Integer; x,y: Word; s: PrcSpan; t: Cardinal;
begin
  io := TMemoryStream.Create;

  io.write(hf.width, sizeof(hf.width));
  io.write(hf.height, sizeof(hf.height));

  io.write(hf.bmin[0], sizeof(hf.bmin));
  io.write(hf.bmax[0], sizeof(hf.bmax));

  io.write(hf.cs, sizeof(hf.cs));
  io.write(hf.ch, sizeof(hf.ch));

  tmp := 0;
  if Length(hf.spans) > 0 then tmp := tmp or 1;

  io.write(tmp, sizeof(tmp));

  for y := 0 to hf.height - 1 do
  begin
    for x := 0 to hf.width - 1 do
    begin
      s := hf.spans[x + y * hf.width];
      while s <> nil do
      begin
        // Write data without the pointer part
        io.write(x, SizeOf(x));
        io.write(y, SizeOf(y));
        t := (s.smin and $1fff) or (s.smax and $1fff) shl 13 or (s.area and $3f) shl 26;
        io.write(t, SizeOf(t));

        s := s.next;
      end;
    end;
  end;

  io.SaveToFile(aFilename);
  io.Free;

  Result := true;
end;

function duDumpCompactHeightfield(chf: PrcCompactHeightfield; aFilename: string): Boolean;
var io: TMemoryStream; tmp: Integer; i: Integer; t: Cardinal;
begin
  io := TMemoryStream.Create;

  io.write(CHF_MAGIC, sizeof(CHF_MAGIC));
  io.write(CHF_VERSION, sizeof(CHF_VERSION));

  io.write(chf.width, sizeof(chf.width));
  io.write(chf.height, sizeof(chf.height));
  io.write(chf.spanCount, sizeof(chf.spanCount));

  io.write(chf.walkableHeight, sizeof(chf.walkableHeight));
  io.write(chf.walkableClimb, sizeof(chf.walkableClimb));
  io.write(chf.borderSize, sizeof(chf.borderSize));

  io.write(chf.maxDistance, sizeof(chf.maxDistance));
  io.write(chf.maxRegions, sizeof(chf.maxRegions));

  io.write(chf.bmin[0], sizeof(chf.bmin));
  io.write(chf.bmax[0], sizeof(chf.bmax));

  io.write(chf.cs, sizeof(chf.cs));
  io.write(chf.ch, sizeof(chf.ch));

  tmp := 0;
  if Length(chf.cells) > 0 then tmp := tmp or 1;
  if Length(chf.spans) > 0 then tmp := tmp or 2;
  if (chf.dist <> nil) then tmp := tmp or 4;
  if (chf.areas <> nil) then tmp := tmp or 8;

  io.write(tmp, sizeof(tmp));

  for i := 0 to Length(chf.cells) - 1 do
  begin
    t := (chf.cells[i].index and $ffffff) or (chf.cells[i].count and $ff shl 24);
    io.write(t, SizeOf(t));
  end;
  for i := 0 to Length(chf.spans) - 1 do
  begin
    t := (chf.spans[i].y and $ffff) or (chf.spans[i].reg and $ffff shl 16);
    io.write(t, SizeOf(t));
    t := (chf.spans[i].con and $ffffff) or (chf.spans[i].h and $ff shl 24);
    io.write(t, SizeOf(t));
  end;
  if (chf.dist <> nil) then
    io.write(chf.dist[0], sizeof(Word)*chf.spanCount);
  if (chf.areas <> nil) then
    io.write(chf.areas[0], sizeof(Byte)*chf.spanCount);

  io.SaveToFile(aFilename);
  io.Free;

  Result := true;
end;

{function duReadCompactHeightfield(chf: PrcCompactHeightfield; aFilename: string): Boolean;
begin
  if (!io)
  begin
    printf('duReadCompactHeightfield: input IO is null.\n');
    return false;
  end;
  if (!io.isReading())
  begin
    printf('duReadCompactHeightfield: input IO not reading.\n');
    return false;
  end;

  int magic = 0;
  int version = 0;

  io.read(&magic, sizeof(magic));
  io.read(&version, sizeof(version));

  if (magic != CHF_MAGIC)
  begin
    printf('duReadCompactHeightfield: Bad voodoo.\n');
    return false;
  end;
  if (version != CHF_VERSION)
  begin
    printf('duReadCompactHeightfield: Bad version.\n');
    return false;
  end;

  io.read(&chf.width, sizeof(chf.width));
  io.read(&chf.height, sizeof(chf.height));
  io.read(&chf.spanCount, sizeof(chf.spanCount));

  io.read(&chf.walkableHeight, sizeof(chf.walkableHeight));
  io.read(&chf.walkableClimb, sizeof(chf.walkableClimb));
  io.write(&chf.borderSize, sizeof(chf.borderSize));

  io.read(&chf.maxDistance, sizeof(chf.maxDistance));
  io.read(&chf.maxRegions, sizeof(chf.maxRegions));

  io.read(chf.bmin, sizeof(chf.bmin));
  io.read(chf.bmax, sizeof(chf.bmax));

  io.read(&chf.cs, sizeof(chf.cs));
  io.read(&chf.ch, sizeof(chf.ch));

  int tmp = 0;
  io.read(&tmp, sizeof(tmp));

  if (tmp & 1)
  begin
    chf.cells = (rcCompactCell*)rcAlloc(sizeof(rcCompactCell)*chf.width*chf.height, RC_ALLOC_PERM);
    if (!chf.cells)
    begin
      printf('duReadCompactHeightfield: Could not alloc cells (%d)\n', chf.width*chf.height);
      return false;
    end;
    io.read(chf.cells, sizeof(rcCompactCell)*chf.width*chf.height);
  end;
  if (tmp & 2)
  begin
    chf.spans = (rcCompactSpan*)rcAlloc(sizeof(rcCompactSpan)*chf.spanCount, RC_ALLOC_PERM);
    if (!chf.spans)
    begin
      printf('duReadCompactHeightfield: Could not alloc spans (%d)\n', chf.spanCount);
      return false;
    end;
    io.read(chf.spans, sizeof(rcCompactSpan)*chf.spanCount);
  end;
  if (tmp & 4)
  begin
    chf.dist = (unsigned short*)rcAlloc(sizeof(unsigned short)*chf.spanCount, RC_ALLOC_PERM);
    if (!chf.dist)
    begin
      printf('duReadCompactHeightfield: Could not alloc dist (%d)\n', chf.spanCount);
      return false;
    end;
    io.read(chf.dist, sizeof(unsigned short)*chf.spanCount);
  end;
  if (tmp & 8)
  begin
    chf.areas = (unsigned char*)rcAlloc(sizeof(unsigned char)*chf.spanCount, RC_ALLOC_PERM);
    if (!chf.areas)
    begin
      printf('duReadCompactHeightfield: Could not alloc areas (%d)\n', chf.spanCount);
      return false;
    end;
    io.read(chf.areas, sizeof(unsigned char)*chf.spanCount);
  end;

  return true;
end;}


procedure logLine(ctx: TrcContext; &label: TrcTimerLabel; name: string; pc: Single);
var t: Integer;
begin
  t := ctx.getAccumulatedTime(&label);
  if (t < 0) then Exit;
  ctx.log(RC_LOG_PROGRESS, Format('%s: %.2fms'#9'(%.1f%%)', [name, t / 1000000.0, t * pc]));
end;

procedure duLogBuildTimes(ctx: TrcContext; totalTimeUsec: Integer);
var pc: Single;
begin
  pc := 100.0 / totalTimeUsec;

  ctx.log(RC_LOG_PROGRESS, 'Build Times');
  logLine(ctx, RC_TIMER_RASTERIZE_TRIANGLES,      '- Rasterize             ', pc);
  logLine(ctx, RC_TIMER_BUILD_COMPACTHEIGHTFIELD, '- Build Compact         ', pc);
  logLine(ctx, RC_TIMER_FILTER_BORDER,            '- Filter Border         ', pc);
  logLine(ctx, RC_TIMER_FILTER_WALKABLE,          '- Filter Walkable       ', pc);
  logLine(ctx, RC_TIMER_ERODE_AREA,               '- Erode Area            ', pc);
  logLine(ctx, RC_TIMER_MEDIAN_AREA,              '- Median Area           ', pc);
  logLine(ctx, RC_TIMER_MARK_BOX_AREA,            '- Mark Box Area         ', pc);
  logLine(ctx, RC_TIMER_MARK_CONVEXPOLY_AREA,     '- Mark Convex Area      ', pc);
  logLine(ctx, RC_TIMER_MARK_CYLINDER_AREA,       '- Mark Cylinder Area    ', pc);
  logLine(ctx, RC_TIMER_BUILD_DISTANCEFIELD,      '- Build Distance Field  ', pc);
  logLine(ctx, RC_TIMER_BUILD_DISTANCEFIELD_DIST, '  - Distance            ', pc);
  logLine(ctx, RC_TIMER_BUILD_DISTANCEFIELD_BLUR, '  - Blur                ', pc);
  logLine(ctx, RC_TIMER_BUILD_REGIONS,            '- Build Regions         ', pc);
  logLine(ctx, RC_TIMER_BUILD_REGIONS_WATERSHED,  '  - Watershed           ', pc);
  logLine(ctx, RC_TIMER_BUILD_REGIONS_EXPAND,     '    - Expand            ', pc);
  logLine(ctx, RC_TIMER_BUILD_REGIONS_FLOOD,      '    - Find Basins       ', pc);
  logLine(ctx, RC_TIMER_BUILD_REGIONS_FILTER,     '  - Filter              ', pc);
  logLine(ctx, RC_TIMER_BUILD_LAYERS,             '- Build Layers          ', pc);
  logLine(ctx, RC_TIMER_BUILD_CONTOURS,           '- Build Contours        ', pc);
  logLine(ctx, RC_TIMER_BUILD_CONTOURS_TRACE,     '  - Trace               ', pc);
  logLine(ctx, RC_TIMER_BUILD_CONTOURS_SIMPLIFY,  '  - Simplify            ', pc);
  logLine(ctx, RC_TIMER_BUILD_POLYMESH,           '- Build Polymesh        ', pc);
  logLine(ctx, RC_TIMER_BUILD_POLYMESHDETAIL,     '- Build Polymesh Detail ', pc);
  logLine(ctx, RC_TIMER_MERGE_POLYMESH,           '- Merge Polymeshes      ', pc);
  logLine(ctx, RC_TIMER_MERGE_POLYMESHDETAIL,     '- Merge Polymesh Details', pc);

  ctx.log(RC_LOG_PROGRESS, Format('=== TOTAL:                %.2fms', [totalTimeUsec / 1000000.0]));
end;

end.
