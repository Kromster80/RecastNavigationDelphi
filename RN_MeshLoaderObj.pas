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

unit RN_MeshLoaderObj;
interface
uses Classes, Math, SysUtils, StrUtils, Types, RN_Helper;

type
  TrcMeshLoaderObj = class
  private
    m_filename: string;
    m_scale: Single;
    m_verts: PSingle;
    m_tris: PInteger;
    m_normals: PSingle;
    m_vertCount: Integer;
    m_triCount: Integer;

    procedure addVertex(x,y,z: Single; cap: PInteger);
    procedure addTriangle(a,b,c: Integer; cap: PInteger);
  public
    constructor Create;
    destructor Destroy; override;

    function load(const fileName: string): Boolean;

    property getVerts: PSingle read m_verts;
    property getTris: PInteger read m_tris;
    property getNormals: PSingle read m_normals;
    property getVertCount: Integer read m_vertCount;
    property getTriCount: Integer read m_triCount;
    property getFileName: string read m_filename;
  end;

implementation
uses RN_RecastHelper;

function StringLow(const aString: string): Integer;
begin
  {$IFDEF FPC}
    Result := Low(aString);
  {$ELSE}
    {$IF CompilerVersion >= 24}
    Result := Low(aString); // Delphi XE3 and up can use Low(s)
    {$ELSE}
    Result := 1;            // Delphi XE2 and below can't use Low(s), but don't have ZEROBASEDSTRINGS either
    {$IFEND}
  {$ENDIF}
end;

function StringHigh(const aString: string): Integer;
begin
  {$IFDEF FPC}
    Result := Length(aString);
  {$ELSE}
    {$IF CompilerVersion >= 24}
    Result := High(aString);    // Delphi XE3 and up can use High(s)
    {$ELSE}
    Result := Length(aString);  // Delphi XE2 and below can't use High(s), but don't have ZEROBASEDSTRINGS either
    {$IFEND}
  {$ENDIF}
end;

constructor TrcMeshLoaderObj.Create;
begin
  inherited;

  m_scale := 1.0;
end;

destructor TrcMeshLoaderObj.Destroy;
begin
  FreeMem(m_verts);
  FreeMem(m_normals);
  FreeMem(m_tris);

  inherited;
end;

procedure TrcMeshLoaderObj.addVertex(x,y,z: Single; cap: PInteger);
var nv,dst: PSingle;
begin
  if (m_vertCount+1 > cap^) then
  begin
    if cap^ = 0 then cap^ := 8 else cap^ := cap^*2;
    GetMem(nv, SizeOf(Single)*cap^*3);
    if (m_vertCount <> 0) then
      Move(m_verts^, nv^, m_vertCount*3*sizeof(Single));
    FreeMem(m_verts);
    m_verts := nv;
  end;
  dst := @m_verts[m_vertCount*3];
  dst[0] := x*m_scale;
  dst[1] := y*m_scale;
  dst[2] := z*m_scale;
  Inc(m_vertCount);
end;

//void rcMeshLoaderObj::addTriangle(int a, int b, int c, int& cap)
procedure TrcMeshLoaderObj.addTriangle(a,b,c: Integer; cap: PInteger);
var nv,dst: PInteger;
begin
  if (m_triCount+1 > cap^) then
  begin
    if cap^ = 0 then cap^ := 8 else cap^ := cap^*2;
    GetMem(nv, SizeOf(Integer)*cap^*3);
    if (m_triCount <> 0) then
      Move(m_tris^, nv^, m_triCount*3*sizeof(integer));
    FreeMem(m_tris);
    m_tris := nv;
  end;
  dst := @m_tris[m_triCount*3];
  dst[0] := a;
  dst[1] := b;
  dst[2] := c;
  Inc(m_triCount);
end;


function parseFace(row: string; var data: array of Integer; n, vcnt: Integer): Integer;
var i,j,vi: Integer; items,elements: TStringDynArray;
begin
  j := 0;
  items := SplitString(row, ' ');
  for i := 1 to High(items) do
  if items[i] <> '' then
  begin
    elements := SplitString(items[i], '/');
    vi := StrToInt(elements[0]);
    if vi < 0 then data[j] := vi+vcnt else data[j] := vi-1;
    Inc(j);
  end;

  Result := j;

  if (j >= n) then Exit;
end;

function TrcMeshLoaderObj.load(const filename: string): Boolean;
var sl: TStringList; i,j: Integer; s: string; nv,vcap,tcap: Integer; x,y,z: Single; a,b,c: Integer; face: array [0..31] of Integer;
v0,v1,v2: PSingle; e0,e1: array [0..2] of Single; n: PSingle; d: Single;
  PrevDecimal: Char;
begin
  sl := TStringList.Create;
  sl.LoadFromFile(fileName);

  PrevDecimal := FormatSettings.DecimalSeparator;
  FormatSettings.DecimalSeparator := '.';

  vcap := 0;
  tcap := 0;

  for j := 0 to sl.Count - 1 do
  begin
    s := sl[j];

    if Length(s) = 0 then Continue;

    if s[StringLow(s)] = '#' then
      Continue
    else
    if (s[StringLow(s)] = 'v') and (s[StringLow(s)+1] <> 'n') and (s[StringLow(s)+1] <> 't')  then
    begin
      // Vertex pos
      Sscanf(RightStr(s, Length(s) - 2), '%f %f %f', [@x, @y, @z]);
      addVertex(x, y, z, @vcap);
    end else
    if s[StringLow(s)] = 'f' then
    begin
      // Faces
      nv := parseFace(s, face, 32, m_vertCount);
      for i := 2 to nv - 1 do
      begin
        a := face[0];
        b := face[i-1];
        c := face[i];
        if (a < 0) or (a >= m_vertCount) or (b < 0) or (b >= m_vertCount) or (c < 0) or (c >= m_vertCount) then
          Continue;
        addTriangle(a, b, c, @tcap);
      end;
    end;
  end;
  sl.Free;

  // Calculate normals.
  GetMem(m_normals, sizeof(Single)*m_triCount*3);

  for i := 0 to m_triCount - 1 do
  begin
    v0 := @m_verts[m_tris[i*3]*3];
    v1 := @m_verts[m_tris[i*3+1]*3];
    v2 := @m_verts[m_tris[i*3+2]*3];

    for j := 0 to 2 do
    begin
      e0[j] := v1[j] - v0[j];
      e1[j] := v2[j] - v0[j];
    end;
    n := @m_normals[i*3];
    n[0] := e0[1]*e1[2] - e0[2]*e1[1];
    n[1] := e0[2]*e1[0] - e0[0]*e1[2];
    n[2] := e0[0]*e1[1] - e0[1]*e1[0];
    d := sqrt(n[0]*n[0] + n[1]*n[1] + n[2]*n[2]);
    if (d > 0) then
    begin
      d := 1.0/d;
      n[0] := n[0] * d;
      n[1] := n[1] * d;
      n[2] := n[2] * d;
    end;
  end;

  m_filename := filename;

  FormatSettings.DecimalSeparator := PrevDecimal;
  Result := true;
end;

end.