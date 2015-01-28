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
unit RN_DetourProximityGrid;
interface
uses Math, SysUtils;

type
  TdtProximityGrid = class
  private
    m_cellSize: Single;
    m_invCellSize: Single;

    type
    PItem = ^TItem;
    TItem = record
      id: Word;
      x,y: SmallInt;
      next: Word;
    end;
    var
    m_pool: PItem;
    m_poolHead: Integer;
    m_poolSize: Integer;

    m_buckets: PWord;
    m_bucketsSize: Integer;

    m_bounds: array [0..3] of Integer;
  public
    constructor Create;
    destructor Destroy; override;

    function init(const poolSize: Integer; const cellSize: Single): Boolean;

    procedure clear();

    procedure addItem(const id: Word; const minx, miny, maxx, maxy: Single);

    function queryItems(const minx, miny, maxx, maxy: Single;
             ids: PWord; const maxIds: Integer): Integer;

    function getItemCountAt(const x, y: Integer): Integer;

    function getBounds(): PInteger; { return m_bounds; }
    function getCellSize(): Single; { return m_cellSize; }
  end;

  function dtAllocProximityGrid(): TdtProximityGrid;
  procedure dtFreeProximityGrid(var ptr: TdtProximityGrid);


implementation
uses RN_DetourCommon;


function dtAllocProximityGrid(): TdtProximityGrid;
begin
	Result := TdtProximityGrid.Create;
end;

procedure dtFreeProximityGrid(var ptr: TdtProximityGrid);
begin
  FreeAndNil(ptr);
end;


function hashPos2(x, y, n: Integer): Integer;
begin
	Result := ((Int64(x)*73856093) xor (Int64(y)*19349663)) and (n-1);
end;


constructor TdtProximityGrid.Create;
begin
  inherited;
end;

destructor TdtProximityGrid.Destroy;
begin
	FreeMem(m_buckets);
	FreeMem(m_pool);
  inherited;
end;

function TdtProximityGrid.init(const poolSize: Integer; const cellSize: Single): Boolean;
begin
	Assert(poolSize > 0);
	Assert(cellSize > 0.0);

	m_cellSize := cellSize;
	m_invCellSize := 1.0 / m_cellSize;

	// Allocate hashs buckets
	m_bucketsSize := dtNextPow2(poolSize);
	GetMem(m_buckets, sizeof(Word)*m_bucketsSize);

	// Allocate pool of items.
	m_poolSize := poolSize;
	m_poolHead := 0;
	GetMem(m_pool, sizeof(TItem)*m_poolSize);

	clear();

	Result := true;
end;

procedure TdtProximityGrid.clear();
begin
	FillChar(m_buckets[0], sizeof(Word)*m_bucketsSize, $ff);
	m_poolHead := 0;
	m_bounds[0] := $ffff;
	m_bounds[1] := $ffff;
	m_bounds[2] := -$ffff;
	m_bounds[3] := -$ffff;
end;

procedure TdtProximityGrid.addItem(const id: Word; const minx, miny, maxx, maxy: Single);
var iminx, iminy, imaxx, imaxy, x, y, h: Integer; idx: Word; item: PItem;
begin
	iminx := Floor(minx * m_invCellSize);
	iminy := Floor(miny * m_invCellSize);
	imaxx := Floor(maxx * m_invCellSize);
	imaxy := Floor(maxy * m_invCellSize);

	m_bounds[0] := dtMin(m_bounds[0], iminx);
	m_bounds[1] := dtMin(m_bounds[1], iminy);
	m_bounds[2] := dtMax(m_bounds[2], imaxx);
	m_bounds[3] := dtMax(m_bounds[3], imaxy);

	for y := iminy to imaxy do
	begin
		for x := iminx to imaxx do
		begin
			if (m_poolHead < m_poolSize) then
			begin
				h := hashPos2(x, y, m_bucketsSize);
				idx := Word(m_poolHead);
				Inc(m_poolHead);
				item := @m_pool[idx];
				item.x := SmallInt(x);
				item.y := SmallInt(y);
				item.id := id;
				item.next := m_buckets[h];
				m_buckets[h] := idx;
			end;
		end;
	end;
end;

function TdtProximityGrid.queryItems(const minx, miny, maxx, maxy: Single;
             ids: PWord; const maxIds: Integer): Integer;
var iminx, iminy, imaxx, imaxy, n, x, y, h: Integer; idx: Word; item: PItem; &end, i: PWord;
begin
	iminx := Floor(minx * m_invCellSize);
	iminy := Floor(miny * m_invCellSize);
	imaxx := Floor(maxx * m_invCellSize);
	imaxy := Floor(maxy * m_invCellSize);

	n := 0;

	for y := iminy to imaxy do
	begin
		for x := iminx to imaxx do
		begin
			h := hashPos2(x, y, m_bucketsSize);
			idx := m_buckets[h];
			while (idx <> $ffff) do
			begin
				item := @m_pool[idx];
				if (item.x = x) and (item.y = y) then
				begin
					// Check if the id exists already.
					&end := ids + n;
					i := ids;
					while (i <> &end) and (i^ <> item.id) do
						Inc(i);
					// Item not found, add it.
					if (i = &end) then
					begin
						if (n >= maxIds) then
							Exit(n);
						ids[n] := item.id;
            Inc(n);
					end;
				end;
				idx := item.next;
			end;
		end;
	end;

	Result := n;
end;

function TdtProximityGrid.getItemCountAt(const x, y: Integer): Integer;
var n, h: Integer; idx: Word; item: PItem;
begin
	n := 0;
	
	h := hashPos2(x, y, m_bucketsSize);
	idx := m_buckets[h];
	while (idx <> $ffff) do
	begin
		item := @m_pool[idx];
		if (item.x = x) and (&item.y = y) then
			Inc(n);
		idx := item.next;
	end;
	
	Result := n;
end;

function TdtProximityGrid.getBounds(): PInteger; begin Result := @m_bounds[0]; end;
function TdtProximityGrid.getCellSize(): Single; begin Result := m_cellSize; end;

end.
