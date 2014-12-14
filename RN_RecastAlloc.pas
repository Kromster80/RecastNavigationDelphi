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

unit RN_RecastAlloc;
interface
uses RN_Helper;

type
  TrcAllocHint =
  (
    RC_ALLOC_PERM,    ///< Memory will persist after a function call.
    RC_ALLOC_TEMP    ///< Memory used temporarily within a function.
  );


  PrcIntArray = ^TrcIntArray;
  TrcIntArray = record
  private
    m_size, m_cap: Integer;
    function GetItem(i: Integer): Integer;
    procedure SetItem(i: Integer; const Value: Integer);
  public
    m_data: PInteger;
    constructor Create(n: Integer);
    procedure Free; // Delphi: Records do not have automatic destructor, we need to dispose of allocated buffer manualy
    procedure Push(aValue: Integer);
    function Pop: Integer;
    procedure resize(n: Integer);
    property Item[i: Integer]: Integer read GetItem write SetItem; default; // Return Pointer because we cant edit values from getter
    property size: Integer read m_size;
  end;
  TArrayOfTrcIntArray = array of TrcIntArray;


implementation
{static void *rcAllocDefault(int size, rcAllocHint)
{
  return malloc(size);
}

{static void rcFreeDefault(void *ptr)
{
  free(ptr);
}

{static rcAllocFunc* sRecastAllocFunc = rcAllocDefault;
static rcFreeFunc* sRecastFreeFunc = rcFreeDefault;

/// @see rcAlloc, rcFree
void rcAllocSetCustom(rcAllocFunc *allocFunc, rcFreeFunc *freeFunc)
{
  sRecastAllocFunc = allocFunc ? allocFunc : rcAllocDefault;
  sRecastFreeFunc = freeFunc ? freeFunc : rcFreeDefault;
}

/// @see rcAllocSetCustom
{void* rcAlloc(int size, rcAllocHint hint)
{
  return sRecastAllocFunc(size, hint);
}

/// @par
///
/// @warning This function leaves the value of @p ptr unchanged.  So it still
/// points to the same (now invalid) location, and not to null.
///
/// @see rcAllocSetCustom
{void rcFree(void* ptr)
{
  if (ptr)
    sRecastFreeFunc(ptr);
}

constructor TrcIntArray.Create(n: Integer);
begin
  m_data := nil;
  m_size := 0;
  m_cap := 0;
  resize(n);
end;

procedure TrcIntArray.Free;
begin
  FreeMem(m_data);
end;

function TrcIntArray.Pop: Integer;
begin
  if (m_size > 0) then Dec(m_size); Result := m_data[m_size];
end;

procedure TrcIntArray.Push(aValue: Integer);
begin
  resize(m_size+1);

  m_data[m_size-1] := aValue;
end;

/// @class rcIntArray
///
/// While it is possible to pre-allocate a specific array size during
/// construction or by using the #resize method, certain methods will
/// automatically resize the array as needed.
///
/// @warning The array memory is not initialized to zero when the size is
/// manually set during construction or when using #resize.

/// @par
///
/// Using this method ensures the array is at least large enough to hold
/// the specified number of elements.  This can improve performance by
/// avoiding auto-resizing during use.
procedure TrcIntArray.resize(n: Integer);
var newData: PInteger;
begin
  if (n > m_cap) then
  begin
    if (m_cap = 0) then m_cap := n;
    while (m_cap < n) do m_cap := m_cap * 2;
    GetMem(newData, m_cap * sizeof(Integer));
    if (m_size <> 0) and (newData <> nil) then
      Move(m_data^, newData^, m_size*sizeof(integer));
    FreeMem(m_data);
    m_data := newData;
  end;

  m_size := n;
end;

function TrcIntArray.GetItem(i: Integer): Integer;
begin
  Result := m_data[i];
end;

procedure TrcIntArray.SetItem(i: Integer; const Value: Integer);
begin
  m_data[i] := Value;
end;


end.
