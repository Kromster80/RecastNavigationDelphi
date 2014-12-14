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

unit RN_DetourNavMeshHelper;
interface


// Undefine (or define in a build cofnig) the following line to use 64bit polyref.
// Generally not needed, useful for very large worlds.
// Note: tiles build using 32bit refs are not compatible with 64bit refs!
//#define DT_POLYREF64 1

// Note: If you want to use 64-bit refs, change the types of both dtPolyRef & dtTileRef.
// It is also recommended that you change dtHashRef() to a proper 64-bit hash.

/// A handle to a polygon within a navigation mesh tile.
/// @ingroup detour
  {$ifdef DT_POLYREF64}
  const
    DT_SALT_BITS = 16;
    DT_TILE_BITS = 28;
    DT_POLY_BITS = 20;
  type
    TdtPolyRef = UInt64;
  {$else}
  type
    TdtPolyRef = Cardinal;
  {$endif}
  PdtPolyRef = ^TdtPolyRef;

/// A handle to a tile within a navigation mesh.
/// @ingroup detour
  {$ifdef DT_POLYREF64}
    TdtTileRef = UInt64;
  {$else}
    TdtTileRef = Cardinal;
  {$endif}
  PdtTileRef = ^TdtTileRef;


implementation


end.