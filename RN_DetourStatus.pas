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
unit RN_DetourStatus;
interface

type TdtStatus = Cardinal;

// High level status.
const DT_FAILURE: Cardinal = Cardinal(1) shl 31;      // Operation failed.
const DT_SUCCESS: Cardinal = Cardinal(1) shl 30;      // Operation succeed.
const DT_IN_PROGRESS: Cardinal = Cardinal(1) shl 29;    // Operation still in progress.

// Detail information for status.
const DT_STATUS_DETAIL_MASK: Cardinal = $0ffffff;
const DT_WRONG_MAGIC: Cardinal = 1 shl 0;    // Input data is not recognized.
const DT_WRONG_VERSION: Cardinal = 1 shl 1;  // Input data is in wrong version.
const DT_OUT_OF_MEMORY: Cardinal = 1 shl 2;  // Operation ran out of memory.
const DT_INVALID_PARAM: Cardinal = 1 shl 3;  // An input parameter was invalid.
const DT_BUFFER_TOO_SMALL: Cardinal = 1 shl 4;  // Result buffer for the query was too small to store all results.
const DT_OUT_OF_NODES: Cardinal = 1 shl 5;    // Query ran out of nodes during search.
const DT_PARTIAL_RESULT: Cardinal = 1 shl 6;  // Query did not reach the end location, returning best guess.

function dtStatusSucceed(status: TdtStatus): Boolean;
function dtStatusFailed(status: TdtStatus): Boolean;
function dtStatusInProgress(status: TdtStatus): Boolean;
function dtStatusDetail(status: TdtStatus; detail: Cardinal): Boolean;


implementation


// Returns true of status is success.
function dtStatusSucceed(status: TdtStatus): Boolean;
begin
  Result := (status and DT_SUCCESS) <> 0;
end;

// Returns true of status is failure.
function dtStatusFailed(status: TdtStatus): Boolean;
begin
  Result := (status and DT_FAILURE) <> 0;
end;

// Returns true of status is in progress.
function dtStatusInProgress(status: TdtStatus): Boolean;
begin
  Result := (status and DT_IN_PROGRESS) <> 0;
end;

// Returns true if specific detail is set.
function dtStatusDetail(status: TdtStatus; detail: Cardinal): Boolean;
begin
  Result := (status and detail) <> 0;
end;

end.
