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

unit RN_PerfTimer;
interface
uses Windows;


function getPerfTime(): Int64;
function getPerfDeltaTimeUsec(const start, &end: Int64): Integer;


implementation


function getPerfTime(): Int64;
var count: Int64;
begin
  QueryPerformanceCounter(count);
  Result := count;
end;

function getPerfDeltaTimeUsec(const start, &end: Int64): Integer;
var freq, elapsed: Int64;
begin
  freq := 0;
  if (freq = 0) then
    QueryPerformanceFrequency(freq);
  elapsed := &end - start;
  Result := Trunc(elapsed*1000000 / freq);
end;

end.
