unit RN_Helper;
interface
uses SysUtils;

type
  TCompareIndicesFunction = function (const A,B: Pointer): Integer;

  procedure qSort(aBuffer: Pointer; aCount: Integer; aSize: Integer; Compare: TCompareIndicesFunction);

  function Sscanf(const s: string; const fmt: string; const Pointers: array of Pointer): Integer;

implementation


//todo: Test this sort with array of record containing array of Integers
procedure QSort(aBuffer: Pointer; aCount: Integer; aSize: Integer; Compare: TCompareIndicesFunction);
  procedure QuickSort(aBuffer: Pointer; iLo, iHi: Integer);
  var Lo, Hi: Integer; Mid, T: Pointer;
  begin
    Lo := iLo;
    Hi := iHi;
    Mid := Pointer(Cardinal(aBuffer) + ((Lo + Hi) div 2) * aSize);
    repeat
      while Compare(Pointer(Cardinal(aBuffer) + Lo*aSize), Mid) < 0 do
        Inc(Lo);
      while Compare(Pointer(Cardinal(aBuffer) + Hi*aSize), Mid) > 0 do
        Dec(Hi);
      if Lo <= Hi then begin
        getmem(T, aSize);
        Move(Pointer(Cardinal(aBuffer) + Lo*aSize)^, T^, aSize);
        Move(Pointer(Cardinal(aBuffer) + Hi*aSize)^, Pointer(Cardinal(aBuffer) + Lo*aSize)^, aSize);
        Move(T^, Pointer(Cardinal(aBuffer) + Hi*aSize)^, aSize);
        freemem(T);
        Inc(Lo);
        Dec(Hi);
      end;
    until Lo > Hi;
    if Hi > iLo then
      QuickSort(aBuffer, iLo, Hi);
    if Lo < iHi then
      QuickSort(aBuffer, Lo, iHi);
  end;
begin
  QuickSort(aBuffer, 0, aCount-1);
end;

{ Sscanf выполняет синтаксический разбор входной строки. Параметры...

s - входная строка для разбора
fmt - 'C' scanf-форматоподобная строка для управления разбором
%d - преобразование в Long Integer
%f - преобразование в Extended Float
%s - преобразование в строку (ограничено пробелами)
другой символ - приращение позиции s на "другой символ"
пробел - ничего не делает
Pointers - массив указателей на присваиваемые переменные

результат - количество действительно присвоенных переменных

Например, ...
Sscanf('Name. Bill   Time. 7:32.77   Age. 8',
'. %s . %d:%f . %d', [@Name, @hrs, @min, @age]);

возвратит ...
Name = Bill  hrs = 7  min = 32.77  age = 8 }

function Sscanf(const s: string; const fmt: string;

  const Pointers: array of Pointer): Integer;
var

  i, j, n, m: integer;
  s1: string;
  L: Integer;
  X: Single;

  function GetInt: Integer;
  begin
    s1 := '';
    while (s[n] = ' ') and (Length(s) > n) do
      inc(n);
    while (n <= Length(s)) and (s[n] in ['0'..'9', '+', '-']) do
    begin
      s1 := s1 + s[n];
      inc(n);
    end;
    Result := Length(s1);
  end;

  function GetFloat: Integer;
  begin
    s1 := '';
    while (s[n] = ' ') and (Length(s) > n) do
      inc(n);
    while (n <= Length(s)) and (s[n] in ['0'..'9', '+', '-', '.', 'e', 'E']) do
    begin
      s1 := s1 + s[n];
      inc(n);
    end;
    Result := Length(s1);
  end;

  function GetString: Integer;
  begin
    s1 := '';
    while (s[n] = ' ') and (Length(s) > n) do
      inc(n);
    while (n <= Length(s)) and (s[n] <> ' ') do
    begin
      s1 := s1 + s[n];
      inc(n);
    end;
    Result := Length(s1);
  end;

  function ScanStr(c: Char): Boolean;
  begin
    while (s[n] <> c) and (Length(s) > n) do
      inc(n);
    inc(n);

    if (n <= Length(s)) then
      Result := True
    else
      Result := False;
  end;

  function GetFmt: Integer;
  begin
    Result := -1;

    while (TRUE) do
    begin
      while (fmt[m] = ' ') and (Length(fmt) > m) do
        inc(m);
      if (m >= Length(fmt)) then
        break;

      if (fmt[m] = '%') then
      begin
        inc(m);
        case fmt[m] of
          'd': Result := vtInteger;
          'f': Result := vtExtended;
          's': Result := vtString;
        end;
        inc(m);
        break;
      end;

      if (ScanStr(fmt[m]) = False) then
        break;
      inc(m);
    end;
  end;

begin

  n := 1;
  m := 1;
  Result := 0;

  for i := 0 to High(Pointers) do
  begin
    j := GetFmt;

    case j of
      vtInteger:
        begin
          if GetInt > 0 then
          begin
            L := StrToInt(s1);
            Move(L, Pointers[i]^, SizeOf(LongInt));
            inc(Result);
          end
          else
            break;
        end;

      vtExtended:
        begin
          if GetFloat > 0 then
          begin
            X := StrToFloat(s1);
            Move(X, Pointers[i]^, SizeOf(Single));
            inc(Result);
          end
          else
            break;
        end;

      vtString:
        begin
          if GetString > 0 then
          begin
            Move(s1, Pointers[i]^, Length(s1) + 1);
            inc(Result);
          end
          else
            break;
        end;

    else
      break;
    end;
  end;
end;


end.
